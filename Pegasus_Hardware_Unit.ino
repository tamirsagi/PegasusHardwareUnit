/*
* PEGASUS HARDWARE UNIT
*
@author - Tamir Sagi, 2015 - 2016.
This code handles the hardware unit of Pegasus Vehicle

Message protocol = key:value,key:value,key:value....#           //# defines end of message
Max Digital Speed 0-255
Steer Angle is 0-40 degrees each side ( Right 50-90, Left, 90-130)
There are 7 ultra sonic sensors on the vehicle which are handled here in array called "sonar".
the system is waiting for handshake and sends echo every 5 seconds until handshake is done.
*/

#include <AFMotor.h>
#include <Servo.h>
#include <NewPing\NewPing.h>


///////////////// Tachometer \\\\\\\\\\\\\\

const int tachometer_interupt_port = 18;
volatile boolean revolution_occured = false;
unsigned int revolution_count = 0;
unsigned long tachometer_current_time = 0;
unsigned long tachometer_previous_time = 0;
unsigned int rps = 0;
const int wheelSlots = 5;
int lastValue = -1;


////////////// Ultra Sonic Sensors \\\\\\\\\\\\\\

const int  MAX_DISTANCE = 60;
const int NUMBER_OF_ULTRA_SONIC_SENSORS = 7;
const int sensors_ids[NUMBER_OF_ULTRA_SONIC_SENSORS] = { 1, 2, 3, 4, 5, 6, 7 };
const int sensors_triger_digital_pins[NUMBER_OF_ULTRA_SONIC_SENSORS] = { 34, 36, 46, 44, 42, 40, 38 };
const int sensors_echo_digital_pins[NUMBER_OF_ULTRA_SONIC_SENSORS] = { 35, 37,47, 45, 43, 41, 39 };
const int sensors_max_distance[NUMBER_OF_ULTRA_SONIC_SENSORS] = { 32, 24, 24, 32, 32, 24, 24 };
NewPing sonar[NUMBER_OF_ULTRA_SONIC_SENSORS] = { NewPing(sensors_triger_digital_pins[0], sensors_echo_digital_pins[0], sensors_max_distance[0]),
												 NewPing(sensors_triger_digital_pins[1], sensors_echo_digital_pins[1], sensors_max_distance[1]),
												 NewPing(sensors_triger_digital_pins[2], sensors_echo_digital_pins[2], sensors_max_distance[2]),
												 NewPing(sensors_triger_digital_pins[3], sensors_echo_digital_pins[3], sensors_max_distance[3]),
												 NewPing(sensors_triger_digital_pins[4], sensors_echo_digital_pins[4], sensors_max_distance[4]),
												 NewPing(sensors_triger_digital_pins[5], sensors_echo_digital_pins[5], sensors_max_distance[5]),
												 NewPing(sensors_triger_digital_pins[6], sensors_echo_digital_pins[6], sensors_max_distance[6])
											  };
bool sensorStatesArray[NUMBER_OF_ULTRA_SONIC_SENSORS] = { false, false, false, false, false, false, false };
int sensors_last_values[NUMBER_OF_ULTRA_SONIC_SENSORS] = { -1, -1, -1, -1, -1, -1, -1 };
unsigned int uS = 0;
unsigned long us_sensors_current_time = 0;
unsigned long us_sensors_previous_time = 0;

/////////////// Motors \\\\\\\\\\\\\\\\\\\\\

AF_DCMotor mBackMotor(4, MOTOR34_64KHZ);
Servo mSteerMotor;


//////////////////////////////////////////////////////////////////
/*
Messages And protocol Abberviations
*/

const char START_MESSAGE = '$';						//Begining message 
const char END_MESSAGE = '#';						//end of message 
const char MESSAGE_SAPERATOR = ',';				   //message saparator , part of the messaging protocol
const char MESSAGE_KEY_VALUE_SAPERATOR = ':';


const char *KEY_MESSAGE_TYPE = "MT";				 // MT = Message Type
const int VALUE_MESSAGE_TYPE_INFO = 1000;			 //INFO
const int VALUE_MESSAGE_TYPE_ACTION = 2000;			 //ACTION
const int VALUE_MESSAGE_TYPE_SETTINGS = 3000;		 //Settings
const int VALUE_MESSAGE_TYPE_ERROR = 4000;			 //ERROR
const char *KEY_INFO_TYPE = "IT";					 //IT = Info Type key


/*
Incoming
*/
const char *KEY_VEHICLE_ACTION_TYPE = "VA";			//Vehicle Action Type key
const char *KEY_DIGITAL_SPEED = "DS";				//Digital Speed Key
const char *KEY_ROTATION_ANGLE = "RA";				//Rotation Angle key
const char *KEY_STEERING_DIRECTION = "SD";			// Steering Direction key
const char *KEY_DRIVING_DIRECTION = "DD";			//Driving Direction key

const char *KEY_SETTINGS_ACTION_TYPE = "ST";		//Settings Type key
const String KEY_PREFIX_ULTRA_SONIC_SENSOR = "S";	//Settings Type key

const String VALUE_STEERING_NONE = "N";				//N = None, no steering
const String VALUE_STEERING_RIGHT = "R";			//R = Right,  Right Steering
const String VALUE_STEERING_LEFT = "L";				//L = Left, Left Steering
const String VALUE_DRIVING_FORWARD = "F";			//F = Forward, driving Forward
const String VALUE_DRIVING_BACKWARD = "B";			//B = Backward, driving backward

//Sensors
const char *KEY_SENSOR_ID = "SID";					//Sensor ID
const char *KEY_SENSOR_STATE = "SS";				//Sensor State
const int VALUE_CHANGE_SENSOR_STATE_DISABLE = 0;
const int VALUE_CHANGE_SENSOR_STATE_ENABLE = 1;


/*
Outgoing
*/

const String VALUE_STATUS = "STATUS";
const String VALUE_SENSOR_DATA = "SENSOR_DATA";			
const char* KEY_SENSOR_DATA = "SD";						//SD = Sensor Data
const char* KEY_STATUS = "ST";						  //ST = STATUS

const String VALUE_OK = "OK";
const String VALUE_ERROR = "ER";

//////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////// ACTIONS \\\\\\\\\\\\\\\\\\\\\\\
/* Action For Arduino */
const int ACTION_DRIVING_DIRECTION = 2001;
const int ACTION_BACK_MOTOR = 2002;
const int ACTION_STEER_MOTOR = 2003;
const int ACTION_CHANGE_SENSOR_STATE = 2004;


/////////////// INFO \\\\\\\\\\\\\\\\\\

const int VALUE_HARDWARE_STATUS_READY = 100;
const int VALUE_LOGIC_UNIT_READY = 200;

/////////////SETTINGS \\\\\\\\\\\\\\\\\\\\\

const int SETTINGS_SET_SENSORS = 3001;


//////////////////////////////////////////////////////////////////////////////////////////////////////
const int UPDATE_INTERVAL = 60 * 1000 * 2;		   // 2 minutes update interval
const int MIN_DIGITAL_SPEED = 0;				  //MIN digital speed value
const int MAX_DIGITAL_SPEED = 255;				 //MAx digital speed value
const int STRAIGHT_STEER_ANGLE = 90;			//MAX steer angle value
const int MIN_STEER_ANGLE = 50;					//MIN steer angle value
const int MAX_STEER_ANGLE = 130;				//MAX steer angle value

String mInputMessage = "";						//keeps incoming messages from Raspberry Pi
boolean mIsLogicUnitReady = false;				//indicates whether the Raspberry pi is ready for communication
double mCurrentteeringAngle;					//keep the current steering angle
int mLastDigitalSpeed;							//keep last digital speed
int mCurrentDrivingDirection;                   //Keep Last Driving direction (Forward, Backword)...
double mLastSteeringAngle;						//keep last steering angle
String mSystemInitialStatus = "";				//keep initiate status of the system , we send this string to logic unit

const int TACHOMETER_INTERVAL_TIME = 1 * 200;		//Measure sensors every 0.2 sec
const int US_INTERVAL_TIME = 1 * 500;				//Measure sensors every 0.5 sec

bool isStopped = true;
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine (ISR)
void revolution()
{
	if (!isStopped){
		revolution_count++;
	}
}

void setup(){

    /*  Tachometer  */
    pinMode(tachometer_interupt_port, INPUT);
    attachInterrupt(digitalPinToInterrupt(tachometer_interupt_port), revolution, RISING);  // attach interrupt handler

    /* Serial Port */
    Serial.begin(115200);						//turn the serial protocol on
    while (!Serial);							//Wait for Serial

    /*
    Motors
    */
    mSteerMotor.attach(9);				//attach motor on the shiled
    mSteerMotor.write(STRAIGHT_STEER_ANGLE);		//from 0-40 degrees to 90-130
    mLastSteeringAngle = STRAIGHT_STEER_ANGLE;
    mBackMotor.run(RELEASE);				//reset back motor
    mBackMotor.run(FORWARD);				//reset To Forward direction by default
    mCurrentDrivingDirection = FORWARD;

    // Build up the status message
    mSystemInitialStatus += START_MESSAGE
        + String(KEY_MESSAGE_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_MESSAGE_TYPE_INFO + MESSAGE_SAPERATOR
        + String(KEY_INFO_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_STATUS + MESSAGE_SAPERATOR
        + String(KEY_STATUS) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_HARDWARE_STATUS_READY
        + END_MESSAGE;



}

void loop(){


	//   wait for handshake, we send status and wait 3 seconds
    if (!mIsLogicUnitReady){
        Serial.println(mSystemInitialStatus);	//send message to logic unit
        delay(5000);	//sleep 5 seconds
    }
	else{		//system is working we send measurements every 1 sec
			sendUltraSonicSensorData();
			sendTachometerData();
		}
}


/*
Serial Event occurs whenever a new data comes in the hardware Serial. this routin is run between each
time loop() runs.
*/
void serialEvent(){
    if (Serial.available()){
        delay(100);
        char temp[] = " ";
        boolean procceed = false;
        while (Serial.available()){
            //read one byte
            temp[0] = Serial.read();
            if (mInputMessage.length() == 0 && temp[0] == START_MESSAGE)
                procceed = true;
            else if (procceed){
                //check if its the end of the message
                if (temp[0] == END_MESSAGE && mInputMessage.length() > 0){
					handleMessage(mInputMessage);
					mInputMessage = "";
                    procceed = false;
                }
                else
                    mInputMessage += temp;
            }
        }
    }
}



/*
Method build up a string including various components(Motors,Sensors etc..) status
prior working.
@returns A string to send over the logic unit
*/
String systemCheckUp(){
    String status = "";



}


/**
* Method will parse the message and call relevant methods
* The Message is in format MessageType(Vehicle_Action, Info,....), params,params...
*/
void handleMessage(String msgFromRaspberry){
	if (!isNullOrEmpty(msgFromRaspberry)){
        int len = msgFromRaspberry.length() + 1;						// + 1 to keep eos '\0' char
        char* msgToHandle = new char[len];
        msgFromRaspberry.toCharArray(msgToHandle, len);					//convert the String to char[] in order to use strtok
        String messageType = getValue(msgToHandle, KEY_MESSAGE_TYPE);	 //get Value
        if (!isNullOrEmpty(messageType) && isNumber(messageType)){
            int messageTypeValue = messageType.toInt();					//return Message type
            switch (messageTypeValue){
				case  VALUE_MESSAGE_TYPE_INFO:
				{
				  handleInfoMessage(msgToHandle);
				  break;
				}
				case VALUE_MESSAGE_TYPE_ACTION:
				{
				   handleAction(msgToHandle);
				   break;
				}
				case VALUE_MESSAGE_TYPE_SETTINGS:
					handleSettings(msgToHandle);
					break;
				case VALUE_MESSAGE_TYPE_ERROR:

					break;

            }//end of switch

        }
        delete[] msgToHandle;										//free allocation
    }

}//end of handle message


/**
* return the value for given key out of releveant Key:Value within the string
*/
String getValue(char *msg, const char *key){
    char *buffer = new char[strlen(msg) + 1];						//allocate buffer to backup msg
    char *relevant = strcpy(buffer, msg);							//copy original msg to buffer
    relevant = strstr(relevant, key);                                                       //find the key
    if (isNullOrEmpty(relevant))
        return "";
    relevant = strchr(relevant, MESSAGE_KEY_VALUE_SAPERATOR);		              //from the key find the ':'
    if (isNullOrEmpty(relevant))
        return "";
    relevant = strtok(relevant, ":,");								//split the message from ':' to ',' which is the value
    if (isNullOrEmpty(relevant))
        return "";

    String value(relevant);											//convert char* to String

    delete[] buffer;												//DELETE allocation
    return value;
}


/*
	handles info messages
*/
void handleInfoMessage(char* msgToHandle){

    String infoType = getValue(msgToHandle, KEY_INFO_TYPE);		//get Value
    if (infoType.compareTo(VALUE_STATUS) == 0){
        String status = getValue(msgToHandle, KEY_STATUS);		//get Value
        if (!isNullOrEmpty(status) && isNumber(status)){
            int statusCode = status.toInt();
            switch (statusCode){
            case VALUE_LOGIC_UNIT_READY:
                mIsLogicUnitReady = true;
                break;
            }
        }
    }
}

/*
 handles messages type of Action (Vehicle manouvering, Sensors etc..)
*/
void handleAction(char* msgToHandle){
    String actionType = getValue(msgToHandle, KEY_VEHICLE_ACTION_TYPE);		//get Value
    if (!isNullOrEmpty(actionType) && isNumber(actionType)) {
        int actionTypeValue = actionType.toInt();					//return Action type
        switch (actionTypeValue){
			case  ACTION_DRIVING_DIRECTION:
			{
				handleDirections(msgToHandle);
				break;
			}
			case ACTION_BACK_MOTOR:
			{
				handleBackMotor(msgToHandle);
				break;
			}
			case ACTION_STEER_MOTOR:
			{
				handleSteerMotor(msgToHandle);
				break;
			}
			case ACTION_CHANGE_SENSOR_STATE:
				handleSensorStateChanged(msgToHandle);
				break;
        }
    }

}

void handleSettings(char* msgToHandle){
	String actionType = getValue(msgToHandle, KEY_SETTINGS_ACTION_TYPE);	//get Settings
	if (!isNullOrEmpty(actionType) && isNumber(actionType)) {
		int actionTypeValue = actionType.toInt();							//return Action type
		switch (actionTypeValue){
		case SETTINGS_SET_SENSORS:
			//setSensors(msgToHandle);
			break;
		}
	}

}

/**
* Check whether a string is null or empty
*/
boolean isNullOrEmpty(String msg){
    return msg == NULL || msg.length() <= 0;
}

/**
* Check whether a string is null or empty
*/
boolean isNullOrEmpty(char *msg){
    return msg == NULL || strlen(msg) <= 0;
}

/**
* Check whether a string is an integer or not
*/
boolean isNumber(String msg){
    int len = msg.length();
    if (len > 0){
        int dotsCounter = 0;
		if (msg[0] == '.'){
			return false;
		}
        for (int i = 1; i < len; i++){
            if (msg[i] == '.')
                dotsCounter++;
            else if ('0' >= msg[i] && msg[i] >= '9')
                return false;
            if (dotsCounter > 1)
                return false;
        }
        return true;
    }
}

/*
DEPERECATED - Did not work well
set sensor max distance 
*/
void setSensors(char* sensorData){
	char* relevantKEy = NULL;
	for (int i = 0; i < NUMBER_OF_ULTRA_SONIC_SENSORS; i++){
		String key = KEY_PREFIX_ULTRA_SONIC_SENSOR + (i + 1);
		int len = key.length() + 1;											// + 1 to keep eos '\0' char
		relevantKEy = new char[len];
		key.toCharArray(relevantKEy, len);									//convert the String to char[] in order to use strtok
		String sensorMaxDistance = getValue(sensorData, relevantKEy);		//get Value
		if (!isNullOrEmpty(sensorMaxDistance) && isNumber(sensorMaxDistance)){
			int maxDistance = sensorMaxDistance.toInt();
			sonar[i] = NewPing(sensors_triger_digital_pins[i], sensors_echo_digital_pins[i], maxDistance);
		}
	}
}



/**
* Method handles actions for back motor
*/
void handleBackMotor(char * msgToHandle){
    String sd = getValue(msgToHandle, KEY_DIGITAL_SPEED);
    if (!isNullOrEmpty(sd) && isNumber(sd)){
        int digitalSpeed = sd.toInt();
        if (MIN_DIGITAL_SPEED <= digitalSpeed && digitalSpeed <= MAX_DIGITAL_SPEED){
            mLastDigitalSpeed = digitalSpeed;
            changeBackMotorSpeed(mLastDigitalSpeed);
        }
    }
}

/**
* Method handles actions for steer motor
*/
void handleSteerMotor(char * msgToHandle){
    String rotationAngle = getValue(msgToHandle, KEY_ROTATION_ANGLE);
    for (int i = 0; i < rotationAngle.length(); i++)
        Serial.println((int)rotationAngle.charAt(i));
    String direction = getValue(msgToHandle, KEY_STEERING_DIRECTION);
    for (int i = 0; i < direction.length(); i++)
        Serial.println((int)direction.charAt(i));
    if (!isNullOrEmpty(rotationAngle) && !isNullOrEmpty(direction)){
        double angle = 0;
        if (isNumber(rotationAngle)){
            int len = rotationAngle.length();
            char* ra = new char[len + 1];
            rotationAngle.toCharArray(ra, len + 1);
            double angle = atof(ra);
			mLastSteeringAngle = angle;
            delete[] ra;
            if (direction.compareTo(VALUE_STEERING_RIGHT) == 0)
                turnSteeringRight(angle);
            else if (direction.compareTo(VALUE_STEERING_LEFT) == 0)
                turnSteeringLeft(angle);
        }
    }
}


/*
Method handles direction changing
*/
void handleDirections(char *msgToHandle){
    String direction = getValue(msgToHandle, KEY_DRIVING_DIRECTION);
    mBackMotor.run(RELEASE);							//reset back motor
    if (direction.compareTo(VALUE_DRIVING_FORWARD) == 0) {
        mCurrentDrivingDirection = FORWARD;
        mBackMotor.run(FORWARD);
    }else if (direction.compareTo(VALUE_DRIVING_BACKWARD) == 0){
        mCurrentDrivingDirection = BACKWARD;
        mBackMotor.run(BACKWARD);
    }
}

/*
handles sensor change state
*/
void handleSensorStateChanged(char* msgToHandle){
	String sensorId = getValue(msgToHandle, KEY_SENSOR_ID);
	String sensorState = getValue(msgToHandle, KEY_SENSOR_STATE);
	if (!isNullOrEmpty(sensorId) && isNumber(sensorId) &&
		!isNullOrEmpty(sensorState) && isNumber(sensorState)){
		int id = sensorId.toInt();
		int state = sensorState.toInt();
		sensorStatesArray[id - 1] = state;
	}
}

/*
	Message send sensor Data
*/
void sendUltraSonicSensorData(){
	us_sensors_current_time = millis();
	if (us_sensors_current_time - us_sensors_previous_time >= US_INTERVAL_TIME){
		for (int i = 0; i < NUMBER_OF_ULTRA_SONIC_SENSORS; i++){
			if (sensorStatesArray[i]){	//sensor is enable and should send data
				int uS = sonar[i].ping();
				int dist = uS / US_ROUNDTRIP_CM;
				if (sensors_last_values[i] != dist){
					sensors_last_values[i] = dist;
					String sensorDataMsg = START_MESSAGE
						+ String(KEY_MESSAGE_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_MESSAGE_TYPE_INFO + MESSAGE_SAPERATOR
						+ String(KEY_INFO_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_SENSOR_DATA + MESSAGE_SAPERATOR
						+ String(KEY_SENSOR_ID) + MESSAGE_KEY_VALUE_SAPERATOR + sensors_ids[i] + MESSAGE_SAPERATOR
						+ String(KEY_SENSOR_DATA) + MESSAGE_KEY_VALUE_SAPERATOR + dist
						+ END_MESSAGE;
					Serial.println(sensorDataMsg);
				}
			}
		}
		us_sensors_previous_time = us_sensors_current_time;
	}
}

/*
	mesaure RPM and send data
*/
void sendTachometerData(){
	tachometer_current_time = millis();
	if (tachometer_current_time - tachometer_previous_time >= TACHOMETER_INTERVAL_TIME && revolution_count > 0) {
		rps = (revolution_count / wheelSlots);		
		String sensorDataMsg = START_MESSAGE
			+ String(KEY_MESSAGE_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_MESSAGE_TYPE_INFO + MESSAGE_SAPERATOR
			+ String(KEY_INFO_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_SENSOR_DATA + MESSAGE_SAPERATOR
			+ String(KEY_SENSOR_ID) + MESSAGE_KEY_VALUE_SAPERATOR + tachometer_interupt_port + MESSAGE_SAPERATOR
			+ String(KEY_SENSOR_DATA) + MESSAGE_KEY_VALUE_SAPERATOR + rps
			+ END_MESSAGE;
		Serial.println(sensorDataMsg);
		tachometer_previous_time = tachometer_current_time;
		revolution_count = 0;
	}



}

/*
*   Method change back dc motor speed
*/
void changeBackMotorSpeed(int digitalSpeed){
	isStopped = digitalSpeed == 0;
	if (isStopped){
		mBackMotor.run(RELEASE);
	}
    else
    {
        mBackMotor.run(mCurrentDrivingDirection);
        mBackMotor.setSpeed(digitalSpeed);
    }
	
}


/*
turning right, from 0-40 degrees to 50-90
*/
void turnSteeringRight(double angle){
    mSteerMotor.write(STRAIGHT_STEER_ANGLE - angle);
}

/*
turning left, from 0-40 degrees to 90-130
*/
void turnSteeringLeft(double angle){
    mSteerMotor.write(STRAIGHT_STEER_ANGLE + angle);		//from 0-40 degrees to 90-130
}

