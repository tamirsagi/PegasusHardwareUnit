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
unsigned long current_time = 0;
unsigned long previous_time = 0;
unsigned int rps = 0;
const int wheelSlots = 5;


////////////// Ultra Sonic Sensors \\\\\\\\\\\\\\

const int  MAX_DISTANCE = 20;
const int NUMBER_OF_ULTRA_SONIC_SENSORS = 7;
const int sensors_ids[NUMBER_OF_ULTRA_SONIC_SENSORS] = { 1, 2, 3, 4, 5, 6, 7 };
const int sensors_triger_digital_pins[NUMBER_OF_ULTRA_SONIC_SENSORS] = { 34, 36, 46, 44, 42, 40, 38 };
const int sensors_echo_digital_pins[NUMBER_OF_ULTRA_SONIC_SENSORS] = { 35, 37,47, 45, 43, 41, 39 };
NewPing sonar[NUMBER_OF_ULTRA_SONIC_SENSORS] = { NewPing(sensors_triger_digital_pins[0], sensors_echo_digital_pins[0], MAX_DISTANCE),
											     NewPing(sensors_triger_digital_pins[1], sensors_echo_digital_pins[1], MAX_DISTANCE),
											     NewPing(sensors_triger_digital_pins[2], sensors_echo_digital_pins[2], MAX_DISTANCE),
											     NewPing(sensors_triger_digital_pins[3], sensors_echo_digital_pins[3], MAX_DISTANCE),
											     NewPing(sensors_triger_digital_pins[4], sensors_echo_digital_pins[4], MAX_DISTANCE),
											     NewPing(sensors_triger_digital_pins[5], sensors_echo_digital_pins[5], MAX_DISTANCE),
											     NewPing(sensors_triger_digital_pins[6], sensors_echo_digital_pins[6], MAX_DISTANCE) 
											  };
bool sensorStatesArray[NUMBER_OF_ULTRA_SONIC_SENSORS] = { false, false, false, false, false, false, false };
unsigned int uS = 0;

/////////////// Motors \\\\\\\\\\\\\\\\\\\\\

AF_DCMotor mBackMotor(4, MOTOR34_64KHZ);
Servo mSteerMotor;


//////////////////////////////////////////////////////////////////
/*
Messages And protocol Abberviations
*/
const char *KEY_MESSAGE_TYPE = "MT";				 // MT = Message Type
const int VALUE_MESSAGE_TYPE_INFO = 1000;			 //INFO
const int VALUE_MESSAGE_TYPE_ACTION = 2000;			 //ACTION
const char *KEY_INFO_TYPE = "IT";					 //IT = Info Type key

const char START_MESSAGE = '$';						//Begining message 
const char END_MESSAGE = '#';						//end of message 
const char MESSAGE_SAPERATOR = ',';				   //message saparator , part of the messaging protocol
const char MESSAGE_KEY_VALUE_SAPERATOR = ':';

/*
Incoming
*/
const char *KEY_VEHICLE_ACTION_TYPE = "VA";			//Vehicle Action Type key
const char *KEY_DIGITAL_SPEED = "DS";				//Digital Speed Key
const char *KEY_ROTATION_ANGLE = "RA";				//Rotation Angle key
const char *KEY_STEERING_DIRECTION = "SD";			// Steering Direction key
const char *KEY_DRIVING_DIRECTION = "DD";			//Driving Direction key

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
//const String KEY_SERIAL_PORT = "SP";				 //SP = Serial Port
//const String KEY_STEER_MOTOR = "SM";				//SM = Steering Motor
//const String KEY_BACK_MOTOR = "BM";				//BM = Back Motor

const String VALUE_OK = "OK";
const String VALUE_ERROR = "ER";

//////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////// ACTIONS \\\\\\\\\\\\\\\\\\\\\\\
/* Action For Arduino */
const int ACTION_DRIVING_DIRECTION = 0;
const int ACTION_BACK_MOTOR = 1;
const int ACTION_STEER_MOTOR = 2;
const int ACTION_CHANGE_SENSOR_STATE = 3;

/////////////// INFO \\\\\\\\\\\\\\\\\\

const int VALUE_HARDWARE_STATUS_READY = 100;
const int VALUE_LOGIC_UNIT_READY = 200;

//////////////////////////////////////////////////////////////////////////////////////////////////////
const int UPDATE_INTERVAL = 60 * 1000 * 3;		   // 5 minutes update interval
const int MIN_DIGITAL_SPEED = 0;				  //MIN digital speed value
const int MAX_DIGITAL_SPEED = 255;				 //MAx digital speed value
const int STRAIGHT_STEER_ANGLE = 90;			//MAX steer angle value
const int MIN_STEER_ANGLE = 50;					//MIN steer angle value
const int MAX_STEER_ANGLE = 130;				//MAX steer angle value

String mInputMessage = "";						//keeps incoming messages from Raspberry Pi
boolean mReceivedEntireMessage = false;			//whether the string is complete
boolean mIsLogicUnitReady = false;				//indicates whether the Raspberry pi is ready for communication
double mCurrentteeringAngle;					//keep the current steering angle
int mLastDigitalSpeed;							//keep last digital speed
int mCurrentDrivingDirection;                            //Keep Last Driving direction (Forward, Backword)...
double mLastSteeringAngle;						//keep last steering angle
String mSystemInitialStatus = "";				//keep initiate status of the system , we send this string to logic unit

int elapsedTime = 0;
int measure = 0;
int lastMeasure = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine (ISR)
void revolution()
{
    revolution_occured = true;
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


	/*mSystemInitialStatus = "MT:2000, VA : 3, SID : 1, SS : 1";
	Serial.println("Before");
	for (int i = 0; i < 7; i++){
		Serial.print("Sensor ");
		Serial.print(i + 1);
		Serial.print(sensorStatesArray[i]);
		Serial.println();
	}
	sendUltraSonicSensorData();
	handleMessage(mSystemInitialStatus);
	Serial.println("after");
	for (int i = 0; i < 7; i++){
		Serial.print("Sensor ");
		Serial.print(i + 1);
		Serial.print(":");
		Serial.print(sensorStatesArray[i]);
		Serial.println();
	}
	sendUltraSonicSensorData();*/
}

void loop(){

	/*
		handle incoming messages
	*/
	if (mReceivedEntireMessage){
		handleMessage(mInputMessage);
		//clear data
		mInputMessage = "";
		mReceivedEntireMessage = false;
	}
	lastMeasure = millis();
    /*
        wait for handshake, we send status and wait 3 seconds
     */
    if (!mIsLogicUnitReady){
        Serial.println(mSystemInitialStatus);	//send message to logic unit
        delay(5000);	//sleep 5 seconds
    }

	else if (lastMeasure - measure > 1000){//system is working we send measurements every 1 sec
			measure = lastMeasure;
			sendUltraSonicSensorData();
			//sendTachometerData();
		}
		

	//// scan from 0 to 180 degrees
	//for (int angle = 50; angle <= 130; angle++)
	//{
	//	mSteerMotor.write(angle);
	//	delay(5);
	//}
	////// now scan back from 180 to 0 degrees
	//for (int angle = 130; angle >= 50; angle--)
	//{
	//	mSteerMotor.write(angle);
	//	delay(5);
	//}
	/*Serial.println("Digital Speed");
	handleMessage(mInputMessage);
	Serial.println("Rotation R");
	mInputMessage = "AT:2,SD:R,RA:80.7";
	handleMessage(mInputMessage);
	Serial.println("Rotation L ");
	mInputMessage = "AT:2,SD:L,RA:130";
	handleMessage(mInputMessage);*/


	//RMP testing

	/*unsigned int time = 0, RPM = 0;
	Serial.println("Reading RPM.....");
	time = delay1();
	Serial.println("Please Wait.....");
	RPM = (time * 12) / 5;
	Serial.println("RPM=");
	Serial.println(RPM);
	delay(2000);*/
	//if (revolution_occured) {
	//	revolution_count++;
	//	revolution_occured = !revolution_occured;
	//}

	//current_time = millis();
	//if (current_time - previous_time >= 1000) {
	//	rpm = (revolution_count / 5);
	//	Serial.print("rpm = ");
	//	Serial.println(rpm);
	//	//Serial.print ("counts = ");
	//	//Serial.println (revolution_count); 
	//	previous_time = current_time;
	//	revolution_count = 0;
	//}

	///////Sonar testing

	//delay(50);

	//for (int i = 0; i < NUMBER_OF_ULTRA_SONIC_SENSORS; i++){

	//	int uS = sonar[i].ping();
	//	Serial.println(sensors_ids[i]);
	//	Serial.print("Ping: ");
	//	Serial.print(uS / US_ROUNDTRIP_CM);
	//	Serial.println("cm");
	//	delay(500);
	//}
	//

	/*
	Serial.println("SONAR 2!!!!!!!!!!!!");
	uS = sonar[1].ping();
	Serial.println("SONAR 1!!!!!!!!!!!!");
	Serial.print("Ping: ");
	Serial.print(uS / US_ROUNDTRIP_CM);
	Serial.println("cm");*/

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
                    mReceivedEntireMessage = true;
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
	Serial.println(msgFromRaspberry);
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
				Serial.println("here");
				handleSensorStateChanged(msgToHandle);
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



/**
* Method handles actions for back motor
*/
void handleBackMotor(char * msgToHandle){
    String sd = getValue(msgToHandle, KEY_DIGITAL_SPEED);
    if (!isNullOrEmpty(sd) && isNumber(sd))
    {
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
    if (direction.compareTo(VALUE_DRIVING_FORWARD) == 0)
    {
        mCurrentDrivingDirection = FORWARD;
        mBackMotor.run(FORWARD);
    }
    else if (direction.compareTo(VALUE_DRIVING_BACKWARD) == 0)
    {
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
	for (int i = 0; i < NUMBER_OF_ULTRA_SONIC_SENSORS; i++){
		if (sensorStatesArray[i]){	//sensor is enable and should send data
			int uS = sonar[i].ping();
			int dist = uS / US_ROUNDTRIP_CM;
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

/*
	mesaure RPM and send data
*/
void sendTachometerData(){
	if (revolution_occured) {
		revolution_count++;
		revolution_occured = !revolution_occured;
	}
	current_time = millis();
	if (current_time - previous_time >= 1000) {
		rps = (revolution_count / wheelSlots);		
		String sensorDataMsg = START_MESSAGE
			+ String(KEY_MESSAGE_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_MESSAGE_TYPE_INFO + MESSAGE_SAPERATOR
			+ String(KEY_INFO_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_SENSOR_DATA + MESSAGE_SAPERATOR
			+ String(KEY_SENSOR_ID) + MESSAGE_KEY_VALUE_SAPERATOR + tachometer_interupt_port + MESSAGE_SAPERATOR
			+ String(KEY_SENSOR_DATA) + MESSAGE_KEY_VALUE_SAPERATOR + rps
			+ END_MESSAGE;
		Serial.println(sensorDataMsg);
		previous_time = current_time;
		revolution_count = 0;
	}



}

/*
*   Method change back dc motor speed
*/
void changeBackMotorSpeed(int digitalSpeed){
    if (digitalSpeed == 0)
        mBackMotor.run(RELEASE);
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

