/*
* PEGASUS HARDWARE UNIT
*
@author - Tamir Sagi, 2015 - 2016.
This code handles the hardware unit of Pegasus Vehicle

Message protocol = key:value,key:value,key:value....#           //# defines end of message
Max Digital Speed 0-255
Steer Angle is 0-40 degrees each side ( Right 50-90, Left, 90-130)

*/

#include <AFMotor.h>
#include <Servo.h>


AF_DCMotor mBackMotor(4, MOTOR34_64KHZ);
Servo mSteerMotor;



//////////////////////////////////////////////////////////////////
/*
	Messages And protocol Abberviations
*/
const char *KEY_MESSAGE_TYPE = "MT";				 // MT = Message Type
const int VALUE_MESSAGE_TYPE_INFO = 1000;			 //INFO
const int VALUE_MESSAGE_TYPE_ACTION = 2000;			 //ACTION
const char   *KEY_INFO_TYPE = "IT";					 //IT = Info Type key


const char END_MESSAGE = '#';						//end of message 
const char MESSAGE_SAPERATOR = ',';				   //message saparator , part of the messaging protocol
const char MESSAGE_KEY_VALUE_SAPERATOR = ':';

/*
	Incoming
*/
const char *KEY_VEHICLE_ACTION_TYPE = "VA";					//Action Type key
const char *KEY_DIGITAL_SPEED = "DS";				//Digital Speed Key
const char *KEY_ROTATION_ANGLE = "RA";				//Rotation Angle key
const char *KEY_STEERING_DIRECTION = "SD";			// Steering Direction key
const char *KEY_DRIVING_DIRECTION = "DD";			//Driving Direction key

const String VALUE_STEERING_NONE = "N";				//N = None, no steering
const String VALUE_STEERING_RIGHT = "R";			//R = Right,  Right Steering
const String VALUE_STEERING_LEFT = "L";				//L = Left, Left Steering
const String VALUE_DRIVING_FORWARD = "F";			//F = Forward, driving Forward
const String VALUE_DRIVING_BACKWARD = "B";			//B = Backward, driving backward

/*
	Outgoing
*/

const String VALUE_STATUS = "STATUS";				   
const String KEY_STATUS = "ST";						  //ST = STATUS
const String KEY_SERIAL_PORT = "SP";				 //SP = Serial Port
const String KEY_STEER_MOTOR = "SM";				//SM = Steering Motor
const String KEY_BACK_MOTOR = "BM";				   //BM = Back Motor
const String VALUE_OK = "OK";						
const String VALUE_ERROR = "ER";



//////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////// ACTIONS \\\\\\\\\\\\\\\\\\\\\\\

/* Action For Arduino */
const int ACTION_DRIVING_DIRECTION = 0;
const int ACTION_BACK_MOTOR = 1;
const int ACTION_STEER_MOTOR = 2;



/////////////// INFO \\\\\\\\\\\\\\\\\\

const int VALUE_HARDWARE_STATUS_READY = 100;
const int VALUE_SERVER_STATUS_READY = 200;



const int MIN_DIGITAL_SPEED = 0;				  //MIN digital speed value
const int MAX_DIGITAL_SPEED = 255;				 //MAx digital speed value
const int STRAIGHT_STEER_ANGLE = 90;			//MAX steer angle value
const int MIN_STEER_ANGLE = 50;					//MIN steer angle value
const int MAX_STEER_ANGLE = 130;				//MAX steer angle value
const int String_Buffer = 100;					//number of bytes to reserve



String mInputMessage = "";						//keeps incoming messages from Raspberry Pi
boolean mReceivedEntireMessage = false;			//whether the string is complete
boolean mIsServerReady = false;					//indicates whether the Raspberry pi is ready for communication
double mCurrentteeringAngle;					//keep the current steering angle
int mLastDigitalSpeed;							//keep last digital speed
double mLastSteeringAngle;						//keep last steering angle
String mSystemInitialStatus = "";					//keep initiate status of the system , we send this string to logic unit

void setup(){
	
	mInputMessage.reserve(String_Buffer);		//allocate buffer for incoming messages

	/*
		Serial Port 
	*/
	Serial.begin(115200);						//turn the serial protocol on
	while (!Serial);							//Wait for Serial
	
	/*
		Motors
	*/
	mSteerMotor.attach(9);					//attach motor on the shiled
	mBackMotor.run(RELEASE);				//reset back motor
	mBackMotor.run(FORWARD);				//reset To Forward direction by default
	

	// Build up the status message
	mSystemInitialStatus += String(KEY_MESSAGE_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_MESSAGE_TYPE_INFO + MESSAGE_SAPERATOR
		+ String(KEY_INFO_TYPE) + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_STATUS + MESSAGE_SAPERATOR
		+ KEY_STATUS + MESSAGE_KEY_VALUE_SAPERATOR + VALUE_HARDWARE_STATUS_READY
		+ END_MESSAGE;

	Serial.println(mSystemInitialStatus);	//send message to logic unit

	//Testing
	/*mInputMessage = "AT:1,SD:R,RA:90";
	handleMessage(mInputMessage);

	mInputMessage = "AT:1,SD:L,RA:90";
	handleMessage(mInputMessage);*/
	
}

void loop(){

	//// scan from 0 to 180 degrees
	//for (int angle = 50; angle <= 130; angle++)
	//{
	//	mSteerMotor.write(angle);
	//	delay(15);
	//}
	//// now scan back from 180 to 0 degrees
	//for (int angle = 130; angle >= 50; angle--)
	//{
	//	mSteerMotor.write(angle);
	//	delay(15);
	//}
	/*Serial.println("Digital Speed");
	handleMessage(mInputMessage);
	Serial.println("Rotation R");
	mInputMessage = "AT:2,SD:R,RA:80.7";
	handleMessage(mInputMessage);
	Serial.println("Rotation L ");
	mInputMessage = "AT:2,SD:L,RA:130";
	handleMessage(mInputMessage);*/
	
	if (mReceivedEntireMessage){
		handleMessage(mInputMessage);
		//clear data
		mInputMessage = "";
		mReceivedEntireMessage = false;
	}

}


/*

Serial Event occurs whenever a new data comes in the hardware Serial. this routin is run between each
time loop() runs.
*/
void serialEvent(){

	while (Serial.available()){
		//read one byte
		char ch = Serial.read();
		//check if its the end of the message
		if (ch == END_MESSAGE){
			Serial.println("received:");
			Serial.println(mInputMessage);
			mInputMessage += '\0';			//we add end of string indicator
			Serial.println(mInputMessage);
			mReceivedEntireMessage = true;
		}
		else
			mInputMessage += ch;

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
		String messageType = getValue(msgToHandle, KEY_MESSAGE_TYPE);		//get Value
		if (!isNullOrEmpty(messageType) && isNumber(messageType)){
			int messageTypeValue = messageType.toInt();					//return Message type
			switch (messageTypeValue){
			case  VALUE_MESSAGE_TYPE_INFO:
			{
				
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
	relevant = strstr(relevant, key);                               //find the key
	if (isNullOrEmpty(relevant))
		return "";
	relevant = strchr(relevant, MESSAGE_KEY_VALUE_SAPERATOR);		//from the key find the ':'
	if (isNullOrEmpty(relevant))
		return "";
	relevant = strtok(relevant, ":,");								//split the message from ':' to ',' which is the value
	if (isNullOrEmpty(relevant))
		return "";
	
	String value(relevant);											//convert char* to String
	delete[] buffer;												//DELETE allocation
	return value;
}

void handleAction(char * msgToHandle){
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
		for (int i = 0; i < len; i++){
			if (msg[i] == '.' && i > 0)
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
	Method handles direction changing
*/
void handleDirections(char *msgToHandle){
	String direction = getValue(msgToHandle, KEY_DRIVING_DIRECTION);
	mBackMotor.run(RELEASE);							//reset back motor
	if (direction.compareTo(VALUE_DRIVING_FORWARD) == 0)
		mBackMotor.run(FORWARD);
	else if (direction.compareTo(VALUE_DRIVING_BACKWARD) == 0)
		mBackMotor.run(BACKWARD);
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
	String direction = getValue(msgToHandle, KEY_STEERING_DIRECTION);
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
*   Method change back dc motor speed
*/
void changeBackMotorSpeed(int digitalSpeed){
	Serial.println("changeBackMotorSpeed");
	Serial.println(digitalSpeed);
	mBackMotor.setSpeed(digitalSpeed);
}


/*
	turning right, from 0-40 degrees to 50-90
*/
void turnSteeringRight(double angle){
	mSteerMotor.write(STRAIGHT_STEER_ANGLE - angle);		
	delay(15);
}

/*
turning left, from 0-40 degrees to 90-130
*/
void turnSteeringLeft(double angle){
	mSteerMotor.write(STRAIGHT_STEER_ANGLE + angle);		//from 0-40 degrees to 90-130
	delay(15);
}



