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

/* Action For Arduino */
const int ACTION_DRIVING_DIRECTION = 0;
const int ACTION_BACK_MOTOR = 1;
const int ACTION_STEER_MOTOR = 2;

const int ACTION_SERVER_READY = 200;

const char END_MESSAGE = '#';                //end of message 
const char MESSAGE_SAPERATOR = ',';         //message saparator , part of the messaging protocol
const char MESSAGE_KEY_VALUE_SAPERATOR = ':';
const char *KEY_ACTION = "AT";
const char *KEY_DIGITAL_SPEED = "DS";
const char *KEY_ROTATION_ANGLE = "RA";
const char *KEY_STEERING_DIRECTION = "SD";

const char *KEY_STEERING_NONE = "N";
const char *KEY_DRIVING_DIRECTION = "DD";

const String VALUE_STEERING_RIGHT = "R";
const String VALUE_STEERING_LEFT = "L";
const String VALUE_DRIVING_FORWARD = "F";
const String VALUE_DRIVING_BACKWARD = "B";





const int MIN_DIGITAL_SPEED = 0;			//MIN digital speed value
const int MAX_DIGITAL_SPEED = 255;			//MAx digital speed value

const int STRAIGHT_STEER_ANGLE = 90;			//MAX steer angle value
const int MIN_STEER_ANGLE = 50;				 //MIN steer angle value
const int MAX_STEER_ANGLE = 130;			//MAX steer angle value
const int String_Buffer = 100;				//number of bytes to reserve

String mInputMessage = "";					//keeps incoming messages from Raspberry Pi
boolean mReceivedEntireMessage = false;		//whether the string is complete
boolean mIsServerReady = false;				//indicates whether the Raspberry pi is ready for communication
double mCurrentteeringAngle;				//keep the current steering angle
int mLastDigitalSpeed;						//keep last digital speed
double mLastSteeringAngle;					//keep last steering angle



void setup(){

	Serial.begin(115200);					//turn the serial protocol on
	while (!Serial);
	mInputMessage.reserve(String_Buffer);
	Serial.println("ready#");

	mSteerMotor.attach(9);					//attach motor on the shiled
	mBackMotor.run(RELEASE);				//reset back motor
	mBackMotor.run(FORWARD);				//reset To Forward direction by default
	mInputMessage = "AT:1,SD:R,RA:90";
	handleMessage(mInputMessage);

	mInputMessage = "AT:1,SD:L,RA:90";
	handleMessage(mInputMessage);


	
}

void loop(){

	// scan from 0 to 180 degrees
	for (int angle = 50; angle <= 130; angle++)
	{
		mSteerMotor.write(angle);
		delay(15);
	}
	// now scan back from 180 to 0 degrees
	for (int angle = 130; angle >= 50; angle--)
	{
		mSteerMotor.write(angle);
		delay(15);
	}
	/*Serial.println("Digital Speed");
	handleMessage(mInputMessage);
	Serial.println("Rotation R");
	mInputMessage = "AT:2,SD:R,RA:80.7";
	handleMessage(mInputMessage);
	Serial.println("Rotation L ");
	mInputMessage = "AT:2,SD:L,RA:130";
	handleMessage(mInputMessage);*/
	
	//if (mReceivedEntireMessage){
	//	handleMessage(mInputMessage);
	//	//clear data
	//	mInputMessage = "";
	//	mReceivedEntireMessage = false;
	//}

}


/*

Serial Event occurs whenever a new data comes in the hardware Serial. this routin is run between each
time loop() runs.
*/
void serialEvent(){

	//while (Serial.available()){
	//	//read one byte
	//	char ch = Serial.read();
	//	//check if its the end of the message
	//	if (ch == END_MESSAGE){
	//		Serial.println("received:");
	//		Serial.println(mInputMessage);
	//		mInputMessage += '\0';			//we add end of string indicator
	//		Serial.println(mInputMessage);
	//		mReceivedEntireMessage = true;
	//	}
	//	else
	//		mInputMessage += ch;

	//}

}


/**
* Method will parse the message and call relevant methods
* The Message is in format MessageType(Action, Data,Fetch etc..), params,params...
*/
void handleMessage(String msgFromRaspberry){
	if (!isNullOrEmpty(msgFromRaspberry)){
		int len = msgFromRaspberry.length() + 1;					// + 1 to keep eos '\0' char
		char* msgToHandle = new char[len];
		msgFromRaspberry.toCharArray(msgToHandle, len);				//convert the String to char[] in order to use strtok
		String actionType = getValue(msgToHandle, KEY_ACTION);		//get Value
		if (!isNullOrEmpty(actionType)){
			int actionTypeValue = actionType.toInt();               //return Action type
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
			case ACTION_SERVER_READY:
			{
				mIsServerReady = true;								//set to true when server notifies its ready
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



