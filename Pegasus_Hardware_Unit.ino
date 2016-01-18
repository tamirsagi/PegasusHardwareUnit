/*
 * PEGASUS HARDWARE UNIT
 *
  @author - Tamir Sagi, 2015 - 2016.
  This code handles the hardware unit of Pegasus Vehicle

  Message protocol = ActionType,key:value,key:value....#           //# defines end of message

*/

#include <AFMotor.h>
#include <Servo.h>



AF_DCMotor mBackMotor(4,MOTOR34_64KHZ);
Servo mSteerMotot; 


const char END_MESSAGE = '#';                //end of message 
const char MESSAGE_SAPERATOR = ',';         //message saparator , part of the messaging protocol
const char MESSAGE_KEY_VALUE_SAPERATOR = ':';
const char *KEY_ACTION = "AT";
const char *KEY_DIGITAL_SPEED = "DS";
const char *KEY_ROTATION_ANGLE = "RA";
const char *KEY_STEERING_DIRECTION = "SD";
const char *KEY_STEERING_RIGHT = "R";
const char *KEY_STEERING_LEFT = "L";
const char *KEY_STEERING_NONE = "N";
const char *KEY_DRIVING_DIRECTION = "DD";
const char *KEY_DRIVING_FORWARD = "F";
const char *KEY_DRIVING_BACKWARD = "B";

/* Action For Arduino */
const int ACTION_DRIVING_DIRECTION = 0; 
const int ACTION_BACK_MOTOR = 1;
const int ACTION_STEER_MOTOR = 2;

const int ACTION_SERVER_READY = 200;

const int MIN_DIGITAL_SPEED =  0;          //min digital speed value
const int MAX_DIGITAL_SPEED = 255;         //MAx digital speed value


String mInputMessage = "" ;                //keeps incoming messages from Raspberry Pi
boolean mReceivedEntireMessage = false;    //whether the string is complete
int mStringBuffer = 200;                  //number of bytes to reserver
boolean mIsServerReady = false;            //Indicates whether the Raspberry pi is ready for communication
double mCurrentteeringAngle;           // keep the current steering angle
int mLastDigitalSpeed;                //keep last digital speed
double mLastSteeringAngle;           //keep last steering angle






void setup(){

 Serial.begin(115200);  //turn the serial protocol on
 while(!Serial);
 mInputMessage.reserve(mStringBuffer);
 Serial.println("hello");

  mSteerMotot.attach(9);        //attach motor on the shiled
  mBackMotor.run(RELEASE);      //reset back motor
 
}

void loop(){
 
  
  if(mReceivedEntireMessage){
     
    //Serial.println("Message from PI : " + inputMessage);
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
  
  while(Serial.available()){
    //read one byte
   char ch = Serial.read() ;
   //check if its the end of the message
   if(ch == END_MESSAGE){
    mInputMessage += '\0';        //we add end of string indicator
    mReceivedEntireMessage = true;
   }
   else
     mInputMessage += ch;

  }

}


  /**
   * Method will parse the message and call relevant methods
   * The Message is in format MessageType(Action, Data,Fetch etc..), params,params...
   */
  void handleMessage(String msgFromRaspberry){
    int len = msgFromRaspberry.length();
    if(len > 0 ){
     char msgToHandle[len];
     msgFromRaspberry.toCharArray(msgToHandle,len);         //convert the String to char[] in order to use strtok
     String actionType = getValue(msgToHandle,KEY_ACTION);     //get Value
     if(!isNullOrEmpty(actionType)){
       int actionTypeValue = actionType.toInt();                   //return Action type
        switch(actionTypeValue){
          case  ACTION_DRIVING_DIRECTION:
          {
            mBackMotor.run(RELEASE);      //reset back motor
          }
          break;
          case ACTION_BACK_MOTOR:
          {
            String sd = getValue(msgToHandle,KEY_DIGITAL_SPEED);  
            int digitalSpeed = sd.toInt();
            if(MIN_DIGITAL_SPEED <=  digitalSpeed && digitalSpeed <= MAX_DIGITAL_SPEED)   
              changeBackMotorSpeed(digitalSpeed);
          }
          break;
          case ACTION_STEER_MOTOR:
          {
            
          }
          break;

          case ACTION_SERVER_READY:
          {
            mIsServerReady = true;        //set to true when server notifies its ready
          }
          break;
          
          default:
          {
          }
          break;
    
    
        }//end of switch
      }
   }
  }//end of handle message


    /**
     * return the value for given key out of releveant Key:Value within the string
     */
    String getValue(char *msg,const char *key){
      char *relevant;
      relevant = strstr(msg,key);                                 //find the key
      if(relevant == NULL)
        return "";
      relevant = strchr(relevant,MESSAGE_KEY_VALUE_SAPERATOR);    //from the key find the ':'
      if(relevant == NULL)
            return "";
      relevant = strtok(relevant,":,");                           //split the message from ':' to ',' which is the value
      if(relevant == NULL)
            return "";
            
      String value(relevant);                                     //convert to String
      return value;
      }



/**
 * Check whether a string is null or empty
 */
  boolean isNullOrEmpty(String msg){
    return msg == NULL || msg.length() <= 0;
  }


  

  /*  
   *   Method change back dc motor speed
   */
  void changeBackMotorSpeed(int digitalSpeed){
    mBackMotor.setSpeed(digitalSpeed);  
  }


 
