#include <DynamixelMotor.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int64.h>
#include <tilt_scanner/MsgSettings.h>


uint8_t situation = 1;
uint16_t result = 0;
uint16_t position = 0;
uint16_t delta = 0;
unsigned long time = 0;

bool startReceived = false;

// --settings for Dynamixel
const uint8_t id = 1;
const uint32_t baudrate = 57600;
uint16_t approachSpeed = 150;
uint16_t homePosition = 2048;


// --settings of service--
uint16_t numberOfScans = 0;
uint16_t speed = 0;
uint16_t holdUpTime = 0;
uint16_t startPosition = 0;
uint16_t endPosition = 0;



ros::NodeHandle nh;

// --Subscriber--

void subsettings(const tilt_scanner::MsgSettings& sub_settings)
{
  numberOfScans = sub_settings.numberOfScans;
  speed = sub_settings.speed;
  holdUpTime = sub_settings.holdUpTime;
  startPosition = sub_settings.startPosition;
  endPosition = sub_settings.endPosition;
  startReceived = true;
}
ros::Subscriber<tilt_scanner::MsgSettings> subSettings("settings", &subsettings);


// --Publisher--

std_msgs::UInt16 pub_angle;
ros::Publisher pubAngle("angle", &pub_angle);

std_msgs::UInt16 pub_situation;
ros::Publisher pubSituation("situation", &pub_situation);

std_msgs::Int64 pub_control;
ros::Publisher pubControl("control", &pub_control);



SoftwareDynamixelInterface interface(9,10,2);     //set (RX,TX,Controlpin)

DynamixelMotor motor(interface, id);                                  //set id of Dynamixel


void setup() {
  
  delay(100);
  interface.begin(baudrate);                                          //set Baudrate
  delay(100);

  nh.initNode();


  // --Subscriber--
  
  nh.subscribe(subSettings);


  // --Publisher--

  nh.advertise(pubAngle);
  nh.advertise(pubSituation);
  nh.advertise(pubControl);

  while(!nh.connected())                                              //waiting for ROS connection
  {
    nh.spinOnce();
  }
  
  motor.init();                                                       //initialize the Dynamixel
  motor.enableTorque();

  motor.jointMode();                                                  //choose 'jointMode' to enable 'goalPosition'
  motor.speed(approachSpeed);                                         //adjustment of speed of the Dynamixel [0.114rpm]
  motor.goalPosition(homePosition);                                   //drive to home position
  
  while(result)                                                       //wait for receiving home position
  {
    delay(6);
    motor.read(0x2E, result);                                         //result = 1 --> moving | result = 0 --> standing
  }
}

void loop() {
 
  switch(situation)
  {
    case 1:
    {
      if(startReceived)
      {
        position = startPosition;
        delta = (endPosition-startPosition)/numberOfScans;            //calculating the increments for the Dynamixel
        motor.speed(speed);                                           //choose speed from the settingsrvs
        situation = 2;
      }
    }
    break;
      
    case 2:
    {      
      motor.goalPosition(position);                                   //drive to new position
      delay(10);
      motor.read(0x2E, result);
      
      if(!result)
      {
        situation = 3;
      }
    }
    break;

    case 3:
    {
      pub_angle.data = motor.currentPosition();                       //publish position after receiving
      pubAngle.publish(&pub_angle);
            
      time = millis();

      situation = 4;
    }
    break;
    
    case 4:
    {     
      if(millis()-time >= holdUpTime)                                 //await laserscan/holdUpTime
      {
        situation = 5;
      }
    }
    break;
    
    case 5:
    {
      if(position <= endPosition-delta)                               //choose right case for the next position
      {
        position += delta;
        situation = 2;
      }

      if(position >= endPosition-delta)
      {
        position = endPosition;
        situation = 2;
      }

      if(position == endPosition)
      {
        motor.goalPosition(homePosition);                             //scan complete, drive back to home position
        startReceived = false;
        nh.loginfo("scan complete");
        situation = 1;
      }
    }
    break;
  }

  nh.spinOnce();
}
