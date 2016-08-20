#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <race/drive_values.h>
#include <race/drive_param.h>
ros::NodeHandle  nh;

boolean flagStop = false;
unsigned int pwm_center_value = 9830;  //  15% duty cycle
unsigned int pwm_lowerlimit = 6554;    //  10% duty cycle
unsigned int pwm_upperlimit = 13108;   //  20% duty cycle

// Publish PWM values for debugging
race::drive_values msg;
ros::Publisher chatter("drivePWM", &msg);

int kill_pin = 2;
unsigned long duration = 0;

float mapfloat(float x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void messageDrive( const race::drive_param& data )
{ 
  if(flagStop == false)
  {
    unsigned long int velocity = mapfloat(data.velocity,-100,100,pwm_lowerlimit,pwm_upperlimit);
    unsigned long int angle = mapfloat(data.angle,-100,100,pwm_lowerlimit,pwm_upperlimit);
    velocity = constrain(velocity,pwm_lowerlimit,pwm_upperlimit);
    angle = constrain(angle,pwm_lowerlimit,pwm_upperlimit);
    analogWrite(5,velocity);     //  Incoming data                    
    analogWrite(6,angle);     //  Incoming data                    
    msg.pwm_drive = velocity; //PWM value
    msg.pwm_angle = angle; //PWM value
    chatter.publish( &msg );
  }
  else
  {
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);    
  }
}

void messageEmergencyStop( const std_msgs::Bool& flag )
{
  flagStop = flag.data;
  if(flagStop == true)
  {
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);    
  }
}

ros::Subscriber<race::drive_param> sub_drive("drive_parameters", &messageDrive );
ros::Subscriber<std_msgs::Bool> sub_stop("eStop", &messageEmergencyStop );

void setup() {
  
  analogWriteFrequency(5, 100);
  analogWriteFrequency(6, 100);
  analogWriteResolution(16);
  analogWrite(5,pwm_center_value);
  analogWrite(6,pwm_center_value);
  pinMode(13,OUTPUT); 
  digitalWrite(13,HIGH);
  pinMode(2,INPUT); 

  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_stop);

  nh.advertise(chatter);
}

void loop() {
  nh.spinOnce();
  duration = pulseIn(kill_pin, HIGH, 30000);
  while(duration > 1900)
  {
    duration = pulseIn(kill_pin, HIGH, 30000);
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);        
  }
}

