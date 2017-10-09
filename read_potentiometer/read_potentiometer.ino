#include <Wire.h>
#include <ros.h>
#include <robotics_controller/Potentiometer.h>


//Set up the ros node and publisher

robotics_controller::Potentiometer joint_msg;
ros::Publisher pub_joint_values("joint_values", &joint_msg);
ros::NodeHandle nh;

#define POT_1 A1
#define POT_2 A2
#define POT_3 A3


int pot1_value;
int pot2_value;
int pot3_value;

void setup()
{
  nh.initNode();
  nh.advertise(pub_joint_values);
  
  pinMode(POT_1, OUTPUT);
  pinMode(POT_2, OUTPUT);
  pinMode(POT_3, OUTPUT);
  
  Serial.begin(57600);

}

long publisher_timer;

void loop()
{

  pot1_value = analogRead(POT_1);
  pot2_value = analogRead(POT_2);
  pot3_value = analogRead(POT_3);

  
  joint_msg.joint_1 = pot1_value;
  joint_msg.joint_2 = pot2_value;
  joint_msg.joint_3 = pot3_value;
  
  pub_joint_values.publish(&joint_msg);
  
  Serial.print("Joint 1: ");
  Serial.println(pot1_value);
  
  Serial.print("Joint 2: ");
  Serial.println(pot2_value);
  
  Serial.print("Joint 3: ");
  Serial.println(pot3_value);

  delay(100);
  
  nh.spinOnce();
}
