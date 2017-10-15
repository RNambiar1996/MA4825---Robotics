#include <ArduinoHardware.h>
#include <ros.h>
#include <robotics_controller/Trajectory.h>
#include <robotics_controller/Task.h>

//Set up the ros node and publisher

robotics_controller::Trajectory trajectory;
robotics_controller::Task task;

ros::Publisher pub_trajectory("trajectory", &trajectory);
ros::Publisher pub_task("task", &task);
ros::NodeHandle nh;

#define POT_1 A1
#define POT_2 A2
#define POT_3 A3

#define BUTTON1 7
#define BUTTON2 8


float pot1_value;
float pot2_value;
float pot3_value;


static int trajectoryCounter = 0;
static int taskCounter = 0;
  
int button1_state = 0;
int button2_state = 0;

float present = 0.0;
float past_trajectory = 0.0;
float past_task = 0.0;
float t_delta_traj = 0.0;
float t_delta_task = 0.0;

void setup()
{
  nh.initNode();
  nh.advertise(pub_trajectory);
  nh.advertise(pub_task);
  
  pinMode(POT_1, OUTPUT);
  pinMode(POT_2, OUTPUT);
  pinMode(POT_3, OUTPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  
  Serial.begin(57600);

}

void loop()
{
  present = millis();
  
  pot1_value = analogRead(POT_1);
  pot2_value = analogRead(POT_2);
  pot3_value = analogRead(POT_3);

  button1_state = digitalRead(BUTTON1);
  button2_state = digitalRead(BUTTON2);

  Serial.print("Pot 1 value: ");
  Serial.println(pot1_value);

  Serial.print("Pot 2 value: ");
  Serial.println(pot2_value);

  Serial.print("Pot 3 value: ");
  Serial.println(pot3_value);
  
  trajectory.joint_1 = pot1_value;
  trajectory.joint_2 = pot2_value;
  trajectory.joint_3 = pot3_value;
  
  //Serial.println(trajectory.joint_1);
  //Serial.println(trajectory.joint_2);
  //Serial.println(trajectory.joint_3);

  pub_trajectory.publish(&trajectory);
  
  //Serial.print("Button1 State: ");
  //Serial.println(button1_state);
  //Serial.print("Button2 State: ");
  //Serial.println(button2_state);
      
  if(button1_state==1)
  {
    t_delta_traj = present - past_trajectory;

    if(t_delta_traj>150)
    {
      Serial.println(trajectoryCounter);
      task.joint_values[trajectoryCounter] = trajectory;
      Serial.println(task.joint_values[trajectoryCounter].joint_1);
      Serial.println("Joint values stored");
      Serial.print("Trajectory Counter value: ");
      Serial.println(trajectoryCounter++);
    } 

    past_trajectory = present;
  }

  if(button2_state == 1)
  {
    t_delta_task = present - past_task;

    if(t_delta_task>150)
    {
      
      Serial.println("Task stored");
      Serial.print("Task Counter Value: ");
      Serial.println(taskCounter++);
    
      pub_task.publish(&task);
      trajectoryCounter = 0;
    }
    
    past_task = present;
  }
  

  delay(100);
 
  nh.spinOnce();


}
