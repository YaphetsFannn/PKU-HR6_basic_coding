#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<stdio.h>

using namespace std;

class Dynamixel{
  private:
    ros::NodeHandle RArm;
    ros::Publisher pub_RArm;
    ros::Subscriber sub_RArm;
  public:
    Dynamixel(string topic);
    int moveMotor(double position);
};

Dynamixel::Dynamixel(string topic){
  pub_RArm = RArm.advertise<std_msgs::Float64>(topic,1000);
  sub_RArm = RArm.subscribe("chatter", 1000, float q[4] = chatterCallback);
}

int Dynamixel::moveMotor(double position){
  std_msgs::Float64 aux;
  aux.data = position;
  pub_RArm.publish(aux);
  return 1;
}

float q[4] = chatterCallback(){
  



}
int main(int argc, char** argv){
  ros::init(argc, argv, "RArm_move_motor");

  Dynamixel RArm1("RArm1_controller/command");
  Dynamixel RArm2("RArm2_controller/command");
  Dynamixel RArm3("RArm3_controller/command");
  Dynamixel RArm4("RArm4_controller/command");

  ros::Rate loop_rate(1);

  float q_init[4] = {0,0,-30,0};
  float q[4] = {60,30,0,30};


  RArm1.moveMotor(q_init[0]*3.14/180);
  RArm2.moveMotor(q_init[1]*3.14/180);
  RArm3.moveMotor(q_init[2]*3.14/180);
  RArm4.moveMotor(q_init[3]*3.14/180);
  loop_rate.sleep();

  while(ros::ok()){

    RArm1.moveMotor(q[0]*3.14/180);
    RArm2.moveMotor(q[1]*3.14/180);
    RArm3.moveMotor(q[2]*3.14/180);
    RArm4.moveMotor(q[3]*3.14/180);

    ros::spinOnce();
    loop_rate.sleep();


  }

}

