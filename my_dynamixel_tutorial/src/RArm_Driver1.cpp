#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include <naoqi_msgs/JointAnglesWithSpeed.h>
#include<stdio.h>
#include <math.h>

using namespace std;
#define JOINT_NUM 24

ros::Publisher pub[JOINT_NUM + 1];
ros::Publisher pub_grasp;

float L1 = 12, L2 = 12.5;
double D2R = acos(-1.0)/180.0;
double del[] = {0, 0, 0, 0, 0, -25*D2R, 20*D2R, 0, 0, -20*D2R, -40*D2R, 20*D2R, -20*D2R,
                10*D2R, 18*D2R, 0, 0, 5*D2R, -15*D2R, 0, -40*D2R, 0, 0, -0*D2R, 0};
int state = 0;
void poseMessageReceived(const naoqi_msgs::JointAnglesWithSpeed& msg){  
  if (state != 0) return;

  ros::Rate loop_rate(60);
  std_msgs::Float64 aux;
  int id[] = {1, 3, 5, 21};
  int sign[] = {-1, -1, 1, -1};

  /*float leg_angle = msg.joint_angles[0];
  aux.data = leg_angle + del[11];
  pub[11].publish(aux);
  aux.data = -leg_angle + del[12];
  pub[12].publish(aux);
  float leg2_angle = asin(L1 / L2 * sin(-leg_angle));
  aux.data = leg2_angle + del[15];
  pub[15].publish(aux);
  aux.data = -leg2_angle + del[16];
  pub[16].publish(aux);
  aux.data = abs(leg_angle) + leg2_angle + del[13];
  pub[13].publish(aux);
  aux.data = -(abs(leg_angle) + leg2_angle) + del[14];
  pub[14].publish(aux);

  for (int j = 0; j < 150; ++j)
      loop_rate.sleep();
  pub_grasp.publish(aux);*/

  for (int i = 0; i < 4; ++i) {
      aux.data = sign[i] * msg.joint_angles[i] + del[id[i]];
      //printf("%f\n", aux.data);
      pub[id[i]].publish(aux);
  } 
  //puts("");
  state = 1;
}
char node_names[][50] = {
    //0
    "Nothing",
    //1
    "/RArmShoulderPitch_controller/command",
    //2
    "/LArmShoulderPitch_controller/command",
    //3
    "/RArmShoulderRoll_controller/command",
    //4
    "/LArmShoulderRoll_controller/command",
    //5
    "/RArmElbowRoll_controller/command",
    //6
    "/LArmElbowRoll_controller/command",
    //7
    "/RHipYaw_controller/command",
    //8
    "/LHipYaw_controller/command",
    //9
    "/RHipRoll_controller/command",
    //10
    "/LHipRoll_controller/command",
    //11
    "/RHipPitch_controller/command",
    //12
    "/LHipPitch_controller/command",
    //13
    "/RKneePitch_controller/command",
    //14
    "/LKneePitch_controller/command",
    //15
    "/RAnklePitch_controller/command",
    //16
    "/LAnklePitch_controller/command",
    //17
    "/RFootRoll_controller/command",
    //18
    "/LFootRoll_controller/command",
    //19
    "/NeckYaw_controller/command",
    //20
    "/HeadPitch_controller/command",
    //21
    "/RArmElbowYaw_controller/command",
    //22
    "/LArmElbowYaw_controller/command",
    //23
    "/RArmHand_controller/command",
    //24
    "/LArmHand_controller/command"
};
int cuboid = 3;
int main(int argc, char** argv){
  
  ros::init(argc, argv, "RArm_move_motor");
  naoqi_msgs::JointAnglesWithSpeed msg;
  //Dynamixel motors;
  ros::NodeHandle nh;
  ros::Subscriber pSub = nh.subscribe("/joint_angles", 1000, &poseMessageReceived);
  pub_grasp = nh.advertise<std_msgs::Float64>("/rarm_state",1);
  for (int i = 1; i <= JOINT_NUM; ++i) {
      pub[i] = nh.advertise<std_msgs::Float64>(node_names[i],1);
  }
  ros::Rate loop_rate(60);
  std_msgs::Float64 aux;
  while(ros::ok()){
    printf("%d\n", state);
    if (state == 0) {
        //aux.data = -0.5 + del[19];
        //pub[19].publish(aux);
    }
    else if (state == 1) {
        if (cuboid == 3) {
            for (int j = 0; j < 100; ++j)
                loop_rate.sleep();
            //break;
            aux.data = -7 + del[23];
            pub[23].publish(aux);
            for (int j = 0; j < 200; ++j)
                loop_rate.sleep();
            aux.data = 3.5 + del[1];
            pub[1].publish(aux);
            aux.data = 0.7 + del[5];
            pub[5].publish(aux);
            aux.data = 0.45 + del[3];
            pub[3].publish(aux);
            break;
            /*aux.data = -0.6 + del[11];
            pub[11].publish(aux);
            aux.data = 0.6 + del[12];
            pub[12].publish(aux);
            aux.data = 1.1 + del[13];
            pub[13].publish(aux);
            aux.data = -1.1 + del[14];
            pub[14].publish(aux);
            aux.data = 0.5 + del[15];
            pub[15].publish(aux);
            aux.data = -0.5 + del[16];
            pub[16].publish(aux);*/
            for (int j = 0; j < 150; ++j)
                loop_rate.sleep();
        }
        if (cuboid == 4) {
            /*
            aux.data = 1.2 + del[5];
            pub[5].publish(aux);
            aux.data = -1.5 + del[21];
            pub[21].publish(aux);
            aux.data = 1.5 + del[1];
            pub[1].publish(aux);
            aux.data = 1.3 + del[3];
            pub[3].publish(aux);
            aux.data = -0.65;
            pub[19].publish(aux);
            for (int j = 0; j < 100; ++j)
                loop_rate.sleep();*/
            for (int j = 0; j < 150; ++j)
                loop_rate.sleep();
            aux.data = 0 + del[23];
            pub[23].publish(aux);
            for (int j = 0; j < 150; ++j)
                loop_rate.sleep();
            aux.data = 2.5 + del[1];
            pub[1].publish(aux);
            for (int j = 0; j < 150; ++j)
                loop_rate.sleep();
        }
        pub_grasp.publish(aux);
        state = 0;
        ++cuboid;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

