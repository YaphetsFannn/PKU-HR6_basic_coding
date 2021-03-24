#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include  <std_srvs/Empty.h>
#include<math.h>
#include "dynamixel_msgs/JointState.h"
#include <naoqi_msgs/JointAnglesWithSpeed.h>
#include <fstream>
using namespace std;

double D2R = acos(-1.0)/180.0;
double del[] = {0, 0, 0, 0, 0, -25*D2R, 20*D2R, 0, 0, -20*D2R, -40*D2R, 20*D2R, -20*D2R,
                10*D2R, 18*D2R, 0, 0, 5*D2R, -15*D2R, 0, -40*D2R, 0, 0, -0*D2R, 0};

float q_head, q_neck, q_shoulder, q_high, q_low, q_wrist, tar_x, tar_y, tar_z;
bool record_data(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
        if(tar_z > 1e-6)
        {

                FILE* fw = fopen("data_res", "a+");
                fprintf(fw, "%f %f %f %f %f %f %f %f %f\n", q_head+40*D2R, q_neck, q_shoulder,  q_high, q_low+25*D2R, q_wrist, tar_x, tar_y, tar_z);
                fclose(fw);
                printf("%f %f %f\n", tar_x, tar_y, tar_z);
        }
}

void poseMessageReceived(const dynamixel_msgs::JointState& msg){
    	if (msg.motor_ids[0] == 19) 
        		q_neck= msg.current_pos;
        else if (msg.motor_ids[0] == 20)
    		q_head = msg.current_pos;
        else if (msg.motor_ids[0] == 1)
    		q_shoulder = msg.current_pos;
        else if(msg.motor_ids[0] == 3)
   		q_high = msg.current_pos;
        else if (msg.motor_ids[0] == 5)
    		q_low = msg.current_pos;
    	else
    		q_wrist = msg.current_pos;
    	return;
}

void object_poseMessageReceived(const naoqi_msgs::JointAnglesWithSpeed& msg)
{
              tar_x = msg.joint_angles[0];
              tar_y = msg.joint_angles[1];
              tar_z = msg.joint_angles[2];
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "record_data");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("record_data", &record_data);

	ros::Subscriber sub1 = nh.subscribe("/NeckYaw_controller/state", 1000, poseMessageReceived);
    	ros::Subscriber sub2 = nh.subscribe("/HeadPitch_controller/state", 1000, poseMessageReceived);
    	ros::Subscriber sub3 = nh.subscribe("/RArmShoulderPitch_controller/state", 1000, poseMessageReceived);
    	ros::Subscriber sub4 = nh.subscribe("/RArmShoulderRoll_controller/state", 1000, poseMessageReceived);
    	ros::Subscriber sub5 = nh.subscribe("/RArmElbowRoll_controller/state", 1000, poseMessageReceived);
    	ros::Subscriber sub6 = nh.subscribe("/RArmElbowYaw_controller/state", 1000, poseMessageReceived);
        ros::Subscriber sub7 = nh.subscribe("/Object_state", 1000, object_poseMessageReceived);

    	ros::spin();
    	return 0;
}
