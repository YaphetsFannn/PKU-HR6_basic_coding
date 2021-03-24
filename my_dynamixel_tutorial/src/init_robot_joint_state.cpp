#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <memory.h>
#define JOINT_NUM 28

ros::Publisher pub[JOINT_NUM + 1];
float D2R = acos(-1.0)/180.0;
float del[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, -20*D2R, -40*D2R, 20*D2R, -20*D2R,
                10*D2R, 18*D2R, 0, 0, 5*D2R, -15*D2R, 0, 0, 0, 0, -0*D2R, 0, 0, 0, 0, 0};
                
/*float del[] = {0, 0, 0, 0, 0, -25*D2R, 20*D2R, 0, 0, -20*D2R, -40*D2R, 20*D2R, -20*D2R,
                10*D2R, 18*D2R, 0, 0, 5*D2R, -15*D2R, 0, -40*D2R, 0, 0, -0*D2R, 0, 0, 0, 0, 0};*/

bool initJointState(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    std_msgs::Float64 aux;
    float angle[29] = {0};
    memset(angle, 0, sizeof(angle));
    /*angle[2] = -2.5;
    angle[4] = -0.45;
    angle[6] = -0.7;
    angle[3] = 0.5;
    angle[5] = 0.3;
    angle[11] = -0.4;
    angle[12] = 0.4;
    angle[13] = 0.78;
    angle[14] = -0.78;
    angle[15] = 0.38;
    angle[16] = -0.38;
    angle[22] = 0.6;
    angle[24] = -1;*/
    /*angle[1] = 2.5;
    angle[4] = -0.5;
    angle[6] = -0.3;
    angle[11] = -0.4;
    angle[12] = 0.4;
    angle[13] = 0.78;
    angle[14] = -0.78;
    angle[15] = 0.38;
    angle[16] = -0.38;
    angle[21] = -0.6;
    angle[3] = 0.45;
    angle[5] = 0.7;*/
    //angle[19] = 0.4;

        // angle[1] = 0.7;
        angle[1] = 0.0;
        angle[4] = -0.25;
        angle[6] = -0.3;
        angle[11] = 0;
        angle[12] = 0;
        angle[13] = 0;
        angle[14] = 0;
        angle[15] = 0;
        angle[16] = 0;
        angle[3] = 0.25;
        angle[5] = 0;
        angle[21] = 0;
        angle[25] = -0.1;
        angle[20] = 0;
        angle[19] = 0.5;


    /*angle[1] = 1.4;
        angle[4] = -1;
        angle[6] = -1;
        angle[11] = -0.4;
        angle[12] = 0.4;
        angle[13] = 0.78;
        angle[14] = -0.78;
        angle[15] = 0.38;
        angle[16] = -0.38;
        angle[21] = 0.3;
        angle[22] = -0.3;
        angle[3] = 1;
        angle[5] = 1;
        angle[2] = -1.4;
        angle[20] = 0.4;*/
   //     angle[1] = 0.7;
    //    angle[19] = -0.45;
    ros::Rate loop(60);
    for (int i = 1; i <= 28; ++i) {
        aux.data = angle[i] + del[i];
        pub[i].publish(aux);
    }
    int release;
        printf("Input (0 for loose, 1 for grasp): ");
        scanf("%d", &release);
        if(release == 1){
            aux.data = -3;
            pub[27].publish(aux);
        }
        else
        {
            aux.data= 0;
            pub[27].publish(aux);
        }
    /*int release;
    scanf("%d", &release);
    if(release == 1){
        aux.data = 3 + del[24];
        pub[24].publish(aux);
    }
    else
    {
        aux.data= -1 + del[24];
        pub[24].publish(aux);
    }*/
	return true;
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
    "/RArmElbowYaw_controller/command",
    //6
    "/LArmElbowYaw_controller/command",
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
    "/RArmElbowRoll_controller/command",
    //22
    "/LArmElbowRoll_controller/command",
    //23
    "/RArmWristYaw_controller/command",
    //24
    "/LArmWristYaw_controller/command",
    //25
    "/RArmWristRoll_controller/command",
    //26
    "/LArmWristRoll_controller/command",
    //27
    "/RArmHand_controller/command",
    //28
    "/LArmHand_controller/command"
};

int main(int argc,char** argv)
{
	ros::init(argc, argv, "init_robot_joint_state");
	ros::NodeHandle n;
    for (int i = 1; i <= JOINT_NUM; ++i) {
        // if(i==1||i==3||i==5||i==21||i==23||i==25)
        //     continue;
        pub[i] = n.advertise<std_msgs::Float64>(node_names[i],1);
    }
//    ros::ServiceServer service = n.advertiseService("init_robot_joint_state", &initJointState);
    ros::Rate loop(60);
    double angle[30] = {0};
    /*angle[1] = 0.8;
    angle[3] = 0.25    ;
    angle[21] = 0.25;
    angle[5] = 0;
    angle[23] = 0;
    angle[25] = 0;
    angle[19] = 0;
    angle[20] = -0.55;*/
    const float Offset = 0;
    
    angle[4] = -0.25;
    angle[6] = -0.3;

    angle[11] = -0.4 - Offset;
    angle[12] = 0.4 + Offset;
    angle[13] = 0.78 + 2 * Offset;
    angle[14] = -0.78 - 2 * Offset;
    angle[15] = 0.38 + Offset;
    angle[16] = -0.38 - Offset;

    //  angle[1] = 1.9;
    //  angle[1] = 0.2;
    //  angle[3] = 0.25;
    //  angle[5] = 0;
    //  angle[21] = 0;
    //  angle[25] = 0.1;
    //  angle[20] = 0.5235;
    //  angle[1] = 90*D2R;
    //  angle[3] = 13*D2R;
    //  angle[5] = 25*D2R;
    //  angle[21] = 14*D2R;
    //  angle[23] = -20*D2R;
    //  angle[25] = 25*D2R;
     angle[1] = 0*D2R;
     angle[3] = 0*D2R;
     angle[5] = 0*D2R;
     angle[21] = 0*D2R;
     angle[23] = 0*D2R;
     angle[25] = 0;
    //  angle[27] = -4;
    //  angle[27] = 0.0;


//    angle[1] = 0;
//    angle[3] = 0;
//    angle[5] = 0;
//    angle[21] = 0;
//    angle[25] = 0;
//    angle[20] = 0;
//    angle[27] = 0;
    std_msgs::Float64 aux;
  while(ros::ok())
  {
   // loop.sleep();
        for (int i = 1; i <= 26; ++i) {
            aux.data = angle[i] + del[i];
            // if(i==1||i==3||i==5||i==21||i==23||i==25)
            //     continue;
            pub[i].publish(aux);
        }
      ros::spinOnce();
      loop.sleep();
  }
//    int release;
//        printf("Input (0 for loose, 1 for grasp): ");
//        scanf("%d", &release);
//        if(release == 1){
//            aux.data = -3;
//            pub[27].publish(aux);
//        }
//        else
//        {
//            aux.data= 0;
//            pub[27].publish(aux);
//        }
/*    angle[27] = -3;
    angle[19] = -0.45;
    angle[10] = 0.2;
    std_msgs::Float64 aux;
    while (ros::ok()) {
        for (int i = 27; i <= 27; ++i) {
            aux.data = angle[i] + del[i];
            pub[i].publish(aux);
        }
        aux.data = angle[19] + del[19];
        pub[19].publish(aux);
        aux.data = angle[10] + del[10];
        pub[10].publish(aux);
        loop.sleep();
    }*/
    ros::spin();
    return 0;
}

