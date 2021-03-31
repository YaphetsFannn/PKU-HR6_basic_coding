// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "rs.hpp"
#include "example.hpp"

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <vector>
#include "ros/ros.h"
#include "dynamixel_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <naoqi_msgs/JointAnglesWithSpeed.h>
#include <memory>
#include "dynamixel_msgs/MotorStateList.h"
#include "dynamixel_msgs/MotorState.h"
#include "realsense_camera/position.h"
#include <map>
//sudo modprobe uvcvideo
using namespace std;

texture_buffer buffers[RS_STREAM_COUNT];
bool align_depth_to_color = true;
bool align_color_to_depth = false;
bool color_rectification_enabled = false;
int no_object = 0, head_move = 0;

int countt = 0, result_count = 0;
int kind; //kind 0-move rand; 1-move with cuboid;
FILE *fr;
//FILE *fw = fopen("result.txt", "a+");

///************** Object Detection Begin **********///
#define HEIGHT 480
#define WIDTH 640
const float INFF = 1e10;
const float PI = acos(-1.0);

float D2R = acos(-1.0)/180.0;
// float del[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, -20*D2R, -40*D2R, 20*D2R, -20*D2R,
//                 10*D2R, 18*D2R, 0, 0, 5*D2R, -15*D2R, 0, 0, 0, 0, -0*D2R, 0, -51*D2R, 0, 0, 0};
int zero_position[29] = {0,
                        512, 543, 506, 512,
                        502, 526, 512, 512,
                        512, 512, 512, 512,
                        512, 512, 512, 682,
                        436, 512, 512, 660,
                        563, 589, 536, 530,
                        578, 424, 628, 454
                        };
// float del[29] = {0,
//                  0, 0, 0, 0,
//                  0, 0, 0, 0,
//                  -20*D2R, -40*D2R, 20*D2R, -20*D2R,
//                  10*D2R, 18*D2R, 0, 0, 
//                  5*D2R, -15*D2R, 0, 0, 
//                  0, 0, -0*D2R, 0, 
//                  -51*D2R, 0, 0, 0};
int arm_joint_index[6] = {1,3,5,21,23,25};
float step_angle = 300.0/1024;
FILE* frame;
vector<vector<float>> read_joint;
int cnt_of_instrct = 0;


float H[HEIGHT][WIDTH], S[HEIGHT][WIDTH], V[HEIGHT][WIDTH];
float X[HEIGHT][WIDTH], Y[HEIGHT][WIDTH], Z[HEIGHT][WIDTH];
int isObj[HEIGHT][WIDTH];
//80 460 50 600
int hb = 20, he = 460, wb = 20, we = 620;
/// 0 - orange, 1 - green, 2 - blue, 3 - purple, 4 - red
float hsv_thresh[5][3] = {{0.0, 0.1, 0.2}, {0.2, 0.4, 0.2}, {0.5, 0.6, 0.2},
                            {0.65, 0.75, 0.2}, {0.9, 1.0, 0.2}};

const uint8_t* data = nullptr;
const uint16_t* depth = nullptr;
float scale;
rs::intrinsics intrin;

float M[4][4];

float M1[4][4] = {{1, 0, 0, -3}, {0, 1, 0, 0}, {0, 0, 1, 14}, {0, 0, 0, 1}};
float M2[4][4] = {{cos(0), 0, sin(0), 0}, 
                    {0, 1, 0, 0},
                  {-sin(0), 0, cos(0), 0},
                   {0, 0, 0, 1}};

/// \return val1 == val2
bool IsEquals(float val1 , float val2)
{
    return fabs(val1 - val2) < 0.001;
}

///
/// \brief Detect
/// \param depth_scale
/// \param depth_intrin
/// \param color
/// \return objector's position

vector<float> Detect(const float& depth_scale,
            const rs::intrinsics& depth_intrin, const int color) {
    // cout <<"start Detect"<<endl;
    //uint8_t *ptr_data = (uint8_t *)data;
    //uint16_t *ptr_depth = (uint16_t *)depth;

    int dep_row = hb * 640;
    // transform from rgb to hsv
    for (int i = hb; i < he; ++i) {
        int dep_col = dep_row;
        int rgb_col = dep_col * 3;
        for (int j = wb; j < we; ++j) {
            // cout<<"i "<<i<<" j "<<j<<endl;
            float h,s,v;
            const rs::float3 point = depth_intrin.deproject(
                {static_cast<float>(i),static_cast<float>(j)},
                        depth[dep_col++]*depth_scale);  // transform from depth to 3D-point
            X[i][j] = point.x, Y[i][j] = point.y, Z[i][j] = point.z;    
            float r = data[rgb_col], g = data[rgb_col + 1], b = data[rgb_col + 2];
            float min_rgb = min(r, min(g, b));
            float max_rgb = max(r, max(g, b));
            v = max_rgb/255;
            float delta = max_rgb - min_rgb;

            if (IsEquals(max_rgb, 0)){
                h = s = 0.0;
            }
            else{
                s = delta / max_rgb;
                if (IsEquals(r, max_rgb) && g >= b){
                    h = 60 * (g - b) / delta + 0;
                }
                else if (IsEquals(r, max_rgb) && g < b){
                    h = 60 * (g - b) / delta + 360;
                }
                else if (IsEquals(g, max_rgb)){
                    h = 60 * (b - r) / delta + 120;
                }
                else if (IsEquals(b, max_rgb)){
                    h = 60 * (r - g) / delta + 240;
                }
            }

            if ((h>340||h<20)&&(s > 0.4)) {
                isObj[i][j] = 1;
            }else{
                isObj[i][j] = 0;
            }
            rgb_col += 3;
        }
        dep_row += 640;
    }



    // get object's position
    vector<float> obj_position{0.0,0.0,0.0};
    vector<float> obj_cols;
    vector<float> obj_rows;
    float mind_obj_depth = 0.0;
    int obj_cnt = 0;

    for(int i=hb;i<he;i++){      
        for(int j=wb;j<we;j++){         
            if(isObj[i][j]){
                obj_cols.push_back(i);
                obj_rows.push_back(j);
                obj_cnt++;
            }
        }
    }
    sort(obj_cols.begin(), obj_cols.end());
    sort(obj_rows.begin(), obj_rows.end());
    int mid_col_index = obj_cols.size()/2;
    int mid_row_index = obj_rows.size()/2;
    // set obj position as median
    int obj_col = 0;
    int obj_row = 0;
    if(obj_cnt>40){
        obj_col = (obj_cols[mid_col_index - 1] + obj_cols[mid_col_index] + obj_cols[mid_col_index +1])/3;
        obj_row = (obj_rows[mid_col_index - 1] + obj_rows[mid_col_index] + obj_rows[mid_col_index +1])/3;
        obj_position[0] = Z[obj_col][obj_row];
        obj_position[1] = -Y[obj_col][obj_row];
        obj_position[2] = -X[obj_col][obj_row];
    }
    cout<<"***********************************************************************"<<endl;
    for(int i=hb; i<he;i+=10){
        for(int j=wb; j<we;j+=10){
            if(i>obj_col-10&&i<obj_col+10&&j>obj_row-10&&j<obj_row+10) cout<<"#";
            else if(isObj[i][j]) cout<<"*";
            else cout<<" ";
        }
        cout<<endl;
    }
    cout<<"***********************************************************************"<<endl;

    return obj_position;
}

///************** Object Detection End **********///
/// \brief Mul
///
void Mul(float (*m1)[4], float (*m2)[4]) {
    float m[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            m[i][j] = 0;
            for (int k = 0; k < 4; ++k)
                m[i][j] += m1[i][k] * m2[k][j];
        }
    }
    memcpy(m1, m, sizeof(m));
}

ros::Publisher pub;
ros::Publisher pub_hn[8];

// int count_ = 1;

/// \brief poseMessageReceived
/// \param msg
///
map<int,int> motorId_states;

int FLAG = 0;
int video_switch = 1;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<realsense_camera::position>("/hand_position", 1);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/motor_states/pan_tilt_port", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const dynamixel_msgs::MotorStateList& msg)
  {
    auto motor_positions = msg.motor_states;
    // printf("motor positions size is:%d\n",motor_positions.size());
    
    for(auto motor_state : motor_positions){
        motorId_states[motor_state.id] = motor_state.position;
    }
    if(data == NULL || depth == NULL) return;
    vector<float> obj_position_cmr = Detect(scale, intrin, 3);
    obj_position_cmr.push_back(1);  //{x,y,z,1}

    // printf("position is %d\n",motorId_states[20]);
    // printf("zeroposition is %d\n",zero_position[20]);
    bool need_debug = false;
    if(need_debug){
        for(auto index:arm_joint_index){
            float angle = (motorId_states[index] - zero_position[index]) * step_angle;
            printf("joint %d : now in %d, angle is %.1f\n",
                    index, motorId_states[index], angle);
        }
    }
    
    float cur_head_pitch = (motorId_states[20] - zero_position[20]) * step_angle * D2R; //head pitch

    M2[0][0] = cos(cur_head_pitch);
    M2[0][2] = sin(cur_head_pitch);
    M2[2][0] = -sin(cur_head_pitch);
    M2[2][2] = cos(cur_head_pitch);
    vector<float> obj_position_base;
    for(int i=0;i<4;i++){
        float tmp = 0.0;
        for(int j=0;j<4;j++){
            tmp+=M2[i][j]*obj_position_cmr[j];
        }
        obj_position_base.push_back(tmp);
    }
    obj_position_base[0] += 4.4/100;
    // obj_position_base[1] -= 3/100;
    obj_position_base[1] += 1.8/100;
    obj_position_base[2] += 5.3/100;
    if(!(obj_position_base[0]>0.40||obj_position_base[0]<0.03)&&!IsEquals(obj_position_cmr[0],0.0)){
        printf("head pitch theta is : %.2f\n", cur_head_pitch);
        printf("position_cmr(cm):  (x,y,z)| (%.4f , %.4f, %.4f)\n",
                obj_position_cmr[0]*100, obj_position_cmr[1]*100, obj_position_cmr[2]*100);
        printf("position_base(cm): (x,y,z)| (%.4f , %.4f, %.4f)\n",
                obj_position_base[0]*100, obj_position_base[1]*100, obj_position_base[2]*100);
        realsense_camera::position output;        
        output.A=obj_position_base[0]*100;
        output.B=obj_position_base[1]*100;
        output.C=obj_position_base[2]*100;
        pub_.publish(output);
    }else{
        cout<<"Object not found!\n";
    }
    ros::Duration(0.3).sleep();
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

int main(int argc, char * argv[]) try
{
    //cin >> kind;
    ros::init(argc, argv, "get_hand_position");

    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");
    srand((unsigned int)(time(NULL))); /// use for object detection

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    dev.enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 15);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    //dev.enable_stream(rs::stream::infrared, rs::preset::best_quality);
    //try { dev.enable_stream(rs::stream::infrared2, 0, 0, rs::format::any, 0); } catch(...) {}
    const float depth_scale = dev.get_depth_scale();
    const rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);

    // Compute field of view for each enabled stream
    for(int i = 0; i < 2; ++i)  // original 4
    {
        auto stream = rs::stream(i);
        if(!dev.is_stream_enabled(stream)) continue;
        auto intrin = dev.get_stream_intrinsics(stream);
        std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
        std::cout << std::setprecision(1) << std::fixed
                  << ", fov = " << intrin.hfov() << " x " 
                  << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
    }

    // Start our device
    dev.start();

    // Open a GLFW window
    // glfwInit();
    std::ostringstream ss; ss << "CPP Capture Example (" << dev.get_name() << ")";
    GLFWwindow * win = glfwCreateWindow(1280, 480, ss.str().c_str(), 0, 0);
    // glfwSetWindowUserPointer(win, &dev);
    glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods)
    {
        auto dev = reinterpret_cast<rs::device *>(glfwGetWindowUserPointer(win));
        if(action != GLFW_RELEASE) switch(key)
        {
        case GLFW_KEY_R: color_rectification_enabled = !color_rectification_enabled; break;
        case GLFW_KEY_C: align_color_to_depth = !align_color_to_depth; break;
        case GLFW_KEY_D:
            {
            if (FLAG == 0) FLAG = 1;
            else if (FLAG == 2) FLAG = 3;
            if (video_switch == 0) {
                video_switch = 1;
                printf("Close successfully\n");
            }

        }break;
            //align_depth_to_color = !align_depth_to_color; break;
        case GLFW_KEY_E:
            if(dev->supports_option(rs::option::r200_emitter_enabled))
            {
                int value = !dev->get_option(rs::option::sr300_auto_range_enable_laser);
                std::cout << "Setting emitter to " << value << std::endl;
                dev->set_option(rs::option::sr300_auto_range_enable_laser, value);
            }
            break;
        case GLFW_KEY_A:
            if(dev->supports_option(rs::option::r200_lr_auto_exposure_enabled))
            {
                int value = !dev->get_option(rs::option::r200_lr_auto_exposure_enabled);
                std::cout << "Setting auto exposure to " << value << std::endl;
                dev->set_option(rs::option::r200_lr_auto_exposure_enabled, value);
            }
            break;
        case GLFW_KEY_V:
            break;
        }
    });
    glfwMakeContextCurrent(win);

    int cnt = 0;
    scale = depth_scale;
    intrin = depth_intrin;

    // ros::Subscriber sub_head_pitch = nh.subscribe("/HeadPitch_controller/state", 1000, poseMessageReceived);
    SubscribeAndPublish SAPObject;
    ros::Rate r(10); // 10 hz 
    rs::log_to_console(rs::log_severity::warn);
    while (!glfwWindowShouldClose(win) && ros::ok())
    {
        // Wait for new images
        glfwPollEvents();
        dev.wait_for_frames();

        // Clear the framebuffer
        int w,h;
        glfwGetFramebufferSize(win, &w, &h);
        // glViewport(0, 0, w, h);
        // glClear(GL_COLOR_BUFFER_BIT);

        // Draw the images
        // glPushMatrix();
        // glfwGetWindowSize(win, &w, &h);
        // glOrtho(0, w, h, 0, -1, +1);
        data = (const uint8_t*)buffers[0].show(\
            dev, align_color_to_depth ? rs::stream::color_aligned_to_depth : \
            (color_rectification_enabled ? rs::stream::rectified_color : \
            rs::stream::color), 0, 0, w/2, h/2);
        depth = (const uint16_t*)buffers[1].show(\
        dev, align_depth_to_color ? (color_rectification_enabled ? \
        rs::stream::depth_aligned_to_rectified_color : rs::stream::depth_aligned_to_color) : \
        rs::stream::depth, w/2, 0, w-w/2, h/2);
        ++cnt;
        r.sleep();
        ros::spinOnce();
        // glPopMatrix();
        glfwSwapBuffers(win);
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
