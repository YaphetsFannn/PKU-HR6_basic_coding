// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "rs.hpp"
#include "example.hpp"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include "ros/ros.h"
#include "dynamixel_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <naoqi_msgs/JointAnglesWithSpeed.h>
#include <memory>
//sudo modprobe uvcvideo
using namespace std;

int isLong = 0;
texture_buffer buffers[RS_STREAM_COUNT];
bool align_depth_to_color = true;
bool align_color_to_depth = false;
bool color_rectification_enabled = false;
int no_object = 0;

ros::Publisher pub_isLong;
int countt = 0, result_count = 0;
//FILE *fr = fopen("data1.txt", "r");
//FILE *fw = fopen("result.txt", "a+");
///************** Object Detection Begin **********///
#define HEIGHT 480
#define WIDTH 640
const float INFF = 1e10;
const float PI = acos(-1.0);
double D2R = acos(-1.0)/180.0;
double del[] = {0, 0, 0, 0, 0, -25*D2R, 20*D2R, 0, 0, -20*D2R, -40*D2R, 20*D2R, -20*D2R,
                10*D2R, 18*D2R, 0, 0, 5*D2R, -15*D2R, 0, -40*D2R, 0, 0, -0*D2R, 0};
char rgb_graph[HEIGHT][WIDTH][3];
float H[HEIGHT][WIDTH], S[HEIGHT][WIDTH];
float X[HEIGHT][WIDTH], Y[HEIGHT][WIDTH], Z[HEIGHT][WIDTH];
//80 460 50 600
int hb = 20, he = 460, wb = 20, we = 620;
bool mask[HEIGHT][WIDTH];
/// 0 - orange, 1 - green, 2 - blue, 3 - purple, 4 - red
float hsv_thresh[5][3] = {{0.0, 0.1, 0.2}, {0.2, 0.4, 0.2}, {0.5, 0.6, 0.2},
                            {0.65, 0.75, 0.2}, {0.9, 1.0, 0.2}};
float p1[3], p2[3], p3[3], w[4];
int x[3][HEIGHT*WIDTH], y[3][HEIGHT*WIDTH], sz;

/// recognize points of three faces;
//int x_red[HEIGHT*WIDTH], y_red[HEIGHT*WIDTH], x_yellow[HEIGHT*WIDTH], y_yellow[HEIGHT*WIDTH], x_cyan[HEIGHT*WIDTH], y_cyan[HEIGHT*WIDTH];
int sz_red, sz_yellow, sz_cyan;
const int red = 0, yellow = 1, cyan = 2;//red, green, blue
bool start_next = true;

int n_face;
int cuboid = 2;
float face[3][5];
const uint8_t* data = nullptr;
const uint16_t* depth = nullptr;
float scale;
rs::intrinsics intrin;

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

void RotateX(float (*R)[4], const float phi) {
   memset(R, 0, sizeof(R));
   R[0][0] = R[3][3] = 1;
   R[1][1] = cos(phi), R[1][2] = -sin(phi);
   R[2][1] = sin(phi), R[2][2] = cos(phi);
}

void RotateY(float (*R)[4], const float phi) {
   memset(R, 0, sizeof(R));
   R[1][1] = R[3][3] = 1;
   R[0][0] = cos(phi), R[0][2] = sin(phi);
   R[2][0] = -sin(phi), R[2][2] = cos(phi);
}

void RotateZ(float (*R)[4], const float phi) {
   memset(R, 0, sizeof(R));
   R[2][2] = R[3][3] = 1;
   R[0][0] = cos(phi), R[0][1] = -sin(phi);
   R[1][0] = sin(phi), R[1][1] = cos(phi);
}

/// \brief GetFace
/// calculate a 3D-plane according to three 3D-point
void GetFace() {
    w[0] = (p2[1] - p1[1])*(p3[2] - p1[2]) - (p3[1] - p1[1])*(p2[2] - p1[2]);
    w[1] = (p2[2] - p1[2])*(p3[0] - p1[0]) - (p3[2] - p1[2])*(p2[0] - p1[0]);
    w[2] = (p2[0] - p1[0])*(p3[1] - p1[1]) - (p3[0] - p1[0])*(p2[1] - p1[1]);
    w[3] = -(w[0]*p1[0] + w[1]*p1[1] + w[2]*p1[2]);
    if (w[2] > 0) {   // keep the 'a' coefficient positive
        w[0] = -w[0], w[1] = -w[1], w[2] = -w[2], w[3] = -w[3];
    }
}
///
/// \brief Sample
/// \param p - to receive a 3D-point
/// \param id - a 3D-point id to which sampled points near
/// \return 0 - seccuss, -1 - failure
/// sample a valid 3D-point near to a specific 3D-point(id)
int Sample(float *p, const int &id, int face_num) {
    int xt, yt, cnt = 0;
    while (++cnt <= 8) {
        //int id2 =
        xt = x[face_num][id] +  ((float)rand() / RAND_MAX - 0.5) * 40;
        yt = y[face_num][id] + ((float)rand() / RAND_MAX - 0.5) * 40;
        if (xt < hb || xt > he || yt < wb || yt > we || Z[xt][yt] < 1e-8)
         {
            //printf("%d %d %f \n", xt, yt, Z[xt][yt]);
            continue;
        }
        p[0] = X[xt][yt], p[1] = Y[xt][yt], p[2] = Z[xt][yt];
        return 0;
    }
    return -1;
}

float M[4][4];
float M1[4][4] = {{1, 0, 0, -3}, {0, 1, 0, 0}, {0, 0, 1, 14}, {0, 0, 0, 1}};
float M2[4][4] = {{cos(0), 0, sin(0), 0}, {0, 1, 0, 0},
                  {-sin(0), 0, cos(0), 0}, {0, 0, 0, 1}};
float M3[4][4] = {{cos(0), -sin(0), 0, 0}, {sin(0), cos(0), 0, 0},
                  {0, 0, 1, 0}, {0, 0, 0, 1}};
//float M4[4][4] = {{0.7543, 0.0204, 0.1490, 9.0713}, {-0.9645, -0.7071, 1.0109, 17.1346},
//                  {0.6330, 0.0171, 0.1251, -8.4453}, {0, 0, 0, 1}};
float M4[4][4] = {{1, 0, 0, 4}, {0, 1, 0, 4}, {0, 0, 1, 2}, {0, 0, 0, 1}};
float M5[4][4] = {{cos(PI/2), 0, sin(PI/2), 0}, {0, 1, 0, 0},
                  {-sin(PI/2), 0, cos(PI/2), 0}, {0, 0, 0, 1}};
float M6[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
float M7[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 1}};

float r, g, b, u, v;
float f1, f2, root, dis, Max, Min, angle, closest;
float sX, sY, sZ;
int dep_row, dep_col, rgb_col;
int cx, cy, m_mod, nTried;
int tx, ty, dx, dy, direction;
int dx_red, tx_red, dy_red, ty_red, tz_red;
float ax = 0, ay = 0, az = 0;
int all = 0;

float Mt[4][4], cur_neck, cur_head, goal_neck, goal_head, cur_shoulder, cur_high, cur_low, cur_wrist, goal_shoulder, goal_high, goal_low, goal_wrist;
float sum_head_pos, sum_neck_pos;
///
/// \brief Detect
/// \param depth_scale
/// \param depth_intrin
/// \param color
/// \return 0 - success, -1 - detect but retract information fail, -2 - detect obeject fail
/// detect an object, and calculate its center of top face and three faces' normal vector
int Detect(const float& depth_scale,
            const rs::intrinsics& depth_intrin, const int color) {
    //int color = 3;
    sz = 0, n_face = 0;
    sz_red = 0, sz_yellow = 0, sz_cyan = 0;
    //uint8_t *ptr_data = (uint8_t *)data;
    //uint16_t *ptr_depth = (uint16_t *)depth;
    dep_row = hb * 640;
    closest = INFF;
    tx = ty = tx_red = ty_red = 0;

    FILE *fw = fopen("rgb.txt", "w");
    if(!fw)
    {
        puts("Read error\n");
        exit(-1);
    }
    //for (int i = 0; i < 480; ++i) {
    for (int i = hb; i <= he; ++i) {
        dep_col = dep_row + wb;
        rgb_col = dep_col * 3;
        //for (int j = 0; j < 640; ++j) {
        for (int j = wb; j <= we; ++j) {
            const rs::float3 point = depth_intrin.deproject(
                {static_cast<float>(i),static_cast<float>(j)},
                        depth[dep_col++]*depth_scale);
            X[i][j] = point.x, Y[i][j] = -point.y, Z[i][j] = point.z;    // transform from depth to 3D-point
            r = data[rgb_col], g = data[rgb_col + 1], b = data[rgb_col + 2];
            u = min(r, min(g, b));
            v = max(r, max(g, b));
            if (v == u) H[i][j] = S[i][j] = 0.0;   // transform from RGB to HSV
            else {
                f1 = (r == u) ? g - b : ((g == u) ? b - r : r - g);
                f2 = (r == u) ? 3 : ((g == u) ? 5.0 : 1.0);
                H[i][j] = (f2 - f1 / (v - u)) / 6.0;
                S[i][j] = (v - u) / v;
            }
            /*if(i == 240 && j == 320){
                printf("%f %f %f\n", H[i][j], S[i][j], Z[i][j]);
                return 0;
            }*/
            //red face
            if (((H[i][j] > 0.9 &&       // find 3D-point
                H[i][j] < 1.1) || (H[i][j] >= 0 && H[i][j] < 0.05)) &&
                S[i][j] > 0.4 &&
                Z[i][j] > 1e-6 && Z[i][j] < 0.4 && v > 0.4) {
                    //r = 255, g = 255, b = 255;
                    x[0][sz_red] = i, y[0][sz_red++] = j;
                    tx += i, ty += j;
                    tx_red += i, ty_red += j;
                    //data[rgb_col] = 0; data[rgb_col + 1] = 0, data[rgb_col + 2] = 0;
                    if (Z[i][j] < closest) {
                        closest = Z[i][j], cx = i, cy = j;
                    }
            }

            else
            {
                Z[i][j] = 0;
                //r = 0, b = 0, g = 0;
            }
            /*int q, w, e;
            q = r, w = g, e= b;
            fprintf(fw, "%d %d %d ", q, w, e);*/
            rgb_col += 3;
        }
        dep_row += 640;
    }

    sz = sz_red;
    if (sz < 120) {
        //puts("No object detected!");
        return -2;
    }
    no_object = 0;
    //tx /= sz, ty /= sz;
    dx = tx / sz - (he - hb + 1) / 2;
    dy = ty / sz - (we - wb + 1) / 2;
    if (abs(dx) > 45 || abs(dy) > 45) {
        //printf("%d %d\n", dx, dy);
        puts("Not centralized yet!");
        return -3;
    }
    //return 0;

    int id, cnt;

    int no_dec = -1;

    float tar_x, tar_y, tar_z, red_x, red_y, red_z;
    dx_red = tx_red / sz_red;
    dy_red = ty_red / sz_red;
    /*int Min = 99999999, mx, my, tmp;
    for (int i = 0 ; i < sz_red; ++i) {
        tmp = abs(x[0][i] - dx_red) + abs(y[0][i] - dy_red);
        if (tmp < Min) {\
            Min = tmp;
            mx = x[0][i], my = y[0][i];
        }
    }
    dx_red = mx, dy_red = my;*/
    red_x = X[dx_red][dy_red] * 100;
    red_y = Y[dx_red][dy_red] * 100;
    red_z = Z[dx_red][dy_red] * 100;
//    if(cuboid == 3)
//    {
//        tar_x = red_x - 3.5 * face[0][0] - 7 * face[2][0];
//        tar_y = red_y - 3.5 * face[0][1] - 7 * face[2][1];
//        tar_z = red_z - 3.5 * face[0][2] - 7 * face[2][2];

//        /*tar_x = red_x - 5 * face[0][0] - 7 * face[2][0];
//        tar_y = red_y - 5 * face[0][1] - 7 * face[2][1];
//        tar_z = red_z - 5 * face[0][2] - 7 * face[2][2];*/

//        M7[0][0] += face[1][0], M7[1][0] += face[1][1], M7[2][0] += face[1][2];
//        M7[0][1] += face[2][0], M7[1][1] += face[2][1], M7[2][1] += face[2][2];
//        M7[0][2] += face[0][0], M7[1][2] += face[0][1], M7[2][2] += face[0][2];
//        /*float len = sqrt(M7[0][0]*M7[0][0] + M7[1][0]*M7[1][0] + M7[2][0]*M7[2][0]);
//        printf("%d vec %f %f %f\n", sz_red, M7[0][0] / len, M7[1][0] / len, M7[2][0] / len);*/
//    M6[0][3] += tar_x, M6[1][3] += tar_y, M6[2][3] += tar_z;

//    }
    if(cuboid == 2)
    {
        /*tar_x = red_x - 1.7 * face[0][0] - 3.4 * face[2][0];
        tar_y = red_y - 1.7 * face[0][1] - 3.4 * face[2][1];
        tar_z = red_z - 1.7 * face[0][2] - 3.4 * face[2][2];*/
        if(isLong == 0)
        {
//            tar_x = red_x - 1.7 * face[0][0] - 1 * face[2][0];
//            tar_y = red_y - 1.7 * face[0][1] - 1 * face[2][1];
//            tar_z = red_z - 1.7 * face[0][2] - 1 * face[2][2];
              tar_x = red_x;
              tar_y = red_y;
              tar_z = red_z;
        }
//        else
//        {
//            std_msgs::Float64 isLong_msg;
//            isLong_msg.data = isLong;
//            pub_isLong.publish(isLong_msg);
//            tar_x = red_x - 1.7 * face[0][0] + 1 * face[2][0] + 5 * face[0][0];
//            tar_y = red_y - 1.7 * face[0][1] + 1 * face[2][1] + 5 * face[0][1];
//            tar_z = red_z - 1.7 * face[0][2] + 1 * face[2][2] + 5 * face[0][2];
//            isLong = 0;
//        }
        /*tar_x = red_x - 1.7 * face[0][0] - 1.5 * face[2][0];
        tar_y = red_y - 1.7 * face[0][1] - 1.5 * face[2][1];
        tar_z = red_z - 1.7 * face[0][2] - 1.5 * face[2][2];*/


//        M7[0][0] += face[1][0], M7[1][0] += face[1][1], M7[2][0] += face[1][2];
//        M7[0][1] += face[2][0], M7[1][1] += face[2][1], M7[2][1] += face[2][2];
//        M7[0][2] += face[0][0], M7[1][2] += face[0][1], M7[2][2] += face[0][2];
        M7[0][0] = M7[1][1] = M7[2][2] = M7[3][3] = 1;
        /*float len = sqrt(M7[0][0]*M7[0][0] + M7[1][0]*M7[1][0] + M7[2][0]*M7[2][0]);
        printf("%d vec %f %f %f\n", sz_red, M7[0][0] / len, M7[1][0] / len, M7[2][0] / len);*/
        M6[0][3] = tar_x, M6[1][3] = tar_y, M6[2][3] = tar_z;
    }
    printf("success\n");

    return 0;
}
///************** Object Detection End **********///

/// \brief Mul
///

ros::Publisher pub;
//input[10] -= 2;
int pub_rate = 1, pub_cnt = 0;
bool stop_adver = false;
float pub_data[12];
int dec_round = 0, state = 0, data_cnt = 0;

ros::Publisher pub_hn[9];
std_msgs::Float64 aux;

void posMessageGrasp(const std_msgs::Float64& msg) {
   if(cuboid == 2) cuboid = 3, state = 0;
   else if (cuboid == 3) cuboid = 4;
}
void posMessageCanGrasp(const std_msgs::Float64& msg) {
    state = 0;
}

/// \brief poseMessageReceived
/// \param msg
///
void poseMessageReceived(const dynamixel_msgs::JointState& msg){
 //   printf("1\n");
    if (msg.motor_ids[0] == 19) {
        //printf("neck %f\n", msg.current_pos);
        //printf("3\n");
        cur_neck = msg.current_pos;
        goal_neck = msg.goal_pos;
        M3[0][0] = cos(msg.current_pos);
        M3[0][1] = -sin(msg.current_pos);
        M3[1][0] = sin(msg.current_pos);
        M3[1][1] = cos(msg.current_pos);
        return ;
    }

   // printf("head %f\n", msg.current_pos);
    if (msg.motor_ids[0] == 20) {
   //     printf("2\n");
        cur_head = msg.current_pos;
        goal_head = msg.goal_pos;
        M2[0][0] = cos(-msg.current_pos);
        M2[0][2] = sin(-msg.current_pos);
        M2[2][0] = -sin(-msg.current_pos);
        M2[2][2] = cos(-msg.current_pos);
    }
    else return ;

    int judge;
    //printf("state = %d, cuboid = %d\n", state, cuboid);
    if (state == 0 && data != NULL && depth != NULL) {
     //   printf("neck = %f, head = %f\n", cur_neck, -cur_head);
        if ((judge = Detect(scale, intrin, cuboid)) == 0) {
            //return;

            memset(M, 0, sizeof(M));
            M[0][0] = M[1][1] = M[2][2] = M[3][3] = 1;
            for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                Mt[i][j] = M[i][j];
            Mul(M, M6); Mul(M, M7);
            naoqi_msgs::JointAnglesWithSpeed msg_pub;
            msg_pub.joint_angles.push_back(-cur_head);
            msg_pub.joint_angles.push_back(cur_neck);
            for (int j = 0; j < 4; ++j)
            for (int i = 0; i < 3; ++i)
                msg_pub.joint_angles.push_back(M[i][j]);   // NEW
            msg_pub.joint_angles.push_back(cuboid);
                //msg_pub.joint_angles.push_back(x[j*3 + i]);
            for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                msg_pub.joint_angles.push_back(Mt[i][j]);
                msg_pub.joint_angles.push_back(M6[i][j]);
                msg_pub.joint_angles.push_back(M7[i][j]);
            }
            pub.publish(msg_pub);
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    printf("%f ", M[i][j]);
                }
                puts("");
            }
            printf("Done!\n");
            state = 1;
            memset(M6, 0, sizeof(M6));
            memset(M7, 0, sizeof(M7));
            M6[0][0] = M6[1][1] = M6[2][2] = M6[3][3] = 1;
            M7[0][0] = M7[1][1] = M7[2][2] = M7[3][3] = 1;
        }

        else if (state == 0 && judge == -2) {
    //        printf("1\n");
            if (fabs(goal_neck - cur_neck) > 0.1) {return ;}
            if (direction == 0) aux.data = goal_neck + 0.05;
            else aux.data = goal_neck - 0.05;
            if (aux.data < 0.75 && aux.data > -0.75) pub_hn[0].publish(aux);//, printf("neck: %f\n", aux.data);
            if (fabs(aux.data) > 0.65)
            {
                direction ^= 1;
                /*no_object++;
                if(no_object > 1)
                {
                    no_object = 0;
                    printf("out!\n");
                    start_next = true;
                }*/
            }
            /*
            printf("%f %f %f\n", goal_head, goal_neck, cur_neck);
            if (fabs(goal_head - cur_head) > 0.015
                    || fabs(goal_neck - cur_neck) > 0.015) return ;
            if (goal_head > 0.1) aux.data = -0.6;
            else aux.data += goal_head + 0.01;
            printf("aux.data = %f\n", aux.data);
            if (aux.data < 0.2 && aux.data > -0.7) pub_hn[1].publish(aux);
            if (direction == 0) aux.data = 0.4, pub_hn[0].publish(aux);
            else aux.data = -0.4, pub_hn[0].publish(aux);
            direction ^= 1;
            */
        }
        else if (judge == -3) {
            //printf("cur = %f\n", cur_head);
            if (fabs(goal_head - cur_head) > 0.15 || fabs(goal_neck - cur_neck) > 0.15) return ;
            if (dx < -20 && cur_head < 0.3) aux.data = cur_head + 0.05, pub_hn[1].publish(aux);//, printf("pub head %f\n", aux.data);
            else if (dx > 20 && cur_head > -0.8) aux.data = cur_head - 0.05, pub_hn[1].publish(aux);//, printf("pub head %f\n", aux.data);
            if (dy < -20 && cur_neck < 0.7) aux.data = cur_neck + 0.05, pub_hn[0].publish(aux);
            else if (dy > 20 && cur_neck > -0.7) aux.data = cur_neck - 0.05, pub_hn[0].publish(aux);
            /*if((dx < -45 && cur_head > 0.37) || (dx > 45 && cur_head < -0.87) || (dy < -45 && cur_neck > 0.67) || (dy > 45 && cur_neck < -0.67))
            {
                printf("out!\n");
                //start_next = true;
            }*/
            memset(M6, 0, sizeof(M6));
            memset(M7, 0, sizeof(M7));
            M6[0][0] = 1, M6[1][1] = 1, M6[2][2] = 1, M6[3][3] = 1;
            M7[3][3] = 1;
        }
        //else if (judge == -1) state = 1;
    }
}
int FLAG = 0;
int video_switch = 1;
FILE *fp_frames;
int main(int argc, char * argv[]) try
{
    FILE* fp = fopen("color", "ab+");
    ros::init(argc, argv, "realsense_capture");
    ros::NodeHandle nh;
//    cout << "Input a number: 0 for short hand, 1 for long hand" << endl;
//    cin >> isLong;
    rs::log_to_console(rs::log_severity::warn);
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
        std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
    }

    // Start our device
    dev.start();

    // Open a GLFW window
    glfwInit();
    std::ostringstream ss; ss << "CPP Capture Example (" << dev.get_name() << ")";
    GLFWwindow * win = glfwCreateWindow(1280, 960, ss.str().c_str(), 0, 0);
    glfwSetWindowUserPointer(win, &dev);
    glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods)
    {
        auto dev = reinterpret_cast<rs::device *>(glfwGetWindowUserPointer(win));
        if(action != GLFW_RELEASE) switch(key)
        {
        case GLFW_KEY_R: color_rectification_enabled = !color_rectification_enabled; break;
        case GLFW_KEY_C: align_color_to_depth = !align_color_to_depth; break;
        case GLFW_KEY_D:
            {
            if (FLAG == 0) FLAG = 4;
            else if (FLAG == 2) FLAG = 3;
            if (video_switch == 0) {
                video_switch = 1;
                fclose(fp_frames);
                printf("Close successfully\n");
            }
            else
            {
                video_switch = 0;
                fp_frames = fopen("frames.txt", "a+");
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
            FILE *locate = fopen("locate.txt", "a+");
            if (locate != NULL) {
                fprintf(locate, "%d %d %f %f %f\n", dx, dy, ax / all, ay / all, az / all);
                fclose(locate);
            }
            break;
        }
    });
    glfwMakeContextCurrent(win);
    int cnt = 0;
    scale = depth_scale;
    intrin = depth_intrin;
    ros::Subscriber sub1 = nh.subscribe("/NeckYaw_controller/state", 1000, poseMessageReceived);
    ros::Subscriber sub2 = nh.subscribe("/HeadPitch_controller/state", 1000, poseMessageReceived);
    //ros::Subscriber sub3 = nh.subscribe("/RArmShoulderPitch_controller/state", 1000, poseMessageReceived);
    //ros::Subscriber sub4 = nh.subscribe("/RArmShoulderRoll_controller/state", 1000, poseMessageReceived);
    //ros::Subscriber sub5 = nh.subscribe("/RArmElbowRoll_controller/state", 1000, poseMessageReceived);
    //ros::Subscriber sub6 = nh.subscribe("/RArmElbowYaw_controller/state", 1000, poseMessageReceived);
    ros::Subscriber sub_grasp = nh.subscribe("/rarm_state", 1000, posMessageGrasp);
    ros::Subscriber sub_can_grasp = nh.subscribe("/rarm_can_grasp_state", 1000, posMessageCanGrasp);
    pub_isLong = nh.advertise<std_msgs::Float64>("/IsLong", 1);
    pub_hn[0] = nh.advertise<std_msgs::Float64>("/NeckYaw_controller/command",1);
    pub_hn[1] = nh.advertise<std_msgs::Float64>("/HeadPitch_controller/command",1);
    pub_hn[2] = nh.advertise<std_msgs::Float64>("/RArmShoulderPitch_controller/command",1);
    pub_hn[3] = nh.advertise<std_msgs::Float64>("/RArmShoulderRoll_controller/command",1);
    pub_hn[4] = nh.advertise<std_msgs::Float64>("/RArmElbowYaw_controller/command",1);
    pub_hn[5] = nh.advertise<std_msgs::Float64>("/RArmElbowRoll_controller/command",1);
    pub_hn[6] = nh.advertise<std_msgs::Float64>("/RArmWristYaw_controller/command",1);
    pub_hn[7] = nh.advertise<std_msgs::Float64>("/RArmWristRoll_controller/command",1);
        pub_hn[8] = nh.advertise<std_msgs::Float64>("/RArmHand_controller/command",1);
    pub = nh.advertise<naoqi_msgs::JointAnglesWithSpeed>("/target_position", 1);
    int cntt = 0;
    while (!glfwWindowShouldClose(win) && ros::ok())
    {
        //printf("%d\n", FLAG);
        // Wait for new images
        glfwPollEvents();
        dev.wait_for_frames();

        // Clear the framebuffer
        int w,h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw the images
        glPushMatrix();
        glfwGetWindowSize(win, &w, &h);
        glOrtho(0, w, h, 0, -1, +1);
        data = (const uint8_t*)buffers[0].show(dev, align_color_to_depth ? rs::stream::color_aligned_to_depth : (color_rectification_enabled ? rs::stream::rectified_color : rs::stream::color), 0, 0, w/2, h/2);
        depth = (const uint16_t*)buffers[1].show(dev, align_depth_to_color ? (color_rectification_enabled ? rs::stream::depth_aligned_to_rectified_color : rs::stream::depth_aligned_to_color) : rs::stream::depth, w/2, 0, w-w/2, h/2);
        /*auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
        if (++cnt == 1000) {
            if (points->z) {
                FILE *fp = fopen("point_c.txt", "w");
                int tol = 480 * 640;
                for (int i = 0; i < tol; ++i) {
                    fprintf(fp, "%f %f %f\n", points->x, points->y, points->z);
                    points++;
                }
                fclose(fp);
            }
            break;
        }*/
        /*if (++cnt >= 200 && data != NULL && depth != NULL) {
            Detect(data, depth, depth_scale, depth_intrin, 3);
            break;
        }*/

        //printf("%d\n", cnt);
        if (video_switch == 0 && data != NULL) {
            cnt = 0;
            int tol = 480 * 640 * 3;
            //fwrite(((const uint8_t *)data), tol, 1, fp_frames);
            for (int i = 0; i < tol; ++i)
                fprintf(fp_frames, "%d ", ((const uint8_t *)data)[i]);
            fprintf(fp_frames, "\n");
            video_switch = 1;
            fclose(fp_frames);
            printf("record1_success!\n");
            /*for(int i = 2; i <= 8; ++i)
            {
                std_msgs::Float64 msg_hh;
                msg_hh.data = 0;
                pub_hn[i].publish(msg_hh);
            }*/
        }
        /*
        if (FLAG == 0 && data != NULL && ++cntt == 5) {
            int tol = 480 * 640 * 3;
            cntt = 0;
            for (int i = 0 ; i < tol; ++i)
                fwrite(&((const uint8_t *)data)[i], sizeof(((const uint8_t *)data)[i]), 1, fp);
        }
        if (FLAG == 4) {
            FLAG = 5;
            fclose(fp);
        }
        if ((FLAG == 1 || FLAG == 3) && data != NULL && depth != NULL) {
            int tol = 480 * 640 * 3;
            FILE *fp;
            if (FLAG == 1) fp = fopen("color_b.txt", "w");
            else if (FLAG == 3) fp = fopen("color_p.txt", "w");
            for (int i = 0; i < tol; ++i)
                fprintf(fp, "%d ", ((const uint8_t *)data)[i]);
            fprintf(fp, "\n");
            fclose(fp);
            if (FLAG == 1) fp = fopen("point_b.txt", "w");
            else if (FLAG == 3) fp = fopen("point_p.txt", "w");
            int cc = 0;
            for (int x = 0; x < depth_intrin.height; ++x) {
                for (int y = 0; y < depth_intrin.width; ++y) {
                    const rs::float3 point = depth_intrin.deproject(
                        {static_cast<float>(x),static_cast<float>(y)},
                                ((const uint16_t*)depth)[cc++]*depth_scale);
                    fprintf(fp, "%f %f %f\n", point.x, point.y, point.z);
                }
            }
            fclose(fp);
            if (FLAG == 1) fp = fopen("depth_b.txt", "w");
            else if (FLAG == 3) fp = fopen("depth_p.txt", "w");
            cc = 0;
            for (int x = 0; x < depth_intrin.height; ++x) {
                for (int y = 0; y < depth_intrin.width; ++y) {
                    fprintf(fp, "%u\n", ((const uint16_t*)depth)[cc++]);
                }
            }
            ++FLAG;
        }
        else if (cnt >= 0) ++cnt;*/
        //buffers[2].show(dev, rs::stream::infrared, 0, h/2, w/2, h-h/2);
        //buffers[3].show(dev, rs::stream::infrared2, w/2, h/2, w-w/2, h-h/2);
        ros::spinOnce();
        glPopMatrix();
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
