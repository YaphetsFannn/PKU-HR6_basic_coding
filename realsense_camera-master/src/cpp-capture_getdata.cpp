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
bool start_next = true, head_next = true;

int n_face;
float face[3][5];
const uint8_t* data = nullptr;
const uint16_t* depth = nullptr;
float scale;
rs::intrinsics intrin;

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

float Mt[4][4], cur_neck, cur_head, goal_neck, goal_head, cur_shoulderPitch, cur_shoulderRoll, cur_elbowYaw, cur_elbowRoll, cur_wristYaw, cur_wristRoll, goal_shoulderPitch, goal_shoulderRoll, goal_elbowYaw, goal_elbowRoll, goal_wristYaw, goal_wristRoll;

///
/// \brief Detect
/// \param depth_scale
/// \param depth_intrin
/// \param color
/// \return 0 - success, -1 - detect but retract information fail, -2 - detect obeject fail
/// detect an object, and calculate its center of top face and three faces' normal vector
int Detect(const float& depth_scale,
            const rs::intrinsics& depth_intrin, const int color) {
    /*if(kind == 0)
    {
        if(fabs(goal_shoulderPitch - cur_shoulderPitch) > 0.1 || fabs(goal_shoulderRoll - cur_shoulderRoll) > 0.1 || fabs(goal_elbowYaw - cur_elbowYaw) > 0.1
               || fabs(goal_elbowRoll - cur_elbowRoll) > 0.1 || fabs(goal_wristYaw - cur_wristYaw) > 0.1 || fabs(goal_wristRoll - cur_wristRoll) > 0.1)
        {
            //printf("%f %f\n", goal_shoulder, cur_shoulder);
            //printf("not reach\n");

        }
        else
            start_next = true;
        if(fabs(goal_head - cur_head) > 0.06 || fabs(goal_head - cur_head) > 0.06)
        {

        }
        else
            head_next = true;
        return 0;
    }*/
    //int color = 3;
    sz = 0, n_face = 0;
    sz_red = 0, sz_yellow = 0, sz_cyan = 0;
    //uint8_t *ptr_data = (uint8_t *)data;
    //uint16_t *ptr_depth = (uint16_t *)depth;
    dep_row = hb * 640;
    closest = INFF;
    tx = ty = tx_red = ty_red = 0;

    /*FILE *fww = fopen("t.txt", "w");
    if(!fww)
    {
        puts("Read error\n");
        exit(-1);
    }*/
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
            if (((H[i][j] > 0.86 &&       // find 3D-point
                H[i][j] < 1.1) || (H[i][j] >= 0 && H[i][j] < 0.05)) &&
                S[i][j] > 0.5 &&
                Z[i][j] > 1e-6 && Z[i][j] < 0.4 && v > 0.4) {
                    //r = 255, g = 0, b = 0;
                    x[0][sz_red] = i, y[0][sz_red++] = j;
                    tx += i, ty += j;
                    tx_red += i, ty_red += j;
                    //data[rgb_col] = 0; data[rgb_col + 1] = 0, data[rgb_col + 2] = 0;
                    if (Z[i][j] < closest) {
                        closest = Z[i][j], cx = i, cy = j;
                    }
            }
            //yellow face
            else if (H[i][j] > 0.25 &&       // find 3D-point
                H[i][j] < 0.4 &&
                S[i][j] > 0.5 &&
                //S[i][j] < 0.6 &&
                Z[i][j] > 1e-6 && Z[i][j] < 0.4  && v > 0.4) {
                //r = 0, g = 255, b = 0;
                x[1][sz_yellow] = i, y[1][sz_yellow++] = j;
                tx += i, ty += j;
                if (Z[i][j] < closest) {
                    closest = Z[i][j], cx = i, cy = j;
                }
            }
            //cyan face
            else if (H[i][j] > 0.5 &&       // find 3D-point
                H[i][j] < 0.6 &&
                S[i][j] > 0.7 &&
                //S[i][j] < 0.7 &&
                Z[i][j] > 1e-6 && Z[i][j] < 0.4  && v > 0.4) {
                //r = 0, g = 0, b = 255;
                x[2][sz_cyan] = i, y[2][sz_cyan++] = j;
                tx += i, ty += j;
                if (Z[i][j] < closest) {
                    closest = Z[i][j], cx = i, cy = j;
                }
            }

            else
            {
                Z[i][j] = 0;
                    //r= 0, b = 0, g =0;
            }
            /*int q, w, e;
            q = r, w = g, e= b;
            fprintf(fww, "%d %d %d ", q, w, e);*/
            rgb_col += 3;
        }
        dep_row += 640;
    }
    //fclose(fww);
    //exit(-1);
    /*int aa[480][640 * 3] = {0};
    for(int i = 0; i < sz_red; ++i)
        aa[x[0][i]][y[0][i] * 3] = Z[x[0][i]][y[0][i]] * 100;
    for(int i = 0; i < sz_yellow; ++i)
        aa[x[1][i]][y[1][i] * 3 + 1] = Z[x[1][i]][y[1][i]] * 100;
    for(int i = 0; i < sz_cyan; ++i)
        aa[x[2][i]][y[2][i] * 3 + 2] = Z[x[2][i]][y[2][i]] * 100;
    for(int i = 0; i < 480; ++i)
    {
        for(int j = 0; j < 640; j++)
            fprintf(fw, "%d %d %d ", aa[i][j * 3], aa[i][j * 3+ 1],aa[i][j * 3 + 2]);
    }*/
    //return 0;
    sz = sz_red + sz_yellow + sz_cyan;
    if (sz < 12000) {
        puts("No object detected!");
        return -2;
    }
    no_object = 0;
    //tx /= sz, ty /= sz;
    dx = tx / sz - (he - hb + 1) / 2;
    dy = ty / sz - (we - wb + 1) / 2;
    if (abs(dx) > 45 || abs(dy) > 45) {
        //printf("%d %d\n", dx, dy);
        //puts("Not centralized yet!");
        return -3;
    }
    if(fabs(goal_shoulderPitch - cur_shoulderPitch) > 0.1 || fabs(goal_shoulderRoll - cur_shoulderRoll) > 0.1 || fabs(goal_elbowYaw - cur_elbowYaw) > 0.1
            || fabs(goal_elbowRoll - cur_elbowRoll) > 0.1 || fabs(goal_wristYaw - cur_wristYaw) > 0.1 || fabs(goal_wristRoll - cur_wristRoll) > 0.1)
    {
        //printf("%f %f\n", goal_shoulder, cur_shoulder);
        //printf("not reach\n");
        return 0;
    }
    //start_next = true;
    //return 0;
    //printf("hh2\n");
    sleep(0.7);
    int id, cnt;

    int no_dec = -1;
    int nored = 0, nogreen = 0, noblue = 0;
    //get red face
    for (int n = 0; n < 2; n++)
    {
        //printf("%d\n", n);
        //sleep(1);
        bool bre = true;
        int ccc = 0, ddd = 0, fff = 0;
        //printf("get red\n");
        while (bre)
        {
            if (sz_red < 5000)
            {
                printf("no red face!\n");
                no_dec = 0;
                nored = 1;
                break;
            }
            float ttt = 0;
            while(ttt < 1e-8)
            {
                id = (float)rand() / RAND_MAX * (sz_red - 1);
                p1[0] = X[x[0][id]][y[0][id]], p1[1] = Y[x[0][id]][y[0][id]], p1[2] = Z[x[0][id]][y[0][id]];
                ttt = p1[2];
            }
            ttt = 0;
            while(ttt < 1e-8)
            {
                id = (float)rand() / RAND_MAX * (sz_red - 1);
                p2[0] = X[x[0][id]][y[0][id]], p2[1] = Y[x[0][id]][y[0][id]], p2[2] = Z[x[0][id]][y[0][id]];
                ttt = p2[2];
            }
            ttt = 0;
            while(ttt < 1e-8)
            {
                id = (float)rand() / RAND_MAX * (sz_red - 1);
                p3[0] = X[x[0][id]][y[0][id]], p3[1] = Y[x[0][id]][y[0][id]], p3[2] = Z[x[0][id]][y[0][id]];
                ttt = p3[2];
            }


            //printf("%f %f %f \n", p1[0], p1[1], p1[2]);
            //printf("%f %f %f \n", p2[0], p2[1], p2[2]);
            //printf("%f %f %f \n", p3[0], p3[1], p3[2]);
            /*if (Sample(p2, id, 0) == -1 || Sample(p3, id, 0) == -1)
            {
                printf("Samle red face error!\n");
                //printf("%d\n", id);
                continue;
            }*/


            GetFace();
            root = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
            if (root < 1e-8) continue;

            cnt = 0;
            for (int j = 0; j < sz_red; ++j)
            {
                dis = fabs(w[0] * X[x[0][j]][y[0][j]] + w[1] * Y[x[0][j]][y[0][j]] +
                    w[2] * Z[x[0][j]][y[0][j]] + w[3]) / root;
                cnt += (dis < 0.004);
            }
            if (cnt < 3000) {
                //printf("red %d\n", cnt);
                fff++;
                if(fff > 1000)
                    return 0;
                    //break;
                continue;

            }
            for(int i = 0; i < 3; ++i)
                w[i] = w[i] / root;
            if (n == 0) {
                for (int j = 0; j < 4; ++j) face[0][j] = w[j];
                face[0][4] = cnt;
                n_face++;
                //printf("red success!\n");
                bre = false;
            }
            else {
                Min = INFF, Max = -INFF;
                if (n != 0 && no_dec != 1) {
                    f1 = sqrt((face[1][0] * face[1][0] + face[1][1] * face[1][1] + face[1][2] * face[1][2]) *
                    (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]));
                    angle = (face[1][0] * w[0] + face[1][1] * w[1] + face[1][2] * w[2]) / f1;
                    if (angle > Max) Max = angle;
                    if (angle < Min) Min = angle;
                }
                if (n != 0 && no_dec != 2)
                {
                    f1 = sqrt((face[2][0] * face[2][0] + face[2][1] * face[2][1] + face[2][2] * face[2][2]) *
                        (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]));
                    angle = (face[2][0] * w[0] + face[2][1] * w[1] + face[2][2] * w[2]) / f1;
                    if (angle > Max) Max = angle;
                    if (angle < Min) Min = angle;
                }
                if (Max < 0.1 && Min > -0.1 && cnt > face[0][4]) {
                    for (int j = 0; j < 4; ++j) face[0][j] = w[j];
                    face[0][4] = cnt;
                    //printf("success %f %f \n", Max, Min);
                    break;
                }
                else if(Max > 0.3 || Min < -0.3)
                    ccc++;
                else
                    ddd++;
                if(ccc > 15 || ddd > 50)
                {
                    //printf("this is!\n");
                    //n = 1;
                    break;
                }
                continue;
                /*else if (Max < 0.3 && n_face < 3) {
                    printf("red success!\n");
                    bre = false;
                }*/
                //break;
            }
        }

        //printf("get yellow\n");
        //get yellow face
        ccc = 0, ddd = 0, fff = 0;
        bre = true;
        while (bre)
        {
            if (sz_yellow < 4000)
            {
                printf("no yellow face!\n");
                no_dec = 1;
                nogreen = 1;
                break;
            }
            float ttt = 0;
            while(ttt < 1e-8)
            {
                id = (float)rand() / RAND_MAX * (sz_yellow - 1);
                p1[0] = X[x[1][id]][y[1][id]], p1[1] = Y[x[1][id]][y[1][id]], p1[2] = Z[x[1][id]][y[1][id]];
                ttt = p1[2];
            }

            /*if (Sample(p2, id, 1) == -1 || Sample(p3, id, 1) == -1)
            {
                printf("Samle yellow face error!\n");
                continue;
            }*/
            ttt = 0;
            while(ttt < 1e-8)
            {
                id = (float)rand() / RAND_MAX * (sz_yellow - 1);
                p2[0] = X[x[1][id]][y[1][id]], p2[1] = Y[x[1][id]][y[1][id]], p2[2] = Z[x[1][id]][y[1][id]];
                ttt = p2[2];
            }
            ttt = 0;
            while(ttt < 1e-8)
            {
                id = (float)rand() / RAND_MAX * (sz_yellow - 1);
                p3[0] = X[x[1][id]][y[1][id]], p3[1] = Y[x[1][id]][y[1][id]], p3[2] = Z[x[1][id]][y[1][id]];
                ttt = p3[2];
            }

            //printf("%f %f %f \n", p1[0], p1[1], p1[2]);
            //printf("%f %f %f \n", p2[0], p2[1], p2[2]);
            //printf("%f %f %f \n", p3[0], p3[1], p3[2]);
            GetFace();
            root = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
            if (root < 1e-8) {
                //--n;
                continue;
            }

            cnt = 0;
            for (int j = 0; j < sz_yellow; ++j)
            {
                dis = fabs(w[0] * X[x[1][j]][y[1][j]] + w[1] * Y[x[1][j]][y[1][j]] +
                    w[2] * Z[x[1][j]][y[1][j]] + w[3]) / root;
                cnt += (dis < 0.004);
            }
            if (cnt < 3000)
            {
                fff++;
                if(fff > 1000)
                    //break;
                    return 0;
                //printf("yellow %d\n", cnt);
                continue;
            }
            for(int i = 0; i < 3; ++i)
                w[i] = w[i] / root;
            if (n == 0){
            for (int j = 0; j < 4; ++j) face[1][j] = w[j];
            face[1][4] = cnt;
            n_face++;
            //printf("yellow success!\n");
            bre = false;
            }
            else {
                Min = INFF, Max = -INFF;
                if (n != 0 && no_dec != 0) {
                    f1 = sqrt((face[0][0] * face[0][0] + face[0][1] * face[0][1] + face[0][2] * face[0][2]) *
                        (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]));
                    angle = (face[0][0] * w[0] + face[0][1] * w[1] + face[0][2] * w[2]) / f1;
                    if (angle > Max) Max = angle;
                    if (angle < Min) Min = angle;
                }
                if (n != 0 && no_dec != 2)
                {
                    f1 = sqrt((face[2][0] * face[2][0] + face[2][1] * face[2][1] + face[2][2] * face[2][2]) *
                        (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]));
                    angle = (face[2][0] * w[0] + face[2][1] * w[1] + face[2][2] * w[2]) / f1;
                    if (angle > Max) Max = angle;
                    if (angle < Min) Min = angle;
                }
                if (Max < 0.1 && Min > -0.1 && cnt > face[1][4]) {
                    for (int j = 0; j < 4; ++j) face[1][j] = w[j];
                    face[1][4] = cnt;
                    //printf("success %f %f \n", Max, Min);
                    break;
                }
                else if(Max > 0.3 || Min < -0.3)
                    ccc++;
                else ddd++;
                if(ccc > 15 || ddd > 50)
                {
                    //n = 1;
                    //printf("%d this is!\n", n);
                    //break;
                    return 0;
                }
                if(nored + nogreen + noblue == 2 || nored == 1)
                {
                    start_next = true;
                    return -1;
                }
                continue;
                /*else if (Max < 0.3 && n_face < 3) {
                    printf("yellow success!\n");
                    bre = false;
                }*/
                //break;
            }
        }

        //get cyan face
        //printf("get cyan\n");
        ccc = 0, ddd = 0, fff = 0;
        bre = true;
        while (bre)
        {
            if (sz_cyan < 4000)
            {
                printf("no cyan face!\n");
                no_dec = 2;
                noblue = 1;
                break;
            }
            float ttt = 0;
            while(ttt < 1e-8){
                id = (float)rand() / RAND_MAX * (sz_cyan - 1);
                p1[0] = X[x[2][id]][y[2][id]], p1[1] = Y[x[2][id]][y[2][id]], p1[2] = Z[x[2][id]][y[2][id]];
                ttt = p1[2];
            }
            /*if (Sample(p2, id, 2) == -1 || Sample(p3, id, 2) == -1)
            {
                printf("Samle cyan face error!\n");
                continue;
            }*/
            ttt = 0;
            while(ttt < 1e-8){
                id = (float)rand() / RAND_MAX * (sz_cyan - 1);
                p2[0] = X[x[2][id]][y[2][id]], p2[1] = Y[x[2][id]][y[2][id]], p2[2] = Z[x[2][id]][y[2][id]];
                ttt = p2[2];
            }
            ttt = 0;
            while(ttt < 1e-8){
                id = (float)rand() / RAND_MAX * (sz_cyan - 1);
                p3[0] = X[x[2][id]][y[2][id]], p3[1] = Y[x[2][id]][y[2][id]], p3[2] = Z[x[2][id]][y[2][id]];
                ttt = p3[2];
            }

            //printf("%f %f %f \n", p1[0], p1[1], p1[2]);
            //printf("%f %f %f \n", p2[0], p2[1], p2[2]);
            //printf("%f %f %f \n", p3[0], p3[1], p3[2]);
            GetFace();
            root = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
            if (root < 1e-8) continue;

            cnt = 0;
            for (int j = 0; j < sz_cyan; ++j)
            {
                dis = fabs(w[0] * X[x[2][j]][y[2][j]] + w[1] * Y[x[2][j]][y[2][j]] +
                    w[2] * Z[x[2][j]][y[2][j]] + w[3]) / root;
                cnt += (dis < 0.004);
            }
            if (cnt < 3000) {
                fff++;
                if(fff > 1000)
                    //break;
                    return 0;
                continue;

            }
            for(int i = 0; i < 3; ++i)
                w[i] = w[i] / root;
            if (n == 0) {
                for (int j = 0; j < 4; ++j) face[2][j] = w[j];
                face[2][4] = cnt;
                n_face++;
                //printf("cyan success!\n");
                bre = false;
            }
            else {
                Min = INFF, Max = -INFF;
                if (n != 0 && no_dec != 1) {
                    f1 = sqrt((face[1][0] * face[1][0] + face[1][1] * face[1][1] + face[1][2] * face[1][2]) *
                        (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]));
                    angle = (face[1][0] * w[0] + face[1][1] * w[1] + face[1][2] * w[2]) / f1;
                    if (angle > Max) Max = angle;
                    if (angle < Min) Min = angle;
                }
                if (n != 0 && no_dec != 0)
                {
                    f1 = sqrt((face[0][0] * face[0][0] + face[0][1] * face[0][1] + face[0][2] * face[0][2]) *
                        (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]));
                    angle = (face[0][0] * w[0] + face[0][1] * w[1] + face[0][2] * w[2]) / f1;
                    if (angle > Max) Max = angle;
                    if (angle < Min) Min = angle;
                }
                if (Max < 0.1 && Min > -0.1 && cnt > face[2][4]) {
                    for (int j = 0; j < 4; ++j) face[2][j] = w[j];
                    face[2][4] = cnt;
                    //printf("success %f %f \n", Max, Min);
                    break;
                }
                else if(Max > 0.3 || Min < -0.3)
                    ccc++;
                else ddd++;
                if(ccc > 15 || ddd > 50)
                {
                    //n = 1;
                    //printf("%d this is!\n", n);
                    break;
                    return 0;
                }
                if(nored + nogreen + noblue == 2 || nored == 1)
                {
                    start_next = true;
                    return -1;
                }
                continue;
                /*else if (Max < 0.3 && n_face < 3) {
                    printf("cyan success!\n");
                    bre = false;
                }*/
                //break;
            }
        }
        if(nored + nogreen + noblue == 2 || nored == 1)
        {
            start_next = true;
            return -1;
        }
    }
    if (n_face < 2) {
        printf("Only one face detected.\n");
        start_next = true;
        return -1;
    }
    if (n_face == 2) {
        //printf("generate face!\n");
        int face1, face2;
        if (no_dec == 0) { face1 = 1; face2 = 2; }
        else if (no_dec == 1) { face1 = 2; face2 = 0; }
        else if (no_dec == 2) { face1 = 0; face2 = 1; }
        face[no_dec][0] = face[face1][1] * face[face2][2] - face[face1][2] * face[face2][1];
        face[no_dec][1] = face[face1][2] * face[face2][0] - face[face1][0] * face[face2][2];
        face[no_dec][2] = face[face1][0] * face[face2][1] - face[face1][1] * face[face2][0];
        /*if (face[no_dec][2] < 0) {
            face[no_dec][0] = -face[no_dec][0];
            face[no_dec][1] = -face[no_dec][1];
            face[no_dec][2] = -face[no_dec][2];
        }*/
        face[no_dec][3] = 0, face[no_dec][4] = 0;
        ++n_face;
    }
    /*for(int i = 1; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            printf("%f ", face[i][j]);
        }
    }
    for(int j = 0; j < 3; j++)
        printf("%f ", face[0][j]);
    exit(-1);*/
    float tar_x, tar_y, tar_z, red_x, red_y, red_z;
    dx_red = tx_red / sz_red;
    dy_red = ty_red / sz_red;
    red_x = X[dx_red][dy_red] * 100;
    red_y = Y[dx_red][dy_red] * 100;
    red_z = Z[dx_red][dy_red] * 100;
//        tar_x = red_x - 1.7 * face[0][0] - 1 * face[2][0];
//        tar_y = red_y - 1.7 * face[0][1] - 1 * face[2][1];
//        tar_z = red_z - 1.7 * face[0][2] - 1 * face[2][2];
    tar_x = red_x - 1.7 * face[0][0] - 3.3 * face[2][0];
    tar_y = red_y - 1.7 * face[0][1] - 3.3 * face[2][1];
    tar_z = red_z - 1.7 * face[0][2] - 3.3 * face[2][2];
    FILE *fw = fopen("DATA/result.txt", "a+");
    //fprintf(fw, "%d ", ++result_count);
    for(int i = 1; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            fprintf(fw, "%f ", face[i][j]);
        }
    }
    for(int j = 0; j < 3; j++)
        fprintf(fw, "%f ", face[0][j]);
    //3.3, 11.7;

    int sign[] = {-1, 1, -1, -1, -1, 1, -1, -1};
    fprintf(fw, "%f %f %f %f %f %f %f %f ", -cur_head, cur_neck, -cur_shoulderPitch, -cur_shoulderRoll, -cur_elbowYaw, cur_elbowRoll, -cur_wristYaw, -cur_wristRoll);
    fprintf(fw, "%f %f %f\n", tar_x, tar_y, tar_z);
    fclose(fw);

    printf("success\n");

    //sleep(3);
    start_next = true;
    return 0;
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
int pub_rate = 5, pub_cnt = 0;
bool stop_adver = false;
float pub_data[12];
int ccc = 0, state = 0, data_cnt = 0, cuboid = 2;

ros::Publisher pub_hn[8];
std_msgs::Float64 aux;

void posMessageGrasp(const std_msgs::Float64& msg) {
   if (cuboid == 2 && state == 0) cuboid = 2, state = 1, stop_adver = false;
   else if(cuboid == 2 && state == 1) cuboid = 3, state = 0;
   else if(cuboid == 3 && state == 0) cuboid = 3, state = 1;
   else if (cuboid == 3 && state == 1) cuboid = -1;
}
void posMessageCanGrasp(const std_msgs::Float64& msg) {
    state = 0;
    stop_adver = false;
}

/// \brief poseMessageReceived
/// \param msg
///
///

void poseMessageReceived(const dynamixel_msgs::JointState& msg){

    //return ;
    if(!fr)
    {
        printf("error!\n");
        exit(-1);
    }
    /*if(kind == 0 && head_next)
    {
        float head_q = (float)rand() / RAND_MAX * 1.2 - 0.8;
        float neck_q = (float)rand() / RAND_MAX * 1.4 - 0.7;
        aux.data = head_q;
        pub_hn[1].publish(aux);
        aux.data = neck_q;
        pub_hn[0].publish(aux);
        head_next = false;
    }*/
    if(start_next && countt < 6000)
    {
        fscanf(fr, "%d", &countt);
        printf("%d\n", countt);
        float temp1, temp2, temp3, temp4, temp5, temp6;
        fscanf(fr, "%f", &temp1);
        fscanf(fr, "%f", &temp2);
        fscanf(fr, "%f", &temp3);
        fscanf(fr, "%f", &temp4);
        fscanf(fr, "%f", &temp5);
        fscanf(fr, "%f", &temp6);
        if(countt < 0)
            return;
        int sign[] = {-1, -1, -1, 1, -1, -1};
        aux.data = -temp1;
        pub_hn[2].publish(aux);
        //printf("finished1\n");
        aux.data = -temp2;
        pub_hn[3].publish(aux);
        //printf("finished2\n");
        aux.data = -temp3;
        pub_hn[4].publish(aux);
        //printf("finished3\n");
        aux.data = temp4;
        //printf("%f\n", temp4);
        pub_hn[5].publish(aux);
        //printf("finished4\n");
        aux.data = -temp5;
        pub_hn[6].publish(aux);
        //printf("finished5\n");
        aux.data = -temp6;
        pub_hn[7].publish(aux);
        //printf("finished6\n");
        start_next = false;
        //printf("finished\n");
    }
    if (msg.motor_ids[0] == 19) {
        //printf("neck %f\n", msg.current_pos);
        cur_neck = msg.current_pos;
        goal_neck = msg.goal_pos;
        M3[0][0] = cos(msg.current_pos);
        M3[0][1] = -sin(msg.current_pos);
        M3[1][0] = sin(msg.current_pos);
        M3[1][1] = cos(msg.current_pos);
        return ;
    }
    else if (msg.motor_ids[0] == 1) {
        //printf("neck %f\n", msg.current_pos);
        cur_shoulderPitch = msg.current_pos;
        goal_shoulderPitch = msg.goal_pos;
        return ;
    }
    else if (msg.motor_ids[0] == 3) {
        //printf("neck %f\n", msg.current_pos);
        cur_shoulderRoll = msg.current_pos;
        goal_shoulderRoll = msg.goal_pos;
        return ;
    }
    else if (msg.motor_ids[0] == 5) {
        //printf("neck %f\n", msg.current_pos);
        cur_elbowYaw = msg.current_pos;
        goal_elbowYaw = msg.goal_pos;
        return ;
    }
    else if (msg.motor_ids[0] == 21) {
        //printf("neck %f\n", msg.current_pos);
        cur_elbowRoll = msg.current_pos;
        goal_elbowRoll = msg.goal_pos;
        return ;
    }
    else if (msg.motor_ids[0] == 23) {
        //printf("neck %f\n", msg.current_pos);
        cur_wristYaw = msg.current_pos;
        goal_wristYaw = msg.goal_pos;
        return ;
    }
    else if (msg.motor_ids[0] == 25) {
        //printf("neck %f\n", msg.current_pos);
        cur_wristRoll = msg.current_pos;
        goal_wristRoll = msg.goal_pos;
        return ;
    }

    //printf("head %f\n", msg.current_pos);
    cur_head = msg.current_pos;
    goal_head = msg.goal_pos;
    M2[0][0] = cos(-msg.current_pos);
    M2[0][2] = sin(-msg.current_pos);
    M2[2][0] = -sin(-msg.current_pos);
    M2[2][2] = cos(-msg.current_pos);
    int judge;
    state = 0;
    //printf("state = %d, cuboid = %d\n", state, cuboid);
    if (state == 0 && data != NULL && depth != NULL) {
        //printf("neck = %f, head = %f\n", cur_neck, -cur_head);
        if ((judge = Detect(scale, intrin, cuboid)) == 0) {
            return;
            if (++pub_cnt == pub_rate && !stop_adver) {
                memset(M, 0, sizeof(M));
                M[0][0] = M[1][1] = M[2][2] = M[3][3] = 1;
                //FILE *fp = fopen("data3.txt", "w");
                //if (fp == NULL) return ;
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j)
                        M7[j][i] /= pub_rate;
                    float len = sqrt(M7[0][i]*M7[0][i] + M7[1][i]*M7[1][i] + M7[2][i]*M7[2][i]);
                    for (int j = 0; j < 3; ++j) {
                        M7[j][i] /= len;
                        //fprintf(fp, "%f ", M7[j][i]);
                    }
                    //puts("");
                    M6[i][3] /= pub_rate;

                }

                //fprintf(fp, "%f %f %f %f %f\n", M6[0][3], M6[1][3], M6[2][3], cur_head, cur_neck);
                //printf("%d\n", ++data_cnt);
                //fclose(fp);
                Mul(M, M1); Mul(M, M2); Mul(M, M3); Mul(M, M4);
                Mul(M, M5);
                for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    Mt[i][j] = M[i][j];
                Mul(M, M6); Mul(M, M7);
                /*float x[] = {-0.503208027707563,
                        0.851367638929433,
                        -0.148399403949059,
                        0.0809757534948907,
                        -0.340582892150413,
                        -0.936658076077445,
                        -0.882009711132881,
                        -0.456108652644697,
                        0.118184223478470,
                        13.6741171751270,
                        -5.79433305451935,
                        8.20310222554532};*/

                naoqi_msgs::JointAnglesWithSpeed msg_pub;
                for (int j = 0; j < 4; ++j)
                for (int i = 0; i < 3; ++i)
                    msg_pub.joint_angles.push_back(M[i][j]);
                msg_pub.joint_angles.push_back(cuboid);
                    //msg_pub.joint_angles.push_back(x[j*3 + i]);
                for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j) {
                    msg_pub.joint_angles.push_back(Mt[i][j]);
                    msg_pub.joint_angles.push_back(M6[i][j]);
                    msg_pub.joint_angles.push_back(M7[i][j]);
                }
                if (++ccc == 3) {
                    pub.publish(msg_pub);
                    for (int i = 0; i < 4; ++i) {
                        for (int j = 0; j < 4; ++j) {
                            printf("%f ", M[i][j]);
                        }
                        puts("");
                    }
                    printf("Done!\n");
                    stop_adver = true, ccc = 0;
                }
                pub_cnt = 0;
                memset(M6, 0, sizeof(M6));
                memset(M7, 0, sizeof(M7));
                M6[0][0] = 1, M6[1][1] = 1, M6[2][2] = 1, M6[3][3] = 1;
                M7[3][3] = 1;
                //memset(pub_data, 0, sizeof(pub_data));
            }
            //else state = 1;
        }
        else if (state == 0 && judge == -2) {
            if (fabs(goal_neck - cur_neck) > 0.1)return ;
            if (direction == 0) aux.data = goal_neck + 0.05; \
            else aux.data = goal_neck - 0.05;
            if (aux.data < 0.75 && aux.data > -0.75) pub_hn[0].publish(aux);
            if (fabs(aux.data) > 0.65)
            {
                direction ^= 1;
                no_object++;
                if(no_object > 1)
                {
                    if(head_move == 0)
                    {
                        aux.data = -0.5;
                        pub_hn[1].publish(aux);
                    }
                    else
                    {
                        aux.data = 0;
                        pub_hn[1].publish(aux);
                    }
                    head_move ^= 1;
                    no_object = 0;
                    printf("out!\n");
                    start_next = true;
                }
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
            if (fabs(goal_head - cur_head) > 0.1 || fabs(goal_neck - cur_neck) > 0.1) return ;
            if (dx < -20 && cur_head < 0.4) aux.data = cur_head + 0.05, pub_hn[1].publish(aux);
            else if (dx > 20 && cur_head > -0.8) aux.data = cur_head - 0.05, pub_hn[1].publish(aux);
            if (dy < -20 && cur_neck < 0.7) aux.data = cur_neck + 0.05, pub_hn[0].publish(aux);
            else if (dy > 20 && cur_neck > -0.7) aux.data = cur_neck - 0.05, pub_hn[0].publish(aux);
            if((dx < -45 && cur_head > 0.37) || (dx > 45 && cur_head < -0.77) || (dy < -45 && cur_neck > 0.67) || (dy > 45 && cur_neck < -0.67))
            {
                if(fabs(goal_shoulderPitch - cur_shoulderPitch) > 0.1 || fabs(goal_shoulderRoll- cur_shoulderRoll) > 0.1 || fabs(goal_elbowYaw - cur_elbowYaw) > 0.06
                        || fabs(goal_elbowRoll - cur_elbowRoll) > 0.06 || fabs(goal_wristYaw - cur_wristYaw) > 0.06 || fabs(goal_wristRoll - cur_wristRoll) > 0.06)
                {
                    //printf("not reach\n");
                    return;
                }
                printf("out!\n");
                start_next = true;
            }
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
    fr = fopen("DATA/HR6.1_1_tool.txt", "r");
    //cin >> kind;
    ros::init(argc, argv, "realsense_capture");
    ros::NodeHandle nh;

    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");
    srand((unsigned int)(time(NULL))); /// use for object detection
    fp_frames = fopen("frames.txt", "w");

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
            if (FLAG == 0) FLAG = 1;
            else if (FLAG == 2) FLAG = 3;
            if (video_switch == 0) {
                video_switch = 1;
                fclose(fp_frames);
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
    ros::Subscriber sub3 = nh.subscribe("/RArmShoulderPitch_controller/state", 1000, poseMessageReceived);
    ros::Subscriber sub4 = nh.subscribe("/RArmShoulderRoll_controller/state", 1000, poseMessageReceived);
    ros::Subscriber sub5 = nh.subscribe("/RArmElbowYaw_controller/state", 1000, poseMessageReceived);
    ros::Subscriber sub6 = nh.subscribe("/RArmElbowRoll_controller/state", 1000, poseMessageReceived);
    ros::Subscriber sub7 = nh.subscribe("/RArmWristYaw_controller/state", 1000, poseMessageReceived);
    ros::Subscriber sub8 = nh.subscribe("/RArmWristRoll_controller/state", 1000, poseMessageReceived);
    ros::Subscriber sub_grasp = nh.subscribe("/rarm_state", 1000, posMessageGrasp);
    ros::Subscriber sub_can_grasp = nh.subscribe("/rarm_can_grasp_state", 1000, posMessageCanGrasp);
    pub_hn[0] = nh.advertise<std_msgs::Float64>("/NeckYaw_controller/command",1);
    pub_hn[1] = nh.advertise<std_msgs::Float64>("/HeadPitch_controller/command",1);
    pub_hn[2] = nh.advertise<std_msgs::Float64>("/RArmShoulderPitch_controller/command",1);
    pub_hn[3] = nh.advertise<std_msgs::Float64>("/RArmShoulderRoll_controller/command",1);
    pub_hn[4] = nh.advertise<std_msgs::Float64>("/RArmElbowYaw_controller/command",1);
    pub_hn[5] = nh.advertise<std_msgs::Float64>("/RArmElbowRoll_controller/command",1);
    pub_hn[6] = nh.advertise<std_msgs::Float64>("/RArmWristYaw_controller/command",1);
    pub_hn[7] = nh.advertise<std_msgs::Float64>("/RArmWristRoll_controller/command",1);
    pub = nh.advertise<naoqi_msgs::JointAnglesWithSpeed>("/target_position", 1);

    while (!glfwWindowShouldClose(win) && ros::ok())
    {
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
        if (video_switch == 0 && data != NULL && cnt % 10 == 0) {
            cnt = 0;
            int tol = 480 * 640 * 3;
            //fwrite(((const uint8_t *)data), tol, 1, fp_frames);
            for (int i = 0; i < tol; ++i)
                fprintf(fp_frames, "%d ", ((const uint8_t *)data)[i]);
            fprintf(fp_frames, "\n");
        }
        ++cnt;
        /*
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
        else if (cnt >= 0) ++cnt*/
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
