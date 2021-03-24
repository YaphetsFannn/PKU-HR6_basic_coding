#include "NN.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <naoqi_msgs/JointAnglesWithSpeed.h>
#include <dynamixel_msgs/JointState.h>

#define INPUTSIZE_I 14
#define OUTPUTSIZE_I 6
#define NETNUM_I 1

#define INPUTSIZE_F 8
#define OUTPUTSIZE_F 12
#define NETNUM_F 1

#define NETNUM_PIM1 1
#define NETNUM_PIM2 1
#define NETNUM_PIM3 1
#define NETNUM_PIM4 1
#define NETNUM_PFM_saef 8
#define NETNUM_PIM_saef 2
#define NETNUM_PIM_saeb 6

// H - 1, N - 1, T = 9, P = 3, Q = 6
float normRangeI[30][2] = {
-0.2,	0.9,

-1,	0.200000000000000,

-1.0,	1.0,
    -1.0,	1.0,
    -1.0,	0.7,
    -1.0,	1.0,
    -1.0,	1.0,
    -1.0,	0.6,
    -1.0,	1.0,
    -1.0,	1.0,
    -1.0,	0.5,

    -11,	4,
    -13,	9,
    -2,	26,

    -2.09439510239320,	0,
    -1.57079632679490,	0,
    -1.57079632679490,	1.57079632679490,
    0,	1.57079632679490,
    -1.57079632679490,	1.57079632679490,
    -0.349065850398866,	1.57079632679490
};

// H - 1, N - 1, Q = 6, T = 9, P = 3
float normRangeF[30][2] = {
    -0.2,	0.9,

    -1,	0.200000000000000,


    -2.09439510239320,	0,
    -1.57079632679490,	0,
    -1.57079632679490,	1.57079632679490,
    0,	1.57079632679490,
    -1.57079632679490,	1.57079632679490,
    -0.349065850398866,	1.57079632679490,

    -1.0,	1.0,
        -1.0,	1.0,
        -1.0,	0.7,
        -1.0,	1.0,
        -1.0,	1.0,
        -1.0,	0.6,
        -1.0,	1.0,
        -1.0,	1.0,
        -1.0,	0.5,

        -11,	4,
        -13,	9,
        -2,	26



};


float off_set;

float D2R = acos(-1.0);
double del_off[] = {0, 0, 0, 0, 0, 0, 20*D2R, 0, 0, -20*D2R, -40*D2R, 20*D2R, -20*D2R,
                10*D2R, 18*D2R, 0, 0, 5*D2R, -15*D2R, 0, -40*D2R, 0, 0, -0*D2R, 0};
void stateMessageReceive(const dynamixel_msgs::JointState& msg)
{
    float cur_pos = msg.current_pos - del_off[11];
    off_set = 12 * cos(cur_pos) + 12.5 * cos(cur_pos);
            12  - 12.5;
}

void InitNetI(NN *n) {
	string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_I] = { path_prefix + "nnIM.txt"};
       for (int i = 0; i < NETNUM_I; ++i) {
        n[i] = NN(path[i]);
        cout << path[i] << endl;
	}
}

void InitNetF(NN *n) {
    string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_F] = { path_prefix + "nnFM.txt"};
       for (int i = 0; i < NETNUM_F; ++i) {
        n[i] = NN(path[i]);
        cout << path[i] << endl;
    }
}

void InitNetPIM1(NN *n)
{
    string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_PIM1] = {path_prefix + "nnhPIM1.txt"};
    //printf("nnPIM1\n");
    for(int i = 0; i < NETNUM_PIM1; ++i)
    {
        //cout << path[i] << endl;
        n[i] = NN(path[i]);
    }
}

void InitNetPIM2(NN *n)
{
    string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_PIM2] = {path_prefix + "nnhPIM2.txt"};
    for(int i = 0; i < NETNUM_PIM2; ++i)
    {
        n[i] = NN(path[i]);
        //cout << path[i] << endl;
    }
}

void InitNetPIM3(NN *n)
{
    string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_PIM3] = {path_prefix + "nnhPIM3.txt"};
    for(int i = 0; i < NETNUM_PIM3; ++i)
    {
        n[i] = NN(path[i]);
        //cout << path[i] << endl;
    }
}

void InitNetPIM4(NN *n)
{
    string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_PIM4] = {path_prefix + "nnhPIM4.txt"};
    for(int i = 0; i < NETNUM_PIM4; ++i)
    {
        n[i] = NN(path[i]);
        //cout << path[i] << endl;
    }
}

void InitNetPFM_saef(NN *n)
{
    string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_PFM_saef] = {path_prefix + "saeHf.txt", path_prefix + "saeNf.txt", path_prefix + "saeQ1f.txt",
                                   path_prefix + "saeQ2f.txt", path_prefix + "saeQ3f.txt", path_prefix + "saeQ4f.txt", path_prefix + "saeQ5f.txt", path_prefix + "saeQ6f.txt"};
    for(int i = 0; i < NETNUM_PFM_saef; ++i)
    {
        n[i] = NN(path[i]);
        //cout << path[i] << endl;
    }
}

void InitNethPIM_saef(NN *n)
{
    string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_PIM_saef] = {path_prefix + "saeHf.txt", path_prefix + "saeNf.txt"};
    for(int i = 0; i < NETNUM_PIM_saef; ++i)
    {
        n[i] = NN(path[i]);
        //cout << path[i] << endl;
    }
}

void InitNethPIM_saeb(NN *n)
{
    string path_prefix = "/home/pku-hr6/catkin_ws/src/network/";
    string path[NETNUM_PIM_saeb] = {path_prefix + "saeQ1b.txt", path_prefix + "saeQ2b.txt", path_prefix + "saeQ3b.txt",
                                    path_prefix + "saeQ4b.txt", path_prefix + "saeQ5b.txt", path_prefix + "saeQ6b.txt"};
    for(int i = 0; i < NETNUM_PIM_saeb; ++i)
    {
        //printf("%d\n", i);
        n[i] = NN(path[i]);
        //cout << path[i] << endl;
    }
}

vector<float> Forecast(NN *n, vector<float> input, int inputsize, int outputsize,
                       int netnum, float normRange[][2]) {

    vector<float> net_input[8], temp, output;
    /*for (int i = 0; i < inputsize; ++i) {
        input[i] = (input[i] - normRange[i][0]) /
                   (normRange[i][1] - normRange[i][0]);
    }*/
    for(int i = 0; i < netnum; ++i)
    {
        for(int j = 0; j < inputsize; ++j)
            net_input[i].push_back(input[i*inputsize + j]);
    }
    for (int i = 0; i < netnum; ++i) {

        temp = n[i].predict(net_input[i]);
        for (int j = 0; j < temp.size(); ++j)
        {
            output.push_back(temp[j]);
            //printf("%f ", output[i * temp.size() + j]);
        }
        //printf("\n");
        //input.push_back(temp[0]);
    }
    /*for (int i = 0; i < outputsize; ++i) {
        output[i] = normRange[i + inputsize][0] + output[i] *
                   (normRange[i + inputsize][1] - normRange[i + inputsize][0]);
    }*/
    return output;
}

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
float T[4][4], T1[4][4], T2[4][4], T3[4][4], T4[4][4], T5[4][4];
float RsY[4][4], RsX[4][4], ReX[4][4], ReZ[4][4];
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

void Transfer(float (*R)[4], const float *T) {
    memset(R, 0, sizeof(R));
    R[0][0] = R[1][1] = R[2][2] = R[3][3] = 1;
    R[0][3] = T[0], R[1][3] = T[1], R[2][3] = T[2];
}

void hr6_arm_fwd_kinematics(const vector<float>& arm_q) {
    float shoulderOffsetY = -6.4;
    float shoulderOffsetZ = 9.5;
    float shoulderShiftY = -3.5;
    float shoulderShiftZ = -0.8;
    float upperArmLength = 5.7;
    float elbowShiftY = 0.6;
    float elbowShiftZ = -6.3;
    float lowerArmLength = 10;

    float leg1_angle = -arm_q[0];
    float leg2_angle = asin(12 * sin(leg1_angle) / 12.5);
    float del_len = 12 * cos(leg1_angle) + 12.5 * cos(leg2_angle) -
            12  - 12.5;
    shoulderOffsetZ += del_len;
    T1[1][3] = shoulderOffsetY, T1[2][3] = shoulderOffsetZ;
    T2[1][3] = shoulderShiftY, T2[2][3] = shoulderShiftZ;
    T3[1][3] = elbowShiftY, T3[2][3] = -upperArmLength;
    T4[2][3] = elbowShiftZ;
    T5[2][3] = -lowerArmLength;
    RsY[0][0] = cos(arm_q[1]), RsY[0][2] = sin(arm_q[1]);
    RsY[2][0] = -sin(arm_q[1]), RsY[2][2] = cos(arm_q[1]);
    RsX[1][1] = cos(arm_q[2]), RsX[1][2] = -sin(arm_q[2]);
    RsX[2][1] = sin(arm_q[2]), RsX[2][2] = cos(arm_q[2]);
    ReX[1][1] = cos(arm_q[3]), ReX[1][2] = -sin(arm_q[3]);
    ReX[2][1] = sin(arm_q[3]), ReX[2][2] = cos(arm_q[3]);
    ReZ[0][0] = cos(arm_q[4]), ReZ[0][1] = -sin(arm_q[4]);
    ReZ[1][0] = sin(arm_q[4]), ReZ[1][1] = cos(arm_q[4]);

    memset(T, 0, sizeof(T));
    T[0][0] = T[1][1] = T[2][2] = T[3][3] = 1;
    Mul(T, T1); Mul(T, RsY); Mul(T, T2); Mul(T, RsX);
    Mul(T, T3); Mul(T, ReX); Mul(T, T4); Mul(T, ReZ); Mul(T, T5);
}

void Init() {
    memset(T1, 0, sizeof(T1));
    memset(T2, 0, sizeof(T2));
    memset(T3, 0, sizeof(T3));
    memset(T4, 0, sizeof(T4));
    memset(T5, 0, sizeof(T5));
    T1[0][0] = T1[1][1] = T1[2][2] = T1[3][3] = 1;
    T2[0][0] = T2[1][1] = T2[2][2] = T2[3][3] = 1;
    T3[0][0] = T3[1][1] = T3[2][2] = T3[3][3] = 1;
    T4[0][0] = T4[1][1] = T4[2][2] = T4[3][3] = 1;
    T5[0][0] = T5[1][1] = T5[2][2] = T5[3][3] = 1;
    memset(RsY, 0, sizeof(RsY));
    memset(RsX, 0, sizeof(RsX));
    memset(ReZ, 0, sizeof(ReZ));
    memset(ReX, 0, sizeof(ReX));
    RsY[0][0] = RsY[1][1] = RsY[2][2] = RsY[3][3] = 1;
    RsX[0][0] = RsX[1][1] = RsX[2][2] = RsX[3][3] = 1;
    ReZ[0][0] = ReZ[1][1] = ReZ[2][2] = ReZ[3][3] = 1;
    ReX[0][0] = ReX[1][1] = ReX[2][2] = ReX[3][3] = 1;
}

NN nI[NETNUM_I], nI1[NETNUM_PIM1], nI2[NETNUM_PIM2], nI3[NETNUM_PIM3], nI4[NETNUM_PIM4], nI_saef[NETNUM_PIM_saef], nI_saeb[NETNUM_PIM_saeb];
NN nF[NETNUM_F], nF_saef[NETNUM_PFM_saef];
vector<float> input(INPUTSIZE_I, 0);
vector<float> output(OUTPUTSIZE_I, 0);
ros::Publisher pub;
ros::Publisher pub_can_grasp;
std_msgs::Float64 aux;
float Mt[4][4], M6[4][4], M7[4][4];
float R1[4][4], R2[4][4], R3[4][4], R[4][4], S[4][4];

vector<float> calculate_hPIM(vector<float> input)
{
    for (int i = 0; i < 14; ++i) {
        input[i] = (input[i] - normRangeI[i][0]) /
                       (normRangeI[i][1] - normRangeI[i][0]);
    }
    vector<float> nnhPIM_saef_input, nnhPIM_saef_output, nnhPIM_input[4], nnhPIM_output[4], nnhPIM_saeb_input, nnhPIM_saeb_output;
    for(int i = 0; i < 2; ++i)
        nnhPIM_saef_input.push_back(input[i]);
    nnhPIM_saef_output = Forecast(nI_saef, nnhPIM_saef_input, 1, 20, NETNUM_PIM_saef, normRangeF);

    for(int i = 0; i < nnhPIM_saef_output.size(); ++i)
        nnhPIM_input[0].push_back(nnhPIM_saef_output[i]);
    for(int i = 2; i < 14; i++)
        nnhPIM_input[0].push_back(input[i]);
    nnhPIM_output[0] = Forecast(nI1, nnhPIM_input[0], 32, 20, NETNUM_PIM1, normRangeF);

    for(int i = 0; i < nnhPIM_input[0].size(); ++i)
        nnhPIM_input[1].push_back(nnhPIM_input[0][i]);
    for(int i = 0; i < nnhPIM_output[0].size(); ++i)
        nnhPIM_input[1].push_back(nnhPIM_output[0][i]);
    nnhPIM_output[1] = Forecast(nI2, nnhPIM_input[1], 52, 20, NETNUM_PIM2, normRangeF);

    for(int i = 0; i < nnhPIM_input[1].size(); ++i)
        nnhPIM_input[2].push_back(nnhPIM_input[1][i]);
    for(int i = 0; i < nnhPIM_output[1].size(); ++i)
        nnhPIM_input[2].push_back(nnhPIM_output[1][i]);
    nnhPIM_output[2] = Forecast(nI3, nnhPIM_input[2], 72, 20, NETNUM_PIM3, normRangeF);

    for(int i = 0; i < nnhPIM_input[1].size(); ++i)
        nnhPIM_input[3].push_back(nnhPIM_input[1][i]);
    for(int i = 0; i < nnhPIM_output[2].size(); ++i)
        nnhPIM_input[3].push_back(nnhPIM_output[2][i]);
    for(int i = 0; i < nnhPIM_output[1].size(); ++i)
        nnhPIM_input[3].push_back(nnhPIM_output[1][i]);
    nnhPIM_output[3] = Forecast(nI4, nnhPIM_input[3], 92, 20, NETNUM_PIM4, normRangeF);

    for(int i = 0; i < nnhPIM_output[0].size(); ++i)
        nnhPIM_saeb_input.push_back(nnhPIM_output[0][i]);
    for(int i = 0; i < 10; ++i)
        nnhPIM_saeb_input.push_back(nnhPIM_output[3][i]);
    for(int i = 10; i < nnhPIM_output[2].size(); ++i)
        nnhPIM_saeb_input.push_back(nnhPIM_output[2][i]);
    for(int i = 10; i < nnhPIM_output[3].size(); ++i)
        nnhPIM_saeb_input.push_back(nnhPIM_output[3][i]);
    for(int i = 10; i < nnhPIM_output[1].size(); ++i)
        nnhPIM_saeb_input.push_back(nnhPIM_output[1][i]);
    nnhPIM_saeb_output = Forecast(nI_saeb, nnhPIM_saeb_input, 10, 6, NETNUM_PIM_saeb, normRangeF);
    vector<float> output = nnhPIM_saeb_output;
    for (int i = 0; i < 6; ++i) {
            output[i] = normRangeI[i + 14][0] + output[i] *
                       (normRangeI[i + 14][1] - normRangeI[i + 14][0]);
    }
    return output;
}

vector<float> calculate_IM(vector<float> IM_input)
{
    for (int i = 0; i < 14; ++i) {
        IM_input[i] = (IM_input[i] - normRangeI[i][0]) /
                       (normRangeI[i][1] - normRangeI[i][0]);
    }
    vector<float> IM_output;
    //PFM_input = Forecast(nF_saef, input, 1, 80, NETNUM_PFM_saef, normRangeF);
    IM_output = Forecast(nI, IM_input, 14, 6, NETNUM_I, normRangeI);
    vector<float> output = IM_output;
    for (int i = 0; i < 6; ++i) {
            output[i] = normRangeI[i + 14][0] + output[i] *
                       (normRangeI[i + 14][1] - normRangeI[i + 14][0]);
    }
    return output;
}

vector<float> calculate_FM(vector<float> FM_input)
{
    for (int i = 0; i < 8; ++i) {
            FM_input[i] = (FM_input[i] - normRangeF[i][0]) /
                       (normRangeF[i][1] - normRangeF[i][0]);
        }
    vector<float> FM_output;
    //PFM_input = Forecast(nF_saef, input, 1, 80, NETNUM_PFM_saef, normRangeF);
    FM_output = Forecast(nF, FM_input, 8, 12, NETNUM_F, normRangeF);
    vector<float> output = FM_output;
    for (int i = 0; i < 12; ++i) {
            output[i] = normRangeF[i + 8][0] + output[i] *
                       (normRangeF[i + 8][1] - normRangeF[i + 8][0]);
    }
    return output;
}

void poseMessageReceived(const naoqi_msgs::JointAnglesWithSpeed& msg) {

    for (int i = 0; i < INPUTSIZE_I; ++i)
        input[i] = msg.joint_angles[i];
    float cuboid = msg.joint_angles[INPUTSIZE_I];
    int pt = INPUTSIZE_I + 1;
    for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
        Mt[i][j] = msg.joint_angles[pt++];
        M6[i][j] = msg.joint_angles[pt++];
        M7[i][j] = msg.joint_angles[pt++];
    }

    //hPIM
    vector<float> output = calculate_IM(input);

    vector<float> input_tmp;
    input_tmp.push_back(input[0]);
    input_tmp.push_back(input[1]);
    for (int i = 0; i < OUTPUTSIZE_I; ++i)
        input_tmp.push_back(output[i]);

    vector<float> output_tmp = calculate_FM(input_tmp);
    //Init();
    //hr6_arm_fwd_kinematics(output);
    vector<float> tmp;
    float del, del_t = 0, tt;
    for (int i = 0; i < 3; ++i){
        tt = fabs(input[INPUTSIZE_I - 3 + i] - output_tmp[OUTPUTSIZE_F - 3 + i]);
        tt = (i == 1) ? 2.0 / 3 * tt : tt;
        del_t = max(del_t, tt);
        //printf("%f %f ", input[9 + i], T[i][3]);
    }
    //puts("");
    del = del_t;

    float final_input[12];
    for (int i = 0; i < 0; ++i) {
        float phi1 = ((float)rand() / RAND_MAX - 0.5) * 40.0 * acos(-1.0) / 180;
        float phi2 = ((float)rand() / RAND_MAX - 0.5) * 40.0 * acos(-1.0) / 180;
        float phi3 = ((float)rand() / RAND_MAX - 0.5) * 40.0 * acos(-1.0) / 180;

        RotateX(R1, phi1);
        RotateY(R2, phi2);
        RotateZ(R3, phi3);
        memset(R, 0, sizeof(R));
        memset(S, 0, sizeof(S));
        R[0][0] = R[1][1] = R[2][2] = R[3][3] = 1;
        S[0][0] = S[1][1] = S[2][2] = S[3][3] = 1;
        Mul(R, R1); Mul(R, R2); Mul(R, R3); Mul(R, M7);
        //Mul(S, Mt);
        Mul(S, M6); Mul(S, R);
        /*for (int j = 0; j < 3; ++j) {
            float len = sqrt(S[0][j]*S[0][j] + S[1][j]*S[1][j] + S[2][j]*S[2][j]);
            for (int k = 0; k < 3; ++k) {
                S[k][j] /= len;
            }
        }*/

        //for (int i = 0; i < 3; ++i)
            //printf("#%f\n", S[i][3]);
        //if (cuboid == 3) S[2][3] += 3.5;
        for (int k = 0; k < 4; ++k)
        for (int j = 0; j < 3; ++j)
            input[k*3 + j + 2] = S[j][k];
        if (input[INPUTSIZE_I - 1] < 12) {
            //input[11] = 7.5;
            //S[2][3] = 7.5;
        }
        //tmp = Forecast(nI, input, INPUTSIZE_I, OUTPUTSIZE_I, NETNUM_I, normRangeI);
        for (int i = 0; i < OUTPUTSIZE_I; ++i)
            input_tmp[i + 2] = tmp[i];
        //hr6_arm_fwd_kinematics(tmp);
        //vector<float> output_tmp = Forecast(nF, input_tmp, INPUTSIZE_F, OUTPUTSIZE_F, NETNUM_F, normRangeF);
        del_t = 0;

        for (int j = 0; j < 3; ++j){
            tt = fabs(S[j][3] - output_tmp[OUTPUTSIZE_F - 3 + j]);
            tt = (j == 1) ? 2.0 / 3.0 * tt : tt;
            del_t = max(del_t, tt);
        }
        if (del_t < del) {
            del = del_t;
            for (int j = 0; j < OUTPUTSIZE_I; ++j) {
                output[j] = tmp[j];
            }
            for(int i = 0; i < 12; i++)
                final_input[i] = input[i];
            for (int j = 0; j < 3; ++j) {
                //printf("%f %f ", S[j][3], output_tmp[OUTPUTSIZE_F - 3 + j]);
            }
            //puts("");
            for (int j = 0; j < 4; ++j) {
                //printf("%f ", output[j]);
            }
            //puts("");
        }
    }
    /*puts("Num2");
    puts("");
    puts("Num3");*/
    if (del > 2.0) {
        puts("I can't grasp this object, sorry!");
        puts("");
        puts("");
        puts("");
        puts("");
        aux.data = 0;
        pub_can_grasp.publish(aux);
        return ;
    }


    naoqi_msgs::JointAnglesWithSpeed msg_pub;
    for (int i = 0; i < OUTPUTSIZE_I; ++i) {
        //printf("%f ", output[i]);
        msg_pub.joint_angles.push_back(output[i]);
    }
    //puts("");
    //for(int i = 0; i < 12; ++i)
        //printf("%f ", final_input[i]);
    //puts("");
    pub.publish(msg_pub);
}
void Test(float arr[]) {
    for (int i = 0; i < INPUTSIZE_I; ++i)
        input[i] = arr[i];
    printf("sdsad\n");
    //vector<float> output = Forecast(nI, input, INPUTSIZE_I, OUTPUTSIZE_I, NETNUM_I, normRangeI);
    for (int i = 0; i < OUTPUTSIZE_I; ++i) {
        printf("%f ", output[i]);
    }
    printf("\n");
    vector<float> input_tmp;
    input_tmp.push_back(input[0]);
    input_tmp.push_back(input[1]);
    for (int i = 0; i < OUTPUTSIZE_I; ++i)
        input_tmp.push_back(output[i]);
    vector<float> output_tmp = Forecast(nF, input_tmp, INPUTSIZE_F, OUTPUTSIZE_F, NETNUM_F, normRangeF);
    for (int i = 0; i < OUTPUTSIZE_F; ++i)
        printf("%f ", output_tmp[i]);
    printf("\n");
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "NN"); 
  	ros::NodeHandle nh;
    pub = nh.advertise<naoqi_msgs::JointAnglesWithSpeed>("/joint_angles", 1);
    ros::Subscriber sub = nh.subscribe("/target_position", 1000, poseMessageReceived);
    //ros::Subscriber sub_state = nh.subscribe("/RHipPitch_controller/state", 1000, stateMessageReceive);
    pub_can_grasp = nh.advertise<std_msgs::Float64>("/rarm_can_grasp_state", 1);
    srand((unsigned int)(time(NULL)));
    ros::Rate rate(62.5);
  	
    float arr[] = {0.720971,	-0.669838,   -0.0767480000000000,	0.620431000000000,	-0.780497000000000,	0.783326000000000,	-0.446764000000000,	-0.432166000000000,	-0.616373000000000,	-0.648220000000000,	-0.447096000000000,	-4.22687900000000,	-1.02900100000000,	17.8470400000000};
    float arr1[] = {0.720971,	-0.669838, -1.948156,	-0.879482000000000,	0.572686000000000,	1.22207100000000,	0.588026000000000,	0.547120000000000};
	//string path[NETNUM] = {"network/net1.txt", "network/net2.txt","network/net3.txt","network/net4.txt","network/net5.txt"};

//    InitNetPIM1(nI1);
//    InitNetPIM2(nI2);
//    InitNetPIM3(nI3);
//    InitNetPIM4(nI4);
//    InitNethPIM_saeb(nI_saeb);
//    InitNethPIM_saef(nI_saef);
//    InitNetPFM_saef(nF_saef);
    InitNetI(nI);
    InitNetF(nF);

    vector<float> input_I, input_F;
    for(int i = 0; i < 14; i++)
        input_I.push_back(arr[i]);
    for(int i = 0; i < 8; i++)
        input_F.push_back(arr1[i]);
    vector<float> output_I = calculate_IM(input_I);
    for(int i = 0; i < output_I.size(); ++i)
        printf("%f ", output_I[i]);
    printf("\n");
   vector<float> output_F = calculate_FM(input_F);
    for(int i = 0; i < output_F.size(); ++i)
        printf("%f ", output_F[i]);
    printf("\n");
    //Test(arr);
    //vector<float> input(arr, arr + INPUTSIZE);
//    for (int i = 0; i < INPUTSIZE_I; ++i)
//        input[i] = arr[i];
//    vector<float> output = Forecast(nI, input, INPUTSIZE_I, OUTPUTSIZE_I, NETNUM_I, normRangeI);
//    for (int i = 0; i < OUTPUTSIZE_I; ++i) {
//        printf("%f ", output[i]);
//    }
//    printf("\n");
    /*
    output.clear();
    output.push_back(-0.3);
    output.push_back(-1);
    output.push_back(1);
    output.push_back(0);
    Init();
    hr6_arm_fwd_kinematics(output);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j)
            printf("%f ", T[i][j]);
        puts("");
    }
    puts("");*/
    //output[0] = -output[0];
    //output[1] = -output[1];
    //output[2] -= 30*acos(-1.0)/180.0;
    //output[3] = -output[3];
    //naoqi_msgs::JointAnglesWithSpeed msg;

    //pub.publish(msg);
    //for (int i = 0; i < 1000; ++i)
    //    rate.sleep();
    //return 0;
    //while (ros::ok()) {
    //	naoqi_msgs::JointAnglesWithSpeed msg;
        //for (int i = 0; i < OUTPUTSIZE; ++i)
         //   msg.joint_angles.push_back(output[i]);
        //msg.joint_angles.push_back(0.0);
        //pub.publish(msg);
    //    for (int i = 0; i < 24; ++i) {
    //        msg.joint_angles.push_back(0);
     //   }
    //    pub.publish(msg);
     //   ros::spinOnce();
     //   rate.sleep();
    //}
    ros::spin();
	return 0;
}

/*double arr1[] = {0.635403915548292,	0.0215764989617515,	0.552712002688412,	0.877753385132496,	0.267663595725672, 0.841309147341502,
					0.318617270199198,	0.443414547634351,	0.815420514967012,	0.397252920811124,	0.485522297940158,	0.171201681871593};
	double arr2[] = {0.213347675588196,	0.100233463628146,	0.559355088579767,	0.836783017769484,	0.218107349329537,	0.522213904979651, 
					0.396926141240341,	0.746308055131944,	0.825764182551928,	0.806279763868134,	0.649191412959803,	0.608859564556705,	
					0.909127503320045,	0.752845460505482,	0.454445939485204,	0.174327653513266,	0.301765934513645,	0.365428416074495,	
					0.870817926791636,	0.110936740990704,	0.233376246888487,	0.479869708543117,	0.906828443593048,	0.334275837875669,
					0.524353938520579,	0.483995664809766,	0.134735195767691,	0.797518764557281,	0.345854045640157,	0.0693404477602320,
					0.103395030341172,	0.752334052566672,	0.190749473869133,	0.987137132249797,	0.220872405026003,	0.170185023493028,
					0.802589720274748,	0.310048344272662,	0.764484890261959,	0.681935121477982,	0.592531178241465,	0.0235612695692768,
					0.0633561671084731,	0.261794079421206,	0.960848263143612,	0.872337816299400,	0.667679243654669,	0.200996811438303,
					0.0659611094631574,	0.254559895947513};
	double arr3[] = {0.213347675588196,0.100233463628146,0.559355088579767,0.836783017769484,0.218107349329537,0.522213904979651,0.396926141240341,0.746308055131944,0.825764182551928,0.806279763868134,0.649191412959803,0.608859564556705,0.909127503320045,0.752845460505482,0.454445939485204,0.174327653513266,0.301765934513645,0.365428416074495,0.870817926791636,0.110936740990704,0.233376246888487,0.479869708543117,0.906828443593048,0.334275837875669,0.524353938520579,0.483995664809766,0.134735195767691,0.797518764557281,0.345854045640157,0.0693404477602320,0.103395030341172,0.752334052566672,0.190749473869133,0.987137132249797,0.220872405026003,0.170185023493028,0.802589720274748,0.310048344272662,0.764484890261959,0.681935121477982,0.592531178241465,0.0235612695692768,0.0633561671084731,0.261794079421206,0.960848263143612,0.872337816299400,0.667679243654669,0.200996811438303,0.0659611094631574,0.254559895947513,0.689556375142169};
	double arr4[] = {0.213347675588196,0.100233463628146,0.559355088579767,0.836783017769484,0.218107349329537,0.522213904979651,0.396926141240341,0.746308055131944,0.825764182551928,0.806279763868134,0.649191412959803,0.608859564556705,0.909127503320045,0.752845460505482,0.454445939485204,0.174327653513266,0.301765934513645,0.365428416074495,0.870817926791636,0.110936740990704,0.233376246888487,0.479869708543117,0.906828443593048,0.334275837875669,0.524353938520579,0.483995664809766,0.134735195767691,0.797518764557281,0.345854045640157,0.0693404477602320,0.103395030341172,0.752334052566672,0.190749473869133,0.987137132249797,0.220872405026003,0.170185023493028,0.802589720274748,0.310048344272662,0.764484890261959,0.681935121477982,0.592531178241465,0.0235612695692768,0.0633561671084731,0.261794079421206,0.960848263143612,0.872337816299400,0.667679243654669,0.200996811438303,0.0659611094631574,0.254559895947513,0.689556375142169,0.654081857593641};
	double arr5[] = {0.213347675588196,0.100233463628146,0.559355088579767,0.836783017769484,0.218107349329537,0.522213904979651,0.396926141240341,0.746308055131944,0.825764182551928,0.806279763868134,0.649191412959803,0.608859564556705,0.909127503320045,0.752845460505482,0.454445939485204,0.174327653513266,0.301765934513645,0.365428416074495,0.870817926791636,0.110936740990704,0.233376246888487,0.479869708543117,0.906828443593048,0.334275837875669,0.524353938520579,0.483995664809766,0.134735195767691,0.797518764557281,0.345854045640157,0.0693404477602320,0.103395030341172,0.752334052566672,0.190749473869133,0.987137132249797,0.220872405026003,0.170185023493028,0.802589720274748,0.310048344272662,0.764484890261959,0.681935121477982,0.592531178241465,0.0235612695692768,0.0633561671084731,0.261794079421206,0.960848263143612,0.872337816299400,0.667679243654669,0.200996811438303,0.0659611094631574,0.254559895947513,0.689556375142169,0.654081857593641,0.250675021636674};
	*/
