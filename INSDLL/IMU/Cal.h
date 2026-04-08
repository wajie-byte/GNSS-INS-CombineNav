#ifndef CAL_H
#define CAL_H



#include"../BASE/DefData.h"

#include"../BASE/IMUFile.h"
#include"../MAT/Mat.h"
#include<cmath>
const static int AccCuttime = 3 * 60 * XW_GI7681_sam_freq;//截取前3*60*freq个数据计算均值

const static int GroyCuttime = 360 / 10 * XW_GI7681_sam_freq;//以10度/秒转速截取前360/10*freq个数据计算均值




//计算单个位置的加速度计均值
void CalMeanAcc(const std::vector<INS::IMUdata>& IMUdata, double onepos[3],double sam_freq);
//计算单个位置的角速率均值
void CalMeanGroy(const std::vector<INS::IMUdata>& IMUdata, double onepos[3], double Cuttime);
//计算六个位置的加速度计均值
void CalMeanAcc_sixpos(const INS::Acc_sixpos& acc_sixpos, double sixpos[6][3],double Cuttime);
//加速度计标定
void Demarcate(const double sixpos[6][3], double acc_bias[3], double acc_scale[3], double acc_m[6]);
//加速度据标定值输出
void DisplayAccCalResult(INS::Acc_sixpos& acc_sixpos, INS::Acc_sixpos& fixed_acc_sixpos, double Cuttime);

//计算单个位置的陀螺仪均值
void CalGroyDeg(const std::vector<INS::IMUdata>& IMUdata, double onepos[3], int Cuttime,double sam_freq);
//获取陀螺采样数量
int GetGroyCuttime(int i, int j);
//计算陀螺两位置法标定结果
void CalGroyResult(const double sixpos[6][3], double bias[3], double scale[3], double xCuttime, double yCuttime, double zCuttime,double freq);
//陀螺标定数据输出
void DisplayGroyCalResult(INS::Groy_twopos& groy_xpos, INS::Groy_twopos& groy_ypos, INS::Groy_twopos& groy_zpos,double sam_freq);
//计算补偿后数据
void Fix_IMUdata(const std::vector<INS::IMUdata>& raw_data, std::vector<INS::IMUdata>& fixed_data, const double acc_bias[3], const double acc_scale[3], const double acc_m[6]);
//求解姿态角
void SPCACal(const std::vector<INS::IMUdata>& SPCA_data, INS::Attitude_angle& angle, double Cuttime);
//补偿六个位置的惯导数据
void Fix_sixposdata(INS::Acc_sixpos& sixpos, INS::Acc_sixpos& sixpos_fixed, const double acc_bias[3], const double acc_scale[3], const double acc_m[3]);
//-------------------加速度计六位置法程序执行---------------------
void Acc_six_func();
//-------------------陀螺两位置法标定三轴数据---------------------
void Groy_twoxyz_func();
//-------------------静态解析粗对准-------------------------------
void SPCA_func();

#endif // !CAL_H