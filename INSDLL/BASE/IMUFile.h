#ifndef IMUREAD_H
#define IMUREAD_H
#include<fstream>
#include<sstream>
#include<string>
#include<vector>
#include<iomanip>
#include "DefData.h"




//读取IMU数据文件
void IMUFileRead(const char* filename, std::vector<INS::IMUdata>& IMUdata, double sam_freq,double acc_scale,double groy_scale);
//数据输出展示
void DataDisplay(const char* filename, const std::vector<INS::IMUdata>& IMUdata);
//读取六个位置的IMU数据
void Acc_SixPosRead(const char* filename[6], INS::Acc_sixpos& acc_sixpos,double sample,double acc_scale,double groy_scale);
//展示六个位置的IMU数据
void Acc_SixPosDisplay(const char* filename[6], const INS::Acc_sixpos& acc_sixpos);
//读取两个位置的陀螺仪数据
void Groy_TwoPosRead(const char* filename[2], INS::Groy_twopos& groy_twopos, double sam_freq, double acc_scale, double groy_scale);
//展示两个位置的陀螺仪数据
void Groy_TwoPosDisplay(const char* filename[2], const INS::Groy_twopos& groy_twopos);
//读取六位置的陀螺数据
void Groy_SixPosRead(const char* filename[6], INS::Groy_twopos& groy_xpos, INS::Groy_twopos& groy_ypos, INS::Groy_twopos& groy_zpos, double sam_freq, double acc_scale, double groy_scale);
//展示六位置的陀螺数据
void Groy_SixPosDisplay(const char* filename[6], INS::Groy_twopos& groy_xpos, INS::Groy_twopos& groy_ypos, INS::Groy_twopos& groy_zpos);
//输出每秒的姿态角
void Att_angle_display(const char* filename, std::vector<INS::Attitude_angle>& angle);

//读取二进制IMU数据文件
void IMUBinFileRead(const char* filename, std::vector<INS::IMUDataEpoch>& IMUdata);
//将读取到的二进制IMU数据文件输出展示
void IMUBindatadisplay(const char* filename, std::vector<INS::IMUDataEpoch>& IMUdata);
//纯惯导解算结果输出
void IMUResultDisplay(const char* filename, const std::vector<INS::IMUResultEpoch>& imuresult);
//读取纯惯导解算结果
void BinResultRead(const char* BinResultfile, const char* displayfile);
//读取二进制纯惯导解算结果
void INSResultRead(const char* BinResultfile, std::vector<INS::IMUResultEpoch>& IMUresult);

//读取惯导解算参考结果
void INSRefResultRead(const char* Resultfile, std::vector<INS::IMUResultEpoch>& IMUrefresult);

//纯惯导解算结果输出，姿态角顺序为滚转、俯仰、航向
void IMUResultDisplay_RPY(const char* filename, const std::vector<INS::IMUResultEpoch>& imuresult);
#endif // !IMUREAD_H
