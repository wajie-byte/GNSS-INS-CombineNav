#ifndef DYNAMIC_H
#define DYNAMIC_H
#include"../MAT/Mat.h"
#include<vector>

namespace INS
{
	struct IMUDataEpoch;
}

//计算GRS80地球椭球模型的正常重力公式
double Cal_g_GRS80(double lat, double h);
//计算姿态矩阵C_b^n
Mat Cal_C_b_n(double Yaw, double Pitch, double Roll);
//四元数转姿态矩阵
Mat quaternion2C_b_n(Mat q_b_n);
//姿态角转四元数
Mat euler2quaternion(double Yaw, double Pitch, double Roll);
//四元数转姿态角
void quaternion2euler(Mat q_b_n, double& Yaw, double& Pitch, double& Roll);
//计算叉乘c=axb
void cross_3_1(const double a[3], const double b[3], double c[3]);//注意向量叉乘的顺序为axb
//根据k-1时刻和k时刻角增量计算b系相对i系转动的等效旋转矢量
void Cal_ERV_ib(const double groy_k_1[3], const double groy_k[3], double ERV[3]);
//根据等效旋转矢量计算b系姿态变化的四元数
Mat Cal_pos_rotate(const double ERV[3]);
//根据当地纬度计算积分区间地球自转角速度在n系下的投影
void Cal_w_ie_n(double lat, double w_ie_n[3]);
//根据当地纬度位移角速度在n系下的投影
void Cal_w_en_n(double vE, double vN, double lat, double h, double w_en_n[3]);
//根据当地纬度计算子午圈半径
double Cal_RM(double lat);
//根据当地纬度计算卯酉圈半径
double Cal_RN(double lat);
//积分区间[tk-1,tk]中间时刻tk-1/2
//根据地球自转角速度和位移角速度计算n系相对i系转动的等效旋转矢量
void Cal_ERV_in(const double w_ie_n_mid[3], const double w_en_n_mid[3], double dt, double ERV[3]);
//根据等效旋转矢量计算n系姿态变化的四元数
Mat Cal_neg_rotate(const double ERV[3]);
//四元数乘法
Mat quaternion_multiply(Mat a, Mat b);
//四元数链乘实现姿态更新
Mat Attupdate_quaternion(Mat q_n_rotate, Mat q_bn_k_1, Mat q_b_rotate);
//四元数归一化
Mat quaternion_normalize(Mat a);
//计算重力加速度在n系下的投影
void Cal_g_p_n(double lat, double h, double g_p_n[3]);
//计算重力哥氏积分项
void Cal_dv_gc_n(const double g_p_n_mid[3], const double w_ie_n_mid[3], const double w_en_n_mid[3],
	const double v_n_mid[3], double dt, double dv_gc_n[3]);
//计算b系比力积分项
void Cal_dv_f_b(const double groy_k_1[3], const double groy_k[3], const double dv_k_1[3],
	const double dv_k[3], double dv_f_b[3]);
//计算比力积分项
void Cal_dv_f_n(const double ERV[3], const Mat C_bn_k_1, const double dv_f_b[3],
	double dv_f_n[3]);
//计算反对称矩阵
Mat Cal_Antisymmmetric_Mat(const double a[3]);
//计算反对称矩阵
Mat Skew(const Mat& a);
Mat Skew(const double a[3]);
//n系速度更新
void Cal_v_k_n(const double v_k_1_n[3], const double dv_f_n[3], const double dv_gc_n[3],
	double v_k_n[3]);
//高程更新
double Cal_h_k(double h_k_1, double vD_k_1, double vD_k, double dt);
//纬度更新
double Cal_lat_k(double lat_k_1, double vN_k_1, double vN_k, double h_mid, double dt);
//经度更新
double Cal_lon_k(double lon_k_1, double vE_k_1, double vE_k, double lat_mid, double h_mid, double dt);

//寻找初始数据索引
bool FindInitialIndex(const std::vector<INS::IMUDataEpoch>& imudata, double starttime, int& index_start, double tolerance = 1e-4);

//n系到e系转换四元数
Mat Cal_qne(const double BLH[3]);
//零速修正
bool isZeroVel(const double dv[3], const double dtheta[3], double lat, double h, double dt,
	 std::vector<double>& history_dv, std::vector<double>& history_dtheta);
//根据向量构造对角矩阵
Mat diag(const double a[3]);




#endif // !DYNAMIC_H
