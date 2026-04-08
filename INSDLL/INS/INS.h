#ifndef INS_H
#define INS_H
#include"../BASE/DefData.h"
#include"../MAT/Mat.h"

//纯惯导算法
void InertialNavigation(const std::vector<INS::IMUDataEpoch>& imudata, const INS::IMUResultEpoch& imustart,
	std::vector<INS::IMUResultEpoch>& imuresult);


//惯导递推更新
void Updatek(const double BLH_k_2[3], const double BLH_k_1[3],
	const double Vn_k_2[3], const double Vn_k_1[3],
	const Mat q_bn_k_1,
	const double groy_k_1[3], const double groy_k[3],
	const double dv_k_1[3], const double dv_k[3], double dt,
	double BLHk[3], double Vk_n[3], double YPRk[3], Mat& q_bn_k);
//保存解算结果
void SaveIMUResultEpoch(double time, const double BLH[3], const double Vn[3],
	const double YPR[3], INS::IMUResultEpoch& imuresult);
//计算与参考结果的差值
void CalIMUResultDiff(const std::vector<INS::IMUResultEpoch>& imuresult,
	const std::vector<INS::IMUResultEpoch>& imuref,
	std::vector<INS::IMUResultEpoch>& imudiff);

#endif // INS_H