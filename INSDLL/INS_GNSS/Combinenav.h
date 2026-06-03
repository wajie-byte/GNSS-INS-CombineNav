#ifndef COMBINENAV
#define COMBINENAV
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include"../MAT/Mat.h"


namespace INS
{
	struct IMUDataEpoch;
	struct IMUResultEpoch;
}

namespace CombineNav
{
	struct Config;//组合导航配置
	struct Param;//组合导航参数

	struct GNSSResult//GNSS解算结果
	{
		double time;
		double pos[3]; // [deg, deg, m]
		double vel[3]; // [m/s]
		double pos_std[3]; // [m]
		double vel_std[3]; // [m/s]

		GNSSResult()//默认构造函数
		{
			time = 0.0;
			pos[0] = pos[1] = pos[2] = 0.0;
			vel[0] = vel[1] = vel[2] = 0.0;
			pos_std[0] = pos_std[1] = pos_std[2] = 0.0;
			vel_std[0] = vel_std[1] = vel_std[2] = 0.0;
		}

	};


	struct KalmanFilter//卡尔曼滤波器
	{
		int x_rank;//状态向量维度
		int noise_rank;//观测噪声维度
		Mat x;//状态向量
		Mat P;//状态协方差矩阵
		Mat Qc;//连续系统过程噪声协方差矩阵
		KalmanFilter() : x_rank(0), noise_rank(0), x(), P(), Qc() {}
		KalmanFilter(int x_rank_in, int noise_rank_in)
		{
			x_rank = x_rank_in;
			noise_rank = noise_rank_in;
			x = Mat(x_rank, 1);
			P = Mat::Identity(x_rank);
			Qc = Mat::Identity(noise_rank);
		}
		KalmanFilter Initialize(const Config& config);
		
	};


	struct NavState
	{
		double time;
		double pos[3]; //[rad, rad, m]
		double vel[3]; //[m/s]
		double att[3]; //[rad] Roll,Pitch,Heading
		Mat Cbn;//姿态矩阵
		Mat qbn;//姿态四元数
		double gyro_bias[3]; //[rad/s]
		double acc_bias[3]; //[m/s^2]
		double gyro_scale[3]; //[1]
		double acc_scale[3]; //[1]
		double odo_scale;//[1]
		double Rm;//子午圈半径
		double Rn;//卯酉圈半径
		double gravity;//重力加速度
		NavState()
		{
			time = 0.0;
			pos[0] = pos[1] = pos[2] = 0.0;
			vel[0] = vel[1] = vel[2] = 0.0;
			att[0] = att[1] = att[2] = 0.0;
			Cbn = Mat(3, 3);
			qbn = Mat(4, 1);
			for (int i = 0; i < 3; i++)
			{
				gyro_bias[i] = 0.0;
				acc_bias[i] = 0.0;
				gyro_scale[i] = 1.0;
				acc_scale[i] = 1.0;
			}
			odo_scale = 0.0;
			Rm = 0.0;
			Rn = 0.0;
			gravity = 0.0;
		}
		NavState Initialize(const Config& config);

	};




	void gnssins();//组合导航主函数
	std::ifstream CreateReadFile(const std::string filepath);//创建并打开读取文件
	int GetGNSSData(std::ifstream& infile, std::vector<GNSSResult>& gnssdata);//读取GNSS数据
	void GetPosVelData(std::ifstream& infile, std::vector<GNSSResult>& gnssdata);//读取有位置速度的GNSS数据
	void GetPosData(std::ifstream& infile, std::vector<GNSSResult>& gnssdata);//读取只有位置的GNSS数据
	std::vector<GNSSResult> CutGnssDataByTime(double starttime, double endtime, const std::vector<GNSSResult>& gnssdata);
	void GNSSUpdate(CombineNav::KalmanFilter& kf, const CombineNav::NavState& navstate, const CombineNav::Config& config,
		const CombineNav::GNSSResult& thisgnss, const INS::IMUDataEpoch& thisimu, double imudt);
	void Nav_ErrorFeedBack(NavState& navstate, const KalmanFilter& kf);
	void KF_ErrorFeedBack(KalmanFilter& kf);
	void ErrorFeedBack(NavState& navstate, KalmanFilter& kf);
	NavState InsMech(const NavState& laststate, const INS::IMUDataEpoch& lastimu, const INS::IMUDataEpoch& thisimu);//惯导机械更新
	void InsPropagate(const NavState& navstate, const INS::IMUDataEpoch& thisimu, double imudt
		, KalmanFilter& kf, double corrtime);
	void interpolate(const INS::IMUDataEpoch& lastimu, const INS::IMUDataEpoch& thisimu, double intertime,
		INS::IMUDataEpoch& firstimu, INS::IMUDataEpoch& secondimu);
	void Test_InsMech(const std::vector<INS::IMUDataEpoch>& imudata, const INS::IMUResultEpoch& imustart,
		std::vector<INS::IMUResultEpoch>& imuresult);
	// 组合导航相关定义



	//SaveResult
	//保存组合导航结果到文件
	void SaveNavResult(std::ofstream& navfp, const CombineNav::NavState& navstate, const CombineNav::Param& param);
	
	void SaveIMUError(std::ofstream& imuerrfp, const CombineNav::NavState& navstate, const CombineNav::Param& param);

	void SaveStateStd(std::ofstream& stdfp, const CombineNav::KalmanFilter& kf, const CombineNav::NavState& navstate, const CombineNav::Param& param);
}

#endif // !COMBINENAV
