#ifndef COMBINENAV
#define COMBINENAV
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include<deque>
#include"../ThirdParty/eigen-3.4.0/Eigen/Dense"


namespace INS
{
	struct IMUDataEpoch;
	struct IMUResultEpoch;
}

namespace CombineNav
{

	#define MAXRAWLEN 40960



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
		Eigen::MatrixXd x;//状态向量
		Eigen::MatrixXd P;//状态协方差矩阵
		Eigen::MatrixXd Qc;//连续系统过程噪声协方差矩阵
		KalmanFilter() : x_rank(0), noise_rank(0), x(), P(), Qc() {}
		KalmanFilter(int x_rank_in, int noise_rank_in)
		{
			x_rank = x_rank_in;
			noise_rank = noise_rank_in;
			x = Eigen::MatrixXd::Zero(x_rank, 1);
			P = Eigen::MatrixXd::Identity(x_rank, x_rank);
			Qc = Eigen::MatrixXd::Identity(noise_rank, noise_rank);
		}
		KalmanFilter Initialize(const Config& config);
		
	};


	struct NavState
	{
		double time;
		double pos[3]; //[rad, rad, m]
		double vel[3]; //[m/s]
		double att[3]; //[rad] Roll,Pitch,Heading
		Eigen::Matrix3d Cbn;//姿态矩阵
		Eigen::Vector4d qbn;//姿态四元数
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
			Cbn = Eigen::MatrixXd::Zero(3, 3);
			qbn = Eigen::MatrixXd::Zero(4, 1);
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

	struct FilterEpoch 
	{
		double time;                          // 当前历元时间
		NavState nav_before_correction;       // 开环导航状态（机械编排后，尚未进行闭环校正）
		Eigen::VectorXd x_pred;               // 预测误差状态 (21x1)
		Eigen::MatrixXd P_pred;               // 预测协方差 (21x21)
		Eigen::VectorXd x_upd;                // 更新误差状态 (21x1)
		Eigen::MatrixXd P_upd;                // 更新协方差 (21x21)
		Eigen::MatrixXd PHI;                  // 从上个历史历元到当前历元的状态转移矩阵
		// 以下为平滑结果（反向计算后填充）
		Eigen::VectorXd x_smooth;
		Eigen::MatrixXd P_smooth;
		FilterEpoch() : time(0.0),x_pred(21), P_pred(21, 21), x_upd(21), P_upd(21, 21),
			PHI(21, 21), x_smooth(21), P_smooth(21, 21) 
		{
			x_pred.setZero(); P_pred.setZero(); x_upd.setZero(); P_upd.setZero();
			PHI.setIdentity(); x_smooth.setZero(); P_smooth.setZero();
		}
	};




	struct SmoothRecord 
	{
		double time;
		CombineNav::NavState navstate;        // 闭环修正后的导航状态
		Eigen::MatrixXd P;           // P矩阵
		Eigen::MatrixXd Phi;    // Phi矩阵
		SmoothRecord() : time(0.0), P(21, 21), Phi(21, 21) 
		{
			
			P.setZero();
			Phi.setIdentity();
		}
	};

	// ============================================================
// ZUPT 零速检测器 - 完整结构体
// 放在 Combinenav.h 的 class CombineNav 内部
// ============================================================

	struct ZUPTDetector {
		// ========== 可调参数 ==========
		static const int WINDOW_SIZE = 30;        // 检测窗口大小（点数）
		// @200Hz: 30点 = 0.15秒
		// @100Hz: 15点 = 0.15秒

		static constexpr double STATIC_RATIO = 0.8;  // 静止判定比例（80%）
		// 窗口内静止点数 >= WINDOW_SIZE * 0.8
		

		static const int EXIT_THRESHOLD = 5;      // 退出静止的运动点数
		// 连续5个运动点 → 退出静止
		// @200Hz: 5点 = 0.025秒
		// 快速响应运动开始

// ========== 内部状态 ==========
		std::deque<bool> history;     // 滑动窗口历史
		int static_count = 0;         // 窗口内静止点数
		int moving_count = 0;         // 连续运动点数
		bool currently_static = false; // 当前是否判定为零速

		// ========== 核心函数 ==========
		// 输入: 当前IMU点是否静止 (true/false)
		// 输出: 是否处于零速状态
		// 调用频率: 每个IMU点调用一次
		bool update(bool is_static_now) 
		{
			// 1. 加入新数据
			history.push_back(is_static_now);
			if (history.size() > WINDOW_SIZE) 
			{
				if (history.front()) static_count--;
				history.pop_front();
			}

			// 2. 更新计数
			if (is_static_now)
			{
				static_count++;
				moving_count = 0;           // 重置连续运动计数
			}
			else 
			{
				moving_count++;
			}

			// 3. 进入静止条件
			//    窗口内静止点数 >= 阈值（默认24/30）
			if (!currently_static && history.size() >= WINDOW_SIZE
				&& static_count >= WINDOW_SIZE * STATIC_RATIO)
			{
				currently_static = true;
				// 可选: 在这里添加日志
				// std::cout << "[ZUPT] 检测到静止 (静态点: " 
				//           << static_count << "/" << WINDOW_SIZE << ")" << std::endl;
			}

			// 4. 退出静止条件
			//    连续运动点数 >= 阈值（默认5个）
			if (currently_static && moving_count >= EXIT_THRESHOLD) 
			{
				currently_static = false;
				static_count = 0;
				moving_count = 0;
				// 可选: 在这里添加日志
				// std::cout << "[ZUPT] 退出静止 (连续运动: " 
				//           << EXIT_THRESHOLD << "点)" << std::endl;
			}

			return currently_static;
		}

		// 获取当前状态（不更新）
		bool isStatic() const 
		{
			return currently_static;
		}

		// 获取窗口内静止比例
		double getStaticRatio() const 
		{
			if (history.empty()) return 0.0;
			return (double)static_count / history.size();
		}

		// 强制重置（GNSS更新后调用，或重新初始化时）
		void reset() 
		{
			history.clear();
			static_count = 0;
			moving_count = 0;
			currently_static = false;
		}

		// 获取窗口是否已满（用于判断检测器是否准备好）
		bool isReady() const 
		{
			return history.size() >= WINDOW_SIZE;
		}
	};



	void gnssins(Config config);//组合导航主函数
	void gnssins_realtime(Config config);//组合导航实时处理主函数
	



	// 反向滤波
	void gnssins_backward();
	void RTS_Smoother(const std::vector<SmoothRecord>& smooth_records,const Config& config,const Eigen::MatrixXd Qc,
		std::vector<NavState>& smoothed_states);
	void gnssins_backwardfilter(
		const NavState& forward_final_state,           // 正向滤波最终状态
		const KalmanFilter& forward_final_kf,          // 正向滤波最终卡尔曼滤波器
		const std::vector<INS::IMUDataEpoch>& imudata_proc,        // 正向用的IMU数据
		const std::vector<GNSSResult>& gnssdata_proc,  // 正向用的GNSS数据
		const Config& config,                          // 配置
		const Param& param,                            // 参数
		std::vector<NavState>& navstate_history_back   // 输出：反向滤波结果
	);
	void ApplyErrorCorrection(NavState& nav,const Eigen::VectorXd& dx);
	void ApplySmoothErrorFeedback(NavState& navstate, const Eigen::VectorXd& x_smooth);
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
	void InsPropagate(const NavState& navstate, const INS::IMUDataEpoch& thisimu,
		double imudt, KalmanFilter& kf, double corrtime,
		Eigen::MatrixXd& PHI_out);
	void interpolate(const INS::IMUDataEpoch& lastimu, const INS::IMUDataEpoch& thisimu, double intertime,
		INS::IMUDataEpoch& firstimu, INS::IMUDataEpoch& secondimu);
	void interpolate_backward(const INS::IMUDataEpoch& lastimu, const INS::IMUDataEpoch& thisimu, double intertime,
		INS::IMUDataEpoch& firstimu, INS::IMUDataEpoch& secondimu);
	void Test_InsMech(const std::vector<INS::IMUDataEpoch>& imudata, const INS::IMUResultEpoch& imustart,
		std::vector<INS::IMUResultEpoch>& imuresult);
	void NHCUpdate(KalmanFilter& kf, const NavState& navstate,
		const Config& config, const INS::IMUDataEpoch& thisimu, double imudt);

	bool detectStatic(const INS::IMUDataEpoch& imu, double GRAVITY, double ACC_THRESHOLD, double GYRO_THRESHOLD,double imudt);
	void ZUPTUpdate(KalmanFilter& kf, NavState& navstate, const Config& config);
	// 组合导航相关定义



	//SaveResult
	//保存组合导航结果到文件
	void SaveNavResult(std::ofstream& navfp, const CombineNav::NavState& navstate, const CombineNav::Param& param);
	
	void SaveIMUError(std::ofstream& imuerrfp, const CombineNav::NavState& navstate, const CombineNav::Param& param);

	void SaveStateStd(std::ofstream& stdfp, const CombineNav::KalmanFilter& kf, const CombineNav::NavState& navstate, const CombineNav::Param& param);
}

#endif // !COMBINENAV
