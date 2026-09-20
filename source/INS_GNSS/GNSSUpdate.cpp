#include"Combinenav.h"
#include"CombConfig.h"
#include"../INS/INS.h"
#include"../INS/Dynamic.h"
//GNSS观测更新
/*
In:
	kf: 卡尔曼滤波器当前状态
	navstate: 当前导航状态
	config: 组合导航配置
	thisgnss: 当前GNSS解算结果
	thisimu: 当前IMU数据
	imudt: IMU数据时间间隔
Out:
	kf_updated: 更新后的卡尔曼滤波器状态
*/

void CombineNav::GNSSUpdate(CombineNav::KalmanFilter& kf, const CombineNav::NavState& navstate, const CombineNav::Config& config,
	const CombineNav::GNSSResult& thisgnss, const INS::IMUDataEpoch& thisimu, double imudt)
{
	CombineNav::Param param;
	
	double gnssposstd[3] = { thisgnss.pos_std[0], thisgnss.pos_std[1], thisgnss.pos_std[2] };
	double Rfactor_pos = 1.0;
	if(gnssposstd[0]>10.0||gnssposstd[1]>10.0||gnssposstd[2]>20.0)
	{
		std::cout << "Warning: GNSS position standard deviation is too large, skipping this update at" << thisgnss.time << "s.\n";
		return;
	}
	else if (gnssposstd[0] > 5.0 || gnssposstd[1] > 5.0 || gnssposstd[2] > 5.0)
	{
		double max_pos_std = std::max({ gnssposstd[0], gnssposstd[1], gnssposstd[2]/2.0 });
		Rfactor_pos = (max_pos_std / 0.5) * (max_pos_std / 0.5);
	}

	

	//measurement innovation
	Eigen::Matrix3d DR = Eigen::Matrix3d::Zero();
	DR(0, 0) = navstate.Rm + navstate.pos[2];
	DR(1, 1) = (navstate.Rn + navstate.pos[2]) * cos(navstate.pos[0]);
	DR(2, 2) = -1.0;
	//ins position in n frame
	Eigen::Vector3d inspos = Eigen::Vector3d::Zero();
	inspos(0, 0) = navstate.pos[0];
	inspos(1, 0) = navstate.pos[1];
	inspos(2, 0) = navstate.pos[2];
	//gnss position in n frame
	Eigen::Vector3d gnsspos = Eigen::Vector3d::Zero();
	gnsspos(0, 0) = thisgnss.pos[0];
	gnsspos(1, 0) = thisgnss.pos[1];
	gnsspos(2, 0) = thisgnss.pos[2];
	//antlever in n frame
	Eigen::Vector3d antlever = Eigen::Vector3d::Zero();
	antlever(0, 0) = config.installation.gnss_lever[0];
	antlever(1, 0) = config.installation.gnss_lever[1];
	antlever(2, 0) = config.installation.gnss_lever[2];
	//N系下NED
	Eigen::Vector3d Z = Eigen::Vector3d::Zero();
	Z = DR * (inspos - gnsspos) + navstate.Cbn * antlever;

	//measurement matrix and noise matrix
	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	R(0, 0) = gnssposstd[0] * gnssposstd[0] * Rfactor_pos;
	R(1, 1) = gnssposstd[1] * gnssposstd[1] * Rfactor_pos;
	R(2, 2) = gnssposstd[2] * gnssposstd[2] * Rfactor_pos * 2;
	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, kf.x_rank);
	H(0, 0) = 1.0; H(1, 1) = 1.0; H(2, 2) = 1.0;//位置状态对位置观测的直接关系
	Eigen::Matrix3d skew_Cbn_antlever = Skew(navstate.Cbn * antlever);
	H.block<3, 3>(0, 6) = skew_Cbn_antlever;
	// update covariance and state vector
	Eigen::MatrixXd K = kf.P * H.transpose() * (H * kf.P * H.transpose() + R).inverse();
	kf.x = kf.x + K * (Z - H * kf.x);
	kf.P = (Eigen::MatrixXd::Identity(kf.x_rank, kf.x_rank) - K * H) * kf.P * (Eigen::MatrixXd::Identity(kf.x_rank, kf.x_rank) - K * H).transpose() + K * R * K.transpose();

	//check if use gnss velocity update
	if (config.switches.usegnssvel)
	{
		double gnssvelstd[3] = { thisgnss.vel_std[0], thisgnss.vel_std[1], thisgnss.vel_std[2] };
		double Rfactor = 1.0;
		if (gnssvelstd[0] > 5.0 || gnssvelstd[1] > 5.0 || gnssvelstd[2] > 5.0)
		{
			std::cout << "Warning: GNSS velocity standard deviation is too large, skipping velocity update at" << thisgnss.time << "s.\n";
			return;
		}
		else if (gnssvelstd[0] > 0.5 || gnssvelstd[1] > 0.5 || gnssvelstd[2] > 0.5)
		{
			// 动态计算R_vel：在原噪声基础上，额外增加一个非线性放大因子
			double max_vel_std = std::max({ gnssvelstd[0], gnssvelstd[1], gnssvelstd[2]/2.0 });
			// 例如：如果速度标准差为2.0m/s，则R矩阵放大 (2.0/0.5)^2 = 16倍
			Rfactor = (max_vel_std / 0.5) * (max_vel_std / 0.5);
		}
		Eigen::MatrixXd Cbn = navstate.Cbn;
		//ins velocity in n frame
		Eigen::Vector3d insvel = Eigen::Vector3d::Zero();
		insvel(0, 0) = navstate.vel[0];
		insvel(1, 0) = navstate.vel[1];
		insvel(2, 0) = navstate.vel[2];
		//gnss velocity in n frame
		Eigen::Vector3d gnssvel = Eigen::Vector3d::Zero();
		gnssvel(0, 0) = thisgnss.vel[0];
		gnssvel(1, 0) = thisgnss.vel[1];
		gnssvel(2, 0) = thisgnss.vel[2];
		//RM,RN
		double RM = navstate.Rm;
		double RN = navstate.Rn;
		
		Eigen::Vector3d w_ib_b = Eigen::Vector3d::Zero();
		w_ib_b(0, 0) = thisimu.gyro[0] / imudt;
		w_ib_b(1, 0) = thisimu.gyro[1] / imudt;
		w_ib_b(2, 0) = thisimu.gyro[2] / imudt;
		Eigen::Vector3d w_en_n = Eigen::Vector3d::Zero();
		w_en_n(0, 0) = insvel(1, 0) / (RN + inspos(2, 0));
		w_en_n(1, 0) = -insvel(0, 0) / (RM + inspos(2, 0));
		w_en_n(2, 0) = -insvel(1, 0) * tan(inspos(0, 0)) / (RN + inspos(2, 0));
		Eigen::Vector3d w_ie_n = Eigen::Vector3d::Zero();
		w_ie_n(0, 0) = param.wie * cos(inspos(0, 0));
		w_ie_n(1, 0) = 0.0;
		w_ie_n(2, 0) = -param.wie * sin(inspos(0, 0));
		Eigen::Vector3d w_in_n = Eigen::Vector3d::Zero();
		w_in_n = w_ie_n + w_en_n;
		//INS推算的GNSS速度观测值（补偿天线杆臂引起的速度误差）
		Eigen::Vector3d ins2gnss_vel = Eigen::Vector3d::Zero();
		ins2gnss_vel = insvel - (Skew(w_en_n) + Skew(w_ie_n)) * Cbn * antlever - Cbn * Skew(antlever) * w_ib_b;
		Eigen::Vector3d Z_vel = Eigen::Vector3d::Zero();
		Z_vel = ins2gnss_vel - gnssvel;
		Eigen::Matrix3d R_vel = Eigen::Matrix3d::Zero();
		R_vel(0, 0) = gnssvelstd[0] * gnssvelstd[0] * Rfactor;
		R_vel(1, 1) = gnssvelstd[1] * gnssvelstd[1] * Rfactor;
		R_vel(2, 2) = gnssvelstd[2] * gnssvelstd[2] * Rfactor * 2;
		Eigen::MatrixXd H_vel = Eigen::MatrixXd::Zero(3, kf.x_rank);
		H_vel(0, 3) = 1.0; H_vel(1, 4) = 1.0; H_vel(2, 5) = 1.0;//速度状态对速度观测的直接关系
		Eigen::Matrix3d H_vG_3 = -Skew(w_in_n) * Skew(Cbn * antlever) - Skew(Cbn * (Skew(antlever) * w_ib_b));
		Eigen::Matrix3d H_vG_4 = -Cbn * Skew(antlever);
		Eigen::Matrix3d diag_wibb = Eigen::Matrix3d::Zero();
		diag_wibb(0, 0) = w_ib_b(0);
		diag_wibb(1, 1) = w_ib_b(1);
		diag_wibb(2, 2) = w_ib_b(2);
		Eigen::Matrix3d H_vG_6 = -Cbn * Skew(antlever) * diag_wibb;
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				H_vel(i, 6+j) = H_vG_3(i, j);
				H_vel(i, 9+j) = H_vG_4(i, j);
				H_vel(i, 15+j) = H_vG_6(i, j);
			}
		}

		// velocity update
		Eigen::MatrixXd K_vel = kf.P * H_vel.transpose() * (H_vel * kf.P * H_vel.transpose() + R_vel).inverse();
		kf.x = kf.x + K_vel * (Z_vel - H_vel * kf.x);
		kf.P = (Eigen::MatrixXd::Identity(kf.x_rank,kf.x_rank) - K_vel * H_vel) * kf.P * (Eigen::MatrixXd::Identity(kf.x_rank,kf.x_rank) - K_vel * H_vel).transpose() + K_vel * R_vel * K_vel.transpose();

	}
	//End

	return;
}









// ================== ODO/NHC 更新函数 ==================
// In:
//   kf: 卡尔曼滤波器当前状态
//   navstate: 当前导航状态
//   config: 组合导航配置
//   thisimu: 当前IMU数据
//   imudt: IMU数据时间间隔
// Out:
//   kf_updated: 更新后的卡尔曼滤波器状态
void CombineNav::NHCUpdate(CombineNav::KalmanFilter& kf, const CombineNav::NavState& navstate,
	const CombineNav::Config& config, const INS::IMUDataEpoch& thisimu, double imudt)
{
	CombineNav::Param param;
	Eigen::Matrix3d Cbv = Eigen::Matrix3d::Identity();
	// 1. 提取变量 
	Eigen::Matrix3d Cbn = navstate.Cbn;
	Eigen::Vector3d vel = { navstate.vel[0], navstate.vel[1], navstate.vel[2] };
	Eigen::Vector3d pos = { navstate.pos[0], navstate.pos[1], navstate.pos[2] };
	double RM = navstate.Rm;
	double RN = navstate.Rn;
	Eigen::Vector3d antlever = { config.installation.odo_lever[0], config.installation.odo_lever[1], config.installation.odo_lever[2] }; // 里程计杆臂

	// 2. 计算角速度
	// w_ib_b = thisimu(2:4) / dt; 
	Eigen::Vector3d w_ib_b = Eigen::Vector3d::Zero();
	w_ib_b(0) = thisimu.gyro[0] / imudt;
	w_ib_b(1) = thisimu.gyro[1] / imudt;
	w_ib_b(2) = thisimu.gyro[2] / imudt;

	// w_en_n 计算
	Eigen::Vector3d w_en_n = Eigen::Vector3d::Zero();
	w_en_n(0) = vel(1) / (RN + pos(2));
	w_en_n(1) = -vel(0) / (RM + pos(2));
	w_en_n(2) = -vel(1) * tan(pos(0)) / (RN + pos(2));

	// w_ie_n 计算
	Eigen::Vector3d w_ie_n = Eigen::Vector3d::Zero();
	w_ie_n(0) = param.wie * cos(pos(0));
	w_ie_n(1) = 0.0;
	w_ie_n(2) = -param.wie * sin(pos(0));

	// w_in_n = w_ie_n + w_en_n
	Eigen::Vector3d w_in_n = w_ie_n + w_en_n;

	// w_nb_b = w_ib_b - Cbn' * w_in_n (MATLAB: Cbn' * w_in_n)
	Eigen::Vector3d w_nb_b = w_ib_b - Cbn.transpose() * w_in_n;

	// 3. 计算INS预测的车体速度（带杆臂补偿）
	// ins_body_vel = Cbv * (Cbn' * vel + skew(w_nb_b) * antlever)
	Eigen::Vector3d ins_body_vel = Cbv * (Cbn.transpose() * vel + Skew(w_nb_b) * antlever);

	// 4. 运动检测：只有车辆在运动时才执行 NHC
	// forward_speed = abs(ins_body_vel(1)); (MATLAB索引1 -> C++索引0)
	double forward_speed = std::abs(ins_body_vel(0));
	if (forward_speed < 0.5) // 阈值 0.5 m/s，低于此值认为静止
	{
		// 静止时跳过 NHC（避免错误约束）
		return;
	}

	
	// Z = ins_body_vel(2:3);  (MATLAB索引2,3 -> C++索引1,2)
	Eigen::Vector2d Z = Eigen::Vector2d::Zero();
	Z(0) = ins_body_vel(1); // 侧向速度 (y轴)
	Z(1) = ins_body_vel(2); // 垂向速度 (z轴)

	// H = zeros(2, kf.RANK);
	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, kf.x_rank);

	// 只取横向和垂向的测量矩阵
	// H(1:2, 4:6) = Cbv(2:3, :) * Cbn'; (MATLAB索引2,3 -> C++索引1,2)
	H.block<2, 3>(0, 3) = Cbv.block<2, 3>(1, 0) * Cbn.transpose();

	// H(1:2, 7:9) = -Cbv(2:3, :) * Cbn' * skew(vel);
	H.block<2, 3>(0, 6) = -Cbv.block<2, 3>(1, 0) * Cbn.transpose() * Skew(vel);

	// H(1:2, 10:12) = -Cbv(2:3, :) * skew(antlever);
	H.block<2, 3>(0, 9) = -Cbv.block<2, 3>(1, 0) * Skew(antlever);

	// 如果状态量包含比例因子 (如果 kf.RANK >= 18)
	if (kf.x_rank >= 18)
	{
		// H(1:2, 16:18) = -Cbv(2:3, :) * skew(antlever) * diag(w_ib_b);
		Eigen::Matrix3d diag_wibb = Eigen::Matrix3d::Zero();
		diag_wibb(0, 0) = w_ib_b(0);
		diag_wibb(1, 1) = w_ib_b(1);
		diag_wibb(2, 2) = w_ib_b(2);
		H.block<2, 3>(0, 15) = -Cbv.block<2, 3>(1, 0) * Skew(antlever) * diag_wibb;
	}

	// 纯NHC噪声
	// R = diag([cfg.odonhc_measnoise(2)^2; cfg.odonhc_measnoise(3)^2]);
	Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
	// 假设 config 中有 odonhc_measnoise 数组，这里用默认值 0.05 代替，您可改为 config 读取
	double std_y = 0.05; // config.odonhc_measnoise[1]; 
	double std_z = 0.05; // config.odonhc_measnoise[2];
	R(0, 0) = std_y * std_y;
	R(1, 1) = std_z * std_z;

	// 6. 卡尔曼滤波更新 (同您之前的风格)
	// K = kf.P * H' / (H * kf.P * H' + R);
	Eigen::MatrixXd K = kf.P * H.transpose() * (H * kf.P * H.transpose() + R).inverse();

	// kf.x = kf.x + K * (Z - H * kf.x);
	kf.x = kf.x + K * (Z - H * kf.x);

	// kf.P = (eye(kf.RANK) - K * H) * kf.P * (eye(kf.RANK) - K * H)' + K * R * K';
	kf.P = (Eigen::MatrixXd::Identity(kf.x_rank, kf.x_rank) - K * H) * kf.P
		* (Eigen::MatrixXd::Identity(kf.x_rank, kf.x_rank) - K * H).transpose()
		+ K * R * K.transpose();
	
		
	
	
}



bool CombineNav::detectStatic(const INS::IMUDataEpoch& imu, double GRAVITY,double ACC_THRESHOLD,double GYRO_THRESHOLD,double imudt)
{
	double acc_norm = sqrt(imu.accl[0] * imu.accl[0] +
		imu.accl[1] * imu.accl[1] +
		imu.accl[2] * imu.accl[2])/imudt;

	double gyro_norm = sqrt(imu.gyro[0] * imu.gyro[0] +
		imu.gyro[1] * imu.gyro[1] +
		imu.gyro[2] * imu.gyro[2])/imudt;

	return (fabs(acc_norm - GRAVITY) < ACC_THRESHOLD) && (gyro_norm < GYRO_THRESHOLD);
}

void CombineNav::ZUPTUpdate(KalmanFilter& kf, NavState& navstate, const Config& config)
{
	// 速度零约束观测
	Eigen::Vector3d ins_vel(navstate.vel[0], navstate.vel[1], navstate.vel[2]);
	Eigen::Vector3d Z = ins_vel;  // 零速约束：观测 = 当前速度（期望为0）

	// H矩阵：观测速度误差
	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, kf.x_rank);
	H(0, 3) = 1.0;
	H(1, 4) = 1.0;
	H(2, 5) = 1.0;

	// R矩阵
	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	double noise = config.obs_noise.zupt_std;
	R(0, 0) = noise * noise;
	R(1, 1) = noise * noise;
	R(2, 2) = noise * noise;

	Eigen::MatrixXd K = kf.P * H.transpose() * (H * kf.P * H.transpose() + R).inverse();

	// kf.x = kf.x + K * (Z - H * kf.x);
	kf.x = kf.x + K * (Z - H * kf.x);

	// kf.P = (eye(kf.RANK) - K * H) * kf.P * (eye(kf.RANK) - K * H)' + K * R * K';
	kf.P = (Eigen::MatrixXd::Identity(kf.x_rank, kf.x_rank) - K * H) * kf.P
		* (Eigen::MatrixXd::Identity(kf.x_rank, kf.x_rank) - K * H).transpose()
		+ K * R * K.transpose();
}