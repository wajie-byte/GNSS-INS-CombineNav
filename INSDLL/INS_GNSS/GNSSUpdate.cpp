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

CombineNav::KalmanFilter CombineNav::GNSSUpdate(const CombineNav::KalmanFilter& kf, const CombineNav::NavState& navstate, const CombineNav::Config& config,
	const CombineNav::GNSSResult& thisgnss, const INS::IMUDataEpoch& thisimu, double imudt)
{
	CombineNav::Param param;
	CombineNav::KalmanFilter kf_updated = kf;
	double gnssposstd[3] = { thisgnss.pos_std[0], thisgnss.pos_std[1], thisgnss.pos_std[2] };
	if(gnssposstd[0]>5.0||gnssposstd[1]>5.0||gnssposstd[2]>5.0)
	{
		std::cout << "Warning: GNSS position standard deviation is too large, skipping this update at" << thisgnss.time << "s.\n";
		return kf_updated;
	}
	//measurement innovation
	Mat DR = Mat::Zero(3, 3);
	DR(0, 0) = navstate.Rm + navstate.pos[2];
	DR(1, 1) = (navstate.Rn + navstate.pos[2]) * cos(navstate.pos[0]);
	DR(2, 2) = -1.0;
	//ins position in n frame
	Mat inspos = Mat::Zero(3, 1);
	inspos(0, 0) = navstate.pos[0];
	inspos(1, 0) = navstate.pos[1];
	inspos(2, 0) = navstate.pos[2];
	//gnss position in n frame
	Mat gnsspos = Mat::Zero(3, 1);
	gnsspos(0, 0) = thisgnss.pos[0];
	gnsspos(1, 0) = thisgnss.pos[1];
	gnsspos(2, 0) = thisgnss.pos[2];
	//antlever in n frame
	Mat antlever = Mat::Zero(3, 1);
	antlever(0, 0) = config.installation.gnss_lever[0];
	antlever(1, 0) = config.installation.gnss_lever[1];
	antlever(2, 0) = config.installation.gnss_lever[2];
	//N系下NED
	Mat Z = Mat::Zero(3, 1);
	Z = DR * (inspos - gnsspos) + navstate.Cbn * antlever;

	//measurement matrix and noise matrix
	Mat R = Mat::Zero(3, 3);
	R(0, 0) = gnssposstd[0] * gnssposstd[0];
	R(1, 1) = gnssposstd[1] * gnssposstd[1];
	R(2, 2) = gnssposstd[2] * gnssposstd[2];
	Mat H = Mat::Zero(3, kf.x_rank);
	H(0, 0) = 1.0; H(1, 1) = 1.0; H(2, 2) = 1.0;//位置状态对位置观测的直接关系
	Mat skew_Cbn_antlever = Skew(navstate.Cbn * antlever);
	H(0, 6) = skew_Cbn_antlever(0, 0); H(0, 7) = skew_Cbn_antlever(0, 1); H(0, 8) = skew_Cbn_antlever(0, 2);//姿态误差对位置观测的影响
	// update covariance and state vector
	Mat K = kf.P * H.transpose() * (H * kf.P * H.transpose() + R).inverse();
	kf_updated.x = kf.x + K * (Z - H * kf.x);
	kf_updated.P = (Mat::Identity(kf.x_rank) - K * H) * kf.P * (Mat::Identity(kf.x_rank) - K * H).transpose() + K * R * K.transpose();

	//check if use gnss velocity update
	if (config.switches.usegnssvel)
	{
		double gnssvelstd[3] = { thisgnss.vel_std[0], thisgnss.vel_std[1], thisgnss.vel_std[2] };
		if (gnssvelstd[0] > 0.5 || gnssvelstd[1] > 0.5 || gnssvelstd[2] > 0.5)
		{
			std::cout << "Warning: GNSS velocity standard deviation is too large, skipping velocity update at" << thisgnss.time << "s.\n";
			return kf_updated;
		}
		Mat Cbn = navstate.Cbn;
		//ins velocity in n frame
		Mat insvel = Mat::Zero(3, 1);
		insvel(0, 0) = navstate.vel[0];
		insvel(1, 0) = navstate.vel[1];
		insvel(2, 0) = navstate.vel[2];
		//gnss velocity in n frame
		Mat gnssvel = Mat::Zero(3, 1);
		gnssvel(0, 0) = thisgnss.vel[0];
		gnssvel(1, 0) = thisgnss.vel[1];
		gnssvel(2, 0) = thisgnss.vel[2];
		//RM,RN
		double RM = navstate.Rm;
		double RN = navstate.Rn;
		
		Mat w_ib_b = Mat::Zero(3, 1);
		w_ib_b(0, 0) = thisimu.gyro[0] / imudt;
		w_ib_b(1, 0) = thisimu.gyro[1] / imudt;
		w_ib_b(2, 0) = thisimu.gyro[2] / imudt;
		Mat w_en_n = Mat::Zero(3, 1);
		w_en_n(0, 0) = insvel(1, 0) / (RN + inspos(2, 0));
		w_en_n(1, 0) = -insvel(0, 0) / (RM + inspos(2, 0));
		w_en_n(2, 0) = -insvel(1, 0) * tan(inspos(0, 0)) / (RN + inspos(2, 0));
		Mat w_ie_n = Mat::Zero(3, 1);
		w_ie_n(0, 0) = param.wie * cos(inspos(0, 0));
		w_ie_n(1, 0) = 0.0;
		w_ie_n(2, 0) = -param.wie * sin(inspos(0, 0));
		Mat w_in_n = Mat::Zero(3, 1);
		w_in_n = w_ie_n + w_en_n;
		//INS推算的GNSS速度观测值（补偿天线杆臂引起的速度误差）
		Mat ins2gnss_vel = Mat::Zero(3, 1);
		ins2gnss_vel = insvel - (Skew(w_en_n) + Skew(w_ie_n)) * Cbn * antlever - Cbn * Skew(antlever) * w_ib_b;
		Mat Z_vel = Mat::Zero(3, 1);
		Z_vel = ins2gnss_vel - gnssvel;
		Mat R_vel = Mat::Zero(3, 3);
		R_vel(0, 0) = gnssvelstd[0] * gnssvelstd[0];
		R_vel(1, 1) = gnssvelstd[1] * gnssvelstd[1];
		R_vel(2, 2) = gnssvelstd[2] * gnssvelstd[2];
		Mat H_vel = Mat::Zero(3, kf.x_rank);
		H_vel(0, 3) = 1.0; H_vel(1, 4) = 1.0; H_vel(2, 5) = 1.0;//速度状态对速度观测的直接关系
		Mat H_vG_3 = -Skew(w_in_n) * Skew(Cbn * antlever) - Skew(Cbn * (Skew(antlever) * w_ib_b));
		Mat H_vG_4 = -Cbn * Skew(antlever);
		Mat diag_wibb = Mat::Zero(3, 3);
		diag_wibb(0, 0) = w_ib_b(0, 0);
		diag_wibb(1, 1) = w_ib_b(1, 0);
		diag_wibb(2, 2) = w_ib_b(2, 0);
		Mat H_vG_6 = -Cbn * Skew(antlever) * diag_wibb;
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
		Mat K_vel = kf_updated.P * H_vel.transpose() * (H_vel * kf_updated.P * H_vel.transpose() + R_vel).inverse();
		kf_updated.x = kf_updated.x + K_vel * (Z_vel - H_vel * kf_updated.x);
		kf_updated.P = (Mat::Identity(kf_updated.x_rank) - K_vel * H_vel) * kf_updated.P * (Mat::Identity(kf_updated.x_rank) - K_vel * H_vel).transpose() + K_vel * R_vel * K_vel.transpose();

	}
	//End
	
	return kf_updated;
}


