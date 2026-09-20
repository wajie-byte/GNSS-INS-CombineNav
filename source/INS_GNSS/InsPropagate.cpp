#include"Combinenav.h"
#include"Param.h"
#include"../INS/INS.h"
#include"../INS/Dynamic.h"

void CombineNav::InsPropagate(const NavState& navstate, const INS::IMUDataEpoch& thisimu,
	double imudt, KalmanFilter& kf, double corrtime,
	Eigen::MatrixXd& PHI_out)
{

	CombineNav::Param param;
	//准备
	double pos[3] = { navstate.pos[0], navstate.pos[1], navstate.pos[2] };
	double vel[3] = { navstate.vel[0], navstate.vel[1], navstate.vel[2] };
	Eigen::Matrix3d Cbn = navstate.Cbn;;
	double Rm = navstate.Rm;
	double Rn = navstate.Rn;
	double gravity = navstate.gravity;

	double e_omega = param.wie;
	double omega_ib[3] = { thisimu.gyro[0]/imudt, thisimu.gyro[1]/imudt, thisimu.gyro[2]/imudt };
	double acce_ib[3] = { thisimu.accl[0]/imudt, thisimu.accl[1]/imudt, thisimu.accl[2]/imudt };

	double w_ie_n[3] = { e_omega * cos(pos[0]), 0.0, -e_omega * sin(pos[0]) };
	double w_en_n[3] = { vel[1] / (Rn + pos[2]), -vel[0] / (Rm + pos[2]), -vel[1] * tan(pos[0]) / (Rn + pos[2]) };
	double w_in_n[3] = { w_ie_n[0] + w_en_n[0], w_ie_n[1] + w_en_n[1], w_ie_n[2] + w_en_n[2] };

	Eigen::MatrixXd F = Eigen::MatrixXd::Zero(kf.x_rank, kf.x_rank);
	Eigen::MatrixXd PHI = Eigen::MatrixXd::Identity(kf.x_rank, kf.x_rank);
	Eigen::MatrixXd G = Eigen::MatrixXd::Zero(kf.x_rank, kf.noise_rank);


	Eigen::Matrix3d fvr = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d fvv = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d fphir = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d fphiv = Eigen::Matrix3d::Zero();

	// frr 矩阵赋值
	// 更好的选择：固定大小矩阵（性能更好）
	Eigen::Matrix3d frr;  
	frr << -vel[2] / (Rm + pos[2]), 0, vel[0] / (Rm + pos[2]),
		vel[1] * tan(pos[0]) / (Rn + pos[2]), -(vel[2] + vel[0] * tan(pos[0])) / (Rn + pos[2]), vel[1] / (Rn + pos[2]),
		0, 0, 0;

	// fvr 矩阵赋值
	fvr(0, 0) = -2 * vel[1] * e_omega * cos(pos[0]) / (Rm + pos[2]) - vel[1] * vel[1] / ((Rn + pos[2]) * (Rm + pos[2]) * pow(cos(pos[0]), 2));
	fvr(0, 2) = vel[0] * vel[2] / pow(Rm + pos[2], 2) - vel[1] * vel[1] * tan(pos[0]) / pow(Rn + pos[2], 2);
	fvr(1, 0) = 2 * e_omega * (vel[0] * cos(pos[0]) - vel[2] * sin(pos[0])) / (Rm + pos[2]) + vel[0] * vel[1] / ((Rn + pos[2]) * (Rm + pos[2]) * pow(cos(pos[0]), 2));
	fvr(1, 2) = vel[1] * vel[2] / pow(Rn + pos[2], 2) + vel[0] * vel[1] * tan(pos[0]) / pow(Rn + pos[2], 2);
	fvr(2, 0) = 2 * vel[1] * e_omega * sin(pos[0]) / (Rm + pos[2]);
	fvr(2, 2) = -vel[1] * vel[1] / pow(Rn + pos[2], 2) - vel[0] * vel[0] / pow(Rm + pos[2], 2) + 2 * gravity / (sqrt(Rm * Rn) + pos[2]);

	// fvv 矩阵赋值
	fvv(0, 0) = vel[2] / (Rm + pos[2]);
	fvv(0, 1) = -2 * e_omega * sin(pos[0]) - 2 * vel[1] * tan(pos[0]) / (Rn + pos[2]);
	fvv(0, 2) = vel[0] / (Rm + pos[2]);
	fvv(1, 0) = 2 * e_omega * sin(pos[0]) + vel[1] * tan(pos[0]) / (Rn + pos[2]);
	fvv(1, 1) = (vel[2] + vel[0] * tan(pos[0])) / (Rn + pos[2]);
	fvv(1, 2) = 2 * e_omega * cos(pos[0]) + vel[1] / (Rn + pos[2]);
	fvv(2, 0) = -2 * vel[0] / (Rm + pos[2]);
	fvv(2, 1) = -2 * e_omega * cos(pos[0]) - 2 * vel[1] / (Rn + pos[2]);

	// fphir 矩阵赋值
	fphir(0, 0) = -e_omega * sin(pos[0]) / (Rm + pos[2]);
	fphir(0, 2) = vel[1] / pow(Rn + pos[2], 2);
	fphir(1, 2) = -vel[0] / pow(Rm + pos[2], 2);
	fphir(2, 0) = -e_omega * cos(pos[0]) / (Rm + pos[2]) - vel[1] / ((Rn + pos[2]) * (Rm + pos[2]) * pow(cos(pos[0]), 2));
	fphir(2, 2) = -vel[1] * tan(pos[0]) / pow(Rn + pos[2], 2);

	// fphiv 矩阵赋值
	fphiv(0, 1) = 1 / (Rn + pos[2]);
	fphiv(1, 0) = -1 / (Rm + pos[2]);
	fphiv(2, 1) = -tan(pos[0]) / (Rn + pos[2]);

	
	//	F矩阵赋值
	// 使用块赋值
	// 位置相关 (行0-2, 列0-2)
	F.block<3, 3>(0, 0) = frr;

	// 速度相关 (行3-5)
	F.block<3, 3>(3, 0) = fvr;      // 列0-2
	F.block<3, 3>(3, 3) = fvv;      // 列3-5

	// 姿态相关 (行6-8)
	F.block<3, 3>(6, 0) = fphir;    // 列0-2
	F.block<3, 3>(6, 3) = fphiv;    // 列3-5

	// 位置-速度耦合
	F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

	// 速度-姿态耦合
	Eigen::Vector3d acce_ib_eigen(acce_ib[0], acce_ib[1], acce_ib[2]);

	// 现在可以正常计算
	Eigen::Vector3d temp = Cbn * acce_ib_eigen;
	F.block<3, 3>(3, 6) = Skew(temp);
	

	// 姿态相关
	F.block<3, 3>(6, 6) = -Skew(w_in_n);
	F.block<3, 3>(6, 9) = -Cbn;

	// 加速度计和陀螺仪误差
	F.block<3, 3>(3, 12) = Cbn;
	F.block<3, 3>(6, 15) = -Cbn * diag(omega_ib);
	F.block<3, 3>(3, 18) = Cbn * diag(acce_ib);

	// 一阶马尔可夫过程
	double beta = -1.0 / corrtime;
	F.block<3, 3>(9, 9) = beta * Eigen::Matrix3d::Identity();
	F.block<3, 3>(12, 12) = beta * Eigen::Matrix3d::Identity();
	F.block<3, 3>(15, 15) = beta * Eigen::Matrix3d::Identity();
	F.block<3, 3>(18, 18) = beta * Eigen::Matrix3d::Identity();

	// 状态转移矩阵更新
	PHI = PHI + F * imudt;

	// G矩阵赋值（噪声驱动矩阵）
// 速度误差对应的加速度计噪声
	G.block<3, 3>(3, 0) = Cbn;                    // G(4:6,1:3) = Cbn
	// 姿态误差对应的陀螺仪噪声
	G.block<3, 3>(6, 3) = Cbn;                    // G(7:9,4:6) = Cbn
	// 陀螺仪漂移噪声（一阶马尔可夫过程）
	G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();      // G(10:12,7:9) = eye(3)
	// 加速度计偏置噪声
	G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();    // G(13:15,10:12) = eye(3)
	// 陀螺仪尺度因子噪声
	G.block<3, 3>(15, 12) = Eigen::Matrix3d::Identity();   // G(16:18,13:15) = eye(3)
	// 加速度计尺度因子噪声
	G.block<3, 3>(18, 15) = Eigen::Matrix3d::Identity();   // G(19:21,16:18) = eye(3)

	Eigen::MatrixXd Q = G * kf.Qc * G.transpose() * imudt; // 离散化过程噪声协方差矩阵
	kf.P = PHI * kf.P * PHI.transpose() + Q; // 预测协方差矩阵

	PHI_out = PHI; // 输出状态转移矩阵
}