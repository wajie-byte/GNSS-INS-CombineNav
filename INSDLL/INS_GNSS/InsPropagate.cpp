#include"Combinenav.h"
#include"Param.h"
#include"../INS/INS.h"
#include"../INS/Dynamic.h"

void CombineNav::InsPropagate(const CombineNav::NavState& navstate, const INS::IMUDataEpoch& thisimu, double imudt
	,CombineNav::KalmanFilter& kf, double corrtime)
{

	CombineNav::Param param;
	//准备
	double pos[3] = { navstate.pos[0], navstate.pos[1], navstate.pos[2] };
	double vel[3] = { navstate.vel[0], navstate.vel[1], navstate.vel[2] };
	Mat Cbn = navstate.Cbn;
	double Rm = navstate.Rm;
	double Rn = navstate.Rn;
	double gravity = navstate.gravity;

	double e_omega = param.wie;
	double omega_ib[3] = { thisimu.gyro[0]/imudt, thisimu.gyro[1]/imudt, thisimu.gyro[2]/imudt };
	double acce_ib[3] = { thisimu.accl[0]/imudt, thisimu.accl[1]/imudt, thisimu.accl[2]/imudt };

	double w_ie_n[3] = { e_omega * cos(pos[0]), 0.0, -e_omega * sin(pos[0]) };
	double w_en_n[3] = { vel[1] / (Rn + pos[2]), -vel[0] / (Rm + pos[2]), -vel[1] * tan(pos[0]) / (Rn + pos[2]) };
	double w_in_n[3] = { w_ie_n[0] + w_en_n[0], w_ie_n[1] + w_en_n[1], w_ie_n[2] + w_en_n[2] };

	Mat F = Mat::Zero(kf.x_rank, kf.x_rank);
	Mat PHI = Mat::Identity(kf.x_rank);
	Mat G = Mat::Zero(kf.x_rank, kf.noise_rank);

	
	Mat fvr = Mat::Zero(3, 3);
	Mat fvv = Mat::Zero(3, 3);
	Mat fphir = Mat::Zero(3, 3);
	Mat fphiv = Mat::Zero(3, 3);

	// frr 矩阵赋值
	Mat frr = Mat::FromVector({
		-vel[2] / (Rm + pos[2]), 0, vel[0] / (Rm + pos[2]),
		vel[1] * tan(pos[0]) / (Rn + pos[2]), -(vel[2] + vel[0] * tan(pos[0])) / (Rn + pos[2]), vel[1] / (Rn + pos[2]),
		0, 0, 0
		}, 3, 3);

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

	// F矩阵赋值
	//位置，速度，姿态
	F(0, 3, 0, 3) = frr;           // F(1:3,1:3)
	F(3, 6, 0, 3) = fvr;           // F(4:6,1:3)
	F(3, 6, 3, 6) = fvv;           // F(4:6,4:6)
	F(6, 9, 0, 3) = fphir;         // F(7:9,1:3)
	F(6, 9, 3, 6) = fphiv;         // F(7:9,4:6)
	//位置-速度耦合
	F(0, 3, 3, 6) = Mat::Identity(3); // F(1:3,4:6)
	//速度-姿态耦合和传感器误差
	F(3, 6, 6, 9) = Skew(Cbn * Mat::FromVector(std::vector<double>(acce_ib, acce_ib + 3), 3, 1));//F(4:6,7:9) = Skew(Cbn * acce_ib);
	//姿态相关
	F(6, 9, 6, 9) = -Skew(w_in_n); // F(7:9,7:9) = -Skew(w_in_n);
	F(6, 9, 9, 12) = -Cbn;//F(7:9,10:12) = -Cbn;
	//加速度计和陀螺仪误差
	F(3, 6, 12, 15) = Cbn;//F(4:6,13:15) = Cbn;
	F(6, 9, 15, 18) = -Cbn * diag(omega_ib);//F(7:9,16:18)=-Cbn*diag(omega_ib);
	F(3, 6, 18, 21) = Cbn * diag(acce_ib);//F(4:6,19:21)=Cbn*diag(acce_ib);
	//一阶马尔可夫过程（传感器误差的时间相关性）
	F(9, 12, 9, 12) = (-1.0 / corrtime) * Mat::Identity(3);   // F(10:12,10:12)
	F(12, 15, 12, 15) = (-1.0 / corrtime) * Mat::Identity(3); // F(13:15,13:15)
	F(15, 18, 15, 18) = (-1.0 / corrtime) * Mat::Identity(3); // F(16:18,16:18)
	F(18, 21, 18, 21) = (-1.0 / corrtime) * Mat::Identity(3); // F(19:21,19:21)

	// 状态转移矩阵更新
	PHI = PHI + F * imudt;

	// G矩阵赋值（噪声驱动矩阵）
	// 速度误差对应的加速度计噪声
	G(3, 6, 0, 3) = Cbn;                    // G(4:6,1:3) = Cbn
	// 姿态误差对应的陀螺仪噪声
	G(6, 9, 3, 6) = Cbn;                    // G(7:9,4:6) = Cbn
	// 陀螺仪漂移噪声（一阶马尔可夫过程）
	G(9, 12, 6, 9) = Mat::Identity(3);      // G(10:12,7:9) = eye(3)
	// 加速度计偏置噪声
	G(12, 15, 9, 12) = Mat::Identity(3);    // G(13:15,10:12) = eye(3)
	// 陀螺仪尺度因子噪声
	G(15, 18, 12, 15) = Mat::Identity(3);   // G(16:18,13:15) = eye(3)
	// 加速度计尺度因子噪声
	G(18, 21, 15, 18) = Mat::Identity(3);   // G(19:21,16:18) = eye(3)

	Mat Q = G * kf.Qc * G.transpose() * imudt; // 离散化过程噪声协方差矩阵
	kf.P = PHI * kf.P * PHI.transpose() + Q; // 预测协方差矩阵

}