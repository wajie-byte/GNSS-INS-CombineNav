#include"Combinenav.h"
#include"../INS/Dynamic.h"
//误差反馈，将卡尔曼滤波器的状态估计误差反馈到导航状态中
/*
In:
	navstate: 当前导航状态
	kf: 卡尔曼滤波器当前状态
Out:
	navstate_new: 反馈误差后的导航状态
*/
void CombineNav::Nav_ErrorFeedBack(CombineNav::NavState& navstate, const CombineNav::KalmanFilter& kf)
{
	
	Eigen::Matrix3d DR = Eigen::Matrix3d::Zero();
	DR(0, 0) = navstate.Rm + navstate.pos[2];
	DR(1, 1) = (navstate.Rn + navstate.pos[2]) * cos(navstate.pos[0]);
	DR(2, 2) = -1.0;
	Eigen::Matrix3d DR_inv = DR.inverse();
	Eigen::Vector3d dx_pos = Eigen::Vector3d::Zero();
	dx_pos(0) = kf.x(0, 0);
	dx_pos(1) = kf.x(1, 0);
	dx_pos(2) = kf.x(2, 0);
	Eigen::Vector3d DR_pos = DR_inv * dx_pos;
	for (int i = 0; i < 3; i++)
	{
		//position
		navstate.pos[i] = navstate.pos[i] - DR_pos(i);
		//velocity
		navstate.vel[i] = navstate.vel[i] - kf.x(3 + i, 0);
		//gyro_bias,acc_bias,gyro_scale,acc_scale
		navstate.gyro_bias[i] = navstate.gyro_bias[i] + kf.x(9 + i, 0);
		navstate.acc_bias[i] = navstate.acc_bias[i] + kf.x(12 + i, 0);
		navstate.gyro_scale[i] = navstate.gyro_scale[i] + kf.x(15 + i, 0);
		navstate.acc_scale[i] = navstate.acc_scale[i] + kf.x(18 + i, 0);

	}
	//attitude
	double erv[3] = { kf.x(6,0),kf.x(7,0),kf.x(8,0) };
	Eigen::Vector4d qpn = Cal_pos_rotate(erv);
	navstate.qbn = quaternion_multiply(qpn, navstate.qbn);
	quaternion2euler(navstate.qbn, navstate.att[2], navstate.att[1], navstate.att[0]);
	//RM,RN,gravity
	navstate.Rm = Cal_RM(navstate.pos[0]);
	navstate.Rn = Cal_RN(navstate.pos[0]);
	navstate.gravity = Cal_g_GRS80(navstate.pos[0], navstate.pos[2]);
	
}

//将卡尔曼滤波器的状态估计误差反馈后，重置卡尔曼滤波器的状态向量
/*
In:
	kf: 卡尔曼滤波器当前状态
Out:
	kf_new: 反馈误差后重置状态向量的卡尔曼滤波器
*/
void CombineNav::KF_ErrorFeedBack(CombineNav::KalmanFilter& kf)
{
	//reset state vector
	kf.x = Eigen::VectorXd::Zero(kf.x_rank);
}

//将卡尔曼滤波器的状态估计误差反馈到导航状态中，并重置卡尔曼滤波器的状态向量
/*
@param navstate: 当前导航状态
@param kf: 卡尔曼滤波器当前状态
*/
void CombineNav::ErrorFeedBack(CombineNav::NavState& navstate, CombineNav::KalmanFilter& kf)
{
	Nav_ErrorFeedBack(navstate, kf);
	KF_ErrorFeedBack(kf);
}


// 应用平滑误差状态的反馈函数
void CombineNav::ApplySmoothErrorFeedback(NavState& navstate, const Eigen::VectorXd& x_smooth)
{
	if (x_smooth.norm() < 1e-10) return;  // 如果误差状态为零，不需要反馈

	// 位置误差反馈
	Eigen::Matrix3d DR = Eigen::Matrix3d::Zero();
	DR(0, 0) = navstate.Rm + navstate.pos[2];
	DR(1, 1) = (navstate.Rn + navstate.pos[2]) * cos(navstate.pos[0]);
	DR(2, 2) = -1.0;
	Eigen::Matrix3d DR_inv = DR.inverse();
	Eigen::Vector3d dx_pos = x_smooth.segment<3>(0);
	Eigen::Vector3d DR_pos = DR_inv * dx_pos;

	for (int i = 0; i < 3; i++)
	{
		// 位置修正
		navstate.pos[i] = navstate.pos[i] - DR_pos(i);
		// 速度修正
		navstate.vel[i] = navstate.vel[i] - x_smooth(3 + i);
	}

	// 姿态修正
	double erv[3] = { x_smooth(6), x_smooth(7), x_smooth(8) };
	Eigen::Vector4d qpn = Cal_pos_rotate(erv);
	navstate.qbn = quaternion_multiply(qpn, navstate.qbn);
	quaternion2euler(navstate.qbn, navstate.att[2], navstate.att[1], navstate.att[0]);

	// 更新曲率半径和重力
	navstate.Rm = Cal_RM(navstate.pos[0]);
	navstate.Rn = Cal_RN(navstate.pos[0]);
	navstate.gravity = Cal_g_GRS80(navstate.pos[0], navstate.pos[2]);
}


void CombineNav::ApplyErrorCorrection(
	CombineNav::NavState& nav,
	const Eigen::VectorXd& dx)
{
	// ===== 1. 位置修正 =====
	// 注意：和 ErrorFeedBack 一样，位置误差需要从弧长转换
	Eigen::Matrix3d DR = Eigen::Matrix3d::Zero();
	DR(0, 0) = nav.Rm + nav.pos[2];
	DR(1, 1) = (nav.Rn + nav.pos[2]) * cos(nav.pos[0]);
	DR(2, 2) = -1.0;
	Eigen::Matrix3d DR_inv = DR.inverse();

	Eigen::Vector3d dx_pos(dx(0), dx(1), dx(2));
	Eigen::Vector3d DR_pos = DR_inv * dx_pos;

	nav.pos[0] -= DR_pos(0);
	nav.pos[1] -= DR_pos(1);
	nav.pos[2] -= DR_pos(2);

	// ===== 2. 速度修正 =====
	nav.vel[0] -= dx(3);
	nav.vel[1] -= dx(4);
	nav.vel[2] -= dx(5);

	// ===== 3. 姿态修正 =====
	double erv[3] = { dx(6), dx(7), dx(8) };
	Eigen::Vector4d qpn = Cal_pos_rotate(erv);
	nav.qbn = quaternion_multiply(qpn, nav.qbn);
	quaternion2euler(nav.qbn, nav.att[2], nav.att[1], nav.att[0]);

	// ===== 4. 陀螺零偏修正 =====
	// ErrorFeedBack 中是 +=，说明零偏误差定义是"估计值 - 真值"
	// 所以这里也用 +=
	nav.gyro_bias[0] += dx(9);
	nav.gyro_bias[1] += dx(10);
	nav.gyro_bias[2] += dx(11);

	// ===== 5. 加计零偏修正 =====
	nav.acc_bias[0] += dx(12);
	nav.acc_bias[1] += dx(13);
	nav.acc_bias[2] += dx(14);

	// ===== 6. 陀螺比例因子修正 =====
	nav.gyro_scale[0] += dx(15);
	nav.gyro_scale[1] += dx(16);
	nav.gyro_scale[2] += dx(17);

	// ===== 7. 加计比例因子修正 =====
	nav.acc_scale[0] += dx(18);
	nav.acc_scale[1] += dx(19);
	nav.acc_scale[2] += dx(20);

	// ===== 8. 更新辅助量 =====
	nav.Rm = Cal_RM(nav.pos[0]);
	nav.Rn = Cal_RN(nav.pos[0]);
	nav.gravity = Cal_g_GRS80(nav.pos[0], nav.pos[2]);
}

