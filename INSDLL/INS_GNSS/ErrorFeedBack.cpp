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
	
	Mat DR = Mat::Zero(3, 3);
	DR(0, 0) = navstate.Rm + navstate.pos[2];
	DR(1, 1) = (navstate.Rn + navstate.pos[2]) * cos(navstate.pos[0]);
	DR(2, 2) = -1.0;
	Mat DR_inv = DR.inverse();
	Mat dx_pos = Mat::Zero(3, 1);
	dx_pos(0, 0) = kf.x(0, 0);
	dx_pos(1, 0) = kf.x(1, 0);
	dx_pos(2, 0) = kf.x(2, 0);
	Mat DR_pos = DR_inv * dx_pos;
	for (int i = 0; i < 3; i++)
	{
		//position
		navstate.pos[i] = navstate.pos[i] - DR_pos(i, 0);
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
	Mat qpn = Cal_pos_rotate(erv);
	navstate.qbn = quaternion_multiply(qpn,navstate.qbn);
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
	kf.x = Mat::Zero(kf.x_rank, 1);
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


