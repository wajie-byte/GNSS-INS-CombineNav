#include"Combinenav.h"
#include"../INS/Dynamic.h"
//Îó²î·´À¡£¬½«¿¨¶ûÂüÂË²¨Æ÷µÄ×´Ì¬¹À¼ÆÎó²î·´À¡µ½µ¼º½×´Ì¬ÖĞ
/*
In:
	navstate: µ±Ç°µ¼º½×´Ì¬
	kf: ¿¨¶ûÂüÂË²¨Æ÷µ±Ç°×´Ì¬
Out:
	navstate_new: ·´À¡Îó²îºóµÄµ¼º½×´Ì¬
*/
CombineNav::NavState CombineNav::Nav_ErrorFeedBack(const CombineNav::NavState& navstate, const CombineNav::KalmanFilter& kf)
{
	CombineNav::NavState navstate_new = navstate;
	Mat DR = Mat::Zero(3, 3);
	DR(0, 0) = navstate.Rm + navstate.pos[2];
	DR(1, 1) = (navstate.Rn + navstate.pos[2]) * cos(navstate.pos[0]);
	DR(2, 2) = -1.0;
	Mat DR_inv = DR.inverse();
	Mat x_pos = Mat::Zero(3, 1);
	x_pos(0, 0) = kf.x(0, 0);
	x_pos(1, 0) = kf.x(1, 0);
	x_pos(2, 0) = kf.x(2, 0);
	Mat DR_pos = DR_inv * x_pos;
	for(int i=0;i<3;i++)
	{
		//position
		navstate_new.pos[i] = navstate.pos[i] - DR_pos(i, 0);
		//velocity
		navstate_new.vel[i] = navstate.vel[i] - kf.x(3 + i, 0);
		//gyro_bias,acc_bias,gyro_scale,acc_scale
		navstate_new.gyro_bias[i] = navstate.gyro_bias[i] + kf.x(9 + i, 0);
		navstate_new.acc_bias[i] = navstate.acc_bias[i] + kf.x(12 + i, 0);
		navstate_new.gyro_scale[i] = navstate.gyro_scale[i] + kf.x(15 + i, 0);
		navstate_new.acc_scale[i] = navstate.acc_scale[i] + kf.x(18 + i, 0);
		
	}
	//attitude
	Mat qpn = Cal_pos_rotate(navstate.att);
	navstate_new.qbn = quaternion_multiply(qpn,navstate.qbn);
	quaternion2euler(navstate_new.qbn, navstate_new.att[2], navstate_new.att[1], navstate_new.att[0]);
	//RM,RN,gravity
	navstate_new.Rm = Cal_RM(navstate_new.pos[0]);
	navstate_new.Rn = Cal_RN(navstate_new.pos[0]);
	navstate_new.gravity = Cal_g_GRS80(navstate_new.pos[0], navstate_new.pos[2]);
	return navstate_new;
}

//½«¿¨¶ûÂüÂË²¨Æ÷µÄ×´Ì¬¹À¼ÆÎó²î·´À¡ºó£¬ÖØÖÃ¿¨¶ûÂüÂË²¨Æ÷µÄ×´Ì¬ÏòÁ¿
/*
In:
	kf: ¿¨¶ûÂüÂË²¨Æ÷µ±Ç°×´Ì¬
Out:
	kf_new: ·´À¡Îó²îºóÖØÖÃ×´Ì¬ÏòÁ¿µÄ¿¨¶ûÂüÂË²¨Æ÷
*/
CombineNav::KalmanFilter CombineNav::KF_ErrorFeedBack(const CombineNav::KalmanFilter& kf)
{
	CombineNav::KalmanFilter kf_new = kf;
	//reset state vector
	kf_new.x = Mat::Zero(kf_new.x_rank, 1);
	return kf_new;
}

//½«¿¨¶ûÂüÂË²¨Æ÷µÄ×´Ì¬¹À¼ÆÎó²î·´À¡µ½µ¼º½×´Ì¬ÖĞ£¬²¢ÖØÖÃ¿¨¶ûÂüÂË²¨Æ÷µÄ×´Ì¬ÏòÁ¿
/*
@param navstate: µ±Ç°µ¼º½×´Ì¬
@param kf: ¿¨¶ûÂüÂË²¨Æ÷µ±Ç°×´Ì¬
*/
void CombineNav::ErrorFeedBack(CombineNav::NavState& navstate, CombineNav::KalmanFilter& kf)
{
	navstate = Nav_ErrorFeedBack(navstate, kf);
	kf = KF_ErrorFeedBack(kf);
}


