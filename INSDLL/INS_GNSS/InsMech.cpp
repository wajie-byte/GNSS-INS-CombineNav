#include"Combinenav.h"
#include"../INS/Dynamic.h"
#include"Param.h"
#include"../BASE/DefData.h"
#include "../INS/INS.h"




//惯导机械更新
/*
In:
	laststate: 上一时刻导航状态
	lastimu: 上一时刻IMU数据
	thisimu: 当前时刻IMU数据
Out:
	navstate: 当前时刻导航状态
*/
CombineNav::NavState CombineNav::InsMech(const CombineNav::NavState& laststate, const INS::IMUDataEpoch& lastimu, const INS::IMUDataEpoch& thisimu)
{
	CombineNav::Param param;
	//准备工作
	double interval = thisimu.time - lastimu.time;
	double deltatheta_k_1[3] = { lastimu.gyro[0], lastimu.gyro[1], lastimu.gyro[2] };
	double deltatheta_k[3] = { thisimu.gyro[0], thisimu.gyro[1], thisimu.gyro[2] };
	double deltav_k_1[3] = { lastimu.accl[0], lastimu.accl[1], lastimu.accl[2] };
	double deltav_k[3] = { thisimu.accl[0], thisimu.accl[1], thisimu.accl[2] };
	double e_omega = param.wie;
	double r_half[3] = { laststate.pos[0], laststate.pos[1], laststate.pos[2] };
	double v_half[3] = { laststate.vel[0], laststate.vel[1], laststate.vel[2] };
	double Rm_half = laststate.Rm;
	double Rn_half = laststate.Rn;
	Mat qbn_k_1 = laststate.qbn;
	double r_k_1[3] = { laststate.pos[0], laststate.pos[1], laststate.pos[2] };
	double v_k_1[3] = { laststate.vel[0], laststate.vel[1], laststate.vel[2] };
	double gravity = laststate.gravity;


	//算法

	double crosstheta_k_1_k[3] = { 0.0,0.0,0.0 };
	double phi_k[3] = { 0.0,0.0,0.0 };
	double omega_ie_n_half[3] = { 0.0,0.0,0.0 };
	double w_en_n_half[3] = { 0.0,0.0,0.0 };
	double zeta_k[3] = { 0.0,0.0,0.0 };
	Mat qbb(4, 1);
	Mat qnn(4, 1);
	Mat qbn_k(4, 1);
	Mat Cbn_tk(3, 3);



	cross_3_1(deltatheta_k_1, deltatheta_k, crosstheta_k_1_k);

	//phi_k = deltatheta_k + crosstheta_k_1_k / 12;
	for(int i=0; i < 3; i++)
	{
		phi_k[i] = deltatheta_k[i] + crosstheta_k_1_k[i] / 12;
	}
	//qbb = Cal_pos_rotate(phi_k);
	qbb = Cal_pos_rotate(phi_k);
	//omega_ie_n_half = Cal_w_ie_n(r_half[0]);
	Cal_w_ie_n(r_half[0], omega_ie_n_half);
	//w_en_n_half = Cal_w_en_n(v_half[0], v_half[1], r_half[0], r_half[2]);
	Cal_w_en_n(v_half[1], v_half[0], r_half[0], r_half[2], w_en_n_half);
	//zeta_k=(omega_ie_n_half + w_en_n_half) * interval;
	for(int i=0; i < 3; i++)
	{
		zeta_k[i] = (omega_ie_n_half[i] + w_en_n_half[i]) * interval;
	}
	//qnn=Cal_neg_rotate(zeta_k);
	qnn=Cal_neg_rotate(zeta_k);
	//qbn_k 四元数链乘实现姿态更新
	qbn_k = Attupdate_quaternion(qnn, qbn_k_1, qbb);
	//Cbn_tk = quaternion2C_b_n(qbn_k);
	Cbn_tk = quaternion2C_b_n(qbn_k);


	//vel
	double gl_n_half[3] = { 0,0,laststate.gravity };
	double deltav_fk_bk_1[3] = { 0.0,0.0,0.0 };
	double ksaik_1_k[3] = { 0.0,0.0,0.0 };
	double deltav_fk_n[3] = {0.0,0.0,0.0};
	double deltavg_cor_k[3] = { 0.0,0.0,0.0 };
	Mat Cb_n_tk_1 = quaternion2C_b_n(qbn_k_1);

	Cal_dv_f_b(deltatheta_k_1, deltatheta_k, deltav_k_1, deltav_k, deltav_fk_bk_1);
	Cal_ERV_in(omega_ie_n_half, w_en_n_half, interval, ksaik_1_k);
	Cal_dv_f_n(ksaik_1_k, Cb_n_tk_1, deltav_fk_bk_1, deltav_fk_n);
	Cal_dv_gc_n(gl_n_half, omega_ie_n_half, w_en_n_half, v_half, interval, deltavg_cor_k);

	double v_k[3] = { 0.0,0.0,0.0 };
	Cal_v_k_n(v_k_1, deltav_fk_n, deltavg_cor_k, v_k);

	//pos
	//update height
	double rk[3] = { 0.0,0.0,0.0 };
	rk[2] = Cal_h_k(r_k_1[2], v_k_1[2], v_k[2], interval);
	double mean_h = (rk[2] + r_k_1[2]) / 2;
	//update latitude
	rk[0] = Cal_lat_k(r_k_1[0], v_k_1[0], v_k[0], mean_h, interval);
	double mean_phi = (rk[0] + r_k_1[0]) / 2;
	//update longitude
	rk[1] = Cal_lon_k(r_k_1[1], v_k_1[1], v_k[1], mean_phi, mean_h, interval);


	//update new value
	CombineNav::NavState navstate;
	navstate.time = thisimu.time;
	for(int i=0; i < 3; i++)
	{
		navstate.pos[i] = rk[i];
		navstate.vel[i] = v_k[i];
		navstate.gyro_bias[i] = laststate.gyro_bias[i];
		navstate.acc_bias[i] = laststate.acc_bias[i];
		navstate.gyro_scale[i] = laststate.gyro_scale[i];
		navstate.acc_scale[i] = laststate.acc_scale[i];
	}
	navstate.qbn = qbn_k;
	navstate.Cbn = quaternion2C_b_n(navstate.qbn);
	quaternion2euler(navstate.qbn, navstate.att[2], navstate.att[1], navstate.att[0]);
	navstate.Rm = Cal_RM(navstate.pos[0]);
	navstate.Rn = Cal_RN(navstate.pos[0]);
	navstate.gravity = Cal_g_GRS80(navstate.pos[0], navstate.pos[2]);
	return navstate;
	//end
}



void CombineNav::Test_InsMech(const std::vector<INS::IMUDataEpoch>& imudata, const INS::IMUResultEpoch& imustart,
	std::vector<INS::IMUResultEpoch>& imuresult)
{
	//寻找初始数据索引
	int index_start = -1;
	bool found = FindInitialIndex(imudata, imustart.time, index_start);
	if (!found)
	{
		std::cout << "Initial time not found in IMU data." << std::endl;
		return;
	}
	//初始化变量
	double dt = 0.0; //采样间隔

	INS::IMUDataEpoch lastimu = imudata[index_start];

	CombineNav::NavState laststate;
	laststate.time = imustart.time;
	for (int i = 0; i < 3; i++)
	{
		laststate.pos[i] = imustart.BLH[i];
		laststate.vel[i] = imustart.Vn[i];
		laststate.att[i] = imustart.YPR[2 - i];
	}
	laststate.qbn = euler2quaternion(laststate.att[2], laststate.att[1], laststate.att[0]);
	laststate.Cbn = quaternion2C_b_n(laststate.qbn);
	laststate.Rm = Cal_RM(laststate.pos[0]);
	laststate.Rn = Cal_RN(laststate.pos[0]);
	laststate.gravity = Cal_g_GRS80(laststate.pos[0], laststate.pos[2]);


	CombineNav::NavState navstate;

	//保存初始时刻结果
	INS::IMUResultEpoch imuresult_epoch;
	INS::IMUDataEpoch thisimu;

	int lastpercent = -1; //上次处理进度百分比
	std::cout << "Starting INS mechanical update test...\n";
	//递推更新
	for (size_t k = index_start + 1; k < imudata.size(); k++)
	{
		thisimu = imudata[k];
		navstate = InsMech(laststate, lastimu, thisimu);
		imuresult_epoch.time = navstate.time;
		for (int i = 0; i < 3; i++)
		{
			imuresult_epoch.BLH[i] = navstate.pos[i];
			imuresult_epoch.Vn[i] = navstate.vel[i];
			imuresult_epoch.YPR[2 - i] = navstate.att[i];
		}
		imuresult.push_back(imuresult_epoch);
		lastimu = thisimu;
		laststate = navstate;
		//print processing information
		int current_percent = (int)(100.0 * (k - index_start) / (imudata.size() - index_start));
		if (current_percent > lastpercent)
		{
			std::cout << "Processing: " << current_percent << "%\n";
			lastpercent = current_percent;
		}
	}
	std::cout << "INS mechanical update test completed.\n";
}