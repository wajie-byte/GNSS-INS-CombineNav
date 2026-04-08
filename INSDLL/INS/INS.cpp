#include"INS.h"
#include"Dynamic.h"



//保存结果
/*
In:
	double time 				//时刻s
	double BLH[3]				//位置经纬高rad,m
	double Vn[3]				//速度在n系下投影m/s(北东地)
	double YPR[3]				//姿态角rad
Out:
	IMUResultEpoch imuresult	//惯导计算结果
*/
void SaveIMUResultEpoch(double time, const double BLH[3], const double Vn[3],
	const double YPR[3], INS::IMUResultEpoch& imuresult)
{
	imuresult.time = time;
	for (int i = 0; i < 3; i++)
	{
		imuresult.BLH[i] = BLH[i];
		imuresult.Vn[i] = Vn[i];
		imuresult.YPR[i] = YPR[i];
	}
}

//惯导递推更新
/*
In:
	double BLH_k_2[3]		//k-2时刻位置经纬高rad,m
	double BLH_k_1[3]		//k-1时刻位置经纬高rad,m
	double Vn_k_2[3]		//k-2时刻速度在n系下投影m/s
	double Vn_k_1[3]		//k-1时刻速度在n系下投影m/s
	Mat q_bn_k_1			//k-1时刻姿态四元数(4x1)
	double gyro_k_1[3]		//k-1时刻角增量rad
	double gyro_k[3]		//k时刻角增量rad
	double dv_k_1[3]		//k-1时刻速度增量m/s
	double dv_k[3]			//k时刻速度增量m/s
	double dt				//采样间隔s
Out:
	double BLHk[3]			//k时刻位置经纬高rad,m
	double Vk_n[3]			//k时刻速度在n系下投影m/s
	double YPRk[3]			//k时刻姿态角rad
	Mat q_bn_k				//k时刻姿态四元数(4x1)
*/
void Updatek(const double BLH_k_2[3], const double BLH_k_1[3],
	const double Vn_k_2[3], const double Vn_k_1[3],
	const Mat q_bn_k_1,
	const double gyro_k_1[3], const double gyro_k[3],
	const double dv_k_1[3], const double dv_k[3], double dt,
	double BLHk[3], double Vk_n[3], double YPRk[3], Mat& q_bn_k)
{
	//-------------------------速度更新-----------------------
	//外推中间时刻的位置和速度
	double BLH_mid[3] = { 0.0,0.0,0.0 };
	double Vn_mid[3] = { 0.0,0.0,0.0 };
	for (int i = 0; i < 3; i++)
	{
		BLH_mid[i] = 1.5 * BLH_k_1[i] - 0.5 * BLH_k_2[i];
		Vn_mid[i] = 1.5 * Vn_k_1[i] - 0.5 * Vn_k_2[i];
	}
	//用中间时刻位置和速度计算地理参数
	double w_ie_n_mid[3] = { 0.0,0.0,0.0 };
	double w_en_n_mid[3] = { 0.0,0.0,0.0 };
	Cal_w_ie_n(BLH_mid[0], w_ie_n_mid);
	Cal_w_en_n(Vn_mid[1], Vn_mid[0], BLH_mid[0], BLH_mid[2], w_en_n_mid);
	//计算b系比力积分项
	double dv_f_b[3] = { 0.0,0.0,0.0 };
	Cal_dv_f_b(gyro_k_1, gyro_k, dv_k_1, dv_k, dv_f_b);
	//计算姿态矩阵
	Mat C_bn_k_1 = quaternion2C_b_n(q_bn_k_1);
	//计算比力积分项
	double dv_f_n[3] = { 0.0,0.0,0.0 };
	//计算等效旋转矢量
	double ERV_in[3] = { 0.0,0.0,0.0 };
	Cal_ERV_in(w_ie_n_mid, w_en_n_mid, dt, ERV_in);
	Cal_dv_f_n(ERV_in, C_bn_k_1, dv_f_b, dv_f_n);
	//计算重力加速度在n系下的投影
	double g_p_n_mid[3] = { 0.0,0.0,0.0 };
	Cal_g_p_n(BLH_mid[0], BLH_mid[2], g_p_n_mid);
	//计算重力哥氏积分项
	double dv_gc_n[3] = { 0.0,0.0,0.0 };
	Cal_dv_gc_n(g_p_n_mid, w_ie_n_mid, w_en_n_mid, Vn_mid, dt, dv_gc_n);
	//速度更新
	Cal_v_k_n(Vn_k_1, dv_f_n, dv_gc_n, Vk_n);
	//-------------------------位置更新-----------------------
	//更新高程，并计算中间时刻高程
	BLHk[2] = Cal_h_k(BLH_k_1[2], Vn_k_1[2], Vk_n[2], dt);
	BLH_mid[2] = (BLH_k_1[2] + BLHk[2]) / 2;
	//更新纬度，并计算中间时刻纬度
	BLHk[0] = Cal_lat_k(BLH_k_1[0], Vn_k_1[0], Vk_n[0], BLH_mid[2], dt);
	BLH_mid[0] = (BLH_k_1[0] + BLHk[0]) / 2;
	//更新经度,并计算中间时刻经度
	BLHk[1] = Cal_lon_k(BLH_k_1[1], Vn_k_1[1], Vk_n[1], BLH_mid[0], BLH_mid[2], dt);
	BLH_mid[1] = (BLH_k_1[1] + BLHk[1]) / 2;
	//-------------------------姿态更新-----------------------
	//重新计算中间时刻的位置和速度(在位置更新中已经完成)
	//重新计算地理参数
	Cal_w_ie_n(BLH_mid[0], w_ie_n_mid);
	Cal_w_en_n(Vn_mid[1], Vn_mid[0], BLH_mid[0], BLH_mid[2], w_en_n_mid);
	//计算n系的旋转四元数
	Cal_ERV_in(w_ie_n_mid, w_en_n_mid, dt, ERV_in);
	Mat q_n_rotate = Cal_neg_rotate(ERV_in);//n系姿态变化四元数
	//计算b系的旋转四元数(补偿圆锥效应)
	double ERV_ib[3] = { 0.0,0.0,0.0 };
	Cal_ERV_ib(gyro_k_1, gyro_k, ERV_ib);
	Mat q_b_rotate = Cal_pos_rotate(ERV_ib);
	//姿态更新
	q_bn_k = Attupdate_quaternion(q_n_rotate, q_bn_k_1, q_b_rotate);
	//四元数转姿态角
	quaternion2euler(q_bn_k, YPRk[0], YPRk[1], YPRk[2]);

}

//纯惯导算法
/*
In:
	std::vector<IMUDataEpoch> imudata		//IMU数据序列
	IMUResultEpoch imustart				//惯导初始参数
Out:
	std::vector<IMUResultEpoch> imuresult		//惯导计算结果序列
*/
void InertialNavigation(const std::vector<INS::IMUDataEpoch>& imudata, const INS::IMUResultEpoch& imustart,
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
	//k-2时刻
	double BLH_k_2[3] = { imustart.BLH[0], imustart.BLH[1], imustart.BLH[2] };
	double Vn_k_2[3] = { imustart.Vn[0], imustart.Vn[1], imustart.Vn[2] };

	//k-1时刻
	double BLH_k_1[3] = { imustart.BLH[0], imustart.BLH[1], imustart.BLH[2] };
	double Vn_k_1[3] = { imustart.Vn[0], imustart.Vn[1], imustart.Vn[2] };
	double YPR_k_1[3] = { imustart.YPR[0], imustart.YPR[1], imustart.YPR[2] };
	Mat q_bn_k_1 = euler2quaternion(imustart.YPR[0], imustart.YPR[1], imustart.YPR[2]);//k-1时刻姿态四元数
	//k时刻
	double BLH_k[3] = { 0.0,0.0,0.0 };
	double Vn_k[3] = { 0.0,0.0,0.0 };
	double YPR_k[3] = { 0.0,0.0,0.0 };
	Mat q_bn_k(4, 1);

	//保存初始时刻结果
	INS::IMUResultEpoch imuresult_epoch;
	//保存零速检测结果
	int static_count = 0;
	int Min_static_count = 5; //连续零速判断阈值
	std::vector<double> history_dv;
	std::vector<double> history_dtheta;
	//零速修正数据量计算
	int is_static_num = 0;
	
	//递推更新
	for (size_t k = index_start + 1; k < imudata.size(); k++)
	{
		dt = imudata[k].time - imudata[k - 1].time;
		Updatek(BLH_k_2, BLH_k_1,
			Vn_k_2, Vn_k_1,
			q_bn_k_1,
			imudata[k - 1].gyro, imudata[k].gyro,
			imudata[k - 1].accl, imudata[k].accl, dt,
			BLH_k, Vn_k, YPR_k, q_bn_k);
		
		//零速检测与修正
		
		bool is_static = isZeroVel(imudata[k].accl, imudata[k].gyro, BLH_k[0], BLH_k[2], dt, history_dv, history_dtheta);
		
		if(is_static)
		{
			static_count++;
			if(static_count >= Min_static_count)
			{
				is_static_num++;
				//零速修正
				for(int i = 0; i < 3; i++)
				{
					Vn_k[i] = 0.0;
				}
			}
		}
		else
		{
			static_count = 0;
		}
		
		
		
		//保存结果
		SaveIMUResultEpoch(imudata[k].time, BLH_k, Vn_k, YPR_k, imuresult_epoch);
		imuresult.push_back(imuresult_epoch);
		//更新k-1,k-2时刻数据
		for (int i = 0; i < 3; i++)
		{
			BLH_k_2[i] = BLH_k_1[i];
			Vn_k_2[i] = Vn_k_1[i];
			BLH_k_1[i] = BLH_k[i];
			Vn_k_1[i] = Vn_k[i];
			YPR_k_1[i] = YPR_k[i];
		}
		//更新姿态四元数
		for (int i = 0; i < 4; i++)
		{
			q_bn_k_1(i, 0) = q_bn_k(i, 0);
		}
	}
	std::cout << "共有" << is_static_num << "个零速修正点，占总数据的" << float(is_static_num) / (imudata.size() - index_start - 1) * 100.0 << "%" << std::endl;
}

//计算与参考结果的差值
/*
In:
	std::vector<IMUResultEpoch> imuresult		//惯导计算结果序列
	std::vector<IMUResultEpoch> imuref			//参考结果序列
Out:
	std::vector<IMUResultEpoch> imudiff			//惯导结果与参考结果差值序列
*/
void CalIMUResultDiff(const std::vector<INS::IMUResultEpoch>& imuresult,
	const std::vector<INS::IMUResultEpoch>& imuref,
	std::vector<INS::IMUResultEpoch>& imudiff)
{
	


	for (int i = 0; i < imuref.size(); i++)
	{
		INS::IMUResultEpoch diff_epoch;
		diff_epoch.time = imuresult[i].time;
		diff_epoch.BLH[0] = imuresult[i].BLH[0] - imuref[i].BLH[0] * deg2rad_scale;
		diff_epoch.BLH[1] = imuresult[i].BLH[1] - imuref[i].BLH[1] * deg2rad_scale;
		diff_epoch.BLH[2] = imuresult[i].BLH[2] - imuref[i].BLH[2];
		for (int j = 0; j < 3; j++)
		{
			diff_epoch.Vn[j] = imuresult[i].Vn[j] - imuref[i].Vn[j];
			diff_epoch.YPR[j] = imuresult[i].YPR[j] - imuref[i].YPR[j] * deg2rad_scale;//输出格式为航向、俯仰、横滚
		}
		imudiff.push_back(diff_epoch);
	}
	std::cout << "Calculated IMU result differences." << std::endl;
}