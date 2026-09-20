#include"Combinenav.h"
#include"CombConfig.h"
#include"../INS/INS.h"
#include"../BASE/IMUFile.h"
#include"../DataSend/DataSend.h"
#include<cmath>

void CombineNav::gnssins_backward()
{
	// 反向滤波
	//
	//SimpleUdp udp("127.0.0.1", 5005);   // 发到本机5005端口

	//组合导航主函数
	CombineNav::Param param;
	CombineNav::Config config;

	//if back_filter
	std::vector<SmoothRecord> smooth_records;
	Eigen::MatrixXd Q(21, 21);

	std::vector<CombineNav::NavState> navstate_history;

	//load data
	//imudata
	std::vector<INS::IMUdata> rawimudata;
	std::vector<INS::IMUDataEpoch> imudata;
	//IMUBinFileRead(config.files.imufilepath.c_str(), imudata);
	IMUdataRead(config.files.imufilepath.c_str(), rawimudata);
	IMUdataProcess(rawimudata, imudata);

	double imustarttime = 0.0, imuendtime = 0.0;
	if (!imudata.empty())
	{
		imustarttime = imudata.front().time;
		imuendtime = imudata.back().time;
	}
	else { std::cout << "imudata is empty!\n"; return; }

	//gnssdata
	std::ifstream gnssfile = CombineNav::CreateReadFile(config.files.gnssfilepath);
	std::vector<CombineNav::GNSSResult> gnssdata;
	int gnssdata_col = CombineNav::GetGNSSData(gnssfile, gnssdata);
	double gnssstarttime = 0.0, gnssendtime = 0.0;
	if (!gnssdata.empty())
	{
		gnssstarttime = gnssdata.front().time;
		gnssendtime = gnssdata.back().time;
	}
	else { std::cout << "gnssdata is empty!\n"; return; }


	if (gnssdata_col == 13) { config.switches.usegnssvel = true; }
	else if (gnssdata_col == 7) { config.switches.usegnssvel = false; }
	else return;


	gnssfile.close();

	//单位转换
	for (int i = 0; i < gnssdata.size(); i++)
	{
		//位置deg转为rad
		gnssdata[i].pos[0] *= param.D2R;
		gnssdata[i].pos[1] *= param.D2R;
	}


	//set save folder
	std::string navpath = config.files.outputfolder + "/NavResult";
	if (config.switches.usegnssvel)
	{
		navpath += "_GNSSVEL";
	}
	navpath += ".nav";
	std::string imuerrpath = config.files.outputfolder + "/ImuError.txt";
	std::string stdpath = config.files.outputfolder + "/NavSTD.txt";


	//get process time
	double starttime = 0.0, endtime = 0.0;
	if (imustarttime > gnssstarttime) { starttime = imustarttime; }
	else { starttime = gnssstarttime; }
	if (imuendtime > gnssendtime) { endtime = gnssendtime; }
	else { endtime = imuendtime; }
	if (config.time.starttime < starttime)config.time.starttime = starttime;
	config.time.endtime = endtime;

	//data in process interval
	std::vector<INS::IMUDataEpoch> imudata_proc;
	std::vector<CombineNav::GNSSResult> gnssdata_proc;
	imudata_proc = INS::CutImuDataByTime(config.time.starttime, config.time.endtime, imudata);
	gnssdata_proc = CombineNav::CutGnssDataByTime(config.time.starttime, config.time.endtime, gnssdata);

	//
	std::cout << "Start GNSS/INS combined navigation processing...\n";

	//Initialize Kalman Filter and Navigation State
	CombineNav::KalmanFilter kf(21, 18);
	kf = kf.Initialize(config);
	Q = kf.Qc;
	CombineNav::NavState navstate;
	navstate = navstate.Initialize(config);

	//data index preprocess
	int imuindex = 0, gnssindex = 0;
	while (gnssindex < gnssdata_proc.size() && gnssdata_proc[gnssindex].time < imudata_proc[0].time)
	{
		gnssindex++;
	}

	//Main 
	CombineNav::NavState laststate = navstate;
	INS::IMUDataEpoch lastimu = imudata_proc[0];
	INS::IMUDataEpoch thisimu = imudata_proc[0];
	double imudt = 0.0;
	CombineNav::ZUPTDetector zupt_detector;


	//记录处理的数据占比
	int lastpercent = 0;
	Eigen::MatrixXd PHI_current(21, 21);   // 用于接收 InsPropagate 输出的 PHI

	for (imuindex = 1; imuindex < imudata_proc.size() - 1; imuindex++)
	{
		//重新赋值
		lastimu = thisimu;
		laststate = navstate;
		thisimu = imudata_proc[imuindex];
		imudt = thisimu.time - lastimu.time;
		for (int i = 0; i < 3; i++)//IMU数据补偿
		{
			thisimu.gyro[i] = (thisimu.gyro[i] - navstate.gyro_bias[i] * imudt) / (1 + navstate.gyro_scale[i]);
			thisimu.accl[i] = (thisimu.accl[i] - navstate.acc_bias[i] * imudt) / (1 + navstate.acc_scale[i]);
		}




		//adjust GNSS index
		while (gnssindex < gnssdata_proc.size() && gnssdata_proc[gnssindex].time < lastimu.time)
		{
			gnssindex++;
		}
		if (gnssindex >= gnssdata_proc.size()) { std::cout << "GNSS FILE END!\n"; break; }//GNSS观测数据用完，结束导航计算

		//determine if GNSS update is needed
		if (lastimu.time == gnssdata_proc[gnssindex].time)
		{
			//do GNSS update for the current state
			CombineNav::GNSSResult thisgnss = gnssdata_proc[gnssindex];
			imudt = thisimu.time - lastimu.time;
			CombineNav::GNSSUpdate(kf, navstate, config, thisgnss, thisimu, imudt);
			CombineNav::ErrorFeedBack(navstate, kf);
			//index++ 
			gnssindex++;
			laststate = navstate;

			//do propagation for current imu data
			imudt = thisimu.time - lastimu.time;
			navstate = InsMech(laststate, lastimu, thisimu);
			InsPropagate(navstate, thisimu, imudt, kf, config.imu_noise.corrtime, PHI_current);
		}

		else if (lastimu.time<gnssdata_proc[gnssindex].time && thisimu.time>gnssdata_proc[gnssindex].time)
		{
			//ineterpolate imu to gnss time
			INS::IMUDataEpoch firstimu, secondimu;
			interpolate(lastimu, thisimu, gnssdata_proc[gnssindex].time, firstimu, secondimu);
			//do propagation for first imu
			imudt = firstimu.time - lastimu.time;
			navstate = InsMech(laststate, lastimu, firstimu);
			InsPropagate(navstate, firstimu, imudt, kf, config.imu_noise.corrtime, PHI_current);
			//do gnss update
			CombineNav::GNSSResult thisgnss = gnssdata_proc[gnssindex];
			CombineNav::GNSSUpdate(kf, navstate, config, thisgnss, firstimu, imudt);
			CombineNav::ErrorFeedBack(navstate, kf);
			//index++
			gnssindex++;
			laststate = navstate;
			lastimu = firstimu;
			//do propagation for second imu
			imudt = secondimu.time - lastimu.time;
			navstate = InsMech(laststate, lastimu, secondimu);
			InsPropagate(navstate, secondimu, imudt, kf, config.imu_noise.corrtime, PHI_current);
		}
		else
		{
			// only do propagation
			// INS mechanization
			navstate = InsMech(laststate, lastimu, thisimu);
			//error propagation
			InsPropagate(navstate, thisimu, imudt, kf, config.imu_noise.corrtime, PHI_current);
		}

		if (config.switches.usenhc)
		{
			//do NHC update
			CombineNav::NHCUpdate(kf, navstate, config, thisimu, imudt);
			CombineNav::ErrorFeedBack(navstate, kf);
		}
		if (config.switches.usezupt)
		{
			// 静止检测


			bool is_static_now = detectStatic(thisimu, navstate.gravity,
				config.thresholds.zupt_acc_threshold,
				config.thresholds.zupt_gyro_threshold, imudt);
			bool is_zero_velocity = zupt_detector.update(is_static_now);

			if (is_zero_velocity && zupt_detector.isReady())
			{
				// ZUPT卡尔曼滤波更新
				CombineNav::ZUPTUpdate(kf, navstate, config);
				CombineNav::ErrorFeedBack(navstate, kf);
			}
		}

		SmoothRecord record;
		record.time = navstate.time;
		record.navstate = navstate;
		
		record.P = kf.P;
		record.Phi = PHI_current;
		smooth_records.push_back(record);
		navstate_history.push_back(navstate);


		//send data
		// 一行发送
		//udp.send(navstate.time, navstate.pos[0], navstate.pos[1], navstate.pos[2], navstate.vel[0], navstate.vel[1], navstate.vel[2], navstate.att[0], navstate.att[1], navstate.att[2]);

		//print processing information
		int current_percent = (int)(100.0 * imuindex / imudata_proc.size());
		if (current_percent > lastpercent)
		{
			std::cout << "Processing: " << current_percent << "%\n";
			lastpercent = current_percent;
		}

	}

	//打开文件
	std::ofstream navfp(navpath);
	if (!navfp.is_open()) {
		std::cout << "Cannot open " << navpath << std::endl;
		return;
	}

	std::ofstream imuerrfp(imuerrpath);
	if (!imuerrfp.is_open()) {
		std::cout << "Cannot open " << imuerrpath << std::endl;
		return;
	}

	std::ofstream stdfp(stdpath);
	if (!stdfp.is_open()) {
		std::cout << "Cannot open " << stdpath << std::endl;
		return;
	}
	for (int i = 0; i < navstate_history.size(); i++)
	{
		CombineNav::NavState nav = navstate_history[i];
		//save data
		//write navresult to file
		SaveNavResult(navfp, nav, param);
		//write imu error to file
		SaveIMUError(imuerrfp, nav, param);
		//write navstate std to file
		SaveStateStd(stdfp, kf, nav, param);
	}


	//close file
	navfp.close();
	imuerrfp.close();
	stdfp.close();
	std::cout << "File Writing Finished!\n";
	std::cout << "GNSS / INS Integration Processing Finished!\n";

	std::cout << "Starting Smoothing...\n";
	std::vector<CombineNav::NavState> smoothed_states;
	RTS_Smoother(smooth_records, config, Q, smoothed_states);
	// 保存平滑结果
	std::string smooth_path = config.files.outputfolder + "/NavResult_RTS";
	if (config.switches.usegnssvel) smooth_path += "_GNSSVEL";
	smooth_path += ".nav";

	std::ofstream smooth_fp(smooth_path);
	for (const auto& nav : smoothed_states) 
	{
		SaveNavResult(smooth_fp, nav, param);
	}
	smooth_fp.close();
	std::cout << "Smoothing Finished! Results saved to: " << smooth_path << std::endl;
}



void CombineNav::RTS_Smoother(
	const std::vector<CombineNav::SmoothRecord>& smooth_records,
	const CombineNav::Config& config,
	const Eigen::MatrixXd Qc,
	std::vector<CombineNav::NavState>& smoothed_states )
{
	int N = smooth_records.size();
	if (N < 2) return;

	std::cout << "Starting RTS Smoother...\n";

	// 平滑修正量，初始全为零
	std::vector<Eigen::VectorXd> dx_smoothed(N, Eigen::VectorXd::Zero(21));

	

	// 从后往前递推
	for (int k = N - 2; k >= 0; k--)
	{
		// 取出第k个记录
		Eigen::MatrixXd P_k = smooth_records[k].P;                        // 公式①
		Eigen::MatrixXd Phi_k = smooth_records[k + 1].Phi;  // 公式②
		CombineNav::NavState nav_k = smooth_records[k].navstate;
		double imudt = smooth_records[k + 1].time -smooth_records[k].time;

		Eigen::MatrixXd G(21, 18);
		// G矩阵赋值（噪声驱动矩阵）
	// 速度误差对应的加速度计噪声
		G.block<3, 3>(3, 0) = nav_k.Cbn;                    // G(4:6,1:3) = Cbn
		// 姿态误差对应的陀螺仪噪声
		G.block<3, 3>(6, 3) = nav_k.Cbn;                    // G(7:9,4:6) = Cbn
		// 陀螺仪漂移噪声（一阶马尔可夫过程）
		G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();      // G(10:12,7:9) = eye(3)
		// 加速度计偏置噪声
		G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();    // G(13:15,10:12) = eye(3)
		// 陀螺仪尺度因子噪声
		G.block<3, 3>(15, 12) = Eigen::Matrix3d::Identity();   // G(16:18,13:15) = eye(3)
		// 加速度计尺度因子噪声
		G.block<3, 3>(18, 15) = Eigen::Matrix3d::Identity();   // G(19:21,16:18) = eye(3)

		Eigen::MatrixXd Q = G * Qc * G.transpose() * imudt; // 离散化过程噪声协方差矩阵

		// 计算预测协方差
		Eigen::MatrixXd P_pred = Phi_k * P_k * Phi_k.transpose() + Q;    // 公式③
		if (P_pred.determinant() < 1e-30) 
		{
			std::cout << "Warning: P_pred nearly singular at k=" << k << "\n";
			P_pred += Eigen::MatrixXd::Identity(21, 21) * 1e-6;  // 正则化
		}

		// 计算平滑增益
		Eigen::MatrixXd G_k = P_k * Phi_k.transpose() * P_pred.inverse(); // 公式④

		// 递推修正量
		dx_smoothed[k] = G_k * dx_smoothed[k + 1];                       // 公式⑤
	}

	// 应用修正
	smoothed_states.clear();
	for (int k = 0; k < N; k++)
	{
		CombineNav::NavState nav = smooth_records[k].navstate;
		ApplyErrorCorrection(nav, dx_smoothed[k]);                        // 公式⑥
		smoothed_states.push_back(nav);
	}

	std::cout << "RTS Smoother Finished!\n";
}

//void CombineNav::gnssins_backwardfilter(
//	const CombineNav::NavState& forward_final_state,
//	const CombineNav::KalmanFilter& forward_final_kf,
//	const std::vector<INS::IMUDataEpoch>& imudata_forward,
//	const std::vector<CombineNav::GNSSResult>& gnssdata_forward,
//	const CombineNav::Config& config,
//	const CombineNav::Param& param,
//	std::vector<CombineNav::NavState>& navstate_history_back)
//{
//	std::cout << "Starting Backward Filtering...\n";
//
//	int N = imudata_forward.size();
//	int G = gnssdata_forward.size();
//	if (N < 2 || G == 0) return;
//
//	// ==================== 1. 找反向起点 ====================
//	int start_imu_index = -1;
//	for (int i = N - 1; i >= 0; --i)
//	{
//		if (imudata_forward[i].time <= forward_final_state.time)
//		{
//			start_imu_index = i;
//			break;
//		}
//	}
//	if (start_imu_index < 0)
//	{
//		std::cout << "Error: Cannot find start IMU for backward filter.\n";
//		return;
//	}
//
//	int gnss_index = -1;
//	for (int i = G - 1; i >= 0; --i)
//	{
//		if (gnssdata_forward[i].time <= forward_final_state.time)
//		{
//			gnss_index = i;
//			break;
//		}
//	}
//	if (gnss_index < 0)
//	{
//		std::cout << "Error: Cannot find start GNSS for backward filter.\n";
//		return;
//	}
//
//	// ==================== 2. 初始化反向滤波器 ====================
//	CombineNav::KalmanFilter kf_back = forward_final_kf;
//	CombineNav::NavState navstate = forward_final_state;
//
//	// ==================== 3. 初始化传播状态 ====================
//	// 关键修改：初始lastimu应该是start_imu_index（时间小的），因为循环中thisimu会是start_imu_index+1
//	CombineNav::NavState laststate = navstate;
//	INS::IMUDataEpoch lastimu = imudata_forward[start_imu_index];
//	double imudt = 0.0;
//	Eigen::MatrixXd PHI_current(21, 21);
//
//	CombineNav::ZUPTDetector zupt_detector;
//	int lastpercent = 0;
//	int total_loop = start_imu_index;
//
//	// ==================== 4. 主循环 ====================
//	for (int i = start_imu_index - 1; i >= 0; --i)
//	{
//		// 方案一：lastimu是时间小的，thisimu是时间大的
//		lastimu = imudata_forward[i];      // 时间小 (如 99.8)
//		INS::IMUDataEpoch thisimu = imudata_forward[i + 1];  // 时间大 (如 99.9)
//
//		// 关键：imudt = thisimu.time - lastimu.time，这是正数！
//		imudt = thisimu.time - lastimu.time;  // 99.9 - 99.8 = 0.1 > 0 
//
//		// 保存上一个状态
//		laststate = navstate;
//
//		// IMU数据补偿（使用补偿后的副本，不修改原始数据）
//		INS::IMUDataEpoch compensated_imu = thisimu;
//		for (int j = 0; j < 3; j++)
//		{
//			compensated_imu.gyro[j] = (thisimu.gyro[j] - navstate.gyro_bias[j] * imudt) /
//				(1 + navstate.gyro_scale[j]);
//			compensated_imu.accl[j] = (thisimu.accl[j] - navstate.acc_bias[j] * imudt) /
//				(1 + navstate.acc_scale[j]);
//		}
//
//		// ==================== 5. GNSS更新判断 ====================
//		if (gnss_index >= 0)
//		{
//			double g_time = gnssdata_forward[gnss_index].time;
//
//			// GNSS时间在lastimu和thisimu之间
//			if (g_time > lastimu.time && g_time < thisimu.time)
//			{
//				// 插值到GNSS时刻
//				INS::IMUDataEpoch firstimu, secondimu;
//				interpolate(lastimu, thisimu, g_time, firstimu, secondimu);
//
//				// 补偿firstimu
//				double dt_first = firstimu.time - lastimu.time;
//				for (int j = 0; j < 3; j++)
//				{
//					firstimu.gyro[j] = (firstimu.gyro[j] - navstate.gyro_bias[j] * dt_first) /
//						(1 + navstate.gyro_scale[j]);
//					firstimu.accl[j] = (firstimu.accl[j] - navstate.acc_bias[j] * dt_first) /
//						(1 + navstate.acc_scale[j]);
//				}
//
//				// 传播到 firstimu
//				navstate = InsMech(laststate, lastimu, firstimu);
//				InsPropagate(navstate, firstimu, dt_first, kf_back, config.imu_noise.corrtime, PHI_current);
//
//				// GNSS更新
//				CombineNav::GNSSResult thisgnss = gnssdata_forward[gnss_index];
//				CombineNav::GNSSUpdate(kf_back, navstate, config, thisgnss, firstimu, dt_first);
//				CombineNav::ErrorFeedBack(navstate, kf_back);
//				gnss_index--;
//
//				// 补偿secondimu
//				double dt_second = secondimu.time - firstimu.time;
//				for (int j = 0; j < 3; j++)
//				{
//					secondimu.gyro[j] = (secondimu.gyro[j] - navstate.gyro_bias[j] * dt_second) /
//						(1 + navstate.gyro_scale[j]);
//					secondimu.accl[j] = (secondimu.accl[j] - navstate.acc_bias[j] * dt_second) /
//						(1 + navstate.acc_scale[j]);
//				}
//
//				// 传播到 thisimu (secondimu)
//				laststate = navstate;
//				navstate = InsMech(laststate, firstimu, secondimu);
//				InsPropagate(navstate, secondimu, dt_second, kf_back, config.imu_noise.corrtime, PHI_current);
//
//				// 更新lastimu为thisimu，准备下一次循环
//				lastimu = thisimu;
//			}
//			// GNSS时间等于thisimu
//			else if (std::fabs(g_time - thisimu.time) < 1e-6)
//			{
//				// 先传播到thisimu
//				navstate = InsMech(laststate, lastimu, compensated_imu);
//				InsPropagate(navstate, compensated_imu, imudt, kf_back, config.imu_noise.corrtime, PHI_current);
//
//				// 再GNSS更新
//				CombineNav::GNSSResult thisgnss = gnssdata_forward[gnss_index];
//				CombineNav::GNSSUpdate(kf_back, navstate, config, thisgnss, compensated_imu, imudt);
//				CombineNav::ErrorFeedBack(navstate, kf_back);
//				gnss_index--;
//
//				// 更新lastimu
//				lastimu = thisimu;
//			}
//			else
//			{
//				// 纯传播
//				navstate = InsMech(laststate, lastimu, compensated_imu);
//				InsPropagate(navstate, compensated_imu, imudt, kf_back, config.imu_noise.corrtime, PHI_current);
//
//				// 更新lastimu
//				lastimu = thisimu;
//			}
//		}
//		else
//		{
//			// 没有GNSS了，纯传播
//			navstate = InsMech(laststate, lastimu, compensated_imu);
//			InsPropagate(navstate, compensated_imu, imudt, kf_back, config.imu_noise.corrtime, PHI_current);
//
//			// 更新lastimu
//			lastimu = thisimu;
//		}
//
//		// NHC
//		if (config.switches.usenhc)
//		{
//			CombineNav::NHCUpdate(kf_back, navstate, config, compensated_imu, imudt);
//			CombineNav::ErrorFeedBack(navstate, kf_back);
//		}
//
//		// ZUPT
//		if (config.switches.usezupt)
//		{
//			bool is_static_now = detectStatic(compensated_imu, navstate.gravity,
//				config.thresholds.zupt_acc_threshold,
//				config.thresholds.zupt_gyro_threshold, imudt);
//			bool is_zero_velocity = zupt_detector.update(is_static_now);
//
//			if (is_zero_velocity && zupt_detector.isReady())
//			{
//				CombineNav::ZUPTUpdate(kf_back, navstate, config);
//				CombineNav::ErrorFeedBack(navstate, kf_back);
//			}
//		}
//
//		navstate_history_back.push_back(navstate);
//
//		// 进度
//		int current_percent = (int)(100.0 * (total_loop - i) / total_loop);
//		if (current_percent > lastpercent)
//		{
//			std::cout << "Backward Processing: " << current_percent << "%\n";
//			lastpercent = current_percent;
//		}
//	}
//
//	// ==================== 6. 反转结果 ====================
//	std::reverse(navstate_history_back.begin(), navstate_history_back.end());
//
//	// ==================== 7. 保存 ====================
//	std::string back_path = config.files.outputfolder + "/NavResult_Backward";
//	if (config.switches.usegnssvel) back_path += "_GNSSVEL";
//	back_path += ".nav";
//
//	std::ofstream back_fp(back_path);
//	for (const auto& nav : navstate_history_back)
//	{
//		SaveNavResult(back_fp, nav, param);
//	}
//	back_fp.close();
//
//	std::cout << "Backward Result Saved!\n";
//	std::cout << "Backward GNSS/INS Integration Processing Finished!\n";
//}

void CombineNav::gnssins_backwardfilter(
	const CombineNav::NavState& forward_final_state,
	const CombineNav::KalmanFilter& forward_final_kf,
	const std::vector<INS::IMUDataEpoch>& imudata_forward,
	const std::vector<CombineNav::GNSSResult>& gnssdata_forward,
	const CombineNav::Config& config,
	const CombineNav::Param& param,
	std::vector<CombineNav::NavState>& navstate_history_back)
{
	std::cout << "Starting Backward Filtering...\n";

	int N = imudata_forward.size();
	int G = gnssdata_forward.size();
	if (N < 2 || G == 0) return;

	// ==================== 1. 找反向起点 ====================
	int start_imu_index = -1;
	for (int i = N - 1; i >= 0; --i)
	{
		if (imudata_forward[i].time <= forward_final_state.time)
		{
			start_imu_index = i;
			break;
		}
	}
	if (start_imu_index < 0)
	{
		std::cout << "Error: Cannot find start IMU for backward filter.\n";
		return;
	}

	int gnss_index = -1;
	for (int i = G - 1; i >= 0; --i)
	{
		if (gnssdata_forward[i].time <= forward_final_state.time)
		{
			gnss_index = i;
			break;
		}
	}
	if (gnss_index < 0)
	{
		std::cout << "Error: Cannot find start GNSS for backward filter.\n";
		return;
	}

	// 跳过起始时刻的GNSS（避免重复使用）
	while (gnss_index >= 0 && gnssdata_forward[gnss_index].time >= imudata_forward[start_imu_index].time - 1e-6)
	{
		gnss_index--;
	}

	// ==================== 2. 初始化反向滤波器 ====================
	CombineNav::KalmanFilter kf_back = forward_final_kf;
	CombineNav::NavState navstate = forward_final_state;

	// ==================== 3. 初始化 ====================
	CombineNav::NavState laststate = navstate;
	// 注意：初始lastimu是start_imu_index，时间最大的那个
	INS::IMUDataEpoch lastimu = imudata_forward[start_imu_index];
	double imudt = 0.0;
	Eigen::MatrixXd PHI_current(21, 21);

	CombineNav::ZUPTDetector zupt_detector;
	int lastpercent = 0;
	int total_loop = start_imu_index;

	// ==================== 4. 主循环 ====================
	for (int i = start_imu_index - 1; i >= 0; --i)
	{
		// 这次保持原始顺序：lastimu时间大，thisimu时间小
		INS::IMUDataEpoch thisimu = imudata_forward[i];      // 时间小 (如 99.8)
		// lastimu已经在上一轮赋值了，时间大 (如 99.9)

		imudt = lastimu.time - thisimu.time;  // 正数：99.9 - 99.8 = 0.1

		//  关键：取反IMU增量用于反向传播
		INS::IMUDataEpoch reversed_imu;
		reversed_imu.time = thisimu.time;
		for (int j = 0; j < 3; j++)
		{
			// 取反增量，让状态倒退
			reversed_imu.gyro[j] = -thisimu.gyro[j];
			reversed_imu.accl[j] = -thisimu.accl[j];
		}

		// IMU数据补偿
		for (int j = 0; j < 3; j++)
		{
			reversed_imu.gyro[j] = (reversed_imu.gyro[j] - navstate.gyro_bias[j] * imudt) /
				(1 + navstate.gyro_scale[j]);
			reversed_imu.accl[j] = (reversed_imu.accl[j] - navstate.acc_bias[j] * imudt) /
				(1 + navstate.acc_scale[j]);
		}

		// ==================== 5. GNSS更新判断 ====================
		if (gnss_index >= 0)
		{
			double g_time = gnssdata_forward[gnss_index].time;

			// GNSS时间在thisimu和lastimu之间
			if (g_time > thisimu.time && g_time < lastimu.time)
			{
				// 需要插值
				INS::IMUDataEpoch raw_firstimu, raw_secondimu;
				interpolate(thisimu, lastimu, g_time, raw_firstimu, raw_secondimu);

				// 取反并补偿firstimu
				INS::IMUDataEpoch firstimu;
				firstimu.time = raw_firstimu.time;
				double dt_first = lastimu.time - firstimu.time;
				for (int j = 0; j < 3; j++)
				{
					firstimu.gyro[j] = -raw_firstimu.gyro[j];
					firstimu.accl[j] = -raw_firstimu.accl[j];
					firstimu.gyro[j] = (firstimu.gyro[j] - navstate.gyro_bias[j] * dt_first) /
						(1 + navstate.gyro_scale[j]);
					firstimu.accl[j] = (firstimu.accl[j] - navstate.acc_bias[j] * dt_first) /
						(1 + navstate.acc_scale[j]);
				}

				// 传播到GNSS时刻
				navstate = InsMech(laststate, lastimu, firstimu);
				InsPropagate(navstate, firstimu, dt_first, kf_back, config.imu_noise.corrtime, PHI_current);

				// GNSS更新
				CombineNav::GNSSResult thisgnss = gnssdata_forward[gnss_index];
				CombineNav::GNSSUpdate(kf_back, navstate, config, thisgnss, firstimu, dt_first);
				CombineNav::ErrorFeedBack(navstate, kf_back);
				gnss_index--;

				// 取反并补偿secondimu
				INS::IMUDataEpoch secondimu;
				secondimu.time = raw_secondimu.time;
				double dt_second = firstimu.time - secondimu.time;
				for (int j = 0; j < 3; j++)
				{
					secondimu.gyro[j] = -raw_secondimu.gyro[j];
					secondimu.accl[j] = -raw_secondimu.accl[j];
					secondimu.gyro[j] = (secondimu.gyro[j] - navstate.gyro_bias[j] * dt_second) /
						(1 + navstate.gyro_scale[j]);
					secondimu.accl[j] = (secondimu.accl[j] - navstate.acc_bias[j] * dt_second) /
						(1 + navstate.acc_scale[j]);
				}

				// 传播到thisimu
				laststate = navstate;
				navstate = InsMech(laststate, firstimu, secondimu);
				InsPropagate(navstate, secondimu, dt_second, kf_back, config.imu_noise.corrtime, PHI_current);

				lastimu = thisimu;
			}
			// GNSS时间等于thisimu
			else if (std::fabs(g_time - thisimu.time) < 1e-6)
			{
				// 先传播到thisimu
				navstate = InsMech(laststate, lastimu, reversed_imu);
				InsPropagate(navstate, reversed_imu, imudt, kf_back, config.imu_noise.corrtime, PHI_current);

				// GNSS更新
				CombineNav::GNSSResult thisgnss = gnssdata_forward[gnss_index];
				CombineNav::GNSSUpdate(kf_back, navstate, config, thisgnss, reversed_imu, imudt);
				CombineNav::ErrorFeedBack(navstate, kf_back);
				gnss_index--;

				lastimu = thisimu;
			}
			else
			{
				// 纯传播
				navstate = InsMech(laststate, lastimu, reversed_imu);
				InsPropagate(navstate, reversed_imu, imudt, kf_back, config.imu_noise.corrtime, PHI_current);

				lastimu = thisimu;
			}
		}
		else
		{
			// 没有GNSS了，纯传播
			navstate = InsMech(laststate, lastimu, reversed_imu);
			InsPropagate(navstate, reversed_imu, imudt, kf_back, config.imu_noise.corrtime, PHI_current);

			lastimu = thisimu;
		}

		laststate = navstate;

		// NHC和ZUPT保持不变...
		if (config.switches.usenhc)
		{
			CombineNav::NHCUpdate(kf_back, navstate, config, reversed_imu, imudt);
			CombineNav::ErrorFeedBack(navstate, kf_back);
		}

		if (config.switches.usezupt)
		{
			bool is_static_now = detectStatic(reversed_imu, navstate.gravity,
				config.thresholds.zupt_acc_threshold,
				config.thresholds.zupt_gyro_threshold, imudt);
			bool is_zero_velocity = zupt_detector.update(is_static_now);

			if (is_zero_velocity && zupt_detector.isReady())
			{
				CombineNav::ZUPTUpdate(kf_back, navstate, config);
				CombineNav::ErrorFeedBack(navstate, kf_back);
			}
		}

		navstate_history_back.push_back(navstate);

		// 进度
		int current_percent = (int)(100.0 * (total_loop - i) / total_loop);
		if (current_percent > lastpercent)
		{
			std::cout << "Backward Processing: " << current_percent << "%\n";
			lastpercent = current_percent;
		}
	}

	// ==================== 6. 反转结果 ====================
	std::reverse(navstate_history_back.begin(), navstate_history_back.end());

	// ==================== 7. 保存 ====================
	std::string back_path = config.files.outputfolder + "/NavResult_Backward";
	if (config.switches.usegnssvel) back_path += "_GNSSVEL";
	back_path += ".nav";

	std::ofstream back_fp(back_path);
	for (const auto& nav : navstate_history_back)
	{
		SaveNavResult(back_fp, nav, param);
	}
	back_fp.close();

	std::cout << "Backward Result Saved!\n";
	std::cout << "Backward GNSS/INS Integration Processing Finished!\n";
}