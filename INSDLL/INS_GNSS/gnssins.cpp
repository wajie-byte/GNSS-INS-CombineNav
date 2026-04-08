#include"Combinenav.h"
#include"CombConfig.h"
#include"../INS/INS.h"
#include"../BASE/IMUFile.h"


void CombineNav::gnssins()
{
	
	//组合导航主函数
	CombineNav::Param param;
	CombineNav::Config config;
	
	//load data
	//imudata
	std::vector<INS::IMUDataEpoch> imudata;
	IMUBinFileRead(config.files.imufilepath.c_str(), imudata);
	double imustarttime = 0.0, imuendtime = 0.0;
	if (!imudata.empty())
	{
		double imustarttime = imudata.front().time;
		double imuendtime = imudata.back().time;
	}
	else std::cout << "imudata is empty!\n" ; return;
	

	//gnssdata
	std::ifstream gnssfile = CombineNav::CreateReadFile(config.files.gnssfilepath);
	std::vector<CombineNav::GNSSResult> gnssdata;
	int gnssdata_col = CombineNav::GetGNSSData(gnssfile, gnssdata);
	double gnssstarttime = 0.0, gnssendtime = 0.0;
	if (!gnssdata.empty())
	{
		double gnssstarttime = gnssdata.front().time;
		double gnssendtime = gnssdata.back().time;
	}
	else std::cout << "gnssdata is empty!\n" ; return;


	if (gnssdata_col == 13)config.switches.usegnssvel = true;
	else if (gnssdata_col == 7)config.switches.usegnssvel = false;
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
	if(config.switches.usegnssvel)
	{
		navpath += "_GNSSVEL";
	}
	navpath += ".nav";
	std::string imuerrpath = config.files.outputfolder + "/ImuError.txt";
	std::string stdpath = config.files.outputfolder + "/NavSTD.txt";


	//get process time
	double starttime = 0.0, endtime = 0.0;
	if(imustarttime>gnssstarttime)starttime = imustarttime;
	else starttime = gnssstarttime;
	if(imuendtime>gnssendtime)endtime = gnssendtime;
	else endtime = imuendtime;
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
	CombineNav::KalmanFilter kf;
	kf.Initialize(config);
	CombineNav::NavState navstate;
	navstate.Initialize(config);

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
		if (gnssindex >= gnssdata_proc.size())std::cout << "GNSS FILE END!\n"; break;//GNSS观测数据用完，结束导航计算

		//determine if GNSS update is needed
		if (lastimu.time == gnssdata_proc[gnssindex].time)
		{
			//do GNSS update for the current state
			CombineNav::GNSSResult thisgnss = gnssdata_proc[gnssindex];
			imudt = thisimu.time - lastimu.time;
			kf = CombineNav::GNSSUpdate(kf, navstate, config, thisgnss, thisimu, imudt);
			CombineNav::ErrorFeedBack(navstate, kf);
			//index++ 
			gnssindex++;
			laststate = navstate;

			//do propagation for current imu data
			imudt = thisimu.time - lastimu.time;
			navstate = InsMech(laststate, lastimu, thisimu);
		}

	}

	



}