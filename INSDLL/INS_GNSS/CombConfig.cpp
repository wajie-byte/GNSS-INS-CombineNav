#include"CombConfig.h"
#include <limits>
CombineNav::Config::Config()//默认构造函数
{

	//FILEPATHS
	files.imufilepath = "D:/visual studio/INS/dataset/IMU.bin";
	files.gnssfilepath = "D:/visual studio/INS/dataset/GNSSPLUSVEL.txt";
	files.odofilepath = "D:/visual studio/INS/dataset/ODO.txt";
	files.outputfolder = "D:/visual studio/INS/dataset/output";

	//SWITCHES
	switches.usegnssvel = true;
	switches.useodo = false;
	switches.usenhc = false;
	switches.usezupt = false;

	// TIMECONFIG [s]
	time.starttime = 527000.0;  // 从527000秒开始
	time.endtime = 604801.0;     // 结束时间

	// INITIALSTATE
	// POS [deg, deg, m]
	init_state.init_pos[0] = 28.1421701503;   // 纬度 (deg)
	init_state.init_pos[1] = 112.9585396051;  // 经度 (deg)
	init_state.init_pos[2] = 38.591;          // 高度 (m)
	// VEL [m/s]
	init_state.init_vel[0] = -0.273;
	init_state.init_vel[1] = -3.942;
	init_state.init_vel[2] = -0.020;
	// ATT [deg]
	init_state.init_att[0] = 1.40822092;
	init_state.init_att[1] = -0.95903515;
	init_state.init_att[2] = 267.77618701;
	// POS_STD [m]
	init_state.init_pos_std[0] = 0.05;
	init_state.init_pos_std[1] = 0.05;
	init_state.init_pos_std[2] = 0.1;
	// VEL_STD [m/s]
	init_state.init_vel_std[0] = 0.05;
	init_state.init_vel_std[1] = 0.05;
	init_state.init_vel_std[2] = 0.05;
	// ATT_STD [deg]
	init_state.init_att_std[0] = 0.1;
	init_state.init_att_std[1] = 0.1;
	init_state.init_att_std[2] = 0.5;

	// InitIMUErrorParams
	// INIT_GYROBIAS [deg/h]
	init_imu_errors.init_gyrobias[0] = 100.0;
	init_imu_errors.init_gyrobias[1] = -300.0;
	init_imu_errors.init_gyrobias[2] = -100.0;
	// INIT_ACCBIAS [mGal]
	init_imu_errors.init_accbias[0] = -1500.0;
	init_imu_errors.init_accbias[1] = -600.0;
	init_imu_errors.init_accbias[2] = 0.0;
	// INIT_GYROSCALE [ppm]
	init_imu_errors.init_gyroscale[0] = 1000.0;
	init_imu_errors.init_gyroscale[1] = 6000.0;
	init_imu_errors.init_gyroscale[2] = -4000.0;
	// INIT_ACCSCALE [ppm]
	init_imu_errors.init_accscale[0] = 2600.0;
	init_imu_errors.init_accscale[1] = 8400.0;
	init_imu_errors.init_accscale[2] = 300.0;
	// INIT_GYROBIAS_STD [deg/h]
	init_imu_errors.init_gyrobias_std[0] = 0.027;
	init_imu_errors.init_gyrobias_std[1] = 0.027;
	init_imu_errors.init_gyrobias_std[2] = 0.027;
	// INIT_ACCBIAS_STD [mGal]
	init_imu_errors.init_accbias_std[0] = 15.0;
	init_imu_errors.init_accbias_std[1] = 15.0;
	init_imu_errors.init_accbias_std[2] = 15.0;
	// INIT_GYROSCALE_STD [ppm]
	init_imu_errors.init_gyroscale_std[0] = 300.0;
	init_imu_errors.init_gyroscale_std[1] = 300.0;
	init_imu_errors.init_gyroscale_std[2] = 300.0;
	// INIT_ACCSCALE_STD [ppm]
	init_imu_errors.init_accscale_std[0] = 300.0;
	init_imu_errors.init_accscale_std[1] = 300.0;
	init_imu_errors.init_accscale_std[2] = 300.0;

	// ImuNoiseParams
	imu_noise.gyro_arw = 0.2;          // [deg/s/sqrt(h)]
	imu_noise.accel_vrw = 0.2;         // [m/s/sqrt(h)]
	imu_noise.gyrobias_std = 48.0;     // [deg/h]
	imu_noise.accbias_std = 50.0;      // [mGal]
	imu_noise.gyroscale_std = 500.0;   // [ppm]
	imu_noise.accscale_std = 500.0;    // [ppm]
	imu_noise.corrtime = 1.0;          // [h]

	// InstallationParams
	// GNSS_LEVER [m]
	installation.gnss_lever[0] = 0.505;
	installation.gnss_lever[1] = -0.145;
	installation.gnss_lever[2] = -1.105;
	// ODO_LEVER [m]
	installation.odo_lever[0] = -0.605;
	installation.odo_lever[1] = -1.025;
	installation.odo_lever[2] = 0.705;
	// GNSS_INSTALL_ANGLE [deg]
	installation.gnss_install_angle[0] = 0.0;
	installation.gnss_install_angle[1] = 0.0;
	installation.gnss_install_angle[2] = 0.0;
	// ODO_INSTALL_ANGLE [deg]
	installation.odo_install_angle[0] = 0.0;
	installation.odo_install_angle[1] = -0.532;  
	installation.odo_install_angle[2] = 1.38;    



	//单位转换
	//POS
	init_state.init_pos[0] *= Param::D2R;
	init_state.init_pos[1] *= Param::D2R;
	//ATT
	init_state.init_att[0] *= Param::D2R;
	init_state.init_att[1] *= Param::D2R;
	init_state.init_att[2] *= Param::D2R;
	init_state.init_att_std[0] *= Param::D2R;
	init_state.init_att_std[1] *= Param::D2R;
	init_state.init_att_std[2] *= Param::D2R;
	//Gyro
	init_imu_errors.init_gyrobias[0] *= (Param::D2R / 3600.0);
	init_imu_errors.init_gyrobias[1] *= (Param::D2R / 3600.0);
	init_imu_errors.init_gyrobias[2] *= (Param::D2R / 3600.0);
	init_imu_errors.init_gyrobias_std[0] *= (Param::D2R / 3600.0);
	init_imu_errors.init_gyrobias_std[1] *= (Param::D2R / 3600.0);
	init_imu_errors.init_gyrobias_std[2] *= (Param::D2R / 3600.0);
	init_imu_errors.init_gyroscale[0] *= 1e-6;
	init_imu_errors.init_gyroscale[1] *= 1e-6;
	init_imu_errors.init_gyroscale[2] *= 1e-6;
	init_imu_errors.init_gyroscale_std[0] *= 1e-6;
	init_imu_errors.init_gyroscale_std[1] *= 1e-6;
	init_imu_errors.init_gyroscale_std[2] *= 1e-6;
	//Acc
	init_imu_errors.init_accbias[0] *= 1e-5;
	init_imu_errors.init_accbias[1] *= 1e-5;
	init_imu_errors.init_accbias[2] *= 1e-5;
	init_imu_errors.init_accbias_std[0] *= 1e-5;
	init_imu_errors.init_accbias_std[1] *= 1e-5;
	init_imu_errors.init_accbias_std[2] *= 1e-5;
	init_imu_errors.init_accscale[0] *= 1e-6;
	init_imu_errors.init_accscale[1] *= 1e-6;
	init_imu_errors.init_accscale[2] *= 1e-6;
	init_imu_errors.init_accscale_std[0] *= 1e-6;
	init_imu_errors.init_accscale_std[1] *= 1e-6;
	init_imu_errors.init_accscale_std[2] *= 1e-6;

	//Noise
	imu_noise.gyro_arw *= (Param::D2R / 60.0);
	imu_noise.accel_vrw /= 60.0;
	imu_noise.gyrobias_std *= (Param::D2R / 3600.0);
	imu_noise.accbias_std *= 1e-5;
	imu_noise.gyroscale_std *= 1e-6;
	imu_noise.accscale_std *= 1e-6;
	imu_noise.corrtime *= 3600.0;

	//Installation
	installation.gnss_install_angle[0] *= Param::D2R;
	installation.gnss_install_angle[1] *= Param::D2R;
	installation.gnss_install_angle[2] *= Param::D2R;
	installation.odo_install_angle[0] *= Param::D2R;
	installation.odo_install_angle[1] *= Param::D2R;
	installation.odo_install_angle[2] *= Param::D2R;


	
	
}