#include"CombConfig.h"

CombineNav::Config::Config()//默认构造函数
{

	//FILEPATHS
	files.imufilepath = "D:/visual studio/INS/dataset/IMU.bin";
	files.gnssfilepath = "D:/visual studio/INS/dataset/GNSSPLUSVEL.txt";
	files.odofilepath = "";
	files.outputfolder = "";

	//SWITCHES
	switches.usegnssvel = false;
	switches.useodo = false;
	switches.usenhc = false;
	switches.usezupt = false;

	//TIMECONFIG[s]
	time.starttime = 0.0;
	time.endtime = 0.0;

	//INITIALSTATE
	//POS[deg, deg, m]
	init_state.init_pos[0] = 0.0;
	init_state.init_pos[1] = 0.0;
	init_state.init_pos[2] = 0.0;
	//VEL[m/s]
	init_state.init_vel[0] = 0.0;
	init_state.init_vel[1] = 0.0;
	init_state.init_vel[2] = 0.0;
	//ATT[deg]
	init_state.init_att[0] = 0.0;
	init_state.init_att[1] = 0.0;
	init_state.init_att[2] = 0.0;
	//POS_STD[m]
	init_state.init_pos_std[0] = 0.0;
	init_state.init_pos_std[1] = 0.0;
	init_state.init_pos_std[2] = 0.0;
	//VEL_STD[m/s]
	init_state.init_vel_std[0] = 0.0;
	init_state.init_vel_std[1] = 0.0;
	init_state.init_vel_std[2] = 0.0;
	//ATT_STD[deg]
	init_state.init_att_std[0] = 0.0;
	init_state.init_att_std[1] = 0.0;
	init_state.init_att_std[2] = 0.0;

	//InitIMUErrorParams
	//INIT_GYROBIAS[deg/h]
	init_imu_errors.init_gyrobias[0] = 0.0;
	init_imu_errors.init_gyrobias[1] = 0.0;
	init_imu_errors.init_gyrobias[2] = 0.0;
	//INIT_ACCBIAS[mGal]
	init_imu_errors.init_accbias[0] = 0.0;
	init_imu_errors.init_accbias[1] = 0.0;
	init_imu_errors.init_accbias[2] = 0.0;
	//INIT_GYROSCALE[ppm]
	init_imu_errors.init_gyroscale[0] = 0.0;
	init_imu_errors.init_gyroscale[1] = 0.0;
	init_imu_errors.init_gyroscale[2] = 0.0;
	//INIT_ACCSCALE[ppm]
	init_imu_errors.init_accscale[0] = 0.0;
	init_imu_errors.init_accscale[1] = 0.0;
	init_imu_errors.init_accscale[2] = 0.0;
	//INIT_GYROBIAS_STD[deg/h]
	init_imu_errors.init_gyrobias_std[0] = 0.0;
	init_imu_errors.init_gyrobias_std[1] = 0.0;
	init_imu_errors.init_gyrobias_std[2] = 0.0;
	//INIT_ACCBIAS_STD[mGal]
	init_imu_errors.init_accbias_std[0] = 0.0;
	init_imu_errors.init_accbias_std[1] = 0.0;
	init_imu_errors.init_accbias_std[2] = 0.0;
	//INIT_GYROSCALE_STD[ppm]
	init_imu_errors.init_gyroscale_std[0] = 0.0;
	init_imu_errors.init_gyroscale_std[1] = 0.0;
	init_imu_errors.init_gyroscale_std[2] = 0.0;
	//INIT_ACCSCALE_STD[ppm]
	init_imu_errors.init_accscale_std[0] = 0.0;
	init_imu_errors.init_accscale_std[1] = 0.0;
	init_imu_errors.init_accscale_std[2] = 0.0;

	//ImuNoiseParams
	imu_noise.gyro_arw = 0.0;//[deg/s/sqrt(h)]
	imu_noise.accel_vrw = 0.0;//[m/s/sqrt(h)]
	imu_noise.gyrobias_std = 0.0;//[deg/h]
	imu_noise.accbias_std = 0.0;//[mGal]
	imu_noise.gyroscale_std = 0.0;//[ppm]
	imu_noise.accscale_std = 0.0;//[ppm]
	imu_noise.corrtime = 0.0;//[h]

	//InstallationParams
	//GNSS_LEVER[m]
	installation.gnss_lever[0] = 0.0;
	installation.gnss_lever[1] = 0.0;
	installation.gnss_lever[2] = 0.0;
	//ODO_LEVER[m]
	installation.odo_lever[0] = 0.0;
	installation.odo_lever[1] = 0.0;
	installation.odo_lever[2] = 0.0;
	//GNSS_INSTALL_ANGLE[deg]
	installation.gnss_install_angle[0] = 0.0;
	installation.gnss_install_angle[1] = 0.0;
	installation.gnss_install_angle[2] = 0.0;
	//ODO_INSTALL_ANGLE[deg]
	installation.odo_install_angle[0] = 0.0;
	installation.odo_install_angle[1] = 0.0;
	installation.odo_install_angle[2] = 0.0;



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