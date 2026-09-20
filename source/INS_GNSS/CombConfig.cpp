#include"CombConfig.h"
#include <limits>
CombineNav::Config::Config()//默认构造函数
{
	//METHODS
	methods.useRealTime = false;


	//FILEPATHS
	files.imufilepath = "D:/visual studio/INS/dataset/IMUdata.txt";
	files.gnssfilepath = "D:/visual studio/INS/dataset/GNSSdata.txt";
	files.odofilepath = "D:/visual studio/INS/dataset/ODO.txt";
	files.outputfolder = "D:/visual studio/INS/dataset/output/new";

	//SWITCHES
	switches.usegnssvel = true;
	switches.useodo = false;
	switches.usenhc = true;
	switches.usezupt = true;
	switches.backfilter = false;

	// TIMECONFIG [s]
	time.starttime = 436209.0;  // 从开始
	time.endtime = 438131.0;     // 结束时间

	// INITIALSTATE
	// POS [deg, deg, m]
	init_state.init_pos[0] = 30.528215550;   // 纬度 (deg)
	init_state.init_pos[1] = 114.355770431 ;  // 经度 (deg)
	init_state.init_pos[2] = 19.319100000;          // 高度 (m)
	// VEL [m/s]
	init_state.init_vel[0] = 0.0048;
	init_state.init_vel[1] = -0.0058;
	init_state.init_vel[2] = -0.0018;
	// ATT [deg] RPY
    init_state.init_att[0] = -0.367644;
	init_state.init_att[1] = -0.0839789;
	init_state.init_att[2] = 3.482746;
	// POS_STD [m]
	init_state.init_pos_std[0] = 0.5;
	init_state.init_pos_std[1] = 0.5;
	init_state.init_pos_std[2] = 1.0;
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
	init_imu_errors.init_gyrobias[0] = 360.0;
	init_imu_errors.init_gyrobias[1] = 240.0;
	init_imu_errors.init_gyrobias[2] = 300.0;
	// INIT_ACCBIAS [mGal]
	init_imu_errors.init_accbias[0] = 2942.0;
	init_imu_errors.init_accbias[1] = 2942.0;
	init_imu_errors.init_accbias[2] = 2942.0;
	// INIT_GYROSCALE [ppm]
	init_imu_errors.init_gyroscale[0] = 2000.0;
	init_imu_errors.init_gyroscale[1] = 17000.0;
	init_imu_errors.init_gyroscale[2] = 14000.0;
	// INIT_ACCSCALE [ppm]
	init_imu_errors.init_accscale[0] = 2600.0;
	init_imu_errors.init_accscale[1] = 8400.0;
	init_imu_errors.init_accscale[2] = 300.0;
	// INIT_GYROBIAS_STD [deg/h]
	init_imu_errors.init_gyrobias_std[0] = 1.2;
	init_imu_errors.init_gyrobias_std[1] = 1.2;
	init_imu_errors.init_gyrobias_std[2] = 1.2;
	// INIT_ACCBIAS_STD [mGal]
	init_imu_errors.init_accbias_std[0] = 14.0;
	init_imu_errors.init_accbias_std[1] = 14.0;
	init_imu_errors.init_accbias_std[2] = 14.0;
	// INIT_GYROSCALE_STD [ppm]
	init_imu_errors.init_gyroscale_std[0] = 360.0;
	init_imu_errors.init_gyroscale_std[1] = 360.0;
	init_imu_errors.init_gyroscale_std[2] = 360.0;
	// INIT_ACCSCALE_STD [ppm]
	init_imu_errors.init_accscale_std[0] = 300.0;
	init_imu_errors.init_accscale_std[1] = 300.0;
	init_imu_errors.init_accscale_std[2] = 300.0;

	// ImuNoiseParams
	imu_noise.gyro_arw = 0.08;          // [deg/sqrt(h)]
	imu_noise.accel_vrw = 0.02;         // [m/s/sqrt(h)]
	imu_noise.gyrobias_std = 1.2;     // [deg/h]
	imu_noise.accbias_std = 14.0;      // [mGal]
	imu_noise.gyroscale_std = 500.0;   // [ppm]
	imu_noise.accscale_std = 500.0;    // [ppm]
	imu_noise.corrtime = 1.0;          // [h]

	// InstallationParams
	// GNSS_LEVER [m]
	installation.gnss_lever[0] = 0.08;
	installation.gnss_lever[1] = 0.15;
	installation.gnss_lever[2] = 0.0;
	// ODO_LEVER [m]
	installation.odo_lever[0] = 0.0;
	installation.odo_lever[1] = 0.0;
	installation.odo_lever[2] = 0.0;
	// GNSS_INSTALL_ANGLE [deg]
	installation.gnss_install_angle[0] = 0.0;
	installation.gnss_install_angle[1] = 0.0;
	installation.gnss_install_angle[2] = 0.0;
	// ODO_INSTALL_ANGLE [deg]
	installation.odo_install_angle[0] = 0.0;
	installation.odo_install_angle[1] = 0.0;  
	installation.odo_install_angle[2] = 0.0;    

	//ObservationNoise
	obs_noise.zupt_std = 0.2; // ZUPT约束标准差

	//Thresholds
	thresholds.gnss_pos_threshold = 5.0; // GNSS位置阈值
	thresholds.gnss_vel_threshold = 0.5; // GNSS速度阈值
	thresholds.odo_vel_threshold = 0.5;  // 里程计速度阈值
	thresholds.nhc_threshold = 0.5;      // NHC约束阈值
	thresholds.zupt_acc_threshold = 0.2;     // ZUPT加速度约束阈值
	thresholds.zupt_gyro_threshold = 0.005;    // ZUPT陀螺约束阈值

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


// ============================================================
// 辅助函数：解析逗号分隔的数组
// ============================================================
void ParseArray(const std::string& value, double* arr, int size)
{
    std::istringstream iss(value);
    std::string token;
    int i = 0;
    while (std::getline(iss, token, ',') && i < size) {
        // 去除前后空格
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);
        if (!token.empty()) {
            arr[i++] = std::stod(token);
        }
    }
}

// ============================================================
// 打印配置（使用内部存储的单位，即转换后的值）
// ============================================================
void CombineNav::Config::Print() const
{
    std::cout << "\n========== 组合导航配置 ==========" << std::endl;

    // ===== 组合导航方法 =====
    std::cout << "\n【组合导航方法】" << std::endl;
    std::cout << "  使用实时组合导航: " << (methods.useRealTime ? "是" : "否") << std::endl;

    // ===== 文件路径 =====
    std::cout << "\n【文件路径】" << std::endl;
    std::cout << "  IMU文件: " << files.imufilepath << std::endl;
    std::cout << "  GNSS文件: " << files.gnssfilepath << std::endl;
    std::cout << "  里程计文件: " << files.odofilepath << std::endl;
    std::cout << "  输出文件夹: " << files.outputfolder << std::endl;

    // ===== 功能开关 =====
    std::cout << "\n【功能开关】" << std::endl;
    std::cout << "  使用GNSS速度: " << (switches.usegnssvel ? "是" : "否") << std::endl;
    std::cout << "  使用里程计: " << (switches.useodo ? "是" : "否") << std::endl;
    std::cout << "  使用NHC约束: " << (switches.usenhc ? "是" : "否") << std::endl;
    std::cout << "  使用零速约束: " << (switches.usezupt ? "是" : "否") << std::endl;
    std::cout << "  后向滤波: " << (switches.backfilter ? "是" : "否") << std::endl;

    // ===== 时间配置 =====
    std::cout << "\n【时间配置】" << std::endl;
    std::cout << "  开始时间: " << std::fixed << std::setprecision(3) << time.starttime << " s" << std::endl;
    std::cout << "  结束时间: " << std::fixed << std::setprecision(3) << time.endtime << " s" << std::endl;

    // ===== 初始状态（转换为度显示） =====
    std::cout << "\n【初始状态】" << std::endl;
    std::cout << std::fixed << std::setprecision(9);
    std::cout << "  位置: 纬度=" << init_state.init_pos[0] * Param::R2D
        << " deg, 经度=" << init_state.init_pos[1] * Param::R2D
        << " deg, 高度=" << init_state.init_pos[2] << " m" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  速度: [" << init_state.init_vel[0] << ", "
        << init_state.init_vel[1] << ", " << init_state.init_vel[2] << "] m/s" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  姿态: Roll=" << init_state.init_att[0] * Param::R2D
        << " deg, Pitch=" << init_state.init_att[1] * Param::R2D
        << " deg, Yaw=" << init_state.init_att[2] * Param::R2D << " deg" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  位置标准差: [" << init_state.init_pos_std[0] << ", "
        << init_state.init_pos_std[1] << ", " << init_state.init_pos_std[2] << "] m" << std::endl;
    std::cout << "  速度标准差: [" << init_state.init_vel_std[0] << ", "
        << init_state.init_vel_std[1] << ", " << init_state.init_vel_std[2] << "] m/s" << std::endl;
    std::cout << "  姿态标准差: [" << init_state.init_att_std[0] * Param::R2D << ", "
        << init_state.init_att_std[1] * Param::R2D << ", "
        << init_state.init_att_std[2] * Param::R2D << "] deg" << std::endl;

    // ===== 初始IMU误差 =====
    std::cout << "\n【初始IMU误差】" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  陀螺零偏: ["
        << init_imu_errors.init_gyrobias[0] * 3600.0 * Param::R2D << ", "
        << init_imu_errors.init_gyrobias[1] * 3600.0 * Param::R2D << ", "
        << init_imu_errors.init_gyrobias[2] * 3600.0 * Param::R2D << "] deg/h" << std::endl;
    std::cout << "  加计零偏: ["
        << init_imu_errors.init_accbias[0] * 1e5 << ", "
        << init_imu_errors.init_accbias[1] * 1e5 << ", "
        << init_imu_errors.init_accbias[2] * 1e5 << "] mGal" << std::endl;
    std::cout << "  陀螺尺度: ["
        << init_imu_errors.init_gyroscale[0] * 1e6 << ", "
        << init_imu_errors.init_gyroscale[1] * 1e6 << ", "
        << init_imu_errors.init_gyroscale[2] * 1e6 << "] ppm" << std::endl;
    std::cout << "  加计尺度: ["
        << init_imu_errors.init_accscale[0] * 1e6 << ", "
        << init_imu_errors.init_accscale[1] * 1e6 << ", "
        << init_imu_errors.init_accscale[2] * 1e6 << "] ppm" << std::endl;
    std::cout << "  陀螺零偏标准差: ["
        << init_imu_errors.init_gyrobias_std[0] * 3600.0 * Param::R2D << ", "
        << init_imu_errors.init_gyrobias_std[1] * 3600.0 * Param::R2D << ", "
        << init_imu_errors.init_gyrobias_std[2] * 3600.0 * Param::R2D << "] deg/h" << std::endl;
    std::cout << "  加计零偏标准差: ["
        << init_imu_errors.init_accbias_std[0] * 1e5 << ", "
        << init_imu_errors.init_accbias_std[1] * 1e5 << ", "
        << init_imu_errors.init_accbias_std[2] * 1e5 << "] mGal" << std::endl;
    std::cout << "  陀螺尺度标准差: ["
        << init_imu_errors.init_gyroscale_std[0] * 1e6 << ", "
        << init_imu_errors.init_gyroscale_std[1] * 1e6 << ", "
        << init_imu_errors.init_gyroscale_std[2] * 1e6 << "] ppm" << std::endl;
    std::cout << "  加计尺度标准差: ["
        << init_imu_errors.init_accscale_std[0] * 1e6 << ", "
        << init_imu_errors.init_accscale_std[1] * 1e6 << ", "
        << init_imu_errors.init_accscale_std[2] * 1e6 << "] ppm" << std::endl;

    // ===== IMU噪声参数 =====
    std::cout << "\n【IMU噪声参数】" << std::endl;
    std::cout << "  陀螺ARW: " << imu_noise.gyro_arw * 60.0 * Param::R2D << " deg/sqrt(h)" << std::endl;
    std::cout << "  加计VRW: " << imu_noise.accel_vrw * 60.0 << " m/s/sqrt(h)" << std::endl;
    std::cout << "  陀螺零偏标准差: " << imu_noise.gyrobias_std * 3600.0 * Param::R2D << " deg/h" << std::endl;
    std::cout << "  加计零偏标准差: " << imu_noise.accbias_std * 1e5 << " mGal" << std::endl;
    std::cout << "  陀螺尺度标准差: " << imu_noise.gyroscale_std * 1e6 << " ppm" << std::endl;
    std::cout << "  加计尺度标准差: " << imu_noise.accscale_std * 1e6 << " ppm" << std::endl;
    std::cout << "  相关时间: " << imu_noise.corrtime / 3600.0 << " h" << std::endl;

    // ===== 安装参数 =====
    std::cout << "\n【安装参数】" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  GNSS杆臂: [" << installation.gnss_lever[0] << ", "
        << installation.gnss_lever[1] << ", " << installation.gnss_lever[2] << "] m" << std::endl;
    std::cout << "  里程计杆臂: [" << installation.odo_lever[0] << ", "
        << installation.odo_lever[1] << ", " << installation.odo_lever[2] << "] m" << std::endl;
    std::cout << "  GNSS安装角: ["
        << installation.gnss_install_angle[0] * Param::R2D << ", "
        << installation.gnss_install_angle[1] * Param::R2D << ", "
        << installation.gnss_install_angle[2] * Param::R2D << "] deg" << std::endl;
    std::cout << "  里程计安装角: ["
        << installation.odo_install_angle[0] * Param::R2D << ", "
        << installation.odo_install_angle[1] * Param::R2D << ", "
        << installation.odo_install_angle[2] * Param::R2D << "] deg" << std::endl;

    // ===== 观测噪声 =====
    std::cout << "\n【观测噪声】" << std::endl;
    std::cout << "  ZUPT约束标准差: " << obs_noise.zupt_std << " m/s" << std::endl;

    // ===== 阈值 =====
    std::cout << "\n【阈值】" << std::endl;
    std::cout << "  GNSS位置阈值: " << thresholds.gnss_pos_threshold << " m" << std::endl;
    std::cout << "  GNSS速度阈值: " << thresholds.gnss_vel_threshold << " m/s" << std::endl;
    std::cout << "  里程计速度阈值: " << thresholds.odo_vel_threshold << " m/s" << std::endl;
    std::cout << "  NHC约束阈值: " << thresholds.nhc_threshold << " m/s" << std::endl;
    std::cout << "  ZUPT加速度阈值: " << thresholds.zupt_acc_threshold << " m/s^2" << std::endl;
    std::cout << "  ZUPT陀螺阈值: " << thresholds.zupt_gyro_threshold << " rad/s" << std::endl;

    std::cout << "\n============================================\n" << std::endl;
}

// ============================================================
// 保存配置到文件（反向转换回原始单位）
// ============================================================
bool CombineNav::Config::SaveToFile(const std::string& filename) const
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << " 无法创建配置文件: " << filename << std::endl;
        return false;
    }

    std::cout << " 保存配置到: " << filename << std::endl;

    file << "# ========== 组合导航配置文件 ==========" << std::endl;
    file << "# 自动生成于 " << __DATE__ << " " << __TIME__ << std::endl;
    file << "# ======================================" << std::endl << std::endl;

    // [Methods]
    file << "[Methods]" << std::endl;
    file << "useRealTime = " << (methods.useRealTime ? "true" : "false") << std::endl;
    file << std::endl;

    // [Files]
    file << "[Files]" << std::endl;
    file << "imufile = " << files.imufilepath << std::endl;
    file << "gnssfile = " << files.gnssfilepath << std::endl;
    file << "odofile = " << files.odofilepath << std::endl;
    file << "outputfolder = " << files.outputfolder << std::endl;
    file << std::endl;

    // [Switches]
    file << "[Switches]" << std::endl;
    file << "usegnssvel = " << (switches.usegnssvel ? "true" : "false") << std::endl;
    file << "useodo = " << (switches.useodo ? "true" : "false") << std::endl;
    file << "usenhc = " << (switches.usenhc ? "true" : "false") << std::endl;
    file << "usezupt = " << (switches.usezupt ? "true" : "false") << std::endl;
    file << "backfilter = " << (switches.backfilter ? "true" : "false") << std::endl;
    file << std::endl;

    // [Time]
    file << "[Time]" << std::endl;
    file << "starttime = " << std::fixed << std::setprecision(3) << time.starttime << std::endl;
    file << "endtime = " << std::fixed << std::setprecision(3) << time.endtime << std::endl;
    file << std::endl;

    // [InitialState] - 反向转换为原始单位（度）
    file << "[InitialState]" << std::endl;
    file << "init_pos = " << std::fixed << std::setprecision(9)
        << init_state.init_pos[0] * Param::R2D << ", "
        << init_state.init_pos[1] * Param::R2D << ", "
        << init_state.init_pos[2] << std::endl;
    file << "init_vel = " << init_state.init_vel[0] << ", "
        << init_state.init_vel[1] << ", " << init_state.init_vel[2] << std::endl;
    file << "init_att = " << init_state.init_att[0] * Param::R2D << ", "
        << init_state.init_att[1] * Param::R2D << ", "
        << init_state.init_att[2] * Param::R2D << std::endl;
    file << "init_pos_std = " << init_state.init_pos_std[0] << ", "
        << init_state.init_pos_std[1] << ", " << init_state.init_pos_std[2] << std::endl;
    file << "init_vel_std = " << init_state.init_vel_std[0] << ", "
        << init_state.init_vel_std[1] << ", " << init_state.init_vel_std[2] << std::endl;
    file << "init_att_std = " << init_state.init_att_std[0] * Param::R2D << ", "
        << init_state.init_att_std[1] * Param::R2D << ", "
        << init_state.init_att_std[2] * Param::R2D << std::endl;
    file << std::endl;

    // [InitIMUErrors] - 反向转换为原始单位
    file << "[InitIMUErrors]" << std::endl;
    file << "init_gyrobias = " << std::fixed << std::setprecision(4)
        << init_imu_errors.init_gyrobias[0] * 3600.0 * Param::R2D << ", "
        << init_imu_errors.init_gyrobias[1] * 3600.0 * Param::R2D << ", "
        << init_imu_errors.init_gyrobias[2] * 3600.0 * Param::R2D << std::endl;
    file << "init_accbias = "
        << init_imu_errors.init_accbias[0] * 1e5 << ", "
        << init_imu_errors.init_accbias[1] * 1e5 << ", "
        << init_imu_errors.init_accbias[2] * 1e5 << std::endl;
    file << "init_gyroscale = "
        << init_imu_errors.init_gyroscale[0] * 1e6 << ", "
        << init_imu_errors.init_gyroscale[1] * 1e6 << ", "
        << init_imu_errors.init_gyroscale[2] * 1e6 << std::endl;
    file << "init_accscale = "
        << init_imu_errors.init_accscale[0] * 1e6 << ", "
        << init_imu_errors.init_accscale[1] * 1e6 << ", "
        << init_imu_errors.init_accscale[2] * 1e6 << std::endl;
    file << "init_gyrobias_std = "
        << init_imu_errors.init_gyrobias_std[0] * 3600.0 * Param::R2D << ", "
        << init_imu_errors.init_gyrobias_std[1] * 3600.0 * Param::R2D << ", "
        << init_imu_errors.init_gyrobias_std[2] * 3600.0 * Param::R2D << std::endl;
    file << "init_accbias_std = "
        << init_imu_errors.init_accbias_std[0] * 1e5 << ", "
        << init_imu_errors.init_accbias_std[1] * 1e5 << ", "
        << init_imu_errors.init_accbias_std[2] * 1e5 << std::endl;
    file << "init_gyroscale_std = "
        << init_imu_errors.init_gyroscale_std[0] * 1e6 << ", "
        << init_imu_errors.init_gyroscale_std[1] * 1e6 << ", "
        << init_imu_errors.init_gyroscale_std[2] * 1e6 << std::endl;
    file << "init_accscale_std = "
        << init_imu_errors.init_accscale_std[0] * 1e6 << ", "
        << init_imu_errors.init_accscale_std[1] * 1e6 << ", "
        << init_imu_errors.init_accscale_std[2] * 1e6 << std::endl;
    file << std::endl;

    // [IMUNoise] - 反向转换为原始单位
    file << "[IMUNoise]" << std::endl;
    file << "gyro_arw = " << imu_noise.gyro_arw * 60.0 * Param::R2D << std::endl;
    file << "accel_vrw = " << imu_noise.accel_vrw * 60.0 << std::endl;
    file << "gyrobias_std = " << imu_noise.gyrobias_std * 3600.0 * Param::R2D << std::endl;
    file << "accbias_std = " << imu_noise.accbias_std * 1e5 << std::endl;
    file << "gyroscale_std = " << imu_noise.gyroscale_std * 1e6 << std::endl;
    file << "accscale_std = " << imu_noise.accscale_std * 1e6 << std::endl;
    file << "corrtime = " << imu_noise.corrtime / 3600.0 << std::endl;
    file << std::endl;

    // [Installation] - 角度反向转换为度
    file << "[Installation]" << std::endl;
    file << "gnss_lever = " << installation.gnss_lever[0] << ", "
        << installation.gnss_lever[1] << ", " << installation.gnss_lever[2] << std::endl;
    file << "odo_lever = " << installation.odo_lever[0] << ", "
        << installation.odo_lever[1] << ", " << installation.odo_lever[2] << std::endl;
    file << "gnss_install_angle = "
        << installation.gnss_install_angle[0] * Param::R2D << ", "
        << installation.gnss_install_angle[1] * Param::R2D << ", "
        << installation.gnss_install_angle[2] * Param::R2D << std::endl;
    file << "odo_install_angle = "
        << installation.odo_install_angle[0] * Param::R2D << ", "
        << installation.odo_install_angle[1] * Param::R2D << ", "
        << installation.odo_install_angle[2] * Param::R2D << std::endl;
    file << std::endl;

    // [ObservationNoise]
    file << "[ObservationNoise]" << std::endl;
    file << "zupt_std = " << obs_noise.zupt_std << std::endl;
    file << std::endl;

    // [Thresholds]
    file << "[Thresholds]" << std::endl;
    file << "gnss_pos_threshold = " << thresholds.gnss_pos_threshold << std::endl;
    file << "gnss_vel_threshold = " << thresholds.gnss_vel_threshold << std::endl;
    file << "odo_vel_threshold = " << thresholds.odo_vel_threshold << std::endl;
    file << "nhc_threshold = " << thresholds.nhc_threshold << std::endl;
    file << "zupt_acc_threshold = " << thresholds.zupt_acc_threshold << std::endl;
    file << "zupt_gyro_threshold = " << thresholds.zupt_gyro_threshold << std::endl;
    file << std::endl;

    file << "# ========== 配置结束 ==========" << std::endl;
    file.close();
    std::cout << " 配置保存完成" << std::endl;
    return true;
}

// ============================================================
// 从文件加载配置（进行正向单位转换）
// ============================================================
bool CombineNav::Config::LoadFromFile(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << " 无法打开配置文件: " << filename << std::endl;
        return false;
    }

    std::cout << " 从文件加载配置: " << filename << std::endl;

    // 先调用默认构造函数初始化所有值
    *this = Config();

    std::string line, section, key, value;

    while (std::getline(file, line)) {
        // 去除注释
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        // 去除首尾空格
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (line.empty()) continue;

        // 检查是否是节
        if (line[0] == '[' && line.back() == ']') {
            section = line.substr(1, line.size() - 2);
            continue;
        }

        // 解析键值对
        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;

        key = line.substr(0, eq_pos);
        value = line.substr(eq_pos + 1);

        // 去除键值空格
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);

        // ===== 根据节和键赋值 =====
        if (section == "Methods") {
            if (key == "useRealTime") {
                methods.useRealTime = (value == "true" || value == "1");
            }
        }

        if (section == "Files") {
            if (key == "imufile") files.imufilepath = value;
            else if (key == "gnssfile") files.gnssfilepath = value;
            else if (key == "odofile") files.odofilepath = value;
            else if (key == "outputfolder") files.outputfolder = value;
        }
        else if (section == "Switches") {
            if (key == "usegnssvel") switches.usegnssvel = (value == "true" || value == "1");
            else if (key == "useodo") switches.useodo = (value == "true" || value == "1");
            else if (key == "usenhc") switches.usenhc = (value == "true" || value == "1");
            else if (key == "usezupt") switches.usezupt = (value == "true" || value == "1");
            else if (key == "backfilter") switches.backfilter = (value == "true" || value == "1");
        }
        else if (section == "Time") {
            if (key == "starttime") time.starttime = std::stod(value);
            else if (key == "endtime") time.endtime = std::stod(value);
        }
        else if (section == "InitialState") {
            if (key == "init_pos") {
                ParseArray(value, init_state.init_pos, 3);
                init_state.init_pos[0] *= Param::D2R;
                init_state.init_pos[1] *= Param::D2R;
            }
            else if (key == "init_vel") {
                ParseArray(value, init_state.init_vel, 3);
            }
            else if (key == "init_att") {
                ParseArray(value, init_state.init_att, 3);
                init_state.init_att[0] *= Param::D2R;
                init_state.init_att[1] *= Param::D2R;
                init_state.init_att[2] *= Param::D2R;
            }
            else if (key == "init_pos_std") {
                ParseArray(value, init_state.init_pos_std, 3);
            }
            else if (key == "init_vel_std") {
                ParseArray(value, init_state.init_vel_std, 3);
            }
            else if (key == "init_att_std") {
                ParseArray(value, init_state.init_att_std, 3);
                init_state.init_att_std[0] *= Param::D2R;
                init_state.init_att_std[1] *= Param::D2R;
                init_state.init_att_std[2] *= Param::D2R;
            }
        }
        else if (section == "InitIMUErrors") {
            if (key == "init_gyrobias") {
                ParseArray(value, init_imu_errors.init_gyrobias, 3);
                init_imu_errors.init_gyrobias[0] *= (Param::D2R / 3600.0);
                init_imu_errors.init_gyrobias[1] *= (Param::D2R / 3600.0);
                init_imu_errors.init_gyrobias[2] *= (Param::D2R / 3600.0);
            }
            else if (key == "init_accbias") {
                ParseArray(value, init_imu_errors.init_accbias, 3);
                init_imu_errors.init_accbias[0] *= 1e-5;
                init_imu_errors.init_accbias[1] *= 1e-5;
                init_imu_errors.init_accbias[2] *= 1e-5;
            }
            else if (key == "init_gyroscale") {
                ParseArray(value, init_imu_errors.init_gyroscale, 3);
                init_imu_errors.init_gyroscale[0] *= 1e-6;
                init_imu_errors.init_gyroscale[1] *= 1e-6;
                init_imu_errors.init_gyroscale[2] *= 1e-6;
            }
            else if (key == "init_accscale") {
                ParseArray(value, init_imu_errors.init_accscale, 3);
                init_imu_errors.init_accscale[0] *= 1e-6;
                init_imu_errors.init_accscale[1] *= 1e-6;
                init_imu_errors.init_accscale[2] *= 1e-6;
            }
            else if (key == "init_gyrobias_std") {
                ParseArray(value, init_imu_errors.init_gyrobias_std, 3);
                init_imu_errors.init_gyrobias_std[0] *= (Param::D2R / 3600.0);
                init_imu_errors.init_gyrobias_std[1] *= (Param::D2R / 3600.0);
                init_imu_errors.init_gyrobias_std[2] *= (Param::D2R / 3600.0);
            }
            else if (key == "init_accbias_std") {
                ParseArray(value, init_imu_errors.init_accbias_std, 3);
                init_imu_errors.init_accbias_std[0] *= 1e-5;
                init_imu_errors.init_accbias_std[1] *= 1e-5;
                init_imu_errors.init_accbias_std[2] *= 1e-5;
            }
            else if (key == "init_gyroscale_std") {
                ParseArray(value, init_imu_errors.init_gyroscale_std, 3);
                init_imu_errors.init_gyroscale_std[0] *= 1e-6;
                init_imu_errors.init_gyroscale_std[1] *= 1e-6;
                init_imu_errors.init_gyroscale_std[2] *= 1e-6;
            }
            else if (key == "init_accscale_std") {
                ParseArray(value, init_imu_errors.init_accscale_std, 3);
                init_imu_errors.init_accscale_std[0] *= 1e-6;
                init_imu_errors.init_accscale_std[1] *= 1e-6;
                init_imu_errors.init_accscale_std[2] *= 1e-6;
            }
        }
        else if (section == "IMUNoise") {
            if (key == "gyro_arw") {
                imu_noise.gyro_arw = std::stod(value);
                imu_noise.gyro_arw *= (Param::D2R / 60.0);
            }
            else if (key == "accel_vrw") {
                imu_noise.accel_vrw = std::stod(value);
                imu_noise.accel_vrw /= 60.0;
            }
            else if (key == "gyrobias_std") {
                imu_noise.gyrobias_std = std::stod(value);
                imu_noise.gyrobias_std *= (Param::D2R / 3600.0);
            }
            else if (key == "accbias_std") {
                imu_noise.accbias_std = std::stod(value);
                imu_noise.accbias_std *= 1e-5;
            }
            else if (key == "gyroscale_std") {
                imu_noise.gyroscale_std = std::stod(value);
                imu_noise.gyroscale_std *= 1e-6;
            }
            else if (key == "accscale_std") {
                imu_noise.accscale_std = std::stod(value);
                imu_noise.accscale_std *= 1e-6;
            }
            else if (key == "corrtime") {
                imu_noise.corrtime = std::stod(value);
                imu_noise.corrtime *= 3600.0;
            }
        }
        else if (section == "Installation") {
            if (key == "gnss_lever") {
                ParseArray(value, installation.gnss_lever, 3);
            }
            else if (key == "odo_lever") {
                ParseArray(value, installation.odo_lever, 3);
            }
            else if (key == "gnss_install_angle") {
                ParseArray(value, installation.gnss_install_angle, 3);
                installation.gnss_install_angle[0] *= Param::D2R;
                installation.gnss_install_angle[1] *= Param::D2R;
                installation.gnss_install_angle[2] *= Param::D2R;
            }
            else if (key == "odo_install_angle") {
                ParseArray(value, installation.odo_install_angle, 3);
                installation.odo_install_angle[0] *= Param::D2R;
                installation.odo_install_angle[1] *= Param::D2R;
                installation.odo_install_angle[2] *= Param::D2R;
            }
        }
        else if (section == "ObservationNoise") {
            if (key == "zupt_std") obs_noise.zupt_std = std::stod(value);
        }
        else if (section == "Thresholds") {
            if (key == "gnss_pos_threshold") thresholds.gnss_pos_threshold = std::stod(value);
            else if (key == "gnss_vel_threshold") thresholds.gnss_vel_threshold = std::stod(value);
            else if (key == "odo_vel_threshold") thresholds.odo_vel_threshold = std::stod(value);
            else if (key == "nhc_threshold") thresholds.nhc_threshold = std::stod(value);
            else if (key == "zupt_acc_threshold") thresholds.zupt_acc_threshold = std::stod(value);
            else if (key == "zupt_gyro_threshold") thresholds.zupt_gyro_threshold = std::stod(value);
        }
    }

    file.close();
    std::cout << " 配置加载完成" << std::endl;
    return true;
}