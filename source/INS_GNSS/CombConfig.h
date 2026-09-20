#ifndef COMBCONFIG_H
#define COMBCONFIG_H
#include<iostream>
#include<fstream>
#include<sstream>
#include<iomanip>
#include"Param.h"

namespace CombineNav
{
	struct Config//组合导航配置
	{
		struct Methods//组合导航方法
		{
			bool useRealTime;//是否使用实时组合导航
		}methods;


		struct FilePaths//文件路径
		{
			std::string imufilepath;//IMU数据文件路径
			std::string gnssfilepath;//GNSS数据文件路径
			std::string odofilepath;//里程计数据文件路径
			std::string outputfolder;//结果输出文件夹路径
		}files;

		struct Switches//功能开关
		{
			bool usegnssvel;//是否使用GNSS速度
			bool useodo;//是否使用里程计
			bool usenhc;//是否使用NHC约束
			bool usezupt;//是否使用零速约束
			bool backfilter;//是否使用后向滤波
		}switches;

		struct TimeConfig//时间配置
		{
			double starttime;//开始时间
			double endtime;//结束时间
		} time;


		struct InitialState //初始状态
		{
			double init_pos[3];		// [deg, deg, m]
			double init_vel[3];		// [m/s]
			double init_att[3];		// [deg]

			double init_pos_std[3];  // [m]
			double init_vel_std[3];  // [m/s]
			double init_att_std[3];  // [deg]
		} init_state;

		
		struct InitIMUErrorParams // 初始IMU误差参数
		{
			double init_gyrobias[3];      // [deg/h]
			double init_accbias[3];     // [mGal]
			double init_gyroscale[3];     // [ppm]
			double init_accscale[3];    // [ppm]

			double init_gyrobias_std[3];   // [deg/h]
			double init_accbias_std[3];  // [mGal]
			double init_gyroscale_std[3];  // [ppm]
			double init_accscale_std[3]; // [ppm]
		} init_imu_errors;

		struct IMUNoiseParams // IMU噪声参数
		{
			double gyro_arw;       // [deg/sqrt(h)]
			double accel_vrw;      // [m/s/sqrt(h)]
			double gyrobias_std;  // [deg/h]
			double accbias_std; // [mGal]
			double gyroscale_std; // [ppm]
			double accscale_std;// [ppm]
			double corrtime;      // [h]
		} imu_noise;

		struct InstallationParams//安装参数
		{
			double gnss_lever[3];      // [m]
			double odo_lever[3];      // [m]
			double gnss_install_angle[3];  // [deg]
			double odo_install_angle[3];  // [deg]
			
		} installation;

		
		struct ObservationNoise// 观测噪声
		{
			double zupt_std;      // ZUPT约束标准差
		} obs_noise;

		struct Thresholds // 阈值参数
		{
			double gnss_pos_threshold; // GNSS位置阈值
			double gnss_vel_threshold; // GNSS速度阈值
			double odo_vel_threshold;  // 里程计速度阈值
			double nhc_threshold;      // NHC约束阈值
			double zupt_acc_threshold;     // ZUPT加速度约束阈值
			double zupt_gyro_threshold;    // ZUPT陀螺约束阈值
		} thresholds;


		// 方法
		Config(); // 默认构造函数
		bool Validate() const; // 验证配置有效性
		void Print() const;    // 打印配置信息
		bool LoadFromFile(const std::string& filename);
		bool SaveToFile(const std::string& filename) const;

	private:
		// 辅助函数
		template<typename T>
		void ReadValue(std::ifstream& file, const std::string& key, T& value) const;
		template<typename T>
		void WriteValue(std::ofstream& file, const std::string& key, const T& value) const;
		void ReadArray(std::ifstream& file, const std::string& key, double* arr, int size);
		void WriteArray(std::ofstream& file, const std::string& key, const double* arr, int size) const;
	};




	// 配置管理器类
	class ConfigManager 
	{
	public:
		

	private:
		
	};



}



#endif // COMBCONFIG_H