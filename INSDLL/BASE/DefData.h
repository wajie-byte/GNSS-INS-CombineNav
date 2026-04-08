#ifndef DEFDATA_H
#define DEFDATA_H
#include<iostream>
#include<vector>
#include<string>

//--------------------XW_GI7681--------------------
const static double XW_GI7681_sam_freq = 100.0;//三轴转台采样率
const static double XW_GI7681_acc_scale = 1.5258789063e-6;//加速度计因子
const static double XW_GI7681_gyro_scale = 1.0850694444e-7;//陀螺仪因子

//--------------------SPAN_100C-------------------
const static double SPAN_100C_sam_freq = 200.0;//三轴转台采样率
const static double SPAN_100C_acc_scale = 2e-8;//加速度计因子
const static double SPAN_100C_gyro_scale = 1e-9;//陀螺仪因子


const static double rads2degh_scale = 206264.8062471;//rad/s转deg/h
const static double mss2mGal_scale = 100000.0;//m/s/s转mGal

const static double local_fai = 30.531651244;//当地纬度deg
const static double h = 28.2134;//椭球高m
const static double g = 9.7936174;//当地重力加速度m/s_2
const static double Omegae = 7.292115e-5;//地球自转角速度rad/s
const static double pi = 3.141592653589793;
const static double rad2deg_scale = 180.0 / pi;
const static double deg2rad_scale = pi / 180.0;

//---------------------WGS84椭球参数-------------------
const static double WGS84_a = 6378137.0;
const static double WGS84_b = 6356752.3142;
const static double WGS84_f = 0.00335281066474748;
const static double WGS84_e1_2 = 0.00669437999013;









namespace INS
{
	//---------------------------数据结构定义---------------------------
	//惯导数据
	struct IMUdata
	{
		std::string header;
		double GPStime;
		double IMUtime;
		//----------------------------原始数据--------------------------
		double accx; double accy; double accz;//比力值m/s^2
		double gyrox; double gyroy; double gyroz;//角速度值rad/s

		IMUdata()
		{
			header = "UnKnown";
			GPStime = 0;
			IMUtime = 0;
			accx = 0; accy = 0; accz = 0;
			gyrox = 0; gyroy = 0; gyroz = 0;

		}
		IMUdata(std::string hd, double gtime, double itime, double ax, double ay, double az, double gx, double gy, double gz)
		{
			header = hd;
			GPStime = gtime;
			IMUtime = itime;
			accx = ax; accy = ay; accz = az;
			gyrox = gx; gyroy = gy; gyroz = gz;
		}

	};


	//六个位置的加速度计数据
	struct Acc_sixpos
	{
		std::vector<IMUdata> x_up;
		std::vector<IMUdata> x_down;
		std::vector<IMUdata> y_up;
		std::vector<IMUdata> y_down;
		std::vector<IMUdata> z_up;
		std::vector<IMUdata> z_down;
		Acc_sixpos()
		{
			x_up.clear(); x_down.clear();
			y_up.clear(); y_down.clear();
			z_up.clear(); z_down.clear();
		}

	};

	//两个位置的陀螺仪数据
	struct Groy_twopos
	{
		std::vector<IMUdata> pos_zheng;
		std::vector<IMUdata> pos_fan;
		Groy_twopos()
		{
			pos_zheng.clear();
			pos_fan.clear();
		}
	};

	//姿态角
	struct Attitude_angle
	{
		double pusai;//航向角
		double theta;//俯仰角
		double fai;//横滚角
		Attitude_angle()
		{
			pusai = theta = fai = 0.0;
		}
		Attitude_angle(double p, double t, double f)
		{
			pusai = p;
			theta = t;
			fai = f;
		}

	};


#pragma pack(push,1)
	struct IMUDataEpoch
	{
		double time;//GNSS sec 历元s
		double gyro[3];//角增量rad
		double accl[3];//速度增量m/s
		// 默认构造函数
		IMUDataEpoch()
		{
			time = 0.0;
			for (int i = 0; i < 3; i++)
			{
				gyro[i] = 0.0;
				accl[i] = 0.0;
			}
		}
	};
#pragma pack(pop)

#pragma pack(push,1)
	struct IMUResultEpoch
	{
		double time;//GNSS sec 历元s
		double BLH[3];//位置经纬高rad/m
		double Vn[3];//速度在n系下投影m/s(北东地)
		double YPR[3];//姿态角rad
		IMUResultEpoch()
		{
			time = 0.0;
			for (int i = 0; i < 3; i++)
			{
				BLH[i] = 0.0;
				Vn[i] = 0.0;
				YPR[i] = 0.0;
			}
		}
		IMUResultEpoch(double t, double blh[3], double vn[3], double ypr[3])
		{
			time = t;
			for (int i = 0; i < 3; i++)
			{
				BLH[i] = blh[i];
				Vn[i] = vn[i];
				YPR[i] = ypr[i];
			}
		}
	};
#pragma pack(pop)

	std::vector<IMUDataEpoch> CutImuDataByTime(double starttime, double endtime, const std::vector<IMUDataEpoch>& imudata);//按时间截取数据
}

//---------------------------函数声明-----------------------------
//原始IMU数据积分
void Integrate_Rawdata(const std::vector<INS::IMUdata>& raw_data, std::vector<INS::IMUDataEpoch>& Integrate_data);
//轴系调整，右前上转为前右下
void RFU2FRD(const std::vector<INS::IMUDataEpoch>& raw_data, std::vector<INS::IMUDataEpoch>& adjusted_data);
extern double normalize_angle(double angle);
extern double normalize_360_angle(double angle);
//轴系调整，右前上转为前右下
void RFU2FRD(const std::vector<INS::IMUdata>& raw_data, std::vector<INS::IMUdata>& adjusted_data);



#endif // !DEFDATA_H

