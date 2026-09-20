#include"DefData.h"

//原始IMU数据积分
/*
In:
	std::vector<IMUdata> raw_data					//原始IMU数据
Out:
	std::vector<IMUDataEpoch> Integrate_data			//积分后的IMU数据
*/
void Integrate_Rawdata(const std::vector<INS::IMUdata>& raw_data, std::vector<INS::IMUDataEpoch>& Integrate_data)
{
	INS::IMUDataEpoch epoch_data;
	double dt = 0.0;
	//遍历原始数据，进行积分
	for (int i = 0; i < raw_data.size() - 1; i++)
	{
		dt = raw_data[i + 1].IMUtime - raw_data[i].IMUtime;
		epoch_data.time = raw_data[i].GPStime;
		epoch_data.gyro[0] = (raw_data[i].gyrox + raw_data[i + 1].gyrox) / 2 * dt;
		epoch_data.gyro[1] = (raw_data[i].gyroy + raw_data[i + 1].gyroy) / 2 * dt;
		epoch_data.gyro[2] = (raw_data[i].gyroz + raw_data[i + 1].gyroz) / 2 * dt;
		epoch_data.accl[0] = (raw_data[i].accx + raw_data[i + 1].accx) / 2 * dt;
		epoch_data.accl[1] = (raw_data[i].accy + raw_data[i + 1].accy) / 2 * dt;
		epoch_data.accl[2] = (raw_data[i].accz + raw_data[i + 1].accz) / 2 * dt;
		Integrate_data.push_back(epoch_data);
	}
	std::cout << "原始IMU数据积分完成！共得到" << Integrate_data.size() << "条数据" << std::endl;
}

//轴系调整，右前上转为前右下
/*
In:
	std::vector<IMUDataEpoch> raw_data				//积分后的IMU数据
Out:
	std::vector<IMUDataEpoch> adjusted_data			//轴系调整后的IMU数据
*/
void RFU2FRD(const std::vector<INS::IMUDataEpoch>& raw_data, std::vector<INS::IMUDataEpoch>& adjusted_data)
{
	adjusted_data.clear();
	INS::IMUDataEpoch adjusted_epoch;
	for (int i = 0; i < raw_data.size(); i++)
	{
		adjusted_epoch.time = raw_data[i].time;
		//陀螺仪数据调整
		adjusted_epoch.gyro[0] = raw_data[i].gyro[1];      //前
		adjusted_epoch.gyro[1] = raw_data[i].gyro[0];      //右
		adjusted_epoch.gyro[2] = -raw_data[i].gyro[2];     //下
		//加速度计数据调整
		adjusted_epoch.accl[0] = raw_data[i].accl[1];      //前
		adjusted_epoch.accl[1] = raw_data[i].accl[0];      //右
		adjusted_epoch.accl[2] = -raw_data[i].accl[2];     //下
		adjusted_data.push_back(adjusted_epoch);
	}
	std::cout << "轴系调整完成！共调整" << adjusted_data.size() << "条数据" << std::endl;
}


//轴系调整，右前上转为前右下
/*
In:
	std::vector<IMUDataEpoch> raw_data				//积分后的IMU数据
Out:
	std::vector<IMUDataEpoch> adjusted_data			//轴系调整后的IMU数据
*/
void RFU2FRD(const std::vector<INS::IMUdata>& raw_data, std::vector<INS::IMUdata>& adjusted_data)
{
	adjusted_data.clear();
	INS::IMUdata adjusted_epoch;
	for (int i = 0; i < raw_data.size(); i++)
	{
		adjusted_epoch.GPStime = raw_data[i].GPStime;
		adjusted_epoch.IMUtime = raw_data[i].IMUtime;
		//陀螺仪数据调整
		adjusted_epoch.gyrox = raw_data[i].gyroy;      //前
		adjusted_epoch.gyroy = raw_data[i].gyrox;      //右
		adjusted_epoch.gyroz = -raw_data[i].gyroz;     //下
		//加速度计数据调整
		adjusted_epoch.accx = raw_data[i].accy;      //前
		adjusted_epoch.accy = raw_data[i].accx;      //右
		adjusted_epoch.accz = -raw_data[i].accz;     //下
		adjusted_data.push_back(adjusted_epoch);
	}
	std::cout << "轴系调整完成！共调整" << adjusted_data.size() << "条数据" << std::endl;
}

// 角度规范化函数


double normalize_angle(double angle)
{
	double result = angle;
	while (result > pi) result -= 2.0 * pi;
	while (result < -pi) result += 2.0 * pi;
	return result;
}

double normalize_360_angle(double angle)
{
	double result = angle;
	while (result < 0) result += 2.0 * pi;
	while (result >= 2.0 * pi) result -= 2.0 * pi;
	return result;
}

