#include"BASE/DefData.h"
#include"IMU/Cal.h"
#include"INS/Dynamic.h"
#include"INS/INS.h"
int main_ins()
{
	//------------------------加速度计六位置法----------------------------
	// //------------------------加速度计数据补偿-----------------------------
	//Acc_six_func();



	//------------------------陀螺两位置法----------------------------------
	//Groy_twoxyz_func();



	//------------------------静态解析粗对准--------------------------------
	SPCA_func();





	//------------------------参考数据纯惯导动态导航定位-----------------------------

	/*
	vector<IMUDataEpoch> ImuDataEpoch;
	const char* BinFile = "D:\\visual studio\\INS\\IMUdata\\Bin\\IMU.bin";
	const char* BindisplayFile = "D:\\visual studio\\INS\\display\\IMU_display.ASC";
	const char* BinResultFile = "D:\\visual studio\\INS\\IMUdata\\Bin\\PureINS.bin";
	const char* BinResultdisplayFile = "D:\\visual studio\\INS\\display\\PureINS_display.ASC";
	//读取二进制IMU数据文件
	IMUBinFileRead(BinFile, ImuDataEpoch);
	//输出二进制IMU数据文件
	//IMUBindatadisplay(BindisplayFile, ImuDataEpoch);
	//读取二进制纯惯导解算结果文件
	BinResultRead(BinResultFile, BinResultdisplayFile);

	//设置惯导初始参数
	IMUResultEpoch IMUstart;
	IMUstart.time = 91620.0; //GNSS秒
	IMUstart.BLH[0] = 23.1373950708 * deg2rad_scale; //纬度rad
	IMUstart.BLH[1] = 113.3713651222 * deg2rad_scale; //经度rad
	IMUstart.BLH[2] = 2.175; //高程m
	IMUstart.Vn[0] = 0.0; //北向速度m/s
	IMUstart.Vn[1] = 0.0; //东向速度m/s
	IMUstart.Vn[2] = 0.0; //垂向速度m/s
	IMUstart.YPR[0] = -75.7498049314083 * deg2rad_scale; //航向角rad
	IMUstart.YPR[1] = -2.14251290749072 * deg2rad_scale; //俯仰角rad
	IMUstart.YPR[2] = 0.0107951084511778 * deg2rad_scale; //横滚角rad
	//纯惯导解算
	vector<IMUResultEpoch> IMUresult;
	InertialNavigation(ImuDataEpoch, IMUstart, IMUresult);
	//输出纯惯导解算结果
	const char* IMUResultFile = "D:\\visual studio\\INS\\display\\INS_result.ASC";
	IMUResultDisplay(IMUResultFile, IMUresult);

	//------------------------计算与参考结果的差值---------------------------
	//读取参考结果
	vector<IMUResultEpoch> IMUref;
	INSResultRead(BinResultFile, IMUref);
	//计算差值
	vector<IMUResultEpoch> IMUdiff;
	CalIMUResultDiff(IMUresult, IMUref, IMUdiff);
	//输出差值结果
	const char* IMUdiffFile = "D:\\visual studio\\INS\\display\\INS_diff.ASC";
	IMUResultDisplay(IMUdiffFile, IMUdiff);
	*/



	//--------------------------小推车实验数据纯惯导动态导航定位-----------------------------


	/*
	vector<IMUdata> raw_IMUdata;
	const char* raw_IMUfile = "D:\\visual studio\\INS\\IMUdata\\car\\car.ASC";
	//读取原始IMU数据文件
	//惯导接收机数据读取参数
	double Para[3] = { XW_GI7681_sam_freq,XW_GI7681_acc_scale,XW_GI7681_gyro_scale };
	IMUFileRead(raw_IMUfile, raw_IMUdata, Para[0], Para[1], Para[2]);
	//原始IMU数据展示
	const char* raw_IMUdisplayfile = "D:\\visual studio\\INS\\display\\car_INS\\car_display.ASC";
	DataDisplay(raw_IMUdisplayfile, raw_IMUdata);
	vector<IMUDataEpoch> car_ImuDataEpoch;
	//原始IMU数据积分
	Integrate_Rawdata(raw_IMUdata, car_ImuDataEpoch);
	//积分后IMU数据输出
	const char* car_ImuDataEpochfile = "D:\\visual studio\\INS\\display\\car_INS\\car_ImuDataEpoch.ASC";
	IMUBindatadisplay(car_ImuDataEpochfile, car_ImuDataEpoch);

	//轴系调整
	vector<IMUDataEpoch> car_ImuDataEpoch_adjust;
	RFU2FRD(car_ImuDataEpoch, car_ImuDataEpoch_adjust);
	//轴系调整后IMU数据输出
	const char* car_ImuData_ajustedfile = "D:\\visual studio\\INS\\display\\car_INS\\car_ImuData_ajusted.ASC";
	IMUBindatadisplay(car_ImuData_ajustedfile, car_ImuDataEpoch_adjust);



	//设置惯导初始参数
	IMUResultEpoch car_INSstart;
	car_INSstart.time = 447390.010010; //GNSS秒

	//初始位置赋值
	car_INSstart.BLH[0] = 30.5280788150 * deg2rad_scale;
	car_INSstart.BLH[1] = 114.3557756049 * deg2rad_scale;
	car_INSstart.BLH[2] = 23.2749;



	Attitude_angle car_initial_angle;
	car_initial_angle.pusai = 272.59242 * deg2rad_scale; //航向角rad
	car_initial_angle.theta = 0.31411 * deg2rad_scale; //俯仰角rad
	car_initial_angle.fai = 0.38616 * deg2rad_scale; //横滚角rad

	double cuttime = 5 * 60.0 * XW_GI7681_sam_freq; //静态解析时间五分钟采样数据


	//初始姿态角赋值
	car_INSstart.YPR[0] = car_initial_angle.pusai;	//航向角rad
	car_INSstart.YPR[1] = car_initial_angle.theta;	//俯仰角rad
	car_INSstart.YPR[2] = car_initial_angle.fai;	//横滚角rad

	//初始速度赋值
	car_INSstart.Vn[0] = 0.0; //北向速度
	car_INSstart.Vn[1] = 0.0; //东向速度
	car_INSstart.Vn[2] = 0.0; //垂向速度

	//纯惯导解算
	vector<IMUResultEpoch>INSResult;
	InertialNavigation(car_ImuDataEpoch_adjust, car_INSstart, INSResult);

	//输出结果文件
	const char* INSResultFile = "D:\\visual studio\\INS\\display\\car_INS\\INSResult_zerofix.ASC";
	IMUResultDisplay_RPY(INSResultFile, INSResult);
	//------------------------计算与参考结果的差值---------------------------
	//读取参考结果
	const char* INSrefFile = "D:\\visual studio\\INS\\display\\car_INS\\RefData\\Ref_ajust.nav";
	vector<IMUResultEpoch> INSref;
	INSRefResultRead(INSrefFile, INSref);
	//计算差值
	vector<IMUResultEpoch> INSdiff;
	CalIMUResultDiff(INSResult, INSref, INSdiff);
	//输出差值结果
	const char* IMUdiffFile = "D:\\visual studio\\INS\\display\\car_INS\\INS_zerofix_diff.ASC";
	IMUResultDisplay(IMUdiffFile, INSdiff);
	*/



	//---------------------------------DeBug----------------------------------
	/*
	// 测试用例1：普通角度
	double Yaw = 1.2;
	double Pitch = 0.3;
	double Roll = 0.5; // 弧度
	double y = 0.0;
	double p = 0.0;
	double r = 0.0;
	Mat q(4, 1);
	q = euler2quaternion(Yaw, Pitch, Roll);
	quaternion2euler(q, y, p, r);
	cout << y << " " << p << " " << r << endl;
	//2
	Yaw = 2.8, Pitch = -1.1, Roll = -2.5;
	q = euler2quaternion(Yaw, Pitch, Roll);
	quaternion2euler(q, y, p, r);
	cout << y << " " << p << " " << r << endl;
	// 测试用例3：正90度万向节锁
	Yaw = 1.5, Pitch = pi / 2 - 1e-7, Roll = 0.8;
	q = euler2quaternion(Yaw, Pitch, Roll);
	quaternion2euler(q, y, p, r);
	cout << y << " " << p << " " << r << endl;
	Yaw = 1.5+pi, Pitch = 0.3+pi, Roll = 0.8+pi;
	q = euler2quaternion(Yaw, Pitch, Roll);
	quaternion2euler(q, y, p, r);
	cout << y << " " << p << " " << r << endl;
	*/
	return 0;
}