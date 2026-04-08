#include"Cal.h"
void CalMeanAcc(const std::vector<INS::IMUdata>& IMUdata, double onepos[3],double Cuttime)
{
	for (int i = 0; i < Cuttime; i++)
	{
		onepos[0] += IMUdata[i].accx;
		onepos[1] += IMUdata[i].accy;
		onepos[2] += IMUdata[i].accz;
	}
	onepos[0] = onepos[0] / Cuttime;
	onepos[1] = onepos[1] / Cuttime;
	onepos[2] = onepos[2] / Cuttime;
	return;
}

void CalMeanGroy(const std::vector<INS::IMUdata>& IMUdata, double onepos[3], double Cuttime)
{
	for (int i = 0; i < Cuttime; i++)
	{
		onepos[0] += IMUdata[i].gyrox;
		onepos[1] += IMUdata[i].gyroy;
		onepos[2] += IMUdata[i].gyroz;
	}
	onepos[0] = onepos[0] / Cuttime;
	onepos[1] = onepos[1] / Cuttime;
	onepos[2] = onepos[2] / Cuttime;
	return;
}

void CalMeanAcc_sixpos(const INS::Acc_sixpos& acc_sixpos, double sixpos[6][3],double Cuttime)
{
	CalMeanAcc(acc_sixpos.x_up, sixpos[0], Cuttime);
	CalMeanAcc(acc_sixpos.x_down, sixpos[1], Cuttime);
	CalMeanAcc(acc_sixpos.y_up, sixpos[2], Cuttime);
	CalMeanAcc(acc_sixpos.y_down, sixpos[3], Cuttime);
	CalMeanAcc(acc_sixpos.z_up, sixpos[4], Cuttime);
	CalMeanAcc(acc_sixpos.z_down, sixpos[5], Cuttime);
	std::cout << "六个位置的加速度计均值计算完成！" << std::endl;
	return;
}


void Demarcate(const double sixpos[6][3], double acc_bias[3], double acc_scale[3],double acc_m[6])
{
	Mat L(3, 6);
	Mat M(3, 4);
	Mat A(4, 6);
	//L矩阵赋值
	for (int i = 0; i < 6; i++)
	{
		L(0, i) = sixpos[i][0];
		L(1, i) = sixpos[i][1];
		L(2, i) = sixpos[i][2];
	}
	//A矩阵赋值
	A(0, 0) = g; A(0, 1) = -g;
	A(1, 2) = g; A(1, 3) = -g;
	A(2, 4) = g; A(2, 5) = -g;
	for (int i = 0; i < 6; i++)
	{
		A(3, i) = 1.0;
	}
	
	//矩阵输出
	std::cout << "L矩阵为：" << std::endl;
	for (int i = 0; i < L.rows(); i++)
	{
		for (int j = 0; j < L.cols(); j++)
		{
			std::cout << L(i, j) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "A矩阵为：" << std::endl;
	for (int i = 0; i < A.rows(); i++)
	{
		for (int j = 0; j < A.cols(); j++)
		{
			std::cout << A(i, j) << " ";
		}
		std::cout << std::endl;
	}
	
	

	//M矩阵计算
	Mat N = A * A.transpose();
	Mat N_inv = N.inverse();
	M = L * A.transpose() * N_inv;

	// 新增：计算残差
	Mat L_calc = M * A;  // 计算值
	Mat residual = L - L_calc;  // 残差

	std::cout << "标定残差：" << std::endl;
	for (int i = 0; i < residual.rows(); i++)
	{
		for (int j = 0; j < residual.cols(); j++)
		{
			std::cout << residual(i, j) << " ";
		}
		std::cout << std::endl;
	}
	acc_bias[0] = M(0, 3); acc_bias[1] = M(1, 3); acc_bias[2] = M(2, 3);
	acc_scale[0] = M(0, 0); acc_scale[1] = M(1, 1); acc_scale[2] = M(2, 2);
	acc_m[0] = M(0, 1); acc_m[1] = M(0, 2); acc_m[2] = M(1, 2);
	acc_m[3] = M(1, 0); acc_m[4] = M(2, 0); acc_m[5] = M(2, 1);
	std::cout << "加速度计标定参数计算完成！" << std::endl;
	
}

void DisplayAccCalResult(INS::Acc_sixpos& acc_sixpos, INS::Acc_sixpos& fixed_acc_sixpos,double Cuttime)
{
	double sixpos[6][3] = { 0 };
	CalMeanAcc_sixpos(acc_sixpos, sixpos, Cuttime);
	std::cout << "六个位置的加速度计均值为：" << std::endl;
	for (int i = 0; i < 6; i++)
	{
		std::cout << "accx=" << sixpos[i][0] << " " << "accy=" << sixpos[i][1] << " " << "accz=" << sixpos[i][2] << std::endl;
	}
	double acc_bias[3] = { 0.0 };
	double acc_scale[3] = { 0.0 };
	double acc_m[6] = { 0.0 };
	Demarcate(sixpos, acc_bias, acc_scale, acc_m);
	std::cout << "加速度计标定参数为：" << std::endl;
	std::cout << "acc_bias_x=" << acc_bias[0] << "m/s_2 " << "acc_bias_y=" << acc_bias[1] << "m/s_2 " << "acc_bias_z=" << acc_bias[2] << "m/s_2" << std::endl;
	double bias_mGal[3] = { 0 };
	bias_mGal[0] = acc_bias[0] * mss2mGal_scale;
	bias_mGal[1] = acc_bias[1] * mss2mGal_scale;
	bias_mGal[2] = acc_bias[2] * mss2mGal_scale;
	std::cout << "acc_bias_x=" << bias_mGal[0] << "mGal " << "acc_bias_y=" << bias_mGal[1] << "mGal " << "acc_bias_z=" << bias_mGal[2] << "mGal" << std::endl;
	std::cout << "acc_scale_x=" << acc_scale[0] << " " << "acc_scale_y=" << acc_scale[1] << " " << "acc_scale_z=" << acc_scale[2] << std::endl;
	std::cout << "acc_scale_bias_x=" << acc_scale[0] - 1.0 << " " << "acc_scale_bias_y=" << acc_scale[1] - 1.0 << " " << "acc_scale_bias_z=" << acc_scale[2] - 1.0 << std::endl;
	std::cout << "acc_m_a_xy=" << acc_m[0] << " " << "acc_m_a_xz=" << acc_m[1] << " " << "acc_m_a_yz=" << acc_m[2] << std::endl;
	std::cout << "acc_m_a_yx=" << acc_m[3] << " " << "acc_m_a_zx=" << acc_m[4] << " " << "acc_m_a_zy=" << acc_m[5] << std::endl;
	Fix_sixposdata(acc_sixpos, fixed_acc_sixpos, acc_bias, acc_scale, acc_m);
}

void CalGroyDeg(const std::vector<INS::IMUdata>& IMUdata, double onepos[3], int Cuttime,double sam_freq)
{
	for (int i = 0; i < Cuttime-1; i++)
	{
		onepos[0] += IMUdata[i].gyrox / sam_freq;
		onepos[1] += IMUdata[i].gyroy / sam_freq;
		onepos[2] += IMUdata[i].gyroz / sam_freq;
	}
	
	std::cout << onepos[0] << " " << onepos[1] << " " << onepos[2] << std::endl;
	return;
}

int GetGroyCuttime(int i, int j)
{
	return (i < j) ? i : j;
}


void CalGroyResult(const double sixpos[6][3],double bias[3],double scale[3],double xCuttime,double yCuttime,double zCuttime,double freq)
{
	//计算得到的bias单位为rad/s,scale单位为1
	bias[0] = (sixpos[0][0] + sixpos[1][0]) / (xCuttime * 2 / freq) - Omegae * sin(local_fai * deg2rad_scale);
	bias[1] = (sixpos[2][1] + sixpos[3][1]) / (yCuttime * 2 / freq) - Omegae * sin(local_fai * deg2rad_scale);
	bias[2] = (sixpos[4][2] + sixpos[5][2]) / (zCuttime * 2 / freq) - Omegae * sin(local_fai * deg2rad_scale);
	scale[0] = (sixpos[0][0] - sixpos[1][0]) / (2 * 2 * pi);
	scale[1] = (sixpos[2][1] - sixpos[3][1]) / (2 * 2 * pi);
	scale[2] = (sixpos[4][2] - sixpos[5][2]) / (2 * 2 * pi);

}

void DisplayGroyCalResult(INS::Groy_twopos& gyro_xpos, INS::Groy_twopos& gyro_ypos, INS::Groy_twopos& gyro_zpos,double sam_freq)
{
	double sixpos[6][3] = { 0 };
	int xCuttime = GetGroyCuttime(gyro_xpos.pos_zheng.size(), gyro_xpos.pos_fan.size());
	int yCuttime = GetGroyCuttime(gyro_ypos.pos_zheng.size(), gyro_ypos.pos_fan.size());
	int zCuttime = GetGroyCuttime(gyro_zpos.pos_zheng.size(), gyro_zpos.pos_fan.size());
	CalGroyDeg(gyro_xpos.pos_zheng, sixpos[0], xCuttime, sam_freq);
	CalGroyDeg(gyro_xpos.pos_fan, sixpos[1], xCuttime, sam_freq);
	CalGroyDeg(gyro_ypos.pos_zheng, sixpos[2], yCuttime, sam_freq);
	CalGroyDeg(gyro_ypos.pos_fan, sixpos[3], yCuttime, sam_freq);
	CalGroyDeg(gyro_zpos.pos_zheng, sixpos[4], zCuttime, sam_freq);
	CalGroyDeg(gyro_zpos.pos_fan, sixpos[5], zCuttime, sam_freq);
	double bias[3] = { 0 };
	double scale[3] = { 0 };
	
	CalGroyResult(sixpos, bias, scale, xCuttime, yCuttime, zCuttime, sam_freq);
	std::cout << "陀螺标定参数为：" << std::endl;
	std::cout << "gyro_bias_x=" << bias[0] << "rad/s " << "gyro_bias_y=" << bias[1] << "rad/s " << "gyro_bias_z=" << bias[2] << "rad/s" << std::endl;
	double bias_degh[3] = { 0 };
	bias_degh[0] = bias[0] * rads2degh_scale;
	bias_degh[1] = bias[1] * rads2degh_scale;
	bias_degh[2] = bias[2] * rads2degh_scale;
	std::cout << "gyro_bias_x=" << bias_degh[0] << "deg/h " << "gyro_bias_y=" << bias_degh[1] << "deg/h " << "gyro_bias_z=" << bias_degh[2] << "deg/h" << std::endl;
	std::cout << "gyro_scale_x=" << scale[0] << " " << "gyro_scale_y=" << scale[1] << " " << "gyro_scale_z=" << scale[2] << std::endl;
	std::cout << "gyro_scale_bias_x=" << scale[0] - 1.0 << " " << "gyro_scale_bias_y=" << scale[1] - 1.0 << " " << "gyro_scale_bias_z=" << scale[2] - 1.0 << std::endl;
}


void SPCACal(const std::vector<INS::IMUdata>& SPCA_data, INS::Attitude_angle& angle,double Cuttime)
{
	//--------------对加速度计输出的比力和陀螺输出的角速度求均值-----------
	double acc_mean[3] = { 0.0,0.0,0.0 };
	double gyro_mean[3] = { 0.0,0.0,0.0 };
	CalMeanAcc(SPCA_data, acc_mean, Cuttime);
	CalMeanGroy(SPCA_data, gyro_mean, Cuttime);


	//----------------------构造矩阵，向量-------------------------
	//w_n_ie,w_b_ie,C_n_b,其中n为上标，b为下标
	//g_n,v_n,g_b,v_b,其中n,b为上标
	Mat g_n(3, 1);
	Mat w_n_ie(3, 1);
	Mat v_n(3, 1);
	Mat g_b(3, 1);
	Mat w_b_ie(3, 1);
	Mat v_b(3, 1);
	Mat C_n_b(3, 3);

	//---------------------矩阵初始化,赋值-------------------------

	//g_n
	g_n(2, 0) = g;
	//w_n_ie
	w_n_ie(0, 0) = Omegae * cos(local_fai * deg2rad_scale);
	w_n_ie(2, 0) = Omegae * sin(local_fai * deg2rad_scale);
	//v_n=g_n x w_n_ie
	v_n = Mat::cross(g_n, w_n_ie);
	//g_b
	g_b(0, 0) = -acc_mean[0];
	g_b(1, 0) = -acc_mean[1];
	g_b(2, 0) = -acc_mean[2];
	//w_b_ie
	w_b_ie(0, 0) = gyro_mean[0];
	w_b_ie(1, 0) = gyro_mean[1];
	w_b_ie(2, 0) = gyro_mean[2];
	//v_b=g_b x w_b_ie
	v_b = Mat::cross(g_b, w_b_ie);

	
	//-------------------矢量正交化和单位化---------------------
	// 检查 v 系向量是否平行
	if (Mat::cross(g_n, w_n_ie).norm() < 1e-10)
	{
		throw std::runtime_error("导航系：重力向量和角速度向量平行，无法构建正交系");
	}

	// 检查 w 系向量是否平行  
	if (Mat::cross(g_b, w_b_ie).norm() < 1e-10)
	{
		throw std::runtime_error("载体系：重力向量和角速度向量平行，无法构建正交系");
	}
	//定义向量
	Mat v_g(3, 1);
	Mat v_w(3, 1);
	Mat v_gw(3, 1);
	Mat w_g(3, 1);
	Mat w_w(3, 1);
	Mat w_gw(3, 1);
	//正交化和单位化
	v_g = g_n / g_n.norm();                          
	v_w = Mat::cross(v_g, w_n_ie);                   
	v_w = v_w / v_w.norm();                          
	v_gw = Mat::cross(v_w, v_g);                     

	w_g = g_b / g_b.norm();
	w_w = Mat::cross(w_g, w_b_ie);
	w_w = w_w / w_w.norm();
	w_gw = Mat::cross(w_w, w_g);                     



	//----------------------构造辅助矩阵-------------------------
	Mat gwv_n(3, 3);
	Mat gwv_b(3, 3);
	//初始化
	for (int i = 0; i < 3; i++)
	{
		//gwv_n
		gwv_n(i, 0) = v_g(i, 0);
		gwv_n(i, 1) = v_w(i, 0);
		gwv_n(i, 2) = v_gw(i, 0);
		//gwv_b
		gwv_b(0, i) = w_g(i, 0);
		gwv_b(1, i) = w_w(i, 0);
		gwv_b(2, i) = w_gw(i, 0);
	}
	// 验证构造的矩阵是否正交

	//cout << "gwv_n * gwv_n^T: " << '\n' << (gwv_n * gwv_n.transpose()) << std::endl;
	//cout << "gwv_b * gwv_b^T: " << '\n' << (gwv_b * gwv_b.transpose()) << std::endl;
	
	//-------------------计算姿态矩阵----------------------
	
	C_n_b = gwv_n * gwv_b;
	// 检查 C_n_b 是否正交
	Mat I_check = C_n_b * C_n_b.transpose();
	//std::cout << "C_n_b * C_n_b^T: "<<'\n' << I_check << std::endl;

	// 检查行列式（应该是+1）
	double det = C_n_b.determinant();
	//std::cout << "det(C_n_b): " << '\n' << det << std::endl;
	//-------------------计算姿态角-------------------------
	double theta = 0.0;
	double fai = 0.0;
	double pusai = 0.0;
	//定义辅助变量
	double C11 = C_n_b(0, 0);
	double C12 = C_n_b(0, 1);
	double C21 = C_n_b(1, 0);
	double C32 = C_n_b(2, 1);
	double C33 = C_n_b(2, 2);
	double C31 = C_n_b(2, 0);
	
	//计算姿态角
	//其中theta要满足绝对值小于等于pi/2,fai和pusai绝对值小于等于pi
	theta =  atan2(-C31 , (sqrt(C32 * C32 + C33 * C33)));
	fai = atan2(C32, C33);
	pusai = atan2(C21, C11);
	

	// 万向节锁处理（俯仰角接近±90°）
	const double EPS = 1e-6;
	bool is_gimbal_lock = (fabs(C31) > 1 - EPS);
	if (is_gimbal_lock)
	{
		// 万向节锁情况：按照惯例设置横滚角为0
		fai = 0.0;

		// 正确的万向节锁航向角计算公式
		if (theta > 0)  // 俯仰角接近+90°
		{
			pusai = atan2(C12, C11);
		}
		else  // 俯仰角接近-90°
		{
			pusai = atan2(-C12, C11);
		}
	}

	//---------------------角度规范化---------------------
	

	
	//没有考虑万向节锁的问题
	fai = normalize_angle(fai);
	pusai = normalize_angle(pusai);
	// 防止浮点误差导致的轻微越界
	if (theta > pi / 2) theta = pi / 2;
	else if (theta < -pi / 2) theta = -pi / 2;
	angle.theta = theta;
	angle.fai = fai;
	angle.pusai = pusai;
	

}




//加速度计六位置法程序执行
void Acc_six_func()
{
	INS::Acc_sixpos acc_sixpos;
	INS::Acc_sixpos fixed_acc_sixpos;
	const char* Accdataname[6] =
	{
		"D:\\visual studio\\IMU\\IMUdata\\x_up.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\x_down.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\y_up.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\y_down.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\z_up.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\z_down.ASC"
	};


	const char* Accdisplayname[6] =
	{
		"D:\\visual studio\\IMU\\display\\x_up_display.ASC",
		"D:\\visual studio\\IMU\\display\\x_down_display.ASC",
		"D:\\visual studio\\IMU\\display\\y_up_display.ASC",
		"D:\\visual studio\\IMU\\display\\y_down_display.ASC",
		"D:\\visual studio\\IMU\\display\\z_up_display.ASC",
		"D:\\visual studio\\IMU\\display\\z_down_display.ASC"

	};

	const char* fixed_Accdataname[6] =
	{
		"D:\\visual studio\\IMU\\display\\x_up_fixed.ASC",
		"D:\\visual studio\\IMU\\display\\x_down_fixed.ASC",
		"D:\\visual studio\\IMU\\display\\y_up_fixed.ASC",
		"D:\\visual studio\\IMU\\display\\y_down_fixed.ASC",
		"D:\\visual studio\\IMU\\display\\z_up_fixed.ASC",
		"D:\\visual studio\\IMU\\display\\z_down_fixed.ASC"
	};



	Acc_SixPosRead(Accdataname, acc_sixpos, XW_GI7681_sam_freq, XW_GI7681_acc_scale, XW_GI7681_gyro_scale);
	Acc_SixPosDisplay(Accdisplayname, acc_sixpos);
	DisplayAccCalResult(acc_sixpos, fixed_acc_sixpos, AccCuttime);
	Acc_SixPosDisplay(fixed_Accdataname, fixed_acc_sixpos);
	
}



//陀螺两位置法标定三轴数据
void Groy_twoxyz_func()
{
	INS::Groy_twopos gyro_xpos;
	INS::Groy_twopos gyro_ypos;
	INS::Groy_twopos gyro_zpos;

	const char* Groydataname[6] =
	{
		"D:\\visual studio\\IMU\\IMUdata\\x_zhengzhuan.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\x_fanzhuan.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\y_zhengzhuan.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\y_fanzhuan.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\z_zhengzhuan.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\z_fanzhuan.ASC"
	};

	const char* Groydisplayname[6] =
	{
		"D:\\visual studio\\IMU\\display\\x_zhengzhuan_display.ASC",
		"D:\\visual studio\\IMU\\display\\x_fanzhuan_display.ASC",
		"D:\\visual studio\\IMU\\display\\y_zhengzhuan_display.ASC",
		"D:\\visual studio\\IMU\\display\\y_fanzhuan_display.ASC",
		"D:\\visual studio\\IMU\\display\\z_zhengzhuan_display.ASC",
		"D:\\visual studio\\IMU\\display\\z_fanzhuan_display.ASC"
	};
	Groy_SixPosRead(Groydataname, gyro_xpos, gyro_ypos, gyro_zpos, XW_GI7681_sam_freq, XW_GI7681_acc_scale, XW_GI7681_gyro_scale);
	Groy_SixPosDisplay(Groydisplayname, gyro_xpos, gyro_ypos, gyro_zpos);
	DisplayGroyCalResult(gyro_xpos, gyro_ypos, gyro_zpos, XW_GI7681_sam_freq);
}



void Fix_IMUdata(const std::vector<INS::IMUdata>& raw_data,std::vector<INS::IMUdata>& fixed_data,const double acc_bias[3], const double acc_scale[3], const double acc_m[6])
{
	//-----------------------------矩阵，向量定义-----------------------------
	Mat fixed_acc(3, 1);
	Mat raw_acc(3, 1);
	Mat fix_Mat(3, 3);
	Mat bias(3, 1);
	//-----------------------------改正矩阵初始化------------------------------
	fix_Mat(0, 0) = acc_scale[0]; fix_Mat(1, 1) = acc_scale[1]; fix_Mat(2, 2) = acc_scale[2];
	fix_Mat(0, 1) = acc_m[0]; fix_Mat(0, 2) = acc_m[1]; fix_Mat(1, 2) = acc_m[2];
	fix_Mat(1, 0) = acc_m[3]; fix_Mat(2, 0) = acc_m[4]; fix_Mat(2, 1) = acc_m[5];
	bias(0, 0) = acc_bias[0]; bias(1, 0) = acc_bias[1]; bias(2, 0) = acc_bias[2];
	Mat fix_Mat_inv = fix_Mat.inverse();
	
	//------------------------------原始数据赋值--------------------------------
	for (int i = 0; i < raw_data.size(); i++)
	{
		raw_acc(0, 0) = raw_data[i].accx;
		raw_acc(1, 0) = raw_data[i].accy;
		raw_acc(2, 0) = raw_data[i].accz;
	//------------------------------计算补偿后数据---------------------------------
		fixed_acc = fix_Mat_inv * (raw_acc - bias);
	//-------------------------创建新的IMUdata对象并赋值----------------------------
		INS::IMUdata fixed_imu;
		fixed_imu.header = raw_data[i].header;
		fixed_imu.GPStime = raw_data[i].GPStime;
		fixed_imu.IMUtime = raw_data[i].IMUtime;

		// 使用补偿后的加速度数据
		fixed_imu.accx = fixed_acc(0, 0);
		fixed_imu.accy = fixed_acc(1, 0);
		fixed_imu.accz = fixed_acc(2, 0);

		// 陀螺数据保持不变（因为没有补偿参数）
		fixed_imu.gyrox = raw_data[i].gyrox;
		fixed_imu.gyroy = raw_data[i].gyroy;
		fixed_imu.gyroz = raw_data[i].gyroz;

		
//------------------------------储存补偿后数据---------------------------------
		fixed_data.push_back(fixed_imu);
	}
	std::cout << "数据补偿完成,共补偿" <<fixed_data.size()<<"条数据"<< std::endl;
}


void Fix_sixposdata(INS::Acc_sixpos& sixpos, INS::Acc_sixpos& sixpos_fixed, const double acc_bias[3], const double acc_scale[3], const double acc_m[3])
{
	Fix_IMUdata(sixpos.x_up, sixpos_fixed.x_up, acc_bias, acc_scale, acc_m);
	Fix_IMUdata(sixpos.x_down, sixpos_fixed.x_down, acc_bias, acc_scale, acc_m);
	Fix_IMUdata(sixpos.y_up, sixpos_fixed.y_up, acc_bias, acc_scale, acc_m);
	Fix_IMUdata(sixpos.y_down, sixpos_fixed.y_down, acc_bias, acc_scale, acc_m);
	Fix_IMUdata(sixpos.z_up, sixpos_fixed.z_up, acc_bias, acc_scale, acc_m);
	Fix_IMUdata(sixpos.z_down, sixpos_fixed.z_down, acc_bias, acc_scale, acc_m);
	std::cout << "六个位置的加速度计数据均补偿完成" << std::endl;
}






void SPCA_func()
{
	//------------------------静态解析粗对准--------------------------------
	std::vector<INS::IMUdata>SPCA_data;
	SPCA_data.clear();

	const char* SPCAdataname = "D:\\visual studio\\INS\\IMUdata\\30min.ASC";
	const char* SPCAdisplayname = "D:\\visual studio\\INS\\display\\30min_display.ASC";
	IMUFileRead(SPCAdataname, SPCA_data, SPAN_100C_sam_freq, SPAN_100C_acc_scale, SPAN_100C_gyro_scale);

	//DataDisplay(SPCAdisplayname, SPCA_data);

	//轴系调整
	
	//RFU2FRD(SPCA_data, adjusted_SPCA_data);
	

	//------------------------整体求姿态角--------------------------------
	INS::Attitude_angle Entire_angle;
	SPCACal(SPCA_data, Entire_angle, SPCA_data.size());
	std::cout << Entire_angle.pusai * rad2deg_scale << " " << Entire_angle.theta * rad2deg_scale << " " << Entire_angle.fai * rad2deg_scale << " deg" << std::endl;

	//-----------------------以一秒为间隔求姿态角-------------------------
	std::vector<INS::Attitude_angle> persec_angle;
	INS::Attitude_angle sec_angle;
	std::vector<INS::IMUdata>persec_data;
	int seccount = 0;
	seccount = SPCA_data.size() / SPAN_100C_sam_freq;
	for (int i = 0; i < seccount; i++)
	{
		for (int j = i * SPAN_100C_sam_freq; j < (i + 1) * SPAN_100C_sam_freq; j++)
		{
			persec_data.push_back(SPCA_data[j]);
		}
		SPCACal(persec_data, sec_angle, SPAN_100C_sam_freq);
		persec_angle.push_back(sec_angle);
		persec_data.clear();
	}

	//----------------------------写入文件------------------------------------
	const char* angledisplayname = "D:\\visual studio\\INS\\display\\persec_angle.txt";
	Att_angle_display(angledisplayname, persec_angle);

}

//计算加速度计标定参数并补偿数据
/*
In：
	std::vector<IMUdata> raw_data：原始IMU数据
Out:
	std::vector<IMUdata> fixed_data：补偿后的IMU数据
*/
void Fix_AccCal_func(const std::vector<INS::IMUdata>& raw_data,std::vector<INS::IMUdata>& fixed_data)
{
	INS::Acc_sixpos acc_sixpos;
	INS::Acc_sixpos fixed_acc_sixpos;
	const char* Accdataname[6] =
	{
		"D:\\visual studio\\IMU\\IMUdata\\x_up.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\x_down.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\y_up.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\y_down.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\z_up.ASC",
		"D:\\visual studio\\IMU\\IMUdata\\z_down.ASC"
	};
	Acc_SixPosRead(Accdataname, acc_sixpos, XW_GI7681_sam_freq, XW_GI7681_acc_scale, XW_GI7681_gyro_scale);
	double sixpos[6][3] = { 0 };
	CalMeanAcc_sixpos(acc_sixpos, sixpos, AccCuttime);
	double acc_bias[3] = { 0.0 };
	double acc_scale[3] = { 0.0 };
	double acc_m[6] = { 0.0 };
	Demarcate(sixpos, acc_bias, acc_scale, acc_m);

	//补偿数据
	Fix_IMUdata(raw_data, fixed_data, acc_bias, acc_scale, acc_m);
	std::cout << "加速度计标定参数补偿数据完成！" << std::endl;

}