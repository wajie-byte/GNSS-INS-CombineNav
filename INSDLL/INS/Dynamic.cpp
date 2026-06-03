#include <cmath>
#include"../BASE/DefData.h"
#include"Dynamic.h"


//计算GRS80地球椭球模型的正常重力公式
/*
	In:
	double lat			//纬度rad
	double h			//高程m
	Out:
	double g			//正常重力m/s^2
*/
double Cal_g_GRS80(double lat, double h)
{
	double g0 = 9.7803267715 * (1 + 0.0052790414 * pow(sin(lat), 2) + 0.0000232718 * pow(sin(lat), 4));
	double g1 = -(3.087691089e-6 - 4.397731e-9 * pow(sin(lat), 2)) * h;
	double g2 = 0.721e-12 * h * h;
	return g0 + g1 + g2;
}

//计算姿态矩阵C_b^n
/*
In:
	double Yaw			//航向角rad
	double Pitch		//俯仰角rad		
	double Roll			//横滚角rad	
Out:
	Mat C_b^n			//姿态矩阵,b为下标，n为上标(3x3)
*/
Mat Cal_C_b_n(double Yaw,double Pitch,double Roll)
{
	//用a,b,c存储姿态角，方便编写计算公式
	double a = Pitch;
	double b = Roll;
	double c = Yaw;
	//定义姿态矩阵
	Mat C_b_n(3, 3);
	//姿态矩阵元素计算
	C_b_n(0, 0) = cos(a) * cos(c);
	C_b_n(0, 1) = -cos(b) * sin(c) + sin(b) * sin(a) * cos(c);
	C_b_n(0, 2) = sin(b) * sin(c) + cos(b) * sin(a) * cos(c);
	C_b_n(1, 0) = cos(a) * sin(c);
	C_b_n(1, 1) = cos(b) * cos(c) + sin(b) * sin(a) * sin(c);
	C_b_n(1, 2) = -sin(b) * cos(c) + cos(b) * sin(a) * sin(c);
	C_b_n(2, 0) = -sin(a);
	C_b_n(2, 1) = sin(b) * cos(a);
	C_b_n(2, 2) = cos(b) * cos(a);
	//返回姿态矩阵
	return C_b_n;
}

//姿态角转四元数
/*
In:
	double Yaw			//航向角rad
	double Pitch		//俯仰角rad
	double Roll			//横滚角rad
Out:
	Mat q_b_n			//四元数,b为下标，n为上标(4x1)
*/
Mat euler2quaternion(double Yaw, double Pitch, double Roll)
{
	//计算半倍欧拉角对应的正弦余弦值
	double cy = cos(Yaw / 2); double sy = sin(Yaw / 2);
	double cp = cos(Pitch / 2); double sp = sin(Pitch / 2);
	double cr = cos(Roll / 2); double sr = sin(Roll / 2);
	//参考坐标系n绕z-y-x旋转到载体坐标系n的姿态四元数
	//q_b^n=q_z*q_y*q_x
	Mat q_b_n(4, 1);
	//元素计算
	double qw = cr * cp * cy + sr * sp * sy;
	double qx = sr * cp * cy - cr * sp * sy;
	double qy = cr * sp * cy + sr * cp * sy;
	double qz = -sr * sp * cy + cr * cp * sy;
	//归一化
	double norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
	qw = qw / norm;
	qx = qx / norm;
	qy = qy / norm;
	qz = qz / norm;
	q_b_n(0, 0) = qw;
	q_b_n(1, 0) = qx;
	q_b_n(2, 0) = qy;
	q_b_n(3, 0) = qz;
	
	//返回四元数
	return q_b_n;
}

//四元数转姿态矩阵
/*
In:
	Mat q_b_n			//四元数,b为下标，n为上标(4x1)
Out:
	Mat C_b^n			//姿态矩阵,b为下标，n为上标(3x3)
*/
Mat quaternion2C_b_n(Mat q_b_n)
{
	//获取四元数四个元的值进行求解
	double qw = q_b_n(0, 0);
	double qx = q_b_n(1, 0);
	double qy = q_b_n(2, 0);
	double qz = q_b_n(3, 0);
	//定义姿态矩阵
	Mat C_b_n(3, 3);
	//这里省略四元数转换为姿态矩阵的过程，直接用结论
	C_b_n(0, 0) = 1 - 2 * (qy * qy + qz * qz);
	C_b_n(0, 1) = 2 * (qx * qy - qw * qz);
	C_b_n(0, 2) = 2 * (qx * qz + qw * qy);
	C_b_n(1, 0) = 2 * (qx * qy + qw * qz);
	C_b_n(1, 1) = 1 - 2 * (qx * qx + qz * qz);
	C_b_n(1, 2) = 2 * (qy * qz - qw * qx);
	C_b_n(2, 0) = 2 * (qx * qz - qw * qy);
	C_b_n(2, 1) = 2 * (qy * qz + qw * qx);
	C_b_n(2, 2) = 1 - 2 * (qx * qx + qy * qy);
	//返回姿态矩阵
	return C_b_n;
}

//四元数转姿态角
/*
In:
	Mat q_b_n			//四元数,b为下标，n为上标(4x1)
	
Out:
	double Yaw			//航向角rad
	double Pitch		//俯仰角rad
	double Roll			//横滚角rad		
*/
void quaternion2euler(Mat q_b_n, double& Yaw, double& Pitch, double& Roll)
{
	//获取四元数四个元的值进行求解
	double qw = q_b_n(0, 0);
	double qx = q_b_n(1, 0);
	double qy = q_b_n(2, 0);
	double qz = q_b_n(3, 0);
	//这里省略四元数转换为姿态矩阵的过程，直接用结论
	double c31 = 2 * (qx * qz - qw * qy);
	double c32 = 2 * (qy * qz + qw * qx);
	double c33 = 1 - 2 * (qx * qx + qy * qy);
	double c21 = 2 * (qx * qy + qw * qz);
	double c11 = 1 - 2 * (qy * qy + qz * qz);
	

	// 计算初始欧拉角
	double pitch = atan2(-c31, sqrt(c32 * c32 + c33 * c33));
	double roll = atan2(c32, c33);
	double yaw = atan2(c21, c11);

	// 万向节锁处理（俯仰角接近±90°）
	const double EPS = 1e-6;
	bool is_gimbal_lock = (fabs(c31) > 1 - EPS);
	if (is_gimbal_lock)
	{
		// 万向节锁情况：按照惯例设置Roll=0
		roll = 0.0;
		// 正确的万向节锁Yaw计算公式
		if (pitch > 0) 
		{
			yaw = 2 * atan2(qx, qw);
		}
		// 当俯仰角为-90°时  
		else 
		{
			yaw = -2 * atan2(qx, qw);
		}
	}

	// 角度规范化
	roll = normalize_angle(roll);
	yaw = normalize_angle(yaw);

	// pitch已经在正确范围内，但如果有数值误差可以稍微约束
	if (pitch > pi / 2)
	{
		pitch = pi / 2;
	}
	else if (pitch < -pi / 2)
	{
		pitch = -pi / 2;
	}

	// 输出
	Roll = roll;
	Pitch = pitch;
	Yaw = yaw;
}

//计算叉乘c=axb
/*
In:
	double a[3]		//a向量
	double b[3]		//b向量
Out:
	double c[3]		//叉乘向量
*/
void cross_3_1(const double a[3], const double b[3], double c[3])//注意向量叉乘的顺序为axb
{
	c[0] = -a[2] * b[1] + a[1] * b[2];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = -a[1] * b[0] + a[0] * b[1];
}

//根据k-1时刻和k时刻角增量计算b系相对i系转动的等效旋转矢量
/*
In:
	double groy_k_1[3]		//k-1时刻角增量rad
	double groy_k[3]		//k时刻角增量rad
Out:
	double ERV[3]			//b系相对i系转动的等效旋转矢量rad
*/
void Cal_ERV_ib(const double groy_k_1[3], const double groy_k[3], double ERV[3])
{
	//计算二阶圆锥误差补偿项
	double groy_cross[3] = { 0.0,0.0,0.0 };
	//k-1项叉乘k项
	cross_3_1(groy_k_1, groy_k, groy_cross);
	//计算等效旋转矢量
	for (int i = 0; i < 3; i++)
	{
		ERV[i] = groy_k[i] + groy_cross[i] / 12;
	}
}

//根据等效旋转矢量正转姿态变化的四元数
/*
In:
	double ERV[3]			//等效旋转矢量
Out:
	Mat q_b_rotate			//姿态变化(4x1)			
*/
Mat Cal_pos_rotate(const double ERV[3])
{
	double angle = sqrt(ERV[0] * ERV[0] + ERV[1] * ERV[1] + ERV[2] * ERV[2]);

	Mat q_b_rotate(4, 1);

	const double eps = 0.000005;  
	if (angle < eps)
	{
		// 小角度近似
		q_b_rotate(0, 0) = 1.0 - angle * angle / 8.0;  // cos(angle/2) 的泰勒展开
		double half_angle = angle / 2.0;
		double scale = 0.5;  // sin(angle/2)/angle 在 angle→0 时的极限
		q_b_rotate(1, 0) = scale * ERV[0];
		q_b_rotate(2, 0) = scale * ERV[1];
		q_b_rotate(3, 0) = scale * ERV[2];
	}
	else
	{
		q_b_rotate(0, 0) = cos(angle / 2.0);
		double scale = sin(angle / 2.0) / angle;
		q_b_rotate(1, 0) = scale * ERV[0];
		q_b_rotate(2, 0) = scale * ERV[1];
		q_b_rotate(3, 0) = scale * ERV[2];
	}

	q_b_rotate = quaternion_normalize(q_b_rotate);
	return q_b_rotate;
}


//根据当地纬度计算积分区间地球自转角速度在n系下的投影
/*
In:
	double lat					//当地纬度rad
Out:
	double w_ie_n[3]			//地球自转角速度在n系下投影,ie为下标，n为上标,rad/s
*/
void Cal_w_ie_n(double lat, double w_ie_n[3])
{
	w_ie_n[0] = Omegae * cos(lat);
	w_ie_n[1] = 0.0;
	w_ie_n[2] = -Omegae * sin(lat);
}


//根据当地纬度位移角速度在n系下的投影
/*
In:
	double vE					//东向速度m/s
	double vN					//北向速度m/s
	double lat					//当地纬度rad
	double h					//当地高程m
Out:
	double w_en_n[3]			//位移角速度在n系下投影,en为下标，n为上标,rad/s
*/
void Cal_w_en_n(double vE, double vN, double lat, double h, double w_en_n[3])
{
	//计算子午圈和卯酉圈半径
	double RM = Cal_RM(lat);
	double RN = Cal_RN(lat);
	//计算投影分量
	w_en_n[0] = vE / (RN + h);
	w_en_n[1] = -vN / (RM + h);
	w_en_n[2] = -vE * tan(lat) / (RN + h);
}

//根据当地纬度计算子午圈半径
/*
In:
	double lat					//当地纬度rad
Out:
	double RM					//子午圈半径m
*/
double Cal_RM(double lat)
{
	//获取椭球长半轴和扁率
	double a = WGS84_a;//长半轴m
	double e2 = WGS84_e1_2;//扁率
	//计算RM
	double RM_1 = a * (1 - e2);
	double RM_2 = 1 - e2* sin(lat) * sin(lat);
	double RM = RM_1 / sqrt(RM_2 * RM_2 * RM_2);
	return RM;
}
//根据当地纬度计算卯酉圈半径
/*
In:
	double lat					//当地纬度rad
Out:
	double RN					//卯酉圈半径m
*/
double Cal_RN(double lat)
{
	//获取椭球长半轴和扁率
	double a = WGS84_a;//长半轴m
	double e2 = WGS84_e1_2;//扁率
	//计算RN
	double RN_1 = 1 - e2 * sin(lat) * sin(lat);
	double RN = a / sqrt(RN_1);
	return RN;
}

//积分区间[tk-1,tk]中间时刻tk-1/2
//根据地球自转角速度和位移角速度计算n系相对i系转动的等效旋转矢量
/*
In:
	double w_ie_n_mid[3]		//tk-1/2时刻地球自转角速度向量rad/s
	double w_en_n_mid[3]		//tk-1/2时刻位移角速度向量rad/s
	double dt					//采样间隔s
Out:
	double ERV[3]				//n系相对i系转动的等效旋转矢量rad
*/
void Cal_ERV_in(const double w_ie_n_mid[3], const double w_en_n_mid[3], double dt, double ERV[3])
{
	for (int i = 0; i < 3; i++)
	{
		ERV[i] = (w_ie_n_mid[i] + w_en_n_mid[i]) * dt;
	}
}

//根据等效旋转矢量反转姿态变化的四元数
/*
In:
	double ERV[3]			//等效旋转矢量rad
Out:
	Mat q_n_rotate			//系姿态变化(4x1)
*/
Mat Cal_neg_rotate(const double ERV[3])
{
	// 计算总角度
	double angle = sqrt(ERV[0] * ERV[0] + ERV[1] * ERV[1] + ERV[2] * ERV[2]);

	Mat q_n_rotate(4, 1);

	const double eps = 0.000005;  // 小角度阈值

	if (angle < eps)
	{
		// 小角度近似（反转: 矢量部分取负）
		// cos(angle/2) ≈ 1 - angle²/8
		q_n_rotate(0, 0) = 1.0 - angle * angle / 8.0;
		// sin(angle/2)/angle ≈ 0.5
		double scale = -0.5;  // 负号表示反转
		q_n_rotate(1, 0) = scale * ERV[0];
		q_n_rotate(2, 0) = scale * ERV[1];
		q_n_rotate(3, 0) = scale * ERV[2];
	}
	else
	{
		// 正常计算（矢量部分取负）
		q_n_rotate(0, 0) = cos(angle / 2.0);
		double scale = -sin(angle / 2.0) / angle;
		q_n_rotate(1, 0) = scale * ERV[0];
		q_n_rotate(2, 0) = scale * ERV[1];
		q_n_rotate(3, 0) = scale * ERV[2];
	}

	q_n_rotate = quaternion_normalize(q_n_rotate);
	return q_n_rotate;
}

//四元数乘法
/*
In:
	Mat a			//四元数a(4x1)
	Mat b			//四元数b(4x1)
Out:
	Mat c			//四元数c(4x1)
*/
Mat quaternion_multiply(Mat a, Mat b)
{
	Mat c(4, 1);
	//定义d(4x4)辅助计算
	Mat d(4, 4);
	d(0, 0) = a(0, 0); d(0, 1) = -a(1, 0); d(0, 2) = -a(2, 0); d(0, 3) = -a(3, 0);
	d(1, 0) = a(1, 0); d(1, 1) = a(0, 0); d(1, 2) = -a(3, 0); d(1, 3) = a(2, 0);
	d(2, 0) = a(2, 0); d(2, 1) = a(3, 0); d(2, 2) = a(0, 0); d(2, 3) = -a(1, 0);
	d(3, 0) = a(3, 0); d(3, 1) = -a(2, 0); d(3, 2) = a(1, 0); d(3, 3) = a(0, 0);
	c = d * b;
	return c;
}


//四元数链乘实现姿态更新
/*
In:
	Mat q_bn_k_1			//k-1时刻姿态四元数(4x1)
	Mat q_b_rotate			//k-1到k时刻b系姿态变化(4x1)
	Mat q_n_rotate			//k-1到k时刻n系姿态变化(4x1)
Out:
	Mat q_bn_k				//k时刻姿态四元数(4x1)
*/
Mat Attupdate_quaternion(Mat q_n_rotate, Mat q_bn_k_1, Mat q_b_rotate)
{
	Mat q_bn_k(4, 1);
	Mat temp = quaternion_multiply(q_bn_k_1, q_b_rotate);
	temp = quaternion_normalize(temp);
	q_bn_k = quaternion_multiply(q_n_rotate, temp);
	q_bn_k = quaternion_normalize(q_bn_k);
	return q_bn_k;
}

//四元数归一化
/*
In:
	Mat a				//归一化前四元数
	
Out:
	Mat b				//归一化后四元数
*/
Mat quaternion_normalize(Mat a)
{
	Mat b(4, 1);
	double norm = sqrt(a(0, 0) * a(0, 0) + a(1, 0) * a(1, 0) + a(2, 0) * a(2, 0) + a(3, 0) * a(3, 0));
	b(0, 0) = a(0, 0) / norm;
	b(1, 0) = a(1, 0) / norm;
	b(2, 0) = a(2, 0) / norm;
	b(3, 0) = a(3, 0) / norm;
	return b;
}

//计算重力加速度在n系下的投影
/*
In:
	double lat			//当地纬度rad
	double h			//当地高程m

Out:
	double g_p_n[3]		//当地重力加速度在n系下的投影m/s^2
*/
void Cal_g_p_n(double lat, double h,double g_p_n[3])
{
	double g = Cal_g_GRS80(lat, h);
	g_p_n[0] = g_p_n[1] = 0.0;
	g_p_n[2] = g;
}

//计算重力哥氏积分项
//需外推数据至时刻tk-1/2
/*
In:
	double g_p_n_mid[3]			//当地重力加速度在n系下的投影m/s^2
	double w_ie_n_mid[3]		//tk-1/2时刻地球自转角速度向量rad/s
	double w_en_n_mid[3]		//tk-1/2时刻位移角速度向量rad/s
	double v_n_mid[3]			//tk-1/2时刻速度在n系下投影m/s
	double dt					//采样间隔s

Out:
	double dv_gc_n[3]			//重力哥氏积分项m/s
*/
void Cal_dv_gc_n(const double g_p_n_mid[3], const double w_ie_n_mid[3], const double w_en_n_mid[3],
	const double v_n_mid[3], double dt, double dv_gc_n[3])
{
	double w_add[3] = { 0.0,0.0,0.0 };
	for (int i = 0; i < 3; i++)
	{
		w_add[i] = 2 * w_ie_n_mid[i] + w_en_n_mid[i];
	}
	double cross_w_v[3] = { 0.0,0.0,0.0 };
	cross_3_1(w_add, v_n_mid, cross_w_v);
	for (int i = 0; i < 3; i++)
	{
		dv_gc_n[i] = (g_p_n_mid[i] - cross_w_v[i]) * dt;
	}
}

//计算b系比力积分项
//需外推数据至时刻tk-1/2
/*
In:
	double groy_k_1[3]		//k-1时刻角增量rad
	double groy_k[3]		//k时刻角增量rad
	double dv_k_1[3]		//k-1时刻速度增量m/s
	double dv_k[3]			//k时刻速度增量m/s
Out:
	double dv_f_b[3]		//b系比力积分项m/s  dv_f,k_b(k-1)
*/
void Cal_dv_f_b(const double groy_k_1[3], const double groy_k[3], const double dv_k_1[3],
	const double dv_k[3], double dv_f_b[3])
{
	//定义三个叉乘项
	double cross1[3] = { 0.0,0.0,0.0 };
	double cross2[3] = { 0.0,0.0,0.0 };
	double cross3[3] = { 0.0,0.0,0.0 };
	//计算叉乘项
	cross_3_1(groy_k, dv_k, cross1);
	cross_3_1(groy_k_1, dv_k, cross2);
	cross_3_1(dv_k_1, groy_k, cross3);
	//计算b系比力积分项
	for (int i = 0; i < 3; i++)
	{
		dv_f_b[i] = dv_k[i] + cross1[i] / 2 + (cross2[i] + cross3[i]) / 12;
	}
}

//计算比力积分项
//需外推数据至时刻tk-1/2
/*
In:
	double ERV[3]			//n系相对i系转动的等效旋转矢量rad
	Mat C_bn_k_1			//k-1时刻姿态矩阵rad(3x3)
	double dv_f_b[3]		//b系比力积分项m/s
	
Out:
	double dv_f_n[3]		//比力积分项m/s  dv_f,k_n(k-1)
*/
void Cal_dv_f_n(const double ERV[3],const Mat C_bn_k_1,const double dv_f_b[3],
	double dv_f_n[3])
{
	Mat I = Mat::Identity(3);//3x3单位矩阵
	Mat antisym_ERV = Cal_Antisymmmetric_Mat(ERV);//等效旋转矢量的反对称矩阵
	Mat dv_fb(3, 1);//将b系比力积分项用Mat类表示，便于后续矩阵运算
	for (int i = 0; i < 3; i++)
	{
		dv_fb(i, 0) = dv_f_b[i];
	}
	Mat I_ERV = I - 0.5 * antisym_ERV;
	Mat dv_fn = I_ERV * C_bn_k_1 * dv_fb;
	for (int i = 0; i < 3; i++)
	{
		dv_f_n[i] = dv_fn(i, 0);
	}
}

//计算反对称矩阵
/*
In:
	double a[3]				//向量a(3x1)
Out:
	Mat ax					//反对称矩阵
*/
Mat Cal_Antisymmmetric_Mat(const double a[3])
{
	Mat ax(3, 3);
	ax(0, 0) = 0.0; ax(0, 1) = -a[2]; ax(0, 2) = a[1];
	ax(1, 0) = a[2]; ax(1, 1) = 0; ax(1, 2) = -a[0];
	ax(2, 0) = -a[1]; ax(2, 1) = a[0]; ax(2, 2) = 0.0;
	return ax;
}

//计算反对称矩阵
/*
In:
	Mat a					//向量a
Out:
	Mat ax					//反对称矩阵
*/
Mat Skew(const Mat& a)
{
	// 检查是否是3维向量（无论行列）
	if ((a.cols() == 1 && a.rows() == 3) || (a.cols() == 3 && a.rows() == 1))
	{
		Mat ax(3, 3);

		// 获取向量分量（处理行向量和列向量）
		double v1 = (a.rows() == 3) ? a(0, 0) : a(0, 0);  // x分量
		double v2 = (a.rows() == 3) ? a(1, 0) : a(0, 1);  // y分量  
		double v3 = (a.rows() == 3) ? a(2, 0) : a(0, 2);  // z分量

		ax(0, 0) = 0.0;   ax(0, 1) = -v3; ax(0, 2) = v2;
		ax(1, 0) = v3;   ax(1, 1) = 0.0; ax(1, 2) = -v1;
		ax(2, 0) = -v2;   ax(2, 1) = v1; ax(2, 2) = 0.0;

		return ax;
	}
	else
	{
		std::cerr << "Error: Input must be a 3-element vector (3x1 or 1x3)."
			<< " Got " << a.rows() << "x" << a.cols() << std::endl;
		return Mat();
	}
}

//计算反对称矩阵
/*
In:
	double a[3]					//向量a
Out:
	Mat ax					//反对称矩阵
*/
Mat Skew(const double a[3])
{
	return Cal_Antisymmmetric_Mat(a);
}




//n系速度更新
/*
In:
	double v_k_1_n[3]			//k-1时刻速度在n系下投影m/s
	double dv_f_n[3]			//比力积分项m/s
	double dv_gc_n[3]			//重力哥氏积分项m/s
Out:
	double v_k_n[3]				//k时刻速度在n系下投影
*/
void Cal_v_k_n(const double v_k_1_n[3], const double dv_f_n[3], const double dv_gc_n[3],
	double v_k_n[3])
{
	//计算k时刻速度
	for (int i = 0; i < 3; i++)
	{
		v_k_n[i] = v_k_1_n[i] + dv_f_n[i] + dv_gc_n[i];
	}
}

//高程更新
/*
In:
	double h_k_1			//k-1时刻高程m
	double vD_k_1			//k-1时刻垂向速度m/s
	double vD_k				//k时刻垂向速度m/s
	double dt				//采样间隔s
Out:
	double h_k				//k时刻高程m
*/
double Cal_h_k(double h_k_1, double vD_k_1, double vD_k, double dt)
{
	double h_k = h_k_1 - (vD_k_1 + vD_k) / 2 * dt;
	return h_k;
}

//纬度更新
/*
In:
	double lat_k_1			//k-1时刻纬度rad
	double vN_k_1			//k-1时刻北向速度m/s
	double vN_k				//k时刻北向速度m/s
	double h_mid			//tk-1/2时刻高程m
	double dt				//采样间隔s
Out:
	double lat_k			//k时刻纬度rad
*/
double Cal_lat_k(double lat_k_1, double vN_k_1, double vN_k, double h_mid, double dt)
{
	//计算子午圈半径
	double RM_k_1 = Cal_RM(lat_k_1);
	//计算k时刻纬度
	double lat_k = lat_k_1 + (vN_k_1 + vN_k) / 2 / (RM_k_1 + h_mid) * dt;
	return lat_k;
}

//经度更新
/*
In:
	double lon_k_1			//k-1时刻经度rad
	double vE_k_1			//k-1时刻东向速度m/s
	double vE_k				//k时刻东向速度m/s
	double lat_mid			//tk-1/2时刻纬度rad
	double h_mid			//tk-1/2时刻高程m
	double dt				//采样间隔s
Out:
	double lon_k			//k时刻经度rad
*/
double Cal_lon_k(double lon_k_1, double vE_k_1, double vE_k, double lat_mid, double h_mid, double dt)
{
	//计算卯酉圈半径
	double RN_mid = Cal_RN(lat_mid);
	//计算k时刻经度
	double lon_k = lon_k_1 + (vE_k_1 + vE_k) / 2 / ((RN_mid + h_mid) * cos(lat_mid)) * dt;
	return lon_k;
}


//寻找初始数据索引
/*
In:
	vector<IMUDataEpoch> imudata		//IMU数据序列
	double starttime					//初始时间s
	double tolerance					//时间容差s
Out:
	int& index_start					//初始数据索引
*/
bool FindInitialIndex(const std::vector<INS::IMUDataEpoch>& imudata, double starttime, int& index_start,double tolerance)
{
	index_start = -1;
	for (size_t i = 0; i < imudata.size(); i++)
	{
		if (fabs(imudata[i].time - starttime) <= tolerance)
		{
			index_start = static_cast<int>(i);
			std::cout << "Initial index found at: " << index_start << std::endl;
			return true;
		}
	}
	return false;
}


//n系到e系转换四元数
/*
In:
	double BLH[3]
Out:
	Mat qne
*/
Mat Cal_qne(const double BLH[3])
{
	double coslat = cos(-pi * 0.25 - BLH[0] * 0.5);
	double sinlat = sin(-pi * 0.25 - BLH[0] * 0.5);
	double coslon = cos(BLH[1] * 0.5);
	double sinlon = sin(BLH[1] * 0.5);
	Mat qne(4, 1);
	qne(0, 0) = coslat * coslon;
	qne(1, 0) = -sinlat * sinlon;
	qne(2, 0) = sinlat * coslon;
	qne(3, 0) = coslat * sinlon;
	qne = quaternion_normalize(qne);
	return qne;
}

//零速修正
/*
In:
	double dv[3]		//速度增量m/s
	double dtheta[3]	//角增量rad
	double lat			//当地纬度rad
	double h			//当地高程m
	double dt			//采样间隔s
	double history_dv[3] //历史速度增量m/s
	double history_dtheta[3] //历史角增量rad
	
Out:
	bool isZeroVel		//是否为零速
*/
bool isZeroVel(const double dv[3], const double dtheta[3], double lat,double h, double dt
	,std::vector<double>& history_dv,std::vector<double>& history_dtheta)
{
	bool condition1 = false;
	bool condition2 = false;
	//计算速度增量阈值
	double g = Cal_g_GRS80(lat, h);
	double vel_threshold = 2 * g * dt; //
	//计算角增量阈值
	double ang_threshold = 3 * Omegae * dt * 100.0;
	double var_vel_threshold = 0.1 * g * dt;
	double var_ang_threshold = Omegae * dt;
	//判断是否为零速
	double dv_norm = sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
	double dtheta_norm = sqrt(dtheta[0] * dtheta[0] + dtheta[1] * dtheta[1] + dtheta[2] * dtheta[2]);
	if (dv_norm < vel_threshold && dtheta_norm < ang_threshold)
	{
		condition1 = true;
	}
	history_dv.push_back(dv_norm);
	history_dtheta.push_back(dtheta_norm);
	if (history_dv.size() > 5)
	{
		history_dv.erase(history_dv.begin());
	}
	if (history_dtheta.size() > 5)
	{
		history_dtheta.erase(history_dtheta.begin());
	}
	double mean_dv = 0.0;
	double mean_dtheta = 0.0;
	double var_dv = 0.0;
	double var_dtheta = 0.0;
	for(int i=0;i<history_dv.size();i++)
	{
		mean_dv += history_dv[i];
		mean_dtheta += history_dtheta[i];
	}
	mean_dv /= history_dv.size();
	mean_dtheta /= history_dtheta.size();
	for (int i = 0; i < history_dv.size(); i++) 
	{
		var_dv += (history_dv[i] - mean_dv) * (history_dv[i] - mean_dv);
		var_dtheta += (history_dtheta[i] - mean_dtheta) * (history_dtheta[i] - mean_dtheta);
	}
	var_dv /= history_dv.size();
	var_dtheta /= history_dtheta.size();

	// 条件2：均值小且方差小（表示稳定静止）
	condition2 = (mean_dv < vel_threshold) &&
		(mean_dtheta < ang_threshold) &&
		(var_dv < var_vel_threshold) &&
		(var_dtheta < var_ang_threshold);
	
	if(condition1 && condition2)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

//根据向量构造对角矩阵
/*
In:
	double a[3]		//向量a
Out:
	Mat D			//对角矩阵
*/
Mat diag(const double a[3])
{
	Mat D = Mat::Zero(3, 3);
	D(0, 0) = a[0];
	D(1, 1) = a[1];
	D(2, 2) = a[2];
	return D;
}