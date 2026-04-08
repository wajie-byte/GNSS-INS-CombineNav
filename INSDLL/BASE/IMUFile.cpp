#include"IMUFile.h"


//读取IMU数据文件
void IMUFileRead(const char* filename, std::vector<INS::IMUdata>& IMUdata,double sam_freq,double acc_scale,double gyro_scale)
{
	std::ifstream infile;
	infile.open(filename, std::ios::in);
	if (!infile.is_open())
	{
		std::cerr << "Error: could not open file " << filename << std::endl;
		return;
	}
	std::string line;//存储每行数据
	std::vector<std::string> Tags = { "%RAWIMUSA","%RAWIMUSXA" };//IMU数据行的标志
	while (getline(infile, line))
	{
		for (const auto &Tag : Tags)
		{
			if (line.find(Tag) != std::string::npos) //找到包含IMU数据的行
			{
				// 替换分隔符以便于解析
				for (char& c : line)
				{
					if (c == ',' || c == ';' || c == '*')
					{
						c = ' ';
					}
				}
				std::istringstream iss(line);
				std::string dummy;//存储不需要的数据
				std::string header;
				double gtime, itime, ax, ay, az, gx, gy, gz;
				if (Tag == "%RAWIMUSXA")iss >> header >> dummy >> gtime >> dummy >> dummy >> dummy >> itime >> dummy >> az >> ay >> ax >> gz >> gy >> gx;
				else if(Tag == "%RAWIMUSA")iss >> header >> dummy >> gtime >> dummy >> itime >> dummy >> az >> ay >> ax >> gz >> gy >> gx;
				

				//转换
				ax = ax * acc_scale * sam_freq;
				ay = -ay * acc_scale * sam_freq;
				az = az * acc_scale * sam_freq;
				gx = gx * gyro_scale * sam_freq;
				gy = -gy * gyro_scale * sam_freq;
				gz = gz * gyro_scale * sam_freq;


				IMUdata.emplace_back(header, gtime, itime, ax, ay, az, gx, gy, gz); //存储数据
			}
		}
		
	}
	infile.close();
	std::cout << filename << "IMU数据读取完成！" << std::endl;
	std::cout << "共读取到" << IMUdata.size() << "条数据" << std::endl;
}

void DataDisplay(const char* filename,const std::vector<INS::IMUdata>& IMUdata)
{
	std::ofstream outfile;
	outfile.open(filename, std::ios::out);
	if (!outfile.is_open())
	{
		std::cerr << "Error: could not open file " << filename << std::endl;
		return;
	}
	// 输出IMU数据
	// 设置输出精度，保留足够的小数位
    outfile << std::fixed << std::setprecision(9);
	for (const auto& data : IMUdata)
	{
		outfile << data.header << " " << data.GPStime << " " << data.IMUtime << " "
			<< data.accz << " " << -data.accy << " " << data.accx << " "
			<< data.gyroz << " " << -data.gyroy << " " << data.gyrox << std::endl;
	}
	std::cout << "IMU数据展示完成！数据输出到文件" << filename << ",共展示" << IMUdata.size() << "条数据" << std::endl;

}

void Acc_SixPosRead(const char* filename[6], INS::Acc_sixpos& acc_sixpos, double sam_freq, double acc_scale, double gyro_scale)
{
	IMUFileRead(filename[0], acc_sixpos.x_up, sam_freq, acc_scale, gyro_scale);
	IMUFileRead(filename[1], acc_sixpos.x_down, sam_freq, acc_scale, gyro_scale);
	IMUFileRead(filename[2], acc_sixpos.y_up, sam_freq, acc_scale, gyro_scale);
	IMUFileRead(filename[3], acc_sixpos.y_down, sam_freq, acc_scale, gyro_scale);
	IMUFileRead(filename[4], acc_sixpos.z_up, sam_freq, acc_scale, gyro_scale);
	IMUFileRead(filename[5], acc_sixpos.z_down, sam_freq, acc_scale, gyro_scale);
	std::cout << "六个位置的加速度计数据读取完成！" << std::endl;
}

void Acc_SixPosDisplay(const char* filename[6], const INS::Acc_sixpos& acc_sixpos)
{
	DataDisplay(filename[0], acc_sixpos.x_up);
	DataDisplay(filename[1], acc_sixpos.x_down);
	DataDisplay(filename[2], acc_sixpos.y_up);
	DataDisplay(filename[3], acc_sixpos.y_down);
	DataDisplay(filename[4], acc_sixpos.z_up);
	DataDisplay(filename[5], acc_sixpos.z_down);
	std::cout << "六个位置的加速度计数据展示完成！" << std::endl;
}

void Groy_TwoPosRead(const char* filename[2], INS::Groy_twopos& gyro_twopos, double sam_freq, double acc_scale, double gyro_scale)
{
	IMUFileRead(filename[0], gyro_twopos.pos_zheng, sam_freq, acc_scale, gyro_scale);
	IMUFileRead(filename[1], gyro_twopos.pos_fan, sam_freq, acc_scale, gyro_scale);
	std::cout << "两个位置的陀螺仪数据读取完成！" << std::endl;
}

void Groy_TwoPosDisplay(const char* filename[2], const INS::Groy_twopos& gyro_twopos)
{
	DataDisplay(filename[0], gyro_twopos.pos_zheng);
	DataDisplay(filename[1], gyro_twopos.pos_fan);
	std::cout << "两个位置的陀螺仪数据展示完成！" << std::endl;
}

void Groy_SixPosRead(const char* filename[6], INS::Groy_twopos& gyro_xpos, INS::Groy_twopos& gyro_ypos, INS::Groy_twopos& gyro_zpos, double sam_freq, double acc_scale, double gyro_scale)
{
	const char* xfile[2] = { filename[0],filename[1] };
	const char* yfile[2] = { filename[2],filename[3] };
	const char* zfile[2] = { filename[4],filename[5] };
	Groy_TwoPosRead(xfile, gyro_xpos, sam_freq, acc_scale, gyro_scale);
	Groy_TwoPosRead(yfile, gyro_ypos, sam_freq, acc_scale, gyro_scale);
	Groy_TwoPosRead(zfile, gyro_zpos, sam_freq, acc_scale, gyro_scale);
	std::cout << "陀螺六位置数据以读取完成" << std::endl;
}

void Groy_SixPosDisplay(const char* filename[6], INS::Groy_twopos& gyro_xpos, INS::Groy_twopos& gyro_ypos, INS::Groy_twopos& gyro_zpos)
{
	DataDisplay(filename[0], gyro_xpos.pos_zheng);
	DataDisplay(filename[1], gyro_xpos.pos_fan);
	DataDisplay(filename[2], gyro_ypos.pos_zheng);
	DataDisplay(filename[3], gyro_ypos.pos_fan);
	DataDisplay(filename[4], gyro_zpos.pos_zheng);
	DataDisplay(filename[5], gyro_zpos.pos_fan);
	std::cout << "六个位置的陀螺仪数据展示完成！" << std::endl;
}


void Att_angle_display(const char* filename, std::vector<INS::Attitude_angle>& angle)
{
	std::ofstream outfile;
	outfile.open(filename, std::ios::out);
	if (!outfile.is_open())
	{
		std::cerr << "Error: could not open file " << filename << std::endl;
		return;
	}
	outfile << std::fixed << std::setprecision(9);
	//outfile << "Yaw" << "            " << "Pitch" << "            " << "Roll" << std::endl;
	for (int i = 0; i < angle.size(); i++)
	{
		outfile << angle[i].pusai*rad2deg_scale << "," << angle[i].theta* rad2deg_scale << "," << angle[i].fai* rad2deg_scale <<"" << std::endl;
	}
	outfile.close();
	std::cout << filename << "已输出完成" << std::endl;

}



void IMUBinFileRead(const char* filename, std::vector<INS::IMUDataEpoch>& IMUdata)
{
	//--------------------打开要读取的bin文件--------------------
	FILE* fp_imu;
	errno_t err = fopen_s(&fp_imu, filename, "rb");
	if (err != 0 || !fp_imu)
	{
		printf("无法打开文件: %s, 错误码: %d\n", filename, err);
		return;
	}
	//输出向量清空
	IMUdata.clear();

	//---------------------循环读取数据----------------------------
	//临时储存一条数据
	INS::IMUDataEpoch epoch_data;
	//记录一条数据中读取到的元素个数
	size_t elementsRead;
	
	while ((elementsRead = fread(&epoch_data, 8, 7, fp_imu)) == 7)
	{
		IMUdata.push_back(epoch_data);
	}

	// 处理文件末尾情况
	if (elementsRead > 0 && elementsRead < 7)
	{
		printf("警告：文件末尾有不完整数据，跳过了 %zu 个元素\n", elementsRead);
	}

	//----------------------关闭文件，提示结束---------------------
	fclose(fp_imu);
	std::cout << filename << "文件读取完毕" << "共读取" << IMUdata.size() << "条数据" << std::endl;

}

void IMUBindatadisplay(const char* filename, std::vector<INS::IMUDataEpoch>& IMUdata)
{
	std::ofstream outfile;
	outfile.open(filename, std::ios::out);
	if (!outfile.is_open())
	{
		std::cerr << "Error: could not open file " << filename << std::endl;
		return;
	}
	// 输出IMU数据
	// 设置输出精度，保留足够的小数位
	outfile << std::fixed << std::setprecision(9);
	for (const auto& data : IMUdata)
	{
		outfile << data.time << " ";
		for (int i = 0; i < 3; i++)
		{
			outfile << data.gyro[i] << " ";
		}
		for (int i = 0; i < 3; i++)
		{
			outfile << data.accl[i] << " ";
		}
		outfile << std::endl;
	}
	std::cout << "IMU数据展示完成！数据输出到文件" << filename << ",共展示" << IMUdata.size() << "条数据" << std::endl;
	
}

//纯惯导解算结果输出
/*
In:
	const char* filename					//输出文件名
	const std::vector<IMUResultEpoch>& imuresult	//惯导解算结果序列
Out:
	文本文件
*/
void IMUResultDisplay(const char* filename, const std::vector<INS::IMUResultEpoch>& imuresult)
{
	std::ofstream outfile;
	outfile.open(filename, std::ios::out);
	if (!outfile.is_open())
	{
		std::cerr << "Error: could not open file " << filename << std::endl;
		return;
	}
	// 输出IMU数据
	// 设置输出精度，保留足够的小数位
	outfile << std::fixed << std::setprecision(12);
	for (const auto& data : imuresult)
	{
		//输出时间
		outfile << data.time << " ";
		//输出BLH，纬度，经度转换为度,高程m
		outfile << data.BLH[0] * rad2deg_scale << " " << data.BLH[1] * rad2deg_scale << " " << data.BLH[2] << " ";
		//输出速度（北东地）m/s
		for (int i = 0; i < 3; i++)
		{
			outfile << data.Vn[i] << " ";
		}
		//输出姿态角，转换为度，顺序为航向、俯仰、横滚
		for (int i = 0; i < 3; i++)
		{
			outfile << data.YPR[i] * rad2deg_scale << " ";
		}
		outfile << std::endl;
	}
	std::cout << "惯导解算结果展示完成！数据输出到文件" << filename << ",共展示" << imuresult.size() << "条数据" << std::endl;
}

//纯惯导解算结果输出，姿态角顺序为滚转、俯仰、航向
/*
In:
	const char* filename					//输出文件名
	const std::vector<IMUResultEpoch>& imuresult	//惯导解算结果序列
Out:
	文本文件
*/
void IMUResultDisplay_RPY(const char* filename, const std::vector<INS::IMUResultEpoch>& imuresult)
{
	std::ofstream outfile;
	outfile.open(filename, std::ios::out);
	if (!outfile.is_open())
	{
		std::cerr << "Error: could not open file " << filename << std::endl;
		return;
	}
	// 输出IMU数据
	// 设置输出精度，保留足够的小数位
	outfile << std::fixed << std::setprecision(12);
	for (const auto& data : imuresult)
	{
		//输出时间
		outfile << data.time << " ";
		//输出BLH，纬度，经度转换为度,高程m
		outfile << data.BLH[0] * rad2deg_scale << " " << data.BLH[1] * rad2deg_scale << " " << data.BLH[2] << " ";
		//输出速度（北东地）m/s
		for (int i = 0; i < 3; i++)
		{
			outfile << data.Vn[i] << " ";
		}
		//输出姿态角，转换为度，顺序为横滚，俯仰、航向
		for (int i = 0; i < 3; i++)
		{
			if (i==2)
			{
				if (data.YPR[0] < 0)
				{
					outfile << (data.YPR[0] + 2 * pi) * rad2deg_scale << " ";
					
				}
				else
				{
					outfile << data.YPR[0] * rad2deg_scale << " ";
				}
				
			}
			else
			{
				outfile << data.YPR[2 - i] * rad2deg_scale << " ";
			}
			
			
		}
		outfile << std::endl;
	}
	std::cout << "惯导解算结果展示完成！数据输出到文件" << filename << ",共展示" << imuresult.size() << "条数据" << std::endl;
}

//读取纯惯导解算结果
/*
In:
	const char* BinResultfile				//输入文件名
	const char* displayfile					//输出文件名
Out:
	文本文件
*/
void BinResultRead(const char* BinResultfile, const char* displayfile)
{
	//--------------------打开要读取的bin文件--------------------
	FILE* fp_ins;
	errno_t err = fopen_s(&fp_ins, BinResultfile, "rb");
	if (err != 0 || !fp_ins)
	{
		printf("无法打开文件: %s, 错误码: %d\n", BinResultfile, err);
		return;
	}
	
	std::vector<INS::IMUResultEpoch> IMUresult;

	//---------------------循环读取数据----------------------------
	//临时储存一条数据
	INS::IMUResultEpoch epoch_data;
	//记录一条数据中读取到的元素个数
	size_t elementsRead;

	while ((elementsRead = fread(&epoch_data, 8, 10, fp_ins)) == 10)
	{
		if (sizeof(epoch_data) != 80)
		{
			std::cout << "错误：结构体大小不是80字节！" << std::endl;
			return;
		}
		IMUresult.push_back(epoch_data);
	}

	// 处理文件末尾情况
	if (elementsRead > 0 && elementsRead < 10)
	{
		printf("警告：文件末尾有不完整数据，跳过了 %zu 个元素\n", elementsRead);
	}

	//----------------------关闭文件，提示结束---------------------
	fclose(fp_ins);
	std::cout << BinResultfile << "文件读取完毕" << "共读取" << IMUresult.size() << "条数据" << std::endl;

	//输出结果展示
	std::ofstream outfile;
	outfile.open(displayfile, std::ios::out);
	if (!outfile.is_open())
	{
		std::cerr << "Error: could not open file " << displayfile << std::endl;
		return;
	}
	// 输出IMU数据
	// 设置输出精度，保留足够的小数位
	outfile << std::fixed << std::setprecision(12);
	for (const auto& data : IMUresult)
	{
		outfile << data.time << " ";
		for (int i = 0; i < 3; i++)
		{
			outfile << data.BLH[i] << " ";
		}
		for (int i = 0; i < 3; i++)
		{
			outfile << data.Vn[i] << " ";
		}
		for (int i = 0; i < 3; i++)
		{
			outfile << data.YPR[2 - i] << " ";//将原数据格式改为航向、俯仰、横滚顺序输出
		}
		outfile << std::endl;
	}
	std::cout << "惯导解算结果展示完成！数据输出到文件" << displayfile << ",共展示" << IMUresult.size() << "条数据" << std::endl;
}

//读取纯惯导解算结果
void INSResultRead(const char* BinResultfile, std::vector<INS::IMUResultEpoch>& IMUresult)
{
	//--------------------打开要读取的bin文件--------------------
	FILE* fp_ins;
	errno_t err = fopen_s(&fp_ins, BinResultfile, "rb");
	if (err != 0 || !fp_ins)
	{
		printf("无法打开文件: %s, 错误码: %d\n", BinResultfile, err);
		return;
	}
	IMUresult.clear();
	size_t elementsRead = 0;
	//---------------------循环读取数据----------------------------
	while (!feof(fp_ins))
	{
		//临时储存一条数据
		INS::IMUResultEpoch epoch_data;
		//记录一条数据中读取到的元素个数
		
		//先读取time字段
		size_t read_count = fread(&epoch_data.time, sizeof(double), 1, fp_ins);
		if (read_count != 1) break; // 如果没读到数据，退出循环
		elementsRead += read_count;
		//读取BLH
		read_count = fread(&epoch_data.BLH, sizeof(double), 3, fp_ins);
		if (read_count != 3) break;
		elementsRead += read_count;
		//读取Vn
		read_count = fread(&epoch_data.Vn, sizeof(double), 3, fp_ins);
		if (read_count != 3) break;
		elementsRead += read_count;
		//注意：这里YPR[2]对应Roll，YPR[1]对应Pitch，YPR[0]对应Yaw
		read_count = fread(&epoch_data.YPR[2], sizeof(double), 1, fp_ins);
		if (read_count != 1) break;
		elementsRead += read_count;
		read_count = fread(&epoch_data.YPR[1], sizeof(double), 1, fp_ins);
		if (read_count != 1) break;
		elementsRead += read_count;
		read_count = fread(&epoch_data.YPR[0], sizeof(double), 1, fp_ins);
		if (read_count != 1) break;
		elementsRead += read_count;
		elementsRead = 0; // 重置计数器
		//将读取到的数据存入向量
		IMUresult.push_back(epoch_data);
	}
	// 处理文件末尾情况
	if (elementsRead > 0 && elementsRead < 10)
	{
		printf("警告：文件末尾有不完整数据，跳过了 %zu 个元素\n", elementsRead);
	}
	//----------------------关闭文件，提示结束---------------------
	fclose(fp_ins);
	std::cout << BinResultfile << "文件读取完毕" << "共读取" << IMUresult.size() << "条数据" << std::endl;
}

//读取惯导解算参考结果
void INSRefResultRead(const char* Resultfile, std::vector<INS::IMUResultEpoch>& IMUrefresult)
{
	//打开参考结果文件
	std::ifstream infile;
	infile.open(Resultfile, std::ios::in);
	if (!infile.is_open())
	{
		std::cerr << "Error: could not open file " << Resultfile << std::endl;
		return;
	}
	//输出向量清空
	IMUrefresult.clear();
	std::string line;//存储每行数据
	std::string dummy;//存储不需要的数据
	//循环读取数据
	while (getline(infile, line))
	{
		std::istringstream iss(line);
		INS::IMUResultEpoch epoch_data;
		iss >> dummy; //跳过第一列解算状态标志
		iss >> epoch_data.time;
		for (int i = 0; i < 3; i++)
		{
			iss >> epoch_data.BLH[i];
		}
		for (int i = 0; i < 3; i++)
		{
			iss >> epoch_data.Vn[i];
		}
		for (int i = 0; i < 3; i++)
		{
			
			
			if (i == 2)
			{
				double angle;
				iss >> angle;
				if (angle>180.0)
				{
					epoch_data.YPR[2 - i] = angle - 360.0;

				}
				else
				{
					epoch_data.YPR[2 - i] = angle;
				}

			}
			else
			{
				iss >> epoch_data.YPR[2 - i];
			}
			//将参考结果格式改为与计算结果一致，顺序为航向、俯仰、横滚
		}
		IMUrefresult.push_back(epoch_data);
	}
	infile.close();
	std::cout << Resultfile << "参考结果文件读取完毕" << "共读取" << IMUrefresult.size() << "条数据" << std::endl;
}