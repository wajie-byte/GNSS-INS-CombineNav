#include"Combinenav.h"

//创建并打开读取文件
/*
In:
	std::string filepath		//文件路径
Out:
	std::ifstream infile	//已打开的文件流
*/
std::ifstream CombineNav::CreateReadFile(const std::string filepath)
{
	std::ifstream infile;
	infile.open(filepath, std::ios::in);
	if (!infile.is_open())
	{
		std::cout << "Error: could not open file " << filepath << std::endl;
		return std::ifstream(); // 返回一个无效的ifstream对象
	}
	return infile;// 返回已打开的文件流
}

//读取GNSS数据
/*
In:
	std::ifstream& infile	//已打开的文件流
	std::vector<GNSSResult>& gnssdata	//存储GNSS数据的向量
Out:
	void
*/
int CombineNav::GetGNSSData(std::ifstream& infile, std::vector<GNSSResult>& gnssdata)
{
	
	//记录文件当前位置
	std::streampos original_pos = infile.tellg();

	std::string line;//存储每行数据
	std::getline(infile, line);
	std::istringstream iss(line);
	std::string token;
	std::vector<std::string> tokens;
	while(iss>> token)
	{
		tokens.push_back(token);
	}
	const size_t column = tokens.size();

	//恢复文件位置
	infile.clear(); // 清除EOF标志
	infile.seekg(original_pos);

	//根据列数选择不同的读取方式
	if (column == 13)
	{
		CombineNav::GetPosVelData(infile, gnssdata);
		return 13;
	}
	else if(column == 7)
	{
		CombineNav::GetPosData(infile, gnssdata);
		return 7;
	}
	else
	{
		std::cout << "Error: GNSS data format not recognized, column count: " << column << std::endl;
		return -1;
	}

	
}


//读取GNSS数据（有位置速度及对应STD）
/*
In:
	std::ifstream& infile	//已打开的文件流
	std::vector<GNSSResult>& gnssdata	//存储GNSS数据的向量
Out:
	void
*/
void CombineNav::GetPosVelData(std::ifstream& infile, std::vector<GNSSResult>& gnssdata)
{
	gnssdata.clear();
	std::string line;//存储每行数据
	while (std::getline(infile, line))
	{
		std::istringstream iss(line);
		GNSSResult epoch_data;
		iss >> epoch_data.time;
		for (int i = 0; i < 3; i++)
		{
			iss >> epoch_data.pos[i];
		}
		for (int i = 0; i < 3; i++)
		{
			iss >> epoch_data.pos_std[i];
		}
		for (int i = 0; i < 3; i++)
		{
			iss >> epoch_data.vel[i];
		}
		for (int i = 0; i < 3; i++)
		{
			iss >> epoch_data.vel_std[i];
		}
		gnssdata.push_back(epoch_data);
	}


}

//读取GNSS数据（只有有位置及对应STD）
/*
In:
	std::ifstream& infile	//已打开的文件流
	std::vector<GNSSResult>& gnssdata	//存储GNSS数据的向量
Out:
	void
*/
void CombineNav::GetPosData(std::ifstream& infile, std::vector<GNSSResult>& gnssdata)
{
	gnssdata.clear();
	std::string line;//存储每行数据
	while (std::getline(infile, line))
	{
		std::istringstream iss(line);
		GNSSResult epoch_data;
		iss >> epoch_data.time;
		for (int i = 0; i < 3; i++)
		{
			iss >> epoch_data.pos[i];
		}
		for (int i = 0; i < 3; i++)
		{
			iss >> epoch_data.pos_std[i];
		}
		gnssdata.push_back(epoch_data);
	}
}