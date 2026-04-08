#include"Combinenav.h"
#include"../BASE/DefData.h"

// 按时间范围截取数据的通用函数模板
/*
In:
	double starttime				//起始时间s
	double endtime				//结束时间s
	const std::vector<DataType>& data	//数据序列，要求DataType包含time成员变量
	const std::string& data_name		//用于错误信息的数据名称
Out:
	std::vector<DataType> cut_data	//截取后的数据序列
*/
template<typename DataType>
std::vector<DataType> CutDataByTime(
	double starttime,
	double endtime,
	const std::vector<DataType>& data,
	const std::string& data_name = "Data")  // 用于错误信息
{
	std::vector<DataType> cut_data;

	// 检查时间范围是否合理
	if (data.empty()) {
		std::cout << data_name << " is empty!" << std::endl;
		return cut_data;
	}

	double data_start_time = data.front().time;
	double data_end_time = data.back().time;

	if (starttime >= endtime) {
		std::cout << "Start time is greater than end time!" << std::endl;
		return cut_data;
	}

	if (starttime < data_start_time || endtime > data_end_time) {
		std::cout << "Start time or end time is out of "
			<< data_name << " data range!" << std::endl;
		return cut_data;
	}

	// 寻找起始索引和结束索引
	auto start_it = std::lower_bound(data.begin(), data.end(), starttime,
		[](const DataType& epoch, double t) {
			return epoch.time < t;
		});

	auto end_it = std::upper_bound(data.begin(), data.end(), endtime,
		[](double t, const DataType& epoch) {
			return t < epoch.time;
		});

	// 截取数据
	cut_data = std::vector<DataType>(start_it, end_it);
	return cut_data;
}

// 按时间范围截取IMU数据
/*
In:
	double starttime						//起始时间s
	double endtime						//结束时间s
	const std::vector<IMUDataEpoch>& imudata	//IMU数据序列
Out:
	std::vector<IMUDataEpoch> cut_imudata	//截取后的IMU数据序列
*/
std::vector<INS::IMUDataEpoch> INS::CutImuDataByTime(double starttime, double endtime, const std::vector<INS::IMUDataEpoch>& imudata)
{
	return CutDataByTime(starttime, endtime, imudata, "IMU");
}
// 按时间范围截取GNSS数据
/*
In:
	double starttime						//起始时间s
	double endtime						//结束时间s
	const std::vector<CombineNav::GNSSResult>& gnssdata	//GNSS数据序列
Out:
	std::vector<CombineNav::GNSSResult> cut_gnssdata	//截取后的GNSS数据序列
*/
std::vector<CombineNav::GNSSResult> CombineNav::CutGnssDataByTime(double starttime, double endtime, const std::vector<GNSSResult>& gnssdata)
{
	return CutDataByTime(starttime, endtime, gnssdata, "GNSS");
}