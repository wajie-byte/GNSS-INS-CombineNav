一、使用的编程环境介绍
1. 核心解算程序
本次自编的松组合导航解算程序采用 C++ 语言编写。使用的编程工具及版本号如下：
开发工具：Microsoft Visual Studio Community 2022 (64 位) - Current
版本号：Version 17.14.21 (November 2025)
依赖库：程序使用了 Eigen 矩阵运算库（Eigen-3.4.0 版本），该库已包含在项目源文件的三方目录（ThirdParty）中，无需额外安装配置。

2. 辅助处理与绘图程序
为了对比分析自编程序与商业软件（POSMind）的解算结果，并绘制误差时序图，本次使用了 MATLAB 作为辅助编程工具。
辅助工具：MATLAB R2024b
辅助功能：包括读取 .nav 结果文件、计算自编程序与 POSMind 结果的差值、绘制位置/速度/姿态时序误差图，以及轨迹对比图。
二、提交的自编程序包的文件结构介绍
1. 源文件列表
本次提交的编程项目包基于 Visual Studio 2022 搭建，项目整体采用模块化文件夹管理。项目根目录下包含各功能文件夹及入口源文件，具体文件结构如下：
（1）根目录源文件
main.cpp：UDP数据回放转发端的主程序入口（包含命令行参数解析）。
main_gnssins.cpp：组合导航核心解算程序的主入口。
main_ins.cpp：纯惯性导航解算测试程序的主入口。

（2）基础通用模块（BASE/ 文件夹）
DefData.cpp、DefData.h：定义IMU和GNSS的基础数据结构，以及文件读取接口。
IMUFile.cpp、IMUFile.h：IMU原始数据文件的读取与预处理函数。

（3）网络通信模块（DataSend/ 文件夹）
DataSend.h：UDP数据发送类的定义与实现（用于伪实时转发）。
DataReceive.h：UDP数据接收与解析类的定义与实现（用于实时解算接收）。
json.hpp：JSON解析第三方头文件（辅助通信数据组包解析）。

（4）底层数学与坐标转换模块（IMU/、MAT/ 文件夹）
IMU/Cal.cpp、Cal.h：惯性导航相关的坐标转换、姿态矩阵计算等辅助函数。
MAT/Mat.cpp、Math.h：基础矩阵运算或数学工具函数。

（5）惯性导航机械编排与动态模型（INS/ 文件夹）
Dynamic.cpp、Dynamic.h：包含地球自转、重力模型、曲率半径计算及惯导动态方程相关函数。
INS.cpp、INS.h：惯性导航解算的核心状态管理与数据处理。
（6）组合导航核心算法模块（INS_GNSS/ 文件夹）
该文件夹是本次松组合算法的主要实现区域，包含以下核心文件：
Param.h：定义 D2R、R2D、地球自转角速度等全局宏常量。

CombConfig.cpp、CombConfig.h：组合导航配置类实现，管理路径、开关、初始状态及噪声参数。
Combinenav.h：组合导航总体数据结构声明（KalmanFilter、NavState、ZUPTDetector等）。
Initialize.cpp：实现卡尔曼滤波器和导航状态的初始化（P0、Qc、初始位置姿态）。
InsMech.cpp：实现惯性导航机械编排算法（姿态、速度、位置更新）。
InsPropagate.cpp：实现误差状态卡尔曼滤波的状态转移矩阵（PHI）构建与协方差预测（P_pred）。
GNSSUpdate.cpp：实现GNSS位置/速度量测更新、NHC非完整性约束更新及ZUPT零速修正更新。
ErrorFeedBack.cpp：实现卡尔曼滤波误差状态到导航状态的闭环反馈校正。
DataProcess.cpp：实现GNSS数据按时间范围的截取与过滤。
GetFileRead.cpp：实现多种格式GNSS数据的读取与列数自适应判断。
SaveResult.cpp：实现解算结果（导航状态、IMU误差、标准差）的输出保存。
interpolate.cpp：实现GNSS历元时间戳与IMU数据之间的时间插值同步。
gnssins.cpp：松组合导航主处理流程实现（数据加载、时序同步、滤波循环）。
gnssins_realtime.cpp：伪实时组合导航处理流程（与UDP接收端对接）。
gnssins_backward.cpp：反向平滑滤波处理实现（含RTS平滑器函数）。

（7）第三方依赖库（ThirdParty/ 文件夹）

eigen-3.4.0/：包含Eigen矩阵运算库的全部头文件，用于支撑卡尔曼滤波中矩阵乘法和求逆运算。


2.编译成功截图


（注：进行调试时使用 Debug 模式会引起 Eigen 库大量的边界检查，导致解算时间异常长。若需要快速得到解算结果，请务必切换至 Release 模式。）


三、可执行程序使用介绍
1. 可执行程序文件名与运行方式
自编可执行程序文件名为：[GINS.exe]。
运行方式：
该程序为控制台应用程序，直接双击运行即可。程序启动后会自动读取预设的默认数据路径。
若要修改参数，可以找到同一路径下的config.txt文件修改。
可修改的配置参数如下：
一、文件路径配置
imufile：IMU数据文件路径。
gnssfile：GNSS数据文件路径。
odofile：里程计数据文件路径（可选）。
outputfolder：结果输出文件夹路径。
二、功能开关
usegnssvel：是否使用GNSS速度观测，可选true或false。
useodo：是否使用里程计，可选true或false。
usenhc：是否使用NHC非完整性约束，可选true或false。
usezupt：是否使用零速约束，可选true或false。
backfilter：是否使用后向滤波，可选true或false。
三、组合导航方法
useRealTime：是否使用实时组合导航，可选true或false。
四、时间配置（单位：秒）
starttime：数据开始时间，单位为GPS周秒。
endtime：数据结束时间，单位为GPS周秒。
五、初始状态
init_pos：初始位置，依次为纬度（度）、经度（度）、高度（米）。
init_vel：初始速度，依次为北向、东向、天向速度（米/秒）。
init_att：初始姿态，依次为横滚角、俯仰角、航向角（度）。
init_pos_std：位置标准差，依次为纬度、经度、高度方向（米）。
init_vel_std：速度标准差，依次为北向、东向、天向（米/秒）。
init_att_std：姿态标准差，依次为横滚、俯仰、航向（度）。
六、初始IMU误差参数
init_gyrobias：初始陀螺零偏，依次为X、Y、Z轴（度/小时）。
init_accbias：初始加速度计零偏，依次为X、Y、Z轴（毫伽）。
init_gyroscale：初始陀螺刻度因子误差，依次为X、Y、Z轴（ppm）。
init_accscale：初始加速度计刻度因子误差，依次为X、Y、Z轴（ppm）。
init_gyrobias_std：陀螺零偏标准差，依次为X、Y、Z轴（度/小时）。
init_accbias_std：加速度计零偏标准差，依次为X、Y、Z轴（毫伽）。
init_gyroscale_std：陀螺刻度标准差，依次为X、Y、Z轴（ppm）。
init_accscale_std：加速度计刻度标准差，依次为X、Y、Z轴（ppm）。
七、IMU噪声参数
gyro_arw：陀螺角度随机游走（度/根号小时）。
accel_vrw：加速度计速度随机游走（米/秒/根号小时）。
gyrobias_std：陀螺零偏稳定性（度/小时）。
accbias_std：加速度计零偏稳定性（毫伽）。
gyroscale_std：陀螺刻度因子稳定性（ppm）。
accscale_std：加速度计刻度因子稳定性（ppm）。
corrtime：相关时间（小时）。
八、安装参数
gnss_lever：GNSS天线杆臂，依次为X、Y、Z方向（米）。
odo_lever：里程计杆臂，依次为X、Y、Z方向（米）。
gnss_install_angle：GNSS安装角，依次为横滚、俯仰、航向（度）。
odo_install_angle：里程计安装角，依次为横滚、俯仰、航向（度）。
九、观测噪声
zupt_std：ZUPT约束标准差（米/秒）。
十、阈值参数
gnss_pos_threshold：GNSS位置跳变检测阈值（米）。
gnss_vel_threshold：GNSS速度跳变检测阈值（米/秒）。
odo_vel_threshold：里程计速度跳变检测阈值（米/秒）。
nhc_threshold：NHC约束阈值（米/秒）。
zupt_acc_threshold：ZUPT加速度检测阈值（米/秒²）。
zupt_gyro_threshold：ZUPT陀螺检测阈值（弧度/秒）。
修改完成后保存config.txt文件，重新运行GINS.exe即可生效。


伪实时转发程序为Transmitdata.exe，配置文件为udp_config.ini，放置路径与exe相同。
可修改的配置参数如下：
一、文件路径配置
gnss_file：GNSS数据文件路径。
imu_file：IMU数据文件路径。
二、网络配置
target_ip：UDP数据包发送的目标IP地址。
target_port：UDP数据包发送的目标端口号。
三、转发参数
speed_multiplier：数据播放速度倍率，数值大于0，1.0表示正常速度，2.0表示两倍速播放。
auto_start：程序启动后是否自动开始转发数据，可选true或false。
imu_frequency：IMU数据的采样频率，单位为Hz。
gnss_frequency：GNSS数据的采样频率，单位为Hz。
四、数据过滤
start_time：数据播放的开始时间，单位为秒，从数据文件中的时间戳进行截取。
end_time：数据播放的结束时间，单位为秒，从数据文件中的时间戳进行截取。
filter_by_time：是否启用时间过滤功能，可选true或false，设为true时仅播放start_time到end_time之间的数据。
修改完成后保存udp_config.ini文件，重新运行Transmitdata.exe即可生效。


2. 运行时所需的数据文件
GINS.exe和Transmitdata.exe运行时，需要读取以下数据文件：
IMU 数据文件：文本格式，包含 7 列数据（Time, gx, gy, gz, ax, ay, az），文件名为 IMUdata.txt。
GNSS 数据文件：文本格式，包含 13 列数据（Time, lat, lon, alt, pos_std_lat, pos_std_lon, pos_std_alt, vn, ve, vd, vel_std_vn, vel_std_ve, vel_std_vd），文件名为 GNSSdata.txt。
说明：以上数据文件由实习工具软件转换后的最终文本格式。

3. 运行成功与结果复现

控制台输出截图：


结果复现：执行完毕后，程序会在 [请填入输出路径，如 D:/visual studio/INS/dataset/output/new] 下生成结果文件（NavResult.nav、ImuError.txt、NavSTD.txt）。使用 MATLAB 辅助脚本读取该结果文件，即可复现实习报告中对应图表的轨迹与误差时序图。

四、其他辅助程序使用介绍
1. 结果比较和画图辅助程序使用方法
本次辅助画图程序由 MATLAB 脚本编写，包含以下文件：
脚本文件名：[compareNavResults.m]
作用与功能：
该脚本是本项目中最为核心的误差评估辅助工具。它能够将 POSMind 商业软件生成的“参考真值”与自编程序生成的“结果文件”进行精确对比。脚本会自动执行以下操作：
自动坐标对齐：以 POSMind 的起点为基准，将两套数据统一转换为 NED 局部米坐标系。
时间轴插值同步：即使两套数据的时间戳存在微小偏差（如零点几毫秒），也会通过线性插值将自编程序的数据对齐到 POSMind 的时间轴上，保证残差计算的物理客观性。
角度卷绕修正：针对横滚角（Roll）和航向角（Yaw）可能存在的跨越 ±180° 的边界跳变，脚本内置了相位展开修正算法。

使用方法：
准备数据：确保有两份 .nav 文件（一份是 POSMind 导出的参考结果，一份是自编程序导出的结果）。


调用函数：在 MATLAB 命令行中，按照以下格式调用该函数：
compareNavResults('POSMind文件路径', '自编程序文件路径', '图片保存路径');
例如：compareNavResults('result/POSMind/ref.nav', 'result/my_loose.nav', 'image/Compare/');
自动输出：脚本运行后，将在指定的保存路径下自动生成 4 张图表：
trajectory_2d.png（二维轨迹对比图）
pos_error_3subplots.png（北、东、地三轴位置误差时序图）
vel_error_3subplots.png（北、东、地三轴速度误差时序图）
att_error_3subplots.png（横滚、俯仰、航向三轴姿态误差时序图）

结果统计：脚本运行结束时，还会在 MATLAB 命令窗口输出“平面位置误差 RMS”（均方根误差）的数值，该数值可直接用于报告中的精度评定。

注意事项：
该脚本假设 POSMind 导出文件为 10列（[TOW, lat, lon, h, vE, vN, vU, roll, pitch, yaw]），自编程序文件为 11列（[week, TOW, lat, lon, h, vN, vE, vD, roll, pitch, yaw]）。若实际列数与假设不符，需调整代码中的索引列。
若遇到姿态误差跨越 ±180° 边界出现锯齿时，脚本会自动依据条件判断进行自动卷绕修正，确保图形呈现物理上真实的误差波动趋势。



