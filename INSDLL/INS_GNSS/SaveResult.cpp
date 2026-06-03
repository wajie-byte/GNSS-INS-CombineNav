#include"CombineNav.h"
#include"CombConfig.h"
#include <fstream>
#include <iomanip>

//保存导航结果
/*
In:
    navfp: 导航结果文件流
    navstate: 当前导航状态
    param: 组合导航参数
 Out:
   无返回值，直接将导航结果写入文件
*/
void CombineNav::SaveNavResult(std::ofstream& navfp, const CombineNav::NavState& navstate, const CombineNav::Param& param)
{
    // 设置格式
    navfp << std::fixed;

    // 直接写入文件，格式化输出导航状态
    navfp << std::setw(2) << 0 << " "
        << std::setw(12) << std::setprecision(6) << navstate.time << " "
        << std::setw(12) << std::setprecision(8) << navstate.pos[0] * param.R2D << " "
        << std::setw(12) << std::setprecision(8) << navstate.pos[1] * param.R2D << " "
        << std::setw(8) << std::setprecision(4) << navstate.pos[2] << " "
        << std::setw(8) << std::setprecision(4) << navstate.vel[0] << " "
        << std::setw(8) << std::setprecision(4) << navstate.vel[1] << " "
        << std::setw(8) << std::setprecision(4) << navstate.vel[2] << " "
        << std::setw(8) << std::setprecision(4) << navstate.att[0] * param.R2D << " "
        << std::setw(8) << std::setprecision(4) << navstate.att[1] * param.R2D << " "
        << std::setw(8) << std::setprecision(4) << navstate.att[2] * param.R2D << std::endl;
}

//保存IMU误差状态
/*
In:
    imuerrfp: IMU误差状态文件流
    navstate: 当前导航状态
    param: 组合导航参数
 Out:
   无返回值，直接将IMU误差状态写入文件
*/
void CombineNav::SaveIMUError(std::ofstream& imuerrfp, const CombineNav::NavState& navstate, const CombineNav::Param& param)
{
    // 设置格式
    imuerrfp << std::fixed;

    // 直接写入，格式：时间 + 3个陀螺仪漂移 + 3个加速度计偏置 
    //       + 3个陀螺仪尺度 + 3个加速度计尺度 + 1个里程计尺度
    imuerrfp << std::setw(12) << std::setprecision(6) << navstate.time << " "
        << std::setw(8) << std::setprecision(4) << navstate.gyro_bias[0] * param.R2D * 3600.0 << " "
        << std::setw(8) << std::setprecision(4) << navstate.gyro_bias[1] * param.R2D * 3600.0 << " "
        << std::setw(8) << std::setprecision(4) << navstate.gyro_bias[2] * param.R2D * 3600.0 << " "
        << std::setw(8) << std::setprecision(4) << navstate.acc_bias[0] * 1e5 << " "
        << std::setw(8) << std::setprecision(4) << navstate.acc_bias[1] * 1e5 << " "
        << std::setw(8) << std::setprecision(4) << navstate.acc_bias[2] * 1e5 << " "
        << std::setw(8) << std::setprecision(4) << navstate.gyro_scale[0] * 1e6 << " "
        << std::setw(8) << std::setprecision(4) << navstate.gyro_scale[1] * 1e6 << " "
        << std::setw(8) << std::setprecision(4) << navstate.gyro_scale[2] * 1e6 << " "
        << std::setw(8) << std::setprecision(4) << navstate.acc_scale[0] * 1e6 << " "
        << std::setw(8) << std::setprecision(4) << navstate.acc_scale[1] * 1e6 << " "
        << std::setw(8) << std::setprecision(4) << navstate.acc_scale[2] * 1e6 << " "
        << std::setw(8) << std::setprecision(4) << navstate.odo_scale * 1e6 << std::endl;
}

//保存导航状态到标准输出
/*
In:
    stdfp: 标准输出文件流
    kf: 卡尔曼滤波器状态
    navstate: 当前导航状态
    param: 组合导航参数
 Out:
   无返回值，直接将导航状态和卡尔曼滤波器状态写入标准输出，格式化输出以便查看
*/
void CombineNav::SaveStateStd(std::ofstream& stdfp, const CombineNav::KalmanFilter& kf, const CombineNav::NavState& navstate, const CombineNav::Param& param)
{
    stdfp << std::fixed << std::setprecision(6);

    // 计算标准差
    double std[23] = { navstate.time };
    for (int idx = 1; idx <= kf.x_rank; idx++)
    {
        std[idx] = sqrt(kf.P(idx - 1, idx - 1));
    }

    // 单位转换
    for (int i = 0; i < 3; i++) {
        std[10 + i] *= param.R2D * 3600.0;  // 姿态
        std[13 + i] *= 1e5;                 // 加速度计偏置
        std[16 + i] *= 1e6;                // 尺度因子
    }
    std[22] *= 1e6;  // 里程计

    // 写入文件
    stdfp << std::setw(12) << std[0] << " ";
    for (int i = 1; i < 23; i++) {
        stdfp << std::setw(8) << std[i] << " ";
    }
    stdfp << std::endl;
}