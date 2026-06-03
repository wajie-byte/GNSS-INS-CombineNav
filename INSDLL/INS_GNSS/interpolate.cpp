#include"Combinenav.h"
#include"../INS/INS.h"
void CombineNav::interpolate(const INS::IMUDataEpoch& lastimu,const INS::IMUDataEpoch& thisimu,double intertime,
    INS::IMUDataEpoch& firstimu,INS::IMUDataEpoch& secondimu)
{
    // 初始化为零
    firstimu = INS::IMUDataEpoch();
    secondimu = INS::IMUDataEpoch();

    // 检查 intertime 是否在 [lastimu.time, thisimu.time] 区间内
    if (intertime >= lastimu.time && intertime <= thisimu.time) 
    {
        // 计算插值系数
        double lambda = (intertime - lastimu.time) / (thisimu.time - lastimu.time);

        // firstimu: 时间 = intertime, 数据 = thisimu.data * lambda
        firstimu.time = intertime;
        for (int i = 0; i < 3; i++) 
        {
            firstimu.gyro[i] = thisimu.gyro[i] * lambda;
            firstimu.accl[i] = thisimu.accl[i] * lambda;
        }

        // secondimu: 时间 = thisimu.time, 数据 = thisimu.data * (1 - lambda)
        secondimu.time = thisimu.time;
        for (int i = 0; i < 3; i++) 
        {
            secondimu.gyro[i] = thisimu.gyro[i] * (1.0 - lambda);
            secondimu.accl[i] = thisimu.accl[i] * (1.0 - lambda);
        }
    }
}