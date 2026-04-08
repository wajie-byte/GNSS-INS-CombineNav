#pragma once

namespace CombineNav
{
    #define M_PI 3.14159265358979323846

    struct Param 
    {
        // 常数定义
        static constexpr double D2R = M_PI / 180.0;
        static constexpr double R2D = 180.0 / M_PI;
        static constexpr double HOUR_TO_SEC = 3600.0;
        static constexpr double MINUTE_TO_SEC = 60.0;
		static constexpr double wie = 15 * D2R / 3600; // 地球自转角速度(rad/s)
    };
}