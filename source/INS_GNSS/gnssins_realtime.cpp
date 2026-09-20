#include"Combinenav.h"
#include"CombConfig.h"
#include"../INS/INS.h"
#include"../BASE/IMUFile.h"
#include"../DataSend/DataSend.h"
#include"../DataSend/DataReceive.h"
#include <iomanip>




//bool CombineNav::





void CombineNav::gnssins_realtime(CombineNav::Config config)
{
    CombineNav::Param param;
    
    SimpleUdp udp("127.0.0.1", 5005);   // 发到本机5005端口

    //set save folder
    std::string navpath = config.files.outputfolder + "/NavResult";
    if (config.switches.usegnssvel)
    {
        navpath += "_GNSSVEL";
    }
    navpath += ".nav";
    std::string imuerrpath = config.files.outputfolder + "/ImuError.txt";
    std::string stdpath = config.files.outputfolder + "/NavSTD.txt";


    

    std::vector<GNSSResult> gnss_list;
    std::vector<INS::IMUdata> imu_list;
    std::mutex mtx;
    std::atomic<bool> running{ true };  // 添加退出控制

    // 创建接收器
    UDPReceiver receiver(8888);

    // 设置回调 - 数据存到外面的数组
    receiver.setCallback([&](const std::string& data) {
        if (data.find("GNSS") == 0) {
            GNSSResult gnss;
            std::istringstream iss(data);
            std::string token;
            int index = 0;

            while (std::getline(iss, token, ',')) {
                switch (index) {
                case 1: gnss.time = std::stod(token); break;
                case 2: gnss.pos[0] = std::stod(token)*param.D2R; break;
                case 3: gnss.pos[1] = std::stod(token)*param.D2R; break;
                case 4: gnss.pos[2] = std::stod(token); break;
                case 5: gnss.pos_std[0] = std::stod(token); break;
                case 6: gnss.pos_std[1] = std::stod(token); break;
                case 7: gnss.pos_std[2] = std::stod(token); break;
                case 8: gnss.vel[0] = std::stod(token); break;
                case 9: gnss.vel[1] = std::stod(token); break;
                case 10: gnss.vel[2] = std::stod(token); break;
                case 11: gnss.vel_std[0] = std::stod(token); break;
                case 12: gnss.vel_std[1] = std::stod(token); break;
                case 13: gnss.vel_std[2] = std::stod(token); break;
                default: break;
                }
                index++;
            }

            std::lock_guard<std::mutex> lock(mtx);
            gnss_list.push_back(gnss);
            std::cout << "[GNSS] #" << gnss_list.size()
                << " time=" << std::fixed << std::setprecision(3) << gnss.time << std::endl;
        }
        else if (data.find("IMU") == 0) {
            INS::IMUdata imu;
            std::istringstream iss(data);
            std::string token;
            int index = 0;

            while (std::getline(iss, token, ',')) {
                switch (index) {
                case 1: imu.IMUtime = std::stod(token); break;
                case 2: imu.gyrox = std::stod(token) * param.D2R; break;
                case 3: imu.gyroy = std::stod(token) * param.D2R; break;
                case 4: imu.gyroz = std::stod(token) * param.D2R; break;
                case 5: imu.accx = std::stod(token); break;
                case 6: imu.accy = std::stod(token); break;
                case 7: imu.accz = std::stod(token); break;
                default: break;
                }
                index++;
            }

            std::lock_guard<std::mutex> lock(mtx);
            imu_list.push_back(imu);

            if (imu_list.size() % 100 == 0) {
                std::cout << "[IMU] #" << imu_list.size()
                    << " time=" << std::fixed << std::setprecision(3) << imu.IMUtime << std::endl;
            }
        }
        });

    // ========== 启动接收 ==========
    receiver.start();
    std::cout << "UDP接收已启动，开始组合导航处理...\n" << std::endl;

    // ========== 主处理循环 ==========
    CombineNav::KalmanFilter kf(21, 18);
    kf = kf.Initialize(config);
    CombineNav::NavState navstate;
    navstate = navstate.Initialize(config);
	CombineNav::NavState laststate = navstate;
    INS::IMUdata last_imu;
	INS::IMUdata this_imu;
    INS::IMUDataEpoch lastimu;
    INS::IMUDataEpoch thisimu;
    Eigen::MatrixXd PHI_current(21, 21);   // 用于接收 InsPropagate 输出的 PHI

    bool has_last_imu = false;
    bool has_lastimu = false;
    size_t gnssindex = 0;

    //打开文件
    std::ofstream navfp(navpath);
    if (!navfp.is_open()) {
        std::cout << "Cannot open " << navpath << std::endl;
        return;
    }

    std::ofstream imuerrfp(imuerrpath);
    if (!imuerrfp.is_open()) {
        std::cout << "Cannot open " << imuerrpath << std::endl;
        return;
    }

    std::ofstream stdfp(stdpath);
    if (!stdfp.is_open()) {
        std::cout << "Cannot open " << stdpath << std::endl;
        return;
    }

    bool was_empty = false;
    bool has_received_data = false;  // 是否收到过数据
    auto first_empty_time = std::chrono::steady_clock::now();

    while (running)  // 用 running 控制退出
    {
        //  从缓冲区复制数据（加锁，快速复制）
        std::vector<GNSSResult> gnss_local;
        std::vector<INS::IMUdata> imu_local;
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (!gnss_list.empty() || !imu_list.empty())
            {
                gnss_local = gnss_list;
                imu_local = imu_list;
                gnss_list.clear();
                imu_list.clear();
				gnssindex = 0;//重置GNSS索引
            }
        }  //  锁在这里释放，后面处理数据不在锁里面

        //  处理数据（在锁外面）
        if (!imu_local.empty())
        {
            for (size_t i = 0; i < imu_local.size(); i++)
            {
                //更新状态
                last_imu = this_imu;
                laststate = navstate;
                lastimu = thisimu;


                this_imu = imu_local[i];
                //获取last_imu初值
                if (!has_last_imu) 
                {
                    last_imu = this_imu;
                    has_last_imu = true;
                    continue;
                }

                // 计算时间间隔
                double imudt = this_imu.IMUtime - last_imu.IMUtime;
                thisimu.time = this_imu.IMUtime;
                thisimu.accl[0] = this_imu.accx * imudt;
                thisimu.accl[1] = this_imu.accy * imudt;
                thisimu.accl[2] = this_imu.accz * imudt;
                thisimu.gyro[0] = this_imu.gyrox * imudt;
                thisimu.gyro[1] = this_imu.gyroy * imudt;
                thisimu.gyro[2] = this_imu.gyroz * imudt;

				//获取lastimu初值
                if (!has_lastimu && has_last_imu)
                {
                    lastimu = thisimu;
                    has_lastimu = true;
                    continue;
                }

                for (int i = 0; i < 3; i++)//IMU数据补偿
                {
                    thisimu.gyro[i] = (thisimu.gyro[i] - navstate.gyro_bias[i] * imudt) / (1 + navstate.gyro_scale[i]);
                    thisimu.accl[i] = (thisimu.accl[i] - navstate.acc_bias[i] * imudt) / (1 + navstate.acc_scale[i]);
                }


                
                while(!gnss_local.empty()&&gnssindex < gnss_local.size() && gnss_local[gnssindex].time <= lastimu.time)
                {
					gnssindex++;
				}

                if (gnssindex >= gnss_local.size())
                {
                    //  没有GNSS数据，只做IMU传播
                    double imudt_prop = thisimu.time - lastimu.time;
                    navstate = InsMech(laststate, lastimu, thisimu);
                    InsPropagate(navstate, thisimu, imudt_prop, kf, config.imu_noise.corrtime, PHI_current);
                }
                //determine if GNSS update is needed
                else if (lastimu.time == gnss_local[gnssindex].time)
                {
                    //do GNSS update for the current state
                    CombineNav::GNSSResult thisgnss = gnss_local[gnssindex];
                    imudt = thisimu.time - lastimu.time;
                    CombineNav::GNSSUpdate(kf, navstate, config, thisgnss, thisimu, imudt);
                    CombineNav::ErrorFeedBack(navstate, kf);
                    //index++ 
                    gnssindex++;
                    laststate = navstate;

                    //do propagation for current imu data
                    imudt = thisimu.time - lastimu.time;
                    navstate = InsMech(laststate, lastimu, thisimu);
                    InsPropagate(navstate, thisimu, imudt, kf, config.imu_noise.corrtime, PHI_current);
                }

                else if (lastimu.time<gnss_local[gnssindex].time && thisimu.time>gnss_local[gnssindex].time)
                {
                    //ineterpolate imu to gnss time
                    INS::IMUDataEpoch firstimu, secondimu;
                    interpolate(lastimu, thisimu, gnss_local[gnssindex].time, firstimu, secondimu);
                    //do propagation for first imu
                    imudt = firstimu.time - lastimu.time;
                    navstate = InsMech(laststate, lastimu, firstimu);
                    InsPropagate(navstate, firstimu, imudt, kf, config.imu_noise.corrtime, PHI_current);
                    //do gnss update
                    CombineNav::GNSSResult thisgnss = gnss_local[gnssindex];
                    CombineNav::GNSSUpdate(kf, navstate, config, thisgnss, firstimu, imudt);
                    CombineNav::ErrorFeedBack(navstate, kf);
                    //index++
                    gnssindex++;
                    laststate = navstate;
                    lastimu = firstimu;
                    //do propagation for second imu
                    imudt = secondimu.time - lastimu.time;
                    navstate = InsMech(laststate, lastimu, secondimu);
                    InsPropagate(navstate, secondimu, imudt, kf, config.imu_noise.corrtime, PHI_current);
                }
                else
                {
                    // only do propagation
                    // INS mechanization
                    navstate = InsMech(laststate, lastimu, thisimu);
                    //error propagation
                    InsPropagate(navstate, thisimu, imudt, kf, config.imu_noise.corrtime, PHI_current);
                }

                if(config.switches.usenhc)
                {
                    //do NHC update
                    CombineNav::NHCUpdate(kf, navstate, config, thisimu, imudt);
                    CombineNav::ErrorFeedBack(navstate, kf);
				}


                //save data
                //write navresult to file
                SaveNavResult(navfp, navstate, param);
                //write imu error to file
                SaveIMUError(imuerrfp, navstate, param);
                //write navstate std to file
                SaveStateStd(stdfp, kf, navstate, param);

                //send data
                // 一行发送
                udp.send(navstate.time, navstate.pos[0], navstate.pos[1], navstate.pos[2], navstate.vel[0], navstate.vel[1], navstate.vel[2], navstate.att[0], navstate.att[1], navstate.att[2]);
              

                

                
            }
        }

        // 3. 没有数据时
        if (imu_local.empty() && gnss_local.empty())
        {
            // 如果还没收到过任何数据，不超时，一直等
            if (!has_received_data)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;  // 跳过超时检测
            }

            if (!was_empty)
            {
                first_empty_time = std::chrono::steady_clock::now();
                was_empty = true;
            }

            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - first_empty_time).count();

            if (elapsed >= 5)
            {
                std::cout << "\n[INFO] 缓冲区已空" << elapsed << "秒，处理结束" << std::endl;
                running = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else
        {
            has_received_data = true;  // 收到数据了
            was_empty = false;
        }

        // 4. 检查退出条件（按ESC退出）
        // 如果不用ESC，可以用其他方式
        // 这里简单判断，您可以用 std::cin 或其他方式
    }
    std::cout << "正在停止接收器..." << std::endl;  
    // ========== 停止接收 ==========
    receiver.stop();
	navfp.close();
	imuerrfp.close();
	stdfp.close();
    std::cout << "处理结束！" << std::endl;
    return;
}