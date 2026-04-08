#include"Combinenav.h"
#include"CombConfig.h"
#include"../INS/Dynamic.h"

CombineNav::KalmanFilter CombineNav::KalmanFilter::Initialize(const Config& config)
{
	int x_rank = 21, noise_rank = 18;
	KalmanFilter kf(x_rank, noise_rank);
	//Qc
	kf.Qc(0, 0) = kf.Qc(1, 1) = kf.Qc(2, 2) = config.imu_noise.accel_vrw * config.imu_noise.accel_vrw;
	kf.Qc(3, 3) = kf.Qc(4, 4) = kf.Qc(5, 5) = config.imu_noise.gyro_arw * config.imu_noise.gyro_arw;
	kf.Qc(6, 6) = kf.Qc(7, 7) = kf.Qc(8, 8) = 2 * config.imu_noise.gyrobias_std * config.imu_noise.gyrobias_std / config.imu_noise.corrtime;
	kf.Qc(9, 9) = kf.Qc(10, 10) = kf.Qc(11, 11) = 2 * config.imu_noise.accbias_std * config.imu_noise.accbias_std / config.imu_noise.corrtime;
	kf.Qc(12, 12) = kf.Qc(13, 13) = kf.Qc(14, 14) = 2 * config.imu_noise.gyroscale_std * config.imu_noise.gyroscale_std / config.imu_noise.corrtime;
	kf.Qc(15, 15) = kf.Qc(16, 16) = kf.Qc(17, 17) = 2 * config.imu_noise.accscale_std * config.imu_noise.accscale_std / config.imu_noise.corrtime;

	//P0
	for (int i = 0; i < 3; i++)
	{
		kf.P(i, i) = config.init_state.init_pos_std[i] * config.init_state.init_pos_std[i];
		kf.P(i + 3, i + 3) = config.init_state.init_vel_std[i] * config.init_state.init_vel_std[i];
		kf.P(i + 6, i + 6) = config.init_state.init_att_std[i] * config.init_state.init_att_std[i];
		kf.P(i + 9, i + 9) = config.init_imu_errors.init_gyrobias_std[i] * config.init_imu_errors.init_gyrobias_std[i];
		kf.P(i + 12, i + 12) = config.init_imu_errors.init_accbias_std[i] * config.init_imu_errors.init_accbias_std[i];
		kf.P(i + 15, i + 15) = config.init_imu_errors.init_gyroscale_std[i] * config.init_imu_errors.init_gyroscale_std[i];
		kf.P(i + 18, i + 18) = config.init_imu_errors.init_accscale_std[i] * config.init_imu_errors.init_accscale_std[i];
	}

	return kf;

}

CombineNav::NavState CombineNav::NavState::Initialize(const Config& config)
{
	NavState navstate;
	navstate.time = config.time.starttime;
	for (int i = 0; i < 3; i++)
	{
		navstate.pos[i] = config.init_state.init_pos[i];
		navstate.vel[i] = config.init_state.init_vel[i];
		navstate.att[i] = config.init_state.init_att[i];
		navstate.gyro_bias[i] = config.init_imu_errors.init_gyrobias[i];
		navstate.acc_bias[i] = config.init_imu_errors.init_accbias[i];
		navstate.gyro_scale[i] = config.init_imu_errors.init_gyroscale[i];
		navstate.acc_scale[i] = config.init_imu_errors.init_accscale[i];
	}
	navstate.Cbn = Cal_C_b_n(config.init_state.init_att[2], config.init_state.init_att[1], config.init_state.init_att[0]);
	navstate.qbn = euler2quaternion(config.init_state.init_att[2], config.init_state.init_att[1], config.init_state.init_att[1]);
	return navstate;
}