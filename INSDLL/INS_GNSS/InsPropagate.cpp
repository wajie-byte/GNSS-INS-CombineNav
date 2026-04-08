#include"Combinenav.h"


CombineNav::KalmanFilter CombineNav::InsPropagate(const CombineNav::NavState& navstate, const INS::IMUDataEpoch& thisimu, const INS::IMUDataEpoch&, double imudt
	, const CombineNav::KalmanFilter& kf, double corrtime)
{
	CombineNav::KalmanFilter kf_new;
}