#ifndef _PREDICTLANE_H_
#define _PREDICTLANE_H_

extern ULONG SetupKalman(KalmanFilter& kalmanFilter);
extern ULONG PredictKalman(KalmanFilter& kalmanFilter, Point& detectLane, Point& predictPoint);


#define KALMAN_STATE_NUM 6
#define KALMAN_MEASUREMENT_NUM 2

#endif