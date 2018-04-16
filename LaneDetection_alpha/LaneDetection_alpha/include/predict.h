#ifndef _PREDICTLANE_H_
#define _PREDICTLANE_H_

#define KALMAN_STATE_NUM 4
#define KALMAN_MEASUREMENT_NUM 2

#define FIT_NUM 100

typedef struct tagKalmanFilter
{
    KalmanFilter kf;
    Mat measure;

}KALMAN_FILTER;

extern ULONG InitKalman(KALMAN_FILTER& kalmanFilter);

extern ULONG PredictLaneOnKalman(KALMAN_FILTER& kalmanFilter, LANE& detectLane, LANE& lastLane, LANE& predictLane);

extern ULONG GetRealRoad(ROAD& detectRoad, ROAD& predictRoad, DOUBLE& roadWidth, ROAD& realRoad);

extern ULONG PredictRoadWidth(ROAD& detectRoad, INT& predictStep, vector<ROAD>& lastRoads, DOUBLE& lastWidthSum, DOUBLE& roadWidth);

extern ULONG RecordLastRoad(ROAD& detectRoad, ROAD& lastRoad);

extern ULONG PredictCubic(ROAD& detectRoad, vector<ROAD>& lastFittingRoad, ROAD& predictRoad, vector<Point>& fittingPoint);

extern ULONG PredictCurve(ROAD& detectRoad, vector<ROAD>& historyRoads, vector<Mat>& curveCoefficients, ROAD& realRoad);

#endif