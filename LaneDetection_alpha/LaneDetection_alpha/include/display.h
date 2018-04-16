#ifndef _DISPLAY_H_
#define _DISPLAY_H_


extern ULONG DisplayFittingPoint(Mat& frame, vector<ROAD>& historyRoad,  vector<Mat>& curveCoefficient);
extern ULONG DisplaySignalLane(Mat& frame, LANE& lane, Scalar color);
extern ULONG DisplayCluster(Mat& frame, vector<vector<Point> >& cluster);
extern ULONG DisplayMultiLanes(Mat& frame, vector<LANE >& lanePoints, Scalar color);
extern ULONG DisplayLastHundredLanes(Mat& frame, LANE& leftFittedtLanes, LANE& rightFittedLanes, LANE& midFittedLanes, 
                                      vector<LANE > & lastLeftHundredLane, vector<LANE >& lastRightHundredLane, vector<LANE >& lastMidHundredLane);
extern ULONG DisplayLastHundredLanes(Mat& frame, LANE& leftLanes, LANE& rightLanes, vector<LANE>& miLanes, 
                              vector<vector<LANE> > & lastLeftHundredLane, vector<vector<LANE> >& lastRightHundredLane, vector<vector<LANE> >& lastMidHundredLane);
extern ULONG DisplayLastRoadXPos(Mat& frame, vector<ROAD>& lastRoad, vector<ROAD>& lastPredictRoad, ROAD& predictRoad);
extern ULONG DisplayRoad(Mat& frame, ROAD& road, INT directDeparture);

#endif