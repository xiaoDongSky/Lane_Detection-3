#ifndef _DETECTLANE_H_
#define _DETECTLANE_H_

extern ULONG DetectLane(Mat& srcFrame, Mat& bindFrame, vector<vector<Point> >& clusters, vector<LANE>& clusterFittedLanes);

extern ULONG GetDetectedRoad(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<ROAD>& lastRoads, DOUBLE& roadWidth, ROAD& detectRoad);

extern ULONG ResizeRightLanes(vector<LANE>& rightLanes, INT offset);

extern ULONG SelectLaneBaseOnMultiFrames(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<LANE>& lastLeftlanes, vector<LANE>& lastRightLanes);

extern ULONG GetMidLanes(ROAD& road);

extern ULONG CvtLaneToPolarCoodinate(LANE& lane);

extern ULONG CvtLaneToRectCoordinate(LANE& lane);

#endif
