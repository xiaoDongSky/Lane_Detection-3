#ifndef _DETECTLANE_H_
#define _DETECTLANE_H_

extern ULONG DetectLane(Mat& srcFrame, Mat& bindFrame, vector<vector<Point> >& clusters, vector<LANE>& clusterFittedLanes);
extern ULONG SelectLane(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<LANE>& lastLanes, vector<LANE>& lanes);
extern ULONG ResizeRightLanes(vector<LANE>& rightLanes, INT offset);
extern ULONG SelectLaneBaseOnMultiFrames(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<LANE>& lastLeftlanes, vector<LANE>& lastRightLanes);
extern ULONG GetMidLanes(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<LANE>& midLanes);

#endif
