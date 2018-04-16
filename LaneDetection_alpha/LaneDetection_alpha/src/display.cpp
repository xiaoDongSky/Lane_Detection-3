#include "include/def.h"
#include "include/display.h"
#include "include/predict.h"

/*****************************************************************************
  函 数 名  : DisplayCluster
  功能描述  : 显示分好的簇
  输入参数  :  
  输出参数  :  
  返 回 值  : 
  调用函数  : N/A
  被调函数  : 
  
  修改历史      :
   1.日    期   : 2018年3月6日
     作    者   : ccy0739
     修改内容   : 新生成函数
 *****************************************************************************/
ULONG DisplayCluster(Mat& frame, vector<vector<Point> >& cluster)
{
    ULONG ret = HCOM_OK;
    vector<Point> tmpCluster(1);
    INT randomR;
    INT randomG;
    INT randomB;

    frame.setTo(cv::Scalar(0, 0, 0)); 
    for (size_t i = 0; i < cluster.size(); i++)
    {
        randomR = rand() % 255;
        randomG = rand() % 255;
        randomB = rand() % 255;

        tmpCluster = cluster.at(i);
        for (size_t j = 0; j < tmpCluster.size(); j++)
        {
            circle(frame, tmpCluster[j], 1, Scalar(randomR, randomG, randomB));
        }
    }

    return ret;
}

ULONG DisplayFittingPoint(Mat& frame, vector<ROAD>& historyRoad,  vector<Mat>& curveCoefficient)
{
    ULONG ret = HCOM_OK;

    vector<Point> fitPoints;
    FLOAT y = 0;

    frame = cv::Mat::zeros(WIDTH, FIT_NUM + 100, CV_8UC3);

    if (0 == curveCoefficient.size())
    {
        return ret;
    }

    /* 得到拟合的曲线 */
    for (INT x = 0; x < FIT_NUM; x++)
    {
        for (INT i = 0; i < curveCoefficient.size(); i++)
        {
            y = curveCoefficient[i].at<DOUBLE>(0, 0) + curveCoefficient[i].at<DOUBLE>(1, 0) * x +  
                curveCoefficient[i].at<DOUBLE>(2, 0)*std::pow(x, 2) + curveCoefficient[i].at<DOUBLE>(3, 0)*std::pow(x, 3);  
            fitPoints.push_back(Point(x, y));
        }
    }

    for (INT i = 0; i < historyRoad.size(); i++)
    {      
        //circle(frame, Point(i, historyRoad[i].leftLane.headPoint.x) , 1, Scalar(0,0,255), -1);    
        //circle(frame, Point(i, historyRoad[i].rightLane.headPoint.x) , 1, Scalar(0,255,0), -1);
        circle(frame, Point(i, historyRoad[i].middleLane.headPoint.x) , 1, Scalar(255,0,0), -1);  
    }

    for (INT i = 0; i < fitPoints.size(); i++)
    {
        circle(frame, fitPoints[i] , 1, Scalar(255,255, 255), -1);
    }

    return ret;
}

ULONG DisplayLastHundredLanes(Mat& frame, LANE& leftFittedtLanes, LANE& rightFittedLanes, LANE& midFittedLanes, 
                              vector<LANE > & lastLeftHundredLane, vector<LANE >& lastRightHundredLane, vector<LANE >& lastMidHundredLane)
{
    ULONG ret = HCOM_OK;

    INT lastLaneNum = 800;
    frame = cv::Mat::zeros(WIDTH, lastLaneNum, CV_8UC3);

    if (lastLeftHundredLane.size() <= lastLaneNum)
    {
        lastLeftHundredLane.push_back(leftFittedtLanes);
    }
    else
    {
        lastLeftHundredLane.erase(lastLeftHundredLane.begin());
        lastLeftHundredLane.push_back(leftFittedtLanes);
    }

    if (lastRightHundredLane.size() <= lastLaneNum)
    {
        lastRightHundredLane.push_back(rightFittedLanes);
    }
    else
    {
        lastRightHundredLane.erase(lastRightHundredLane.begin());
        lastRightHundredLane.push_back(rightFittedLanes);
    }

    if (lastMidHundredLane.size() <= lastLaneNum)
    {               
        lastMidHundredLane.push_back(midFittedLanes);
    }
    else
    {
        lastMidHundredLane.erase(lastMidHundredLane.begin());
        lastMidHundredLane.push_back(midFittedLanes);
    }

    //for (size_t i = 0; i < lastLeftHundredLane.size(); i++)
    //{
    //    circle(frame, Point(i, lastLeftHundredLane[i].headPoint.x), 1, Scalar(0,0,255), -1);     
    //}

    //for (size_t i = 0; i < lastRightHundredLane.size(); i++)
    //{
    //    circle(frame, Point(i, lastRightHundredLane[i].headPoint.x), 1, Scalar(0,255,0), -1);      
    //}

    for (size_t i = 0; i < lastMidHundredLane.size(); i++)
    {
        circle(frame, Point(i, lastMidHundredLane[i].headPoint.x), 1, Scalar(255,0,0), -1);      
    }

    line(frame, Point(0, WIDTH / 2), Point(lastLaneNum, WIDTH / 2), Scalar(0,0,255));
    
    return ret;
}

ULONG DisplayLastRoadXPos(Mat& frame, vector<ROAD>& lastRoad, vector<ROAD>& lastPredictRoad, ROAD& predictRoad)
{
    ULONG ret = HCOM_OK;

    frame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);


    if (lastPredictRoad.size() <= 200)
    {
        lastPredictRoad.push_back(predictRoad);
    }
    else
    {
        lastPredictRoad.erase(lastPredictRoad.begin());
        lastPredictRoad.push_back(predictRoad);
    }

    //for (size_t i = 0; i < lastRoad.size(); i++)
    //{
    //    circle(frame, Point((lastRoad[i].leftLane.headPoint.x + lastRoad[i].leftLane.backPoint.x) / 2, i), 1, Scalar(0,0,255), -1);
    //}

    //for (size_t i = 0; i < lastRoad.size(); i++)
    //{
    //    circle(frame, Point((lastRoad[i].rightLane.headPoint.x + lastRoad[i].rightLane.backPoint.x) / 2, i), 1, Scalar(0,255,0), -1);
    //}

    for (size_t i = 0; i < lastPredictRoad.size(); i++)
    {
        circle(frame, Point(lastPredictRoad[i].width, i), 1, Scalar(255,0,0), -1);
    }

    return ret;
}

ULONG DisplayRoad(Mat& frame, ROAD& road, INT directDeparture)
{
    ULONG ret = HCOM_OK;
    Scalar leftColor;
    Scalar rightColor;
    INT width = 4;
    //line(frame, road.midLane.headPoint, road.midLane.backPoint, color, 2, CV_AA);

    if (DIRECT_LEFT == directDeparture)
    {
        leftColor = Scalar(0, 0, 255);
    }
    else
    {
        leftColor = Scalar(0, 255, 0);
    }

    if (DIRECT_RIGHT == directDeparture)
    {
        rightColor = Scalar(0, 0, 255);
    }
    else
    {
        rightColor = Scalar(0, 255, 0);
    }

    line(frame, road.leftLane.headPoint, road.leftLane.backPoint, leftColor, width, CV_AA);

    line(frame, road.rightLane.headPoint, road.rightLane.backPoint, rightColor, width, CV_AA);

    return ret;
}
/*****************************************************************************
  函 数 名  : DisplaySignalLane
  功能描述  : 显示直线
  输入参数  :  
  输出参数  :  
  返 回 值  : 
  调用函数  : N/A
  被调函数  : 
  
  修改历史      :
   1.日    期   : 2018年3月6日
     作    者   : ccy0739
     修改内容   : 新生成函数
 *****************************************************************************/
ULONG DisplaySignalLane(Mat& frame, LANE& lane, Scalar color)
{
    ULONG ret = HCOM_OK;

    line(frame, lane.headPoint, lane.backPoint, color, 2, CV_AA);
    
    return ret;

}

ULONG DisplayMultiLanes(Mat& frame, vector<LANE>& lanes, Scalar color){

    ULONG ret = HCOM_OK;

    for (size_t i = 0; i < lanes.size(); i++)
    {
        line(frame, lanes[i].headPoint, lanes[i].backPoint, color, 2, CV_AA);
    }

    return ret;
}