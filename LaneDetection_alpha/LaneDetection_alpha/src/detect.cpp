#include "include/def.h"
#include "include//detect.h"

ULONG FilteWidthEachRow(Mat& srcFrame, Mat& dstFrame, vector<Point>& candidatePoints);
ULONG ClusteLanePoints(vector<Point>& candidatePoints, vector<vector<Point> >&clusters);
ULONG FittingClusterLanes(vector<vector<Point> >& clusters, vector<LANE>& clusterFittedLanes);
ULONG FittingStraightLine(vector<Point> point, LANE& fittedLane);
ULONG GetMostSimilarCluster(Point& point, vector<vector<Point> >& clusters, INT& mostSimilar, INT& clusterIndex);
ULONG SelectMultiLaneBaseOnMinWidth(LANE& oneLane, vector<LANE>& multiLanes, vector<LANE>& lanes);
ULONG SelectLongestLane(vector<LANE>& lanes, LANE& longestLane);
ULONG SelectCarNearestLane(vector<LANE>& multiLanes, LANE& oneLane,INT& index);
bool comp(LANE& a, LANE& b);
ULONG DeleteAbnormalLanes(vector<LANE>& lanes, vector<LANE>& lastLanes);
ULONG DeleteLanesNearMid(vector<LANE>& multiLanes, vector<LANE>& midLanes);
ULONG DeleteOutlierLane(ROAD& road, vector<ROAD> lastRoads);
ULONG CalRoadWidth(ROAD& road);
ULONG GenUnkwonLaneOnWidth(LANE& knowLane, LANE& unknowLane, DOUBLE& roadWidth);
ULONG DeleteLaneCoveredByCar(ROAD& road);
ULONG SelectParallelLanes(ROAD& road);
ULONG CalOneStepDiffer(ROAD& presentRoad, vector<ROAD>& lastRoads, vector<INT>& differs);

/*****************************************************************************
  函 数 名  : DetectLane
  功能描述  : 在二值化的图像的基础上进行车道线的检测
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
ULONG DetectLane(Mat& srcFrame, Mat& bindFrame, vector<vector<Point> >& clusters, vector<LANE>& clusterFittedLanes){

    ULONG ret = HCOM_OK;

    vector<Point> candidatePoints; 

    /* 去噪 */
    ret = FilteWidthEachRow(srcFrame, bindFrame, candidatePoints);

    /* 分簇 */
    ret = ClusteLanePoints(candidatePoints, clusters);     

    /* 拟合 */
    ret = FittingClusterLanes(clusters, clusterFittedLanes);

    return ret;
}

/*****************************************************************************
  函 数 名  : SelectLanes
  功能描述  : 对检测出的车道线进行进一步的筛选
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
ULONG GetDetectedRoad(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<ROAD>& lastRoads, DOUBLE& roadWidth, ROAD& detectRoad)
{

    ULONG ret =  HCOM_OK;

    /* 选线 */
    if (0 != leftLanes.size())
    {
        ret = SelectLongestLane(leftLanes, detectRoad.leftLane); 
    }
    if (0 != rightLanes.size())
    {
        ret = SelectLongestLane(rightLanes, detectRoad.rightLane);
    }

    /* 有两条线，计算道路宽度 */
    if (0 != detectRoad.leftLane.length && 0 != detectRoad.rightLane.length)
    {
        ret = CalRoadWidth(detectRoad);

        if (detectRoad.width < MIN_ROAD_WIDTH)
        {
            /* 宽度小于车宽，删除被车所在范围覆盖的线 */
            ret = DeleteLaneCoveredByCar(detectRoad);
        }
    }

    /* 若仅有一条 */
    if (0 != detectRoad.leftLane.length && 0 == detectRoad.rightLane.length)
    {
        ret = GenUnkwonLaneOnWidth(detectRoad.leftLane, detectRoad.rightLane, roadWidth);
        detectRoad.width = roadWidth;
    }
    if (0 != detectRoad.rightLane.length && 0 == detectRoad.leftLane.length)
    {     
        ret = GenUnkwonLaneOnWidth(detectRoad.rightLane, detectRoad.leftLane, roadWidth);
        detectRoad.width = roadWidth;   
    }


    /* 转化为极坐标 */
    ret = CvtLaneToPolarCoodinate(detectRoad.leftLane);

    ret = CvtLaneToPolarCoodinate(detectRoad.rightLane);

    /* 删除离群线 */
    ret = DeleteOutlierLane(detectRoad, lastRoads);

    return ret ;
}

ULONG CvtLaneToRectCoordinate(LANE& lane)
{
    ULONG ret = HCOM_OK;

    lane.headPoint.x = lane.s + WIDTH / 2;
    lane.headPoint.y = 0;

    lane.backPoint.y = HEIGHT;
    lane.backPoint.x =  (lane.backPoint.y - lane.headPoint.y) / tan(lane.theta) + lane.headPoint.x;

    lane.length = HEIGHT;

    return ret;
}

ULONG CvtLaneToPolarCoodinate(LANE& lane)
{
    ULONG ret = HCOM_OK;
    FLOAT sople = 0;

    if (0 == lane.length)
    {
        return ret;
    }

    lane.s = lane.headPoint.x - WIDTH / 2;

    if (lane.headPoint.x == lane.backPoint.x)
    {
        lane.theta = PI / 2;
    }
    else
    {
        lane.theta = atan2f(lane.backPoint.y - lane.headPoint.y, lane.backPoint.x - lane.headPoint.x);
    }


    return ret;
}

/*****************************************************************************
  函 数 名  : GetMidLanes
  功能描述  : 求中线
  输入参数  :  
  输出参数  :  
  返 回 值  : 
  调用函数  : N/A
  被调函数  : 
  
  修改历史      :
   1.日    期   : 2018年3月14日
     作    者   : ccy0739
     修改内容   : 新生成函数
 *****************************************************************************/
ULONG GetMidLanes(ROAD& road)
{
    ULONG ret = HCOM_OK;

    road.middleLane.headPoint = Point((road.leftLane.headPoint.x + road.rightLane.headPoint.x) / 2 , 0);
    road.middleLane.backPoint = Point((road.leftLane.backPoint.x + road.rightLane.backPoint.x) / 2 , HEIGHT);
    road.middleLane.length = HEIGHT;

    return ret;
}

/*****************************************************************************
  函 数 名  : DeleteLaneCoveredByCar
  功能描述  : 宽度过小时，删除被车的区域覆盖的线，目的是滤除箭头的干扰
  输入参数  :  
  输出参数  :  
  返 回 值  : 
  调用函数  : N/A
  被调函数  : 
  
  修改历史      :
   1.日    期   : 2018年3月19日
     作    者   : ccy0739
     修改内容   : 新生成函数
 *****************************************************************************/
ULONG DeleteLaneCoveredByCar(ROAD& road)
{
    ULONG ret =  HCOM_OK;
    INT midPos;

    midPos = (road.leftLane.headPoint.x + road.leftLane.backPoint.x) / 2; 
    if (midPos > CAR_POS && midPos < (CAR_POS + MIN_ROAD_WIDTH))
    {
        road.leftLane.backPoint = Point(0,0);
        road.leftLane.headPoint = Point(0,0);
        road.leftLane.length = 0;
        road.width = 0;
    }

    midPos = (road.rightLane.headPoint.x + road.rightLane.backPoint.x) / 2; 
    if (midPos > CAR_POS && midPos < (CAR_POS + MIN_ROAD_WIDTH))
    {
        road.rightLane.backPoint = Point(0,0);
        road.rightLane.headPoint = Point(0,0);
        road.rightLane.length = 0;
        road.width = 0;
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : DeleteArrowLane
  功能描述  : 删除离群线
  输入参数  :  
  输出参数  :  
  返 回 值  : 
  调用函数  : N/A
  被调函数  : 
  
  修改历史      :
   1.日    期   : 2018年3月19日
     作    者   : ccy0739
     修改内容   : 新生成函数
 *****************************************************************************/
ULONG DeleteOutlierLane(ROAD& road, vector<ROAD> lastRoads)
{
    ULONG ret =  HCOM_OK;

    const INT outTrh = 50;

    LANE resetLane = {Point(0,0), Point(0,0), 0};
    INT lastMiddlePoint = 0;

    if (0 == lastRoads.size())
    {
        return ret;
    }

    lastMiddlePoint = (lastRoads.back().leftLane.headPoint.x + lastRoads.back().rightLane.headPoint.x) / 2;

    /* 点与上一帧检测出的中点重合，认为是噪点 */
    if (abs(road.leftLane.headPoint.x - lastMiddlePoint) < outTrh
        || abs(road.rightLane.headPoint.x - lastMiddlePoint) < outTrh)
    {
        road.leftLane = resetLane;
        road.rightLane = resetLane;
        road.width = 0;
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : GenUnkwonLaneOnWidth
  功能描述  : 根据已检测出的线推断出另一条线
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
ULONG GenUnkwonLaneOnWidth(LANE& knowLane, LANE& unknowLane, DOUBLE& roadWidth)
{
    ULONG ret =  HCOM_OK;
    INT flag = 1;

    if (0 == roadWidth)
    {
        return HCOM_ERR;
    }

    flag = (knowLane.headPoint.x < ROI_WIDTH) ? 1 : -1;

    unknowLane.headPoint.x = knowLane.headPoint.x + roadWidth * flag;
    unknowLane.headPoint.y = 0;
    unknowLane.backPoint.x = knowLane.backPoint.x + roadWidth * flag;
    unknowLane.backPoint.y = HEIGHT;
    unknowLane.length = knowLane.length;

    return ret ;
}

/*****************************************************************************
  函 数 名  : CalDoubleLanesWidth
  功能描述  : 计算两条线之间的宽度
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
ULONG CalRoadWidth(ROAD& road)
{
    ULONG ret = HCOM_OK;
    DOUBLE width;

    /* 两线近似平行，取首尾宽度平均值 */
    width = (road.rightLane.backPoint.x - road.leftLane.backPoint.x) 
        + (road.rightLane.headPoint.x - road.leftLane.headPoint.x);
    road.width = width /2.0;
    
    return ret; 

}


/*****************************************************************************
  函 数 名  : ResizeRightLanes
  功能描述  : 将右侧的线返回到原图坐标
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
ULONG ResizeRightLanes(vector<LANE>& rightLanes, INT offset)
{
    ULONG ret = HCOM_OK;

    /* 将右侧的坐标回归至原图 */
    for (size_t i = 0; i < rightLanes.size(); i++)
    {      
        rightLanes[i].headPoint.x = rightLanes[i].headPoint.x + offset;
        rightLanes[i].backPoint.x = rightLanes[i].backPoint.x + offset;
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : comp
  功能描述  : 用于排序
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
bool comp(LANE& a, LANE& b)
{
    return a.length > b.length;
}

/*****************************************************************************
  函 数 名  : SelectLongestLane
  功能描述  : 找最长的车道线
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
ULONG SelectLongestLane(vector<LANE>& lanes, LANE& longestLane){

    ULONG ret = HCOM_OK;
    INT maxLength = 0;

    /* 认为最长的即为车道线 */
    for (size_t i = 0; i < lanes.size(); i++)
    {
        if (lanes[i].length > maxLength && lanes[i].length > MIN_LANE_LENGTH)
        {
            longestLane = lanes[i];
            maxLength = lanes[i].length;
        }
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : SelectLongestLane
  功能描述  : 找最靠近车的车道线
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
ULONG SelectCarNearestLane(vector<LANE>& multiLanes, LANE& oneLane, INT& index){
    
    ULONG ret = HCOM_OK;
    INT minDis;

    if (1 == multiLanes.size())
    {
        return ret;
    }

    minDis = abs(WIDTH / 2 - multiLanes[0].headPoint.x);
    for (size_t i = 0; i < multiLanes.size(); i++)
    {
        if (minDis >= abs(WIDTH / 2 - multiLanes[i].headPoint.x))
        {
            minDis = abs(WIDTH / 2 - multiLanes[i].headPoint.x);
            oneLane = multiLanes[i];
            index = i;
        }
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : DeleteLanesNearMid
  功能描述  : 
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
ULONG DeleteLanesNearMid(vector<LANE>& multiLanes, vector<LANE>& midLanes)
{
    ULONG ret = HCOM_OK;
    INT error = 150;    

    BOOL isError = FALSE;
    vector<LANE> tmpLanes;

    for (size_t i = 0 ; i < multiLanes.size(); i ++)
    {
        for (size_t j = 0; j < midLanes.size(); j++)
        {
            if (abs(multiLanes[i].headPoint.x - midLanes[j].headPoint.x) < error)
            {
                isError = TRUE;
            }
        }

        if (!isError)
        {
            tmpLanes.push_back(multiLanes[i]);
        }
        isError = FALSE;
    }
    multiLanes.clear();
    multiLanes = tmpLanes;
    tmpLanes.clear();

    return ret;

}

/*****************************************************************************
  函 数 名  : SelectBaseOnMinWidth
  功能描述  : 基于最小宽度筛选
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
ULONG SelectMultiLaneBaseOnMinWidth(LANE& oneLane, vector<LANE>& multiLanes, vector<LANE>& lanes){
    
    ULONG ret =  HCOM_OK;
    LANE tmpLane;
    INT headDistance = 0;
    INT backDistance = 0;
    INT error = 20;     //左右两车道线平行的误差

    for (size_t i = 0 ; i < multiLanes.size(); i++)
    {
        tmpLane = multiLanes[i];
        headDistance = tmpLane.headPoint.x - oneLane.headPoint.x;
        backDistance = tmpLane.backPoint.x - oneLane.backPoint.x;

        /* 左右两条线的首尾宽度近似（平行），且宽度在一定范围内 */
        if ((abs(headDistance - backDistance) < error)
            && (headDistance * backDistance > 0)   
            && (abs(headDistance) > MIN_ROAD_WIDTH)
            && (abs(backDistance) > MIN_ROAD_WIDTH))
        {
            lanes.push_back(tmpLane);
        }
    }

    return ret ;
}

/*****************************************************************************
  函 数 名  : SelectSignalLaneBaseOnMinWidth
  功能描述  : 基于最小宽度筛选
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
ULONG SelectParallelLanes(ROAD& road)
{
    
    ULONG ret =  HCOM_OK;
    INT headDistance = 0;
    INT backDistance = 0;
    INT error = 15;     //左右两车道线平行的误差

    headDistance = road.leftLane.headPoint.x - road.rightLane.headPoint.x;
    backDistance = road.leftLane.backPoint.x - road.rightLane.backPoint.x;

    /* 判断是否平行 */
    if ((abs(headDistance - backDistance) > error)
        || headDistance > 0
        || backDistance > 0)
    {
        road.leftLane.length = 0;
        road.rightLane.length = 0;
    }

    return ret ;
}

/*****************************************************************************
  函 数 名  : FilteWidthEachRow
  功能描述  : 根据车道线具有一定宽度的特性去噪
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
ULONG FilteWidthEachRow(Mat& srcFrame, Mat& dstFrame, vector<Point>& candidatePoints){

    ULONG ret = HCOM_OK;

    Mat widthSelectFrame = Mat::zeros(srcFrame.rows, srcFrame.cols, CV_8UC1);
    dstFrame = Mat::zeros(srcFrame.rows, srcFrame.cols, CV_8UC1);

    UCHAR* srcData = srcFrame.data;
    UCHAR* widthSelectData = widthSelectFrame.data;
    UCHAR* dstData = dstFrame.data;

    INT col = 0;  //列
    INT row = 0;  //行
    INT startPos = 0;
    INT endPos = 0;

    /* 一定宽度内应连续不为0 */
    for (row = 0; row < srcFrame.rows; row++)
    {
        while (col < srcFrame.cols - 1)
        {
            if (255 == srcData[row * srcFrame.cols + col])
            {
                startPos = col;   /* 记录不为0像素的开始位置 */

                /* 查找该行连续不为0 的像素 */
                while (col < srcFrame.cols - 1)
                {
                    col++;
                    if (0 == srcData[row * srcFrame.cols + col])
                    {
                        break;
                    }
                }
                endPos = col;     /* 记录不为0像素的结束位置 */

                /* 起止点坐标之差即连续非零像素的宽度，为候选车道线区域 */
                if (endPos - startPos >= MIN_LANE_WIDTH && endPos - startPos <= MAX_LANE_WIDTH)
                {
                    dstData[startPos + row * srcFrame.cols] = 255;
                    dstData[endPos + row * srcFrame.cols] = 255;
                    candidatePoints.push_back(Point((startPos + endPos) / 2,row));
                }
                col = endPos;
            }
            else
            {
                col++;
            }
        }
        col = 0;
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : ClusterSameColumPoints
  功能描述  : 根据横坐标相似度分簇
  输入参数  : 候选车道线上的点
  输出参数  : 聚类得到的簇
  返 回 值  : 
  调用函数  : N/A
  被调函数  : 
  
  修改历史      :
   1.日    期   : 2018年3月6日
     作    者   : ccy0739
     修改内容   : 新生成函数
 *****************************************************************************/
ULONG ClusteLanePoints(vector<Point>& candidatePoints, vector<vector<Point> >& clusters)
{
    ULONG ret = HCOM_OK;

    vector<Point> tmpCluster;
    INT mostSimilarClusterIndex;
    INT mostSimilar;
    INT minError = 8;

    if (0 == candidatePoints.size())
    {
        return ret;
    }

    /* 取第一个候选点作为第一个簇 */
    tmpCluster.push_back(candidatePoints[0]);
    clusters.push_back(tmpCluster);
    tmpCluster.clear();

    /* 根据横坐标计算相似度，分簇 */
    for (size_t i = 1; i < candidatePoints.size(); i++)
    {
        ret = GetMostSimilarCluster(candidatePoints[i], clusters, mostSimilar, mostSimilarClusterIndex);

        /* 若最相似簇的相似度小于阈值，入簇，否则为新簇 */
        if (mostSimilar < minError)
        { 
            clusters[mostSimilarClusterIndex].push_back(candidatePoints[i]);
        }
        else
        {
            tmpCluster.push_back(candidatePoints[i]);
            clusters.push_back(tmpCluster);
            tmpCluster.clear();
        }
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : GetMostSimilarCluster
  功能描述  : 找出最相似的簇
  输入参数  : 候选车道线上的点
  输出参数  : 簇的索引
  返 回 值  : 
  调用函数  : N/A
  被调函数  : 
  
  修改历史      :
   1.日    期   : 2018年3月6日
     作    者   : ccy0739
     修改内容   : 新生成函数
 *****************************************************************************/
ULONG GetMostSimilarCluster(Point& point, vector<vector<Point> >& clusters, INT& mostSimilar, INT& clusterIndex){
    
    ULONG ret = HCOM_OK;
    Point clusterLastPoint;
    vector<INT> similar;

    for (size_t i = 0; i < clusters.size(); i++)
    {
        clusterLastPoint = clusters[i].back();
        similar.push_back(abs(point.x - clusterLastPoint.x));
    }

    /* similar越小，越相似 */
    mostSimilar = similar[0];
    clusterIndex = 0;
    for (size_t i = 0; i < similar.size(); i++)
    {
        if (similar[i] < mostSimilar)
        {
            mostSimilar = similar[i];
            clusterIndex = i;
        }
    }

    similar.clear();
    return ret;
}

/*****************************************************************************
  函 数 名  : FitClusterLine
  功能描述  : 对各个分好的簇进行拟合
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
ULONG FittingClusterLanes(vector<vector<Point> >& clusters, vector<LANE>& clusterFittedLanes){

    ULONG ret =HCOM_OK;

    vector<Point> cluster;
    LANE clusterFittedLane;

    if (0 == clusters.size())
    {
        return ret;
    }

    for (size_t i = 0; i < clusters.size(); i++)
    {     
        cluster = clusters[i];

        /* 簇的大小即为线的长度 */
        if (cluster.size() > MIN_LANE_LENGTH)
        {
            ret = FittingStraightLine(cluster, clusterFittedLane);
            clusterFittedLane.length = cluster.size();
            clusterFittedLanes.push_back(clusterFittedLane);
        }
        cluster.clear();
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : FittingStraightLine
  功能描述  : 调用OpenCV方法进行直线拟合
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
ULONG FittingStraightLine(vector<Point> point, LANE& fittedLane)
{
    ULONG ret = HCOM_OK;
    Vec4f linePara;
    DOUBLE a;
    DOUBLE b;
    DOUBLE c;

    if (0 == point.size())
    {
        return ret;
    }

    fitLine(point, linePara, CV_DIST_L2, 0, 0.001, 0.001);    /*最小二乘*/

    /* 拟合出的直线方程即为 a * x + b * y + c = 0 */
    a = (-1) *linePara[1];
    b = linePara[0];
    c = (-1) * (a * linePara[2] + b * linePara[3]);

    if (0 == a && 0 != b)
    {
        /* 此时方程 b * y + c = 0 */
        fittedLane.headPoint = Point(0, (-1) * c / b);
        fittedLane.backPoint = Point(WIDTH, (-1) * c / b);
    }
    if (0 == b && 0 != a)
    {
        /* 此时方程 a * x + c = 0 */
        fittedLane.headPoint = Point((-1) * c / a, 0);
        fittedLane.backPoint = Point((-1) * c / a, HEIGHT);
    }

    if (0 != a && 0 != b)
    {
        fittedLane.headPoint = Point( (-1) * (c + b * 0) / a, 0);
        fittedLane.backPoint = Point((-1) * (c + b * HEIGHT) / a, HEIGHT);
    }

    if (0 == a && 0 == b)
    {
        return HCOM_ERR;
    }

    return ret;
}

/*****************************************************************************
  函 数 名  : DeleteAbnormalLanes
  功能描述  : 与上一帧比较，滤除有突然跳变的异常点
  输入参数  :  
  输出参数  :  
  返 回 值  : 
  调用函数  : N/A
  被调函数  : 
  
  修改历史      :
   1.日    期   : 2018年3月14日
     作    者   : ccy0739
     修改内容   : 新生成函数
 *****************************************************************************/
ULONG DeleteAbnormalLanes(vector<LANE>& lanes, vector<LANE>& lastLanes)
{
    ULONG ret = HCOM_OK;
    INT errorPoint = 20;    
    BOOL isError = FALSE;

    vector<LANE> tmpLanes;

    for (size_t i = 0 ; i < lanes.size(); i ++)
    {
        for (size_t j = 0; j < lastLanes.size(); j++)
        {
            /* 相邻两帧检测出的车道线顶点之差大于errorPoint，则认为是异常 */
            if (abs(lanes[i].headPoint.x - lastLanes[j].headPoint.x) > errorPoint)
            {
                isError = TRUE;
            }
        }

        if (!isError)
        {
            tmpLanes.push_back(lanes[i]);
        }
        isError = FALSE;
    }

    /* 如检测出的线都是异常突变点，上一帧结果作为本帧结果 */
    if (0 == tmpLanes.size())
    {
        lanes = lastLanes;
    }
    else
    {
        lanes.clear();
        lanes = tmpLanes;
        tmpLanes.clear();
    }

    return ret;
}
