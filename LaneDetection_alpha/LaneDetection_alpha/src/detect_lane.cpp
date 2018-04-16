#include "include/def.h"
#include "include//detect_lane.h"

ULONG FilteWidthEachRow(Mat& srcFrame, Mat& dstFrame, vector<Point>& candidatePoints);
ULONG ClusteLanePoints(vector<Point>& candidatePoints, vector<vector<Point> >&clusters);
ULONG FittingClusterLanes(vector<vector<Point> >& clusters, vector<LANE>& clusterFittedLanes);
ULONG FittingStraightLine(vector<Point> point, LANE& fittedLane);
ULONG GetMostSimilarCluster(Point& point, vector<vector<Point> >& clusters, INT& mostSimilar, INT& clusterIndex);
ULONG SelectMultiLaneBaseOnMinWidth(LANE& oneLane, vector<LANE>& multiLanes, vector<LANE>& lanes);
ULONG SelectParallelLanes(vector<LANE>& leftLanes, vector<LANE>& rightLanes, LANE& leftLane, LANE& rightLane);
ULONG SelectBaseOnLength(vector<LANE>& multiLanes);
ULONG SelectCarNearestLane(vector<LANE>& multiLanes, LANE& oneLane,INT& index);
bool comp(LANE& a, LANE& b);
ULONG DeleteAbnormalLanes(vector<LANE>& lanes, vector<LANE>& lastLanes);
ULONG DeleteLanesNearMid(vector<LANE>& multiLanes, vector<LANE>& midLanes);

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
ULONG SelectLane(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<LANE>& lastLanes, vector<LANE>& lanes){

    ULONG ret =  HCOM_OK;
    vector<LANE> selectedLeftLanes;
    vector<LANE> selectedRightLanes;
    LANE leftLane;
    LANE rightLane;
    INT index;
    vector<LANE> midLanes;

    /* 根据长度筛选 */
    if (1 < leftLanes.size())
    {
       ret = SelectBaseOnLength(leftLanes);
    }

    if (1 < rightLanes.size())
    {
        ret = SelectBaseOnLength(rightLanes);
    }

    ret = GetMidLanes(leftLanes, rightLanes, midLanes);
    if (1 < leftLanes.size())
    {
        ret = DeleteLanesNearMid(leftLanes, midLanes);
    }

    if (1 < rightLanes.size())
    {
        ret = DeleteLanesNearMid(rightLanes, midLanes);
    }
    
    /* 两边均只有一条，判断两条的间距 */
    if (1 == leftLanes.size() && 1 == rightLanes.size())
    {
        ret = SelectParallelLanes(leftLanes, rightLanes, leftLane, rightLane);
    }

    /* 某边没有， 另一边有 */
    if (0 == leftLanes.size() && 0 != rightLanes.size())
    {
        ret = SelectCarNearestLane(rightLanes, rightLane, index); 
    }
    
    if (0 == rightLanes.size() && 0 != leftLanes.size())
    {
        rightLane = lastLanes[1];
        ret = SelectCarNearestLane(leftLanes, leftLane, index); 
    }
   
    /* 某边只有一条，另一边多条，基于单条边筛选另一边 */
    if (1 == leftLanes.size() && 1 != rightLanes.size())
    {
        leftLane = leftLanes[0];

        ret = SelectMultiLaneBaseOnMinWidth(leftLane, rightLanes, selectedRightLanes);

        if (1 <= selectedRightLanes.size())
        {
            ret = SelectCarNearestLane(selectedRightLanes, rightLane, index);          
        }
    }
    
    if (1 == rightLanes.size() && 1 != leftLanes.size())
    {
        rightLane = rightLanes[0];

        ret = SelectMultiLaneBaseOnMinWidth(rightLane, leftLanes, selectedLeftLanes);

        if (1 <= selectedLeftLanes.size())
        {
            ret = SelectCarNearestLane(selectedLeftLanes, leftLane, index);
        }
    }
    
    /* 两边都是多条 */
    if (1 < leftLanes.size() && 1 < rightLanes.size())
    {
        index = 0;
        while (leftLanes.size() > 1)
        {
            /* 取左边最近车的 */
            ret = SelectCarNearestLane(leftLanes, leftLane, index);

            ret = SelectMultiLaneBaseOnMinWidth(leftLane, rightLanes, selectedRightLanes);

            if (1 <= selectedRightLanes.size())
            {
                ret = SelectCarNearestLane(selectedRightLanes, rightLane, index);
                break;
            }
            else
            {
                leftLanes.erase(leftLanes.begin() +index);
            }
        }
    }

    /* 若最终某边未检测出， 用上一帧检测结果代替*/
    if (0 == leftLane.headPoint.x && 0 == leftLane.backPoint.x)
    {
        leftLane = lastLanes[0];
    }
    if (0 == leftLane.backPoint.x && 0 == leftLane.backPoint.x)
    {
        rightLane = lastLanes[1];
    }

    selectedRightLanes.clear();
    selectedLeftLanes.clear();

    lanes.push_back(leftLane);
    lanes.push_back(rightLane);

    return ret ;
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
ULONG SelectBaseOnLength(vector<LANE>& multiLanes){

    ULONG ret = HCOM_OK;
    INT errorWithHeight = 20;
    INT errorWithSecond = 300;
    LANE longestLane;

    if (1 == multiLanes.size())
    {
        return ret;
    }

    /* 按长度排序,从大到小 */
    sort(multiLanes.begin(),multiLanes.end(),comp);

    /* 如最长的接近图像高或远远大于第二长的，认为最长的即为车道线 */
    if (HEIGHT - multiLanes[0].length < errorWithHeight 
        || multiLanes[0].length - multiLanes[1].length > errorWithSecond)
    {
        longestLane = multiLanes[0];
        multiLanes.clear();
        multiLanes.push_back(longestLane);
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
            && (abs(headDistance) > MIN_DOUBLE_LANE_WIDTH)
            && (abs(backDistance) > MIN_DOUBLE_LANE_WIDTH))
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
ULONG SelectParallelLanes(vector<LANE>& leftLanes, vector<LANE>& rightLanes, LANE& leftLane, LANE& rightLane)
{
    
    ULONG ret =  HCOM_OK;
    INT headDistance = 0;
    INT backDistance = 0;
    INT error = 20;     //左右两车道线平行的误差

    headDistance = leftLanes[0].headPoint.x - rightLanes[0].headPoint.x;
    backDistance = leftLanes[0].backPoint.x - rightLanes[0].backPoint.x;

    /* 左右两条线的首尾宽度近似（平行），且宽度在一定范围内 */
    if ((abs(headDistance - backDistance) < error)
        && (headDistance * backDistance > 0)
        && (abs(headDistance) > MIN_DOUBLE_LANE_WIDTH)
        && (abs(backDistance) > MIN_DOUBLE_LANE_WIDTH))
    {
        leftLane = leftLanes[0];
        rightLane = rightLanes[0];
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
                if (endPos - startPos >= MIN_SINGLE_LANE_WIDTH && endPos - startPos <= MAX_SINGLE_LANE_WIDTH)
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
    DOUBLE sople;
    DOUBLE base;

    if (0 == point.size())
    {
        return ret;
    }

    fitLine(point, linePara, CV_DIST_L2, 0, 0.001, 0.001);    /*最小二乘*/

    sople = linePara[1] / linePara[0];
    base = linePara[3] - sople * linePara[2];

    fittedLane.sople = sople;
    fittedLane.base = base;
    fittedLane.headPoint = Point(INT((-base) / sople) ,0);
    fittedLane.backPoint = Point(INT((HEIGHT - base) / sople) ,HEIGHT);

    return ret;
}

/*****************************************************************************
  函 数 名  : SelectLaneBaseOnMultiFrames
  功能描述  : 基于连续帧筛选车道线
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
ULONG SelectLaneBaseOnMultiFrames(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<LANE>& lastLeftlanes, vector<LANE>& lastRightLanes)
{
    ULONG ret = HCOM_OK;

    


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
ULONG GetMidLanes(vector<LANE>& leftLanes, vector<LANE>& rightLanes, vector<LANE>& midLanes)
{
    ULONG ret = HCOM_OK;
    LANE midLane;

    for (size_t i = 0; i < leftLanes.size(); i++)
    {
        for (size_t j = 0; j < rightLanes.size(); j++)
        {
            midLane.headPoint = Point((leftLanes[i].headPoint.x + rightLanes[j].headPoint.x) / 2 , 0);
            midLane.backPoint = Point((leftLanes[i].backPoint.x + rightLanes[j].backPoint.x) / 2 , HEIGHT);
            midLanes.push_back(midLane);
        }

    }

    return ret;
}