#include "include/def.h"
#include "include/preprocess.h"

/*****************************************************************************
  函 数 名  : resizeSrcFrame
  功能描述  : 裁剪环视图像， 获得左右感兴趣区域
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
ULONG GetRoiFrame(Mat& srcFrame, Mat& roundView, Mat& leftRoiFrame, Mat& rightRoiFrame){

    ULONG ret = HCOM_OK;

    if (NULL_PTR == srcFrame.data)
    {
        return HCOM_ERR;
    }

    Rect roundViewRoi(0, 0, WIDTH, HEIGHT);
    Rect carRegion(140, 150, 120, 300);
    Mat dstFrame;
    Mat carRegionFrame;

    dstFrame = srcFrame(roundViewRoi);
    dstFrame.copyTo(roundView);
    
    /* 车置黑色 */
    carRegionFrame = srcFrame(carRegion);
    carRegionFrame.setTo(cv::Scalar(0, 0, 0));

    leftRoiFrame = dstFrame(Rect(0, 0, ROI_WIDTH, HEIGHT));
    rightRoiFrame = dstFrame(Rect(RIGHT_ROI_POS, 0, ROI_WIDTH, HEIGHT));

    return ret;
}

/*****************************************************************************
  函 数 名  : ConvertToBind
  功能描述  : 基于颜色特征，获取二值图像
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
ULONG ConvertToBind(Mat& srcFrame, Mat& dstFrame){

    ULONG ret = HCOM_OK;
    Mat hlsFrame;
    vector<Mat> hslChannels;
    vector<Mat> rgbChannels;
    Mat sChannel;
    Mat rChannel;
    Mat rsAddFrame;
    INT bindThreshold = 90;

    /* 转至HLS空间 */
    cvtColor(srcFrame, hlsFrame,COLOR_BGR2HLS);

    /* 分离出S通道 */
    split(hlsFrame, hslChannels);
    sChannel = hslChannels.at(2);
    //imshow("s", sChannel);

    /* 分离R通道 */
    split(srcFrame, rgbChannels);
    rChannel = rgbChannels.at(2);
    //imshow("r", rChannel);

    /* R、S通道加权和 */
    addWeighted(sChannel, 0.5, rChannel, 0.5, 0, rsAddFrame);
    //imshow("r+s", rsAddFrame);

    /* 阈值化 */
    threshold(rsAddFrame, dstFrame, bindThreshold, 255, CV_THRESH_BINARY);

    return ret;

}