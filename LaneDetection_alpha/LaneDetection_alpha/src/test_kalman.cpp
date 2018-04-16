#include "include/def.h"
#include <cv.h>
#include <highgui.h>

const int winWidth = 800;
const int winHeight = 600;

Point mousePosition = Point(winWidth>>1, winHeight>>1);

//mouse call back
void mouseEvent(int event, int x, int y, int flags, void *param)
{
    if(event==CV_EVENT_MOUSEMOVE)
    {
        mousePosition=Point(x,y);
    }
}

ULONG TestKalman(){

    //1.kalman filter setup   
    const int stateNum=4;  
    const int measureNum=2; 

    KalmanFilter KF(stateNum, measureNum, 0);
    Mat state (stateNum, 1, CV_32FC1); //state(x,y,detaX,detaY)
    Mat processNoise(stateNum, 1, CV_32F);  //processNoise(x,y,detaX,detaY)
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F); //measurement(x,y)
    
    randn( state, Scalar::all(0), Scalar::all(0.1) ); //随机生成一个矩阵，期望是0，标准差为0.1;
    KF.transitionMatrix = *(Mat_<float>(4, 4) << 
        1,0,1,0, 
        0,1,0,1, 
        0,0,1,0, 
        0,0,0,1 );//元素导入矩阵，按行;

    //setIdentity: 缩放的单位对角矩阵;
    //!< measurement matrix (H) 观测模型
    setIdentity(KF.measurementMatrix);

    //!< process noise covariance matrix (Q)
    // wk 是过程噪声，并假定其符合均值为零，协方差矩阵为Qk(Q)的多元正态分布;
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));

    //!< measurement noise covariance matrix (R)
    //vk 是观测噪声，其均值为零，协方差矩阵为Rk,且服从正态分布;
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));

    //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  A代表F: transitionMatrix
    //预测估计协方差矩阵;
    setIdentity(KF.errorCovPost, Scalar::all(1));

    //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    //initialize post state of kalman filter at random 
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
    Mat showImg(winWidth, winHeight,CV_8UC3);

    for(;;)
    {
        setMouseCallback("Kalman", mouseEvent);
        showImg.setTo(0);

        Point statePt = Point( (int)KF.statePost.at<float>(0), (int)KF.statePost.at<float>(1));

        //2.kalman prediction   
        Mat prediction = KF.predict();
        Point predictPt = Point( (int)prediction.at<float>(0), (int)prediction.at<float>(1));

        //3.update measurement
        measurement.at<float>(0)= (float)mousePosition.x;
        measurement.at<float>(1) = (float)mousePosition.y;

        //4.update
        KF.correct(measurement);

        //randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
        //state = KF.transitionMatrix*state + processNoise;
        //draw
        circle(showImg, statePt, 5, CV_RGB(255,0,0),1);//former point
        circle(showImg, predictPt, 5, CV_RGB(0,255,0),1);//predict point
        circle(showImg, mousePosition, 5, CV_RGB(0,0,255),1);//ture point

        //    CvFont font;//字体
        //    cvInitFont(&font, CV_FONT_HERSHEY_SCRIPT_COMPLEX, 0.5f, 0.5f, 0, 1, 8);
        putText(showImg, "Red: Former Point", cvPoint(10,30), FONT_HERSHEY_SIMPLEX, 1 ,Scalar :: all(255));
        putText(showImg, "Green: Predict Point", cvPoint(10,60), FONT_HERSHEY_SIMPLEX, 1 ,Scalar :: all(255));
        putText(showImg, "Blue: Ture Point", cvPoint(10,90), FONT_HERSHEY_SIMPLEX, 1 ,Scalar :: all(255));

        imshow( "Kalman", showImg );
        int key = waitKey(3);
        if (key == 27)
        {
            break;
        }
    } 

    return 1;
}
