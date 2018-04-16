#include "include/def.h"
#include "include/predict_lane.h"

ULONG SetupKalman(KalmanFilter& kalmanFilter)
{
    ULONG ret = HCOM_OK;

    Mat state (KALMAN_STATE_NUM, 1, CV_32FC1);                      //state(x,y,detaX,detaY)
    Mat processNoise(KALMAN_STATE_NUM, 1, CV_32F);                  //processNoise(x,y,detaX,detaY)

    randn( state, Scalar::all(0), Scalar::all(0.1) );   //随机生成一个矩阵，期望是0，标准差为0.1;
    kalmanFilter.transitionMatrix = *(Mat_<float>(KALMAN_STATE_NUM, KALMAN_STATE_NUM) << 
        1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1 );

    //setIdentity: 缩放的单位对角矩阵;
    //!< measurement matrix (H) 观测模型
    setIdentity(kalmanFilter.measurementMatrix, Scalar::all(1));

    //!< process noise covariance matrix (Q)
    // wk 是过程噪声，并假定其符合均值为零，协方差矩阵为Qk(Q)的多元正态分布;
    setIdentity(kalmanFilter.processNoiseCov, Scalar::all(1e-5));

    //!< measurement noise covariance matrix (R)
    //vk 是观测噪声，其均值为零，协方差矩阵为Rk,且服从正态分布;
    setIdentity(kalmanFilter.measurementNoiseCov, Scalar::all(1e-1));

    //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  A代表F: transitionMatrix
    //预测估计协方差矩阵;
    setIdentity(kalmanFilter.errorCovPost, Scalar::all(1));

    //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    //initialize post state of kalman filter at random 
    randn(kalmanFilter.statePost, Scalar::all(0), Scalar::all(400));

    return ret;
}

ULONG PredictKalman(KalmanFilter& kalmanFilter, Point& detectLane, Point& predictPoint)
{
    ULONG ret = HCOM_OK;

    Mat measurement = Mat::zeros(KALMAN_MEASUREMENT_NUM, 1, CV_32F);

    //Point statePt = Point( (kalmanFilter.statePost.at<INT>(0), kalmanFilter.statePost.at<INT>(1)) );

    //2.kalman prediction   
    Mat prediction = kalmanFilter.predict();
    predictPoint = Point( (INT)prediction.at<FLOAT>(0), (INT)prediction.at<FLOAT>(1));

    //3.update measurement
    measurement.at<FLOAT>(0)= (FLOAT)detectLane.x;
    measurement.at<FLOAT>(1) = (FLOAT)detectLane.y;

    //4.update
    kalmanFilter.correct(measurement);

    return ret;
}