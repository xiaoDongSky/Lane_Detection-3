#ifndef _DEF_H_ 
#define _DEF_H_

#include<opencv2\opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>

using namespace cv;
using namespace std;

/* 信息描述宏定义 */
#define SEP(x)  1

#define PI 3.1415926535897932384626433832795f
#define PIXEL_SIZE_RGB 3

#define HCOM_OK     0
#define HCOM_ERR    1
#define NULL_PTR    0
#define ERROR_PTR_NULL    0

#ifndef NULL
#define NULL        0
#endif

#define NULL_UCHAR  0xff
#define NULL_UINT   0xffffffff
#define NULL_ULONG  0xffffffff

#define WIDTH 400
#define HEIGHT 600
#define ROI_WIDTH 200
#define RIGHT_ROI_POS 200
#define CAR_POS 130

#define MIN_LANE_LENGTH 20      //车道线的最短长度
#define MIN_LANE_WIDTH 8        //单个车道线的最小宽度
#define MAX_LANE_WIDTH 40       //单个车道线的最大宽度
#define MIN_ROAD_WIDTH 135      //最小车道宽度，或车的宽度
#define DEFAULT_ROAD_WIDTH 250  //默认车道宽度

#define CHECK_PTR_IS_NULL(ptr)  if (NULL_PTR == (ptr))  \
{                       \
    printf("%s","ERROR_NULL_PTR");                    \
    return ERROR_PTR_NULL;    \
}


#if SEP("常用类型别名")

#define TRUE 1
#define FALSE 0

#ifndef _WINDOWS_

typedef void    VOID;

typedef unsigned int  UINT;
typedef unsigned long ULONG;
typedef unsigned char UCHAR;
typedef unsigned char BYTE;
typedef short int SINT;

typedef bool    BOOL;
typedef char    CHAR;
typedef int     INT;
typedef long    LONG;

typedef float   FLOAT;
typedef double  DOUBLE;

#endif

#endif 

typedef enum tagDirection
{
    DIRECT_LEFT,
    DIRECT_RIGHT,
    DIRECT_NONE

}DIRECTION;

typedef struct tagLane
{
    Point headPoint;
    Point backPoint;
    INT length;
    FLOAT s;        /* 侧向距离 */
    FLOAT theta;    /* 方向角 */
}LANE;

typedef struct tagRoad 
{
    DOUBLE width;
    LANE leftLane;
    LANE rightLane;
    LANE middleLane;

}ROAD;

#endif 

