#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


const int ANGLE_TO_UP = 1;
const int WIDTH_GREATER_THAN_HEIGHT = 0;
#define DELAT_MAX 30
typedef	int filter_type;

filter_type filter(filter_type effective_value, filter_type new_value, filter_type delat_max)
{
    if ( ( new_value - effective_value > delat_max ) || ( effective_value - new_value > delat_max ))
    {
        new_value=effective_value;
        return effective_value;
    }
    else
    {
        new_value=effective_value;
        return new_value;
    }
}

RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
   {
       using std::swap;

       float& width = rec.size.width;
       float& height = rec.size.height;
       float& angle = rec.angle;

       if (mode == WIDTH_GREATER_THAN_HEIGHT)
       {
           if (width < height)
           {
               swap(width, height);
               angle += 90.0;
           }
       }

       while (angle >= 90.0) angle -= 180.0;
       while (angle < -90.0) angle += 180.0;

       if (mode == ANGLE_TO_UP)
       {
           if (angle >= 45.0)
           {
               swap(width, height);
               angle -= 90.0;
           }
           else if (angle < -45.0)
           {
               swap(width, height);
               angle += 90.0;
           }
       }
   return rec;
}

int main()
{
    cv::VideoCapture cap(0);//改为0
    cap.set(CAP_PROP_FRAME_WIDTH,640);  //图片宽设为640
    cap.set(CAP_PROP_FRAME_HEIGHT,480); //图片高设为480

    const cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1222.10, 0,335.48, 0, 1218.81, 226.77, 0,0,1);//内参矩阵
    const cv::Mat distCoeffs = (cv::Mat_<double> (5,1) <<  -0.61290972, 1.20511056, -0.00701041, 0.00149212, 7.2167083);//畸变参数
    int g_nStructElementSize = 3; //结构元素(内核矩阵)的尺寸
    Mat kernel = getStructuringElement(MORPH_RECT,Size(g_nStructElementSize,g_nStructElementSize));//5*5的卷积核
    vector<vector<Point>> contours;//轮廓
    RotatedRect rect;
    Point2f center;


    Mat src,hsv,mask,mask1,mask2,dst,bgr[3],gray;
    while(cap.isOpened()!=0)
    {
       cap >> src;
       imshow("src",src);

       /*图像处理*/
       split(src,bgr);
       gray = bgr[2] - bgr[0];
       /*
       cvtColor(src,hsv,COLOR_BGR2HSV); //bgr ->hsv

       vector<Mat> hsvsplit;
       split(hsv,hsvsplit); //分离hsv
       equalizeHist(hsvsplit[2],hsvsplit[2]);
       merge(hsvsplit,hsv);
       imshow("hsv_process",hsv);//重新合并*/

       Mat thresHold;
       threshold(gray,thresHold,240,250,THRESH_BINARY); //图像二值化
       imshow("二值图",thresHold);

       Mat element = getStructuringElement(MORPH_ELLIPSE,Size(3,3));//
       //erode(thresHold,thresHold,element);
       blur(thresHold,thresHold, Size(1,1));
       //GaussianBlur(thresHold,thresHold,Size(3,3),100,100);
       dilate(thresHold,thresHold,element);
       imshow("处理后的二值图",thresHold);

       vector<RotatedRect> vc; //存放第一轮筛选出的轮廓
       vector<RotatedRect> vRec;
       vector<vector<Point>> Contour; //查找轮廓

       findContours(thresHold,Contour,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
       for(int i = 0; i < Contour.size(); i++)
       {
           float area = contourArea(Contour[i]);
           if(area < 35)
               continue;
           RotatedRect Light_Rec = fitEllipse(Contour[i]); //拟合矩形
           Light_Rec = adjustRec(Light_Rec, ANGLE_TO_UP);
                       if (Light_Rec.angle > 10 )
                           continue;
                       // 长宽比和轮廓面积比限制
                       if (Light_Rec.size.width / Light_Rec.size.height > 2.3
                               || area / Light_Rec.size.area() < 0.55)
                           continue;
                       Light_Rec.size.height *= 1.2;
                       Light_Rec.size.width *= 1.2;
                       vc.push_back(Light_Rec);
         }
       int x,y;
       for (size_t i = 0; i < vc.size(); i++) //对灯条进行二次筛选
          {
            for (size_t j = i + 1; (j < vc.size()); j++)
            {
              //判断是否为相同灯条
              float Contour_angle = abs(vc[i].angle - vc[j].angle); //角度差
              if (Contour_angle >= 7)
                  continue;
              //长度差比
              float Contour_Len1 = abs(vc[i].size.height - vc[j].size.height) / max(vc[i].size.height, vc[j].size.height);
              //宽度差比
              float Contour_Len2 = abs(vc[i].size.width - vc[j].size.width) / max(vc[i].size.width, vc[j].size.width);
              if (Contour_Len1 > 0.25 || Contour_Len2 > 0.25)
                  continue;

              RotatedRect ARMOR;
              ARMOR.center.x = (vc[i].center.x + vc[j].center.x) / 2.; //x坐标
              ARMOR.center.y = (vc[i].center.y + vc[j].center.y) / 2.; //y坐标
              ARMOR.angle = (vc[i].angle + vc[j].angle) / 2.; //角度
              float nh, nw, yDiff, xDiff;
              nh = (vc[i].size.height + vc[j].size.height) / 2; //高度
               // 宽度
              nw = sqrt((vc[i].center.x - vc[j].center.x) * (vc[i].center.x - vc[j].center.x) + (vc[i].center.y - vc[j].center.y) * (vc[i].center.y - vc[j].center.y));
              float ratio = nw / nh; //匹配到的装甲板的长宽比
              xDiff = abs(vc[i].center.x - vc[j].center.x) / nh; //x差比率
              yDiff = abs(vc[i].center.y - vc[j].center.y) / nh; //y差比率
              if (ratio < 0.95 || ratio > 5.0 || xDiff < 0.5 || yDiff > 2.0)
                       continue;
              ARMOR.size.height = nh;
              ARMOR.size.width = nw;
              vRec.push_back(ARMOR);
              Point2f point1;
              Point2f point2;
              point1.x=vc[i].center.x;point1.y=vc[i].center.y+30;
              point2.x=vc[j].center.x;point2.y=vc[j].center.y-30;
              int xmidnum = (point1.x+point2.x)/2;
              int ymidnum = (point1.y+point2.y)/2;
              //轮廓已筛选完毕
              ARMOR.center.x = filter(ARMOR.center.x,xmidnum, DELAT_MAX);
              ARMOR.center.y = filter(ARMOR.center.y,ymidnum, DELAT_MAX);
              Scalar color(0,0,255);
              rectangle(src, point1,point2, color, 2);//将装甲板框起来

              line(src, Point(ARMOR.center.x - 36, ARMOR.center.y - 36),
              Point(ARMOR.center.x + 36, ARMOR.center.y + 36), Scalar(100, 25, 150), 5);
              line(src, Point(ARMOR.center.x + 36, ARMOR.center.y - 36),
              Point(ARMOR.center.x - 36, ARMOR.center.y + 36), Scalar(100, 25, 150), 5);

              line(src, Point(ARMOR.center.x - 10, ARMOR.center.y - 10),
              Point(ARMOR.center.x + 10, ARMOR.center.y + 10), Scalar(255, 255, 0), 5);
              line(src, Point(ARMOR.center.x + 10, ARMOR.center.y - 10),
              Point(ARMOR.center.x - 10, ARMOR.center.y + 10), Scalar(255, 255, 0), 5);
              circle(src,ARMOR.center,7,Scalar(0, 0, 255),2,8,0);//在装甲板中心画一个圆
              cout << "中心点位置" << "["<<ARMOR.center.x << ", "<< ARMOR.center.y << "]" << endl;
              x = ARMOR.center.x, y = ARMOR.center.y;
              imshow("锁定装甲区域",src);
            }
       }

        waitKey(20);
    }
    cap.release();
    return 0;
}


