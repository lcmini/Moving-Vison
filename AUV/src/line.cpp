#include <line.h>
int C_X,C_Y;//全局变量,中心点坐标
int k;


int find_line(Mat frame,int start) //巡直线函数
{

    int flag = 0; //判断位,初始值为0
    int g_nStructElementSize = 3; //结构元素(内核矩阵)的尺寸
    int iterations = 1;
    int not_findtime = 0;
    Mat kernel = getStructuringElement(MORPH_RECT,Size(g_nStructElementSize,g_nStructElementSize));//5*5的卷积核
    Mat mask,mask1,mask2,mask3,edge,out,hsv;


    int w,h,dims,center_number;//对应mask图像的宽,高,通道数,左右边界点个数
    center_number = 0; //中心检测次数赋值为0
    int white_point_left_x = 0,white_point_left_y = 0,white_point_right_x = 0,white_point_right_y = 0,
    sum_x = 0,sum_y = 0,last_x = 0,last_y = 0;

    int white_point_center_x = 0,white_point_center_y = 0;
    int white_point_number = 0;

    int x1,x2,y1,y2;


    if (start == 1) //开启巡直线
    {
        medianBlur(frame,frame,3); //中值滤波
        GaussianBlur(frame,frame,Size(1,1),3);//高斯滤波
        cvtColor(frame,hsv,COLOR_BGR2HSV);
        inRange(hsv,Scalar(158,43,46),Scalar(180,255,255),mask1);//提取红色
        mask1 = 255 -mask1;
        medianBlur(mask1,mask1,3);
        GaussianBlur(mask1,mask1,Size(1,1),1.5);//高斯滤波
        erode(mask1,mask1,kernel);
        inRange(hsv,Scalar(0,43,46),Scalar(8,255,255),mask2);//提取红色
        mask2 = 255 - mask2;
        medianBlur(mask2,mask2,3);
        GaussianBlur(mask2,mask2,Size(1,1),1.5);//高斯滤波
        erode(mask2,mask2,kernel);
        mask1 = 255 - mask1;
        mask2 = 255 - mask2;
        cv::add(mask1,mask2,mask3); //相加
        medianBlur(mask3,mask3,5);
        erode(mask3,mask3,kernel);//白色区域腐蚀操作
        dilate(mask3,mask3,kernel,Point(1,1), iterations);//白色区域膨胀2次
        erode(mask3,mask3,kernel);//白色区域腐蚀
        GaussianBlur(mask3,mask3,Size(1,1),1.5);//高斯滤波
        medianBlur(mask3,mask3,3);
        imshow("mask",mask3);//掩膜mask*/
        w = mask3.cols; //w为mask宽度
        h = mask3.rows; //h为mask高度
        dims = mask3.channels(); //单通道
        for(int row = 20; row < 180; row++) //从第20行开始遍历至110行
          {
            for(int col = 0 ; col < w; col++) //第一行第一列向右遍历直至结束,再开始第二行
            {
              if ((mask3.at<uchar>(row,col)==0) && (mask3.at<uchar>(row,col+1) == 255))//寻找线左侧点
              {
                white_point_left_x = col+1;
                white_point_left_y = row;
                white_point_number += 1;
              }
              if ((mask3.at<uchar>(row,col)==255) && (mask3.at<uchar>(row,col+1) == 0)) //寻找线右侧点
              {
                white_point_right_x = col;
                white_point_right_y = row;
                white_point_number += 1;
              }
                white_point_center_x = (white_point_left_x + white_point_right_x) / 2;
                white_point_center_y = (white_point_left_y + white_point_right_y) / 2;
             }
            if ((white_point_center_x != 0) && (white_point_center_y != 0))
            {
                cv::circle(frame, Point(int(white_point_center_x),int(white_point_center_y)), 5, Scalar(0,255,0),-1,4); //画圆心
                center_number += 1; //每检测到一次白色中心点,次数加1,计算检测白点次数
                flag = 1;
                sum_x += white_point_center_x; //所有白色中心点的横坐标相加
                sum_y += white_point_center_y; //所有白色中心点的纵坐标相加
                if (row == 30)
                {
                    x1 = white_point_center_x;
                    y1 = white_point_center_y;
                }
                if (row == 150)
                {
                    x2 = white_point_center_x;
                    y2 = white_point_center_y;
                }
                if((x2-x1) != 0)
                  {
                    k = abs((y2-y1)/(x2-x1));
                  }
                else
                    cout << "straight line" << endl;
            }

          }
       if (flag == 1)
       {
         last_x = sum_x / center_number; //横坐标和除白点次数,得到中心横坐标的平均值
         last_y = sum_y / center_number; //纵坐标和除白点次数,得到中心纵坐标的平均值
         C_X = int(last_x);
         C_Y = int(last_y);
         cv::circle(frame, Point(C_X,C_Y), 5, Scalar(0,255,0),-1,4); //画圆心
         imshow("out",frame);
         return flag;
       }
      else
         return 0;
    }
    else
        return -1;

}
