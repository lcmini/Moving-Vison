#include <rec.h>

int REC_X,REC_Y;


int rec(Mat frame,int start)
{
    int g_nStructElementSize = 3; //结构元素(内核矩阵)的尺寸
    int iterations = 3;
    Mat kernel = getStructuringElement(MORPH_RECT,Size(g_nStructElementSize,g_nStructElementSize));//5*5的卷积核
    Mat mask1,mask2,mask3,edge,out,hsv,dst;

    if (start == 1)
    {
        cv::resize(frame,frame,Size(640,480));
        medianBlur(frame,frame,3); //中值滤波
        GaussianBlur(frame,frame,Size(1,1),3);//高斯滤波
        cvtColor(frame,hsv,COLOR_BGR2HSV);
        inRange(hsv,Scalar(156,45,47),Scalar(180,255,255),mask1);//提取红色
        mask1 = 255-mask1;
        medianBlur(mask1,mask1,3);
        GaussianBlur(mask1,mask1,Size(1,1),3);//高斯滤波
        erode(mask1,mask1,kernel);
        inRange(hsv,Scalar(0,43,46),Scalar(8,255,255),mask2);//提取红色
        mask2 = 255 - mask2;
        medianBlur(mask2,mask2,3);
        GaussianBlur(mask2,mask2,Size(1,1),3);//高斯滤波
        erode(mask2,mask2,kernel);
        mask1 = 255 - mask1;
        mask2 = 255 - mask2;
        cv::add(mask1,mask2,mask3); //相加
        medianBlur(mask3,mask3,5);
        erode(mask3,mask3,kernel);//白色区域腐蚀操作
        dilate(mask3,mask3,kernel,Point(1,1), iterations);//白色区域膨胀2次
        erode(mask3,mask3,kernel);//白色区域腐蚀
        GaussianBlur(mask3,mask3,Size(1,1),3);//高斯滤波
        medianBlur(mask3,mask3,3);
        //imshow("mask",mask3);//掩膜mask*/
        Canny(mask3,edge,100,100);
        //imshow("edge",edge);

        vector<vector<Point>> contours, squares;
        squares.clear();
        vector<Point> approx;
        vector<Vec4i> Hierarchy;
        findContours(mask3, contours,Hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE);

        Scalar color(0,255, 0);//线条颜色绿色
        float rec_x[20],rec_y[20]; //rec数组，存储焦点的坐标
        float MAX_X,MAX_Y,MIN_X,MIN_Y;
        //int maxxpos,minxpos;
        //MAX_X_V = MAX_Y_V = MIN_X_V = MIN_Y_V = 0;
        //int maxPosition = max_element(a,a+6) - a;


        vector<cv::Point2f> corners;
        int max_corners = 16;
        double quality_level = 0.01;
        double min_distance = 45;
        int block_size = 3;
        bool use_harris = false;
        double k = 0.04;
        if (!contours.empty() && !Hierarchy.empty())
        {
            //角点检测
            cv::goodFeaturesToTrack(edge,corners,max_corners,quality_level,min_distance,cv::Mat(),block_size,use_harris,k);
            //将检测到的角点绘制到原图上
            for (int i = 0; i < corners.size(); i++)
            {
                    cv::circle(frame, corners[i], 5, color, 2, 8, 0);
                    rec_x[i] = corners[i].x;
                    rec_y[i] = corners[i].y;
            }
            MAX_X = *max_element(rec_x,rec_x+corners.size());
            MIN_X = *min_element(rec_x,rec_x+corners.size());
            MAX_Y = *max_element(rec_y,rec_y+corners.size());
            MIN_Y = *min_element(rec_y,rec_y + corners.size());
            //cout << MAX_X << endl;
            //cout << MIN_X << endl;
            //maxxpos = max_element(rec_x,rec_x + corners.size()) - rec_x;
            //minxpos = min_element(rec_x,rec_x + corners.size()) - rec_x;
            /*
            for (int i = 0 ;i < corners.size(); i++)
            {
                if (corners[i].x == MAX_X)
                    MAX_Y = corners[i].y;
                if (corners[i].x == MIN_Y)
                    MIN_Y = corners[i].y;
            }*/
            cv::line(frame,Point(MAX_X,MAX_Y),Point(MIN_X,MIN_Y),color,1);
            //cv::circle(frame,Point((MAX_X + MIN_X)/2.0,(MAX_Y + MIN_Y)/2.0),-1,Scalar(0,0,255),2,8,0);
            REC_X = int((MAX_X + MIN_X)/2.0 - 320);
            REC_Y = int((MAX_Y + MIN_Y)/2.0 - 320);
            imshow("frame",frame);
            return 1;
        }
        else
            return 0;

   }
    else
        cout << "close find rec" << endl;

}




/*
 *         vector<vector<Point>> contours, squares;
        squares.clear();
        vector<Point> approx;
        vector<Vec4i> Hierarchy;
        findContours(mask3, contours,Hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE);
if (!contours.empty() && !Hierarchy.empty())
{
    drawContours(frame, contours, 0, color, FILLED, 8, Hierarchy);//画出轮廓
    for (size_t i = 0; i < contours.size(); i++)
       {
           approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.04, true);
           if (approx.size() >= 4 && approx.size() <= 10)
           {
               //声明一个图像的矩
               Moments M;
               //计算要绘制轮廓的矩
               M = moments(contours[0]);
               //求取轮廓重心的X坐标
               int cX = int(M.m10 / M.m00);
               //求取轮廓重心的Y坐标
               int cY = int(M.m01 / M.m00);
               circle(frame, Point2d(cX, cY), 1, Scalar(0, 0, 255), 10, 15);
               imshow("frame",frame);
               frame_x = cX;
               frame_y = cY;
               cout << "rec_X" << frame_x << "rec_Y" <<frame_y<< endl;
               return 1;
           }
           else
               return 0;
      }

}
else
    return 0;
}
else
return 0;*/
