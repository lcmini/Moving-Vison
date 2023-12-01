#include <ball.h>

#define UnitRadius 150 //球距离1米时在屏幕中的半径，以像素为单位
#define DebugMode      //实际使用时注释此行，只输出指令而不输出图片
                   /****************** 小球搜索函数 *********************

                   其中 TargetErr 定义为 TargetErr = 当前位置 - 参考位置**/


/****定义结构体,成员为球面中心和半径****/
struct Circle
{
    Point2f center;
    float radius;
};
//

/**** cmp 函数，排序中用到****/
bool cmp(const Circle x, const Circle y)
{
    return x.center.x > y.center.x; //从左到右排序
}
//


/****得到横纵方能的差值(像素值)****/
static double angle(Point pt1, Point pt2, Point pt0)
  {
   double dx1 = pt1.x - pt0.x;
   double dy1 = pt1.y - pt0.y;
   double dx2 = pt2.x - pt0.x;
   double dy2 = pt2.y - pt0.y;
   return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
//
                                    /****蓝色球识别函数****/
             /*入口参数分别为图像、误差距离、需要寻找的球的个数、最终找到的个数,返回字符F or N*/
char BlueBallSearch(Mat SrcMat, Point3i *TargetErr, int FindNum, int &ResultNum)
{

    cv::Mat outMat, HSVMat,mask;
    int g_nStructElementSize = 5; //结构元素(内核矩阵)的尺寸
    int iterations = 3;
    Mat kernel = getStructuringElement(MORPH_RECT,Size(g_nStructElementSize,g_nStructElementSize));//5*5的卷积核


    medianBlur(SrcMat,SrcMat,5);
    cvtColor(SrcMat,HSVMat,COLOR_BGR2HSV);
    inRange(HSVMat,Scalar(80,95,110),Scalar(124,255,255),mask);//提取蓝色
    medianBlur(mask,mask,5);
    mask = 255 - mask;
    erode(mask,mask,kernel);//白色区域腐蚀操作
    dilate(mask,mask,kernel,Point(1,1), iterations);//白色区域膨胀3次
    erode(mask,mask,kernel);//白色区域腐蚀
    GaussianBlur(mask,mask,Size(1,1),1.5);//高斯滤波
    mask = 255 -mask;//黑白互换
    outMat = mask;
    imshow("mask",mask);//掩膜mask


    //寻找区域轮廓
    vector<vector<Point>> contours, Circles;
    vector<Vec4i> Hierarchy;
    vector<Point> approx;

    findContours(outMat, contours,Hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE, Point());
    //根据轮廓画最小包围圆

    vector<Circle> EncCircle(contours.size());

    //使用图像轮廓点进行多边形拟合
    for (size_t i = 0; i < contours.size(); i++)
    {
        //这里弧长越小，近似出的圆轮廓边数越多
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.01, true);
        //计算轮廓面积后，得到所有顶点，把不小于5个和面积较大的放进备选
       if (approx.size() >= 5 &&fabs(contourArea(Mat(approx))) > 100 &&isContourConvex(Mat(approx)))
        {
            Circles.push_back(approx);
        }
    }
    //

    //按需要的数量寻找圆
   for (int i = 0;i< Circles.size(); i++)
   {
       minEnclosingCircle(Circles[i], EncCircle[i].center, EncCircle[i].radius);
   }
   //按cmp的标准排序
   sort(EncCircle.begin(), EncCircle.end(), cmp);
    //如果没有输入的那么多数量的球可以找到，那么找到尽可能多的
   if (FindNum > EncCircle.size())
       FindNum = EncCircle.size();
   if (FindNum > 0)
   {
       for (int i = 0; i < FindNum;i++)
       {
           if (EncCircle[i].radius > 10)
           {
#ifdef DebugMode
                circle(SrcMat, EncCircle[i].center, (int)EncCircle[i].radius, Scalar(0,0,255),2,8,0);
#endif
                //计算圆心偏差
                TargetErr[ResultNum].y = (int)EncCircle[i].center.x - 320; //y方向偏差，向右为正
                TargetErr[ResultNum].z = (int)EncCircle[i].center.y - 240; //z方向偏差，向下为正
                TargetErr[ResultNum].x = UnitRadius / (int)EncCircle[i].radius; //x方向偏差，用比例计算
                ResultNum++;
            }
        }
   }

#ifdef DebugMode
   //画近似多边形
   for (int i = 0; i< Circles.size(); i++)
   {
       const Point* p = &Circles[i][0];

       int n = (int)Circles[i].size();
       polylines(SrcMat, &p, &n, 1, true, Scalar(255,0,0),3, LINE_AA);
   }
   cv::imshow("show out", outMat);  //打印阈值图
   cv::imshow("srcMat out", SrcMat);
#endif

   if (ResultNum > 0)
   {
       return 'F'; //F表示找到了
   }
   else
   {
       return 'N'; //N表示没找到
   }
}





                                  /****绿色球识别函数****/
             /*入口参数分别为图像、误差距离、需要寻找的球的个数、最终找到的个数,返回字符F or N*/
char GreenBallSearch(Mat SrcMat, Point3i *TargetErr, int FindNum, int &ResultNum)
{

    cv::Mat outMat, HSVMat,mask;   //输出图像,HSV图像,掩膜mask
    int g_nStructElementSize = 5; //结构元素(内核矩阵)的尺寸
    int iterations = 3;    //膨胀次数3次
    Mat kernel = getStructuringElement(MORPH_RECT,Size(g_nStructElementSize,g_nStructElementSize));//5*5的卷积核



    //resize(SrcMat,SrcMat,Size(256,256));
    medianBlur(SrcMat,SrcMat,5);
    cvtColor(SrcMat,HSVMat,COLOR_BGR2HSV);
    inRange(HSVMat,Scalar(35,40,45),Scalar(80,255,255),mask);//提取绿色
    medianBlur(mask,mask,5);
    mask = 255 - mask;
    erode(mask,mask,kernel);//白色区域腐蚀操作
    dilate(mask,mask,kernel,Point(1,1), iterations);//白色区域膨胀3次
    erode(mask,mask,kernel);//白色区域腐蚀
    GaussianBlur(mask,mask,Size(1,1),1.5);//高斯滤波
    mask = 255 -mask;//黑白互换
    outMat = mask;
    imshow("mask",mask);//掩膜mask


//寻找区域轮廓
    vector<vector<Point>> contours, Circles;
    vector<Vec4i> Hierarchy;
    vector<Point> approx;

    findContours(outMat, contours,Hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE, Point());
    //根据轮廓画最小包围圆

    vector<Circle> EncCircle(contours.size());

    //使用图像轮廓点进行多边形拟合
    for (size_t i = 0; i < contours.size(); i++)
    {
         //这里弧长越小，近似出的圆轮廓边数越多
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.01, true);
    //计算轮廓面积后，得到所有顶点，把不小于5个和面积较大的放进备选
       if (approx.size() >= 5 &&fabs(contourArea(Mat(approx))) > 100 &&isContourConvex(Mat(approx)))
      {
       Circles.push_back(approx);
      }
     }
    //

    //按需要的数量寻找圆
    for (int i = 0;i< Circles.size(); i++)
    {
         minEnclosingCircle(Circles[i], EncCircle[i].center, EncCircle[i].radius);
    }
    //按cmp的标准排序
    sort(EncCircle.begin(), EncCircle.end(), cmp);
    //如果没有输入的那么多数量的球可以找到，那么找到尽可能多的
    if (FindNum > EncCircle.size())
       FindNum = EncCircle.size();
    if (FindNum > 0)
    {
       for (int i = 0; i < FindNum;i++)
       {
          if (EncCircle[i].radius > 10)
          {
            #ifdef DebugMode
            circle(SrcMat, EncCircle[i].center, (int)EncCircle[i].radius, Scalar(0,0,255),2,8,0);
            #endif
            //计算圆心偏差
            TargetErr[ResultNum].y = (int)EncCircle[i].center.x - 320; //y方向偏差，向右为正
            TargetErr[ResultNum].z = (int)EncCircle[i].center.y - 240; //z方向偏差，向下为正
            TargetErr[ResultNum].x = UnitRadius / (int)EncCircle[i].radius; //x方向偏差，用比例计算
            ResultNum++;
          }
       }
    }

    #ifdef DebugMode
    //画近似多边形
    for (int i = 0; i< Circles.size(); i++)
    {
        const Point* p = &Circles[i][0];

        int n = (int)Circles[i].size();
        polylines(SrcMat, &p, &n, 1, true, Scalar(255,0,0),3, LINE_AA);
    }
       cv::imshow("show out", outMat);  //打印阈值图
       cv::imshow("srcMat out", SrcMat);
    #endif

    if (ResultNum > 0)
    {
      return 'F'; //F表示找到了
    }
    else
    {
       return 'N'; //N表示没找到
    }
}

