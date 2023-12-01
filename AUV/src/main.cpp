#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <line.h>
#include <rec.h>
#include <usart.h>


using namespace std;
using namespace cv;

int main()
{
  int fd;
  fd = usart_boot();//usart boot


  Mat src,pic;
  VideoCapture cap1(2); //cap1 ->巡线，cap2框，球
  VideoCapture cap2(0);
  cap1.set(cv::CAP_PROP_FRAME_WIDTH,640);
  cap2.set(cv::CAP_PROP_FRAME_WIDTH,640);
  cap1.set(cv::CAP_PROP_FRAME_HEIGHT,480);
  cap2.set(cv::CAP_PROP_FRAME_HEIGHT,480);
  int rec_flag = 0;
  int line_flag = 0;


  while(true)
  {
      cap1 >> src;
      cap2 >> pic;
      imshow("src",src);
      //imshow("pic",pic);
      //int flag = 0;
      //line_flag = find_line(src,1);
      //rec_flag = rec(src,1);
      line_flag = find_line(src,1);
      cout << line_flag << endl;
      waitKey(10);

      if (line_flag == 1)
      {
          cout << "send data of line to stm32" << endl;
          data_send(fd,000,C_X-320,C_Y-240,3);
          waitKey(10);
      }

      if(line_flag != 1)
      {
          cout << "start find rec" << endl;
          rec_flag = rec(pic,1);
          if(rec_flag == 1)
          {
              cout << "send data of rec to stm32" << endl;
              data_send(fd,000,REC_X,REC_Y,3);
          }
          else
          {
              cout << "go on" << endl;
               data_send(fd,000,000,000,1);
          }
      }
   }
  return 0;

}
