#include <usart.h>


int usart_boot(void)
{

    int fd;
    if(wiringPiSetup() == -1)
    {
            printf("硬件初始化失败\n");
    }
    fd = serialOpen("/dev/ttyAMA0",115200);//串口序号待定,波特率13400

   return fd;

}

void data_send(int fd,int X,int Y,int Z,int i)
{
    fd = usart_boot();
    char str1[13]; //字符数组
    char str2[13];
    char str3[13];

    string str_1,str_2,str_3;

    if (i == 1)
    {
        sprintf(str1,"TX%dsY%dS", X,Y);
        str_1 = str1;
        serialPuts(fd,str_1);
     }
    if (i == 2)
    {
        sprintf(str2,"TY%dsZ%dS", X,Z);
        str_2 = str2;
        serialPuts(fd,str_2);

    }
    if (i == 3)
    {
        sprintf(str3,"TY%dsZ%dS", Y,Z);
        str_3 = str3;
        serialPuts(fd,str_3);
    }

}
