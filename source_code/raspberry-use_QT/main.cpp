#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_function.h"
#include <list>
#include <sstream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
using namespace cv;
int i,j;

int main()
{
    int fd;
    string temp;
    //char buffer;
    if(wiringPiSetup() == -1){
        cout << "Unble to start wiringPi";
        return 0;
    }
    if((fd = serialOpen("/dev/ttyACM0", 115200)) < 0){
        cout << "Unable to open serial device";
        return 0;
    }

   int cm = system("sudo modprobe bcm2835-v4l2");
   wiringPiSetup();
   VideoCapture cap(0);
   cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
   cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
   cap.set(CV_CAP_PROP_FPS,10);
   Mat img;
   int angle_drive;
   while(cap.isOpened()){
       cap >> img;

       if(! img.data )
       {
           cout <<  "Could not open or find the image" << endl ;
           return -1;
       }

       angle_drive = findAngleDrive(img);
       sendData(fd, angle_drive);
       /*if((char)49== waitKey(1)){
                break;
       }*/
   }
    return 0;
}

