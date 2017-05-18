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
extern int i,j;



Mat calcEdgeAndROI(Mat gray)
{
    Mat edges;
    Canny(gray, edges, 100, 110);
    i = edges.rows;
    j = edges.cols;
    for(int x=0; x < i/2; x++){
        for(int y=0; y < j; y++){
            edges.at<uchar>(x,y)= 0;
        }
    }

    /*for(int x=i*85/100; x < i; x++){
        for(int y=0; y < j; y++){
            edges.at<uchar>(x,y)= 0;
        }
    }*/

    /*for(int x=i/2; x < i; x++){
        for(int y=j*40/100; y < j*60/100; y++){
            edges.at<uchar>(x,y)= 0;
        }
    }*/

    /*int k = j*1/5;
    int start = i/2;

    while(k>0){
        for(int x=start; x < i; x++){
            for(int y=0; y < k; y++){
                edges.at<uchar>(x,y)= 0;
            }
            k = k - 2;
        }
    }

    k = j*4/5;
    while(k<=j){
        for(int x=start; x < i; x++){
            for(int y=k; y < j; y++){
                edges.at<uchar>(x,y)= 0;
            }
            k = k + 2;
        }
    }*/

    return edges;
}

list<float> selectRadians(vector<Vec2f> lines)
{
    list<float> rads;
    for(size_t i=0; i<lines.size(); i++){
        float theta = lines[i][1];
        float radian = theta;
        theta = theta * 180 / CV_PI;
        if(theta > 20 and theta < 80){
            rads.push_back(radian);
        }
        if(theta > 110 and theta < 170){
            rads.push_back(radian);
        }
    }
    return rads;
}

StraightLine calcX1Y1X2Y2(float rad, float rho)
{
    StraightLine line;
    float a = cos(rad);
    float b = sin(rad);\
    float X0 = a*rho;
    float Y0 = b*rho;
    line.A.x = int(X0 + 2000*(-b));
    line.A.y = int(Y0 + 2000*(a));
    line.B.x = int(X0 - 2000*(-b));
    line.B.y = int(Y0 - 2000*(a)) ;
    return line;
}

float findMaxRads(list<float> rads)
{
    list<float>::iterator position = rads.begin();
    float max = *position;
    for(size_t i=1; i< rads.size()-1; i++){
        position++;
        if(*position > max){
            max = *position;
        }
    }
    return max;
}
float findMinRads(list<float> rads)
{
    list<float>::iterator rad = rads.begin();
    float min = *rad;
    for(size_t i=1; i< rads.size()-1; i++){
        rad++;
        if(*rad < min){
            min = *rad;
        }
    }
    return min;
}

Point_<int> findIntersection(StraightLine l1, StraightLine l2)
{
    Point_<int> I;
    float a1, a2;
    Point_<float> a = l1.A;
    Point_<float> b = l1.B;
    Point_<float> c = l2.A;
    Point_<float> d = l2.B;
    if(c.x == d.x){
        I.x = j/2;
        I.y = (a.y - b.y)*(I.x - a.x)/(a.x-b.x) + a.y;
    }
    else{
        a1 = ((a.y-b.y)/(a.x-b.x)); //slope of the line 1
        a2 = ((c.y-d.y)/(c.x-d.x)); //slope of the line 2
        if(a1 != a2){
        I.x = (a1*a.x - a2*c.x - a.y + c.y)/(a1-a2);
        I.y = a1*(I.x-a.x) + a.y;
        }
    }
    return I;
}

float findDistance(Point I, StraightLine l)
{
    float a = l.A.y - l.B.y;
    float b = l.B.x - l.A.x;
    float c = -a*l.A.x - b*l.A.y;
    float d = fabs(a*I.x + b*I.y + c) / sqrt(pow(a,2) + pow(b,2));
    return d;
}

int findAngleDrive(Mat img)
{
    static int tTime=0, k = 0;
    Mat gray;
    cvtColor(img, gray, CV_RGB2GRAY);

    Mat edges;
    edges = calcEdgeAndROI(gray);

    vector<Vec2f> lines;
    HoughLines(edges, lines, 6, CV_PI/180, 180);

    list<float> rads = selectRadians(lines);

    bool flag1 = true, flag2 = true;

    //find the best line to the left and right of the vehicle
    StraightLine leftlane, rightlane;
    float maxr ;
    float minr ;
    if(rads.size() > 0 ){
        maxr = findMaxRads(rads);
        minr = findMinRads(rads);
        StraightLine l;
        float theta;
        float rho;
        for(size_t i=0; i<lines.size(); i++){
            theta = lines[i][1];
            rho = lines[i][0];
            //for RIGHT LANE
            if(maxr == theta and flag1 == true){
                l = calcX1Y1X2Y2(maxr, rho);
                if((flag2 == false && maxr-minr > 0.5) || flag2 == true){
                    //line(img, l.A, l.B,CV_RGB(0,255,0),5);
                    rightlane.A.x = l.A.x;
                    rightlane.A.y = l.A.y;
                    rightlane.B.x = l.B.x;
                    rightlane.B.y = l.B.y;
                    flag1 = false;
                }
            }

            //for LEFT LANE
            if(minr == theta and flag2 == true){
                l = calcX1Y1X2Y2(minr, rho);
                if((flag1 == false && maxr-minr > 0.5) || flag1 == true){
                    leftlane.A.x = l.A.x;
                    leftlane.A.y = l.A.y;
                    leftlane.B.x = l.B.x;
                    leftlane.B.y = l.B.y;
                    //line(img, l.A, l.B,CV_RGB(0,0,255),5);
                    flag2 = false;
                }
            }
        }
    }

    //line(img, Point(j/2,i), Point(j/2,0),CV_RGB(255,0,0), 5);

    k++;
    if(millis() - tTime > 1000){
        tTime = millis();
        cout << k << endl;
        fflush(stdout);
        k = 0;
    }

    float left_distance = findDistance(Point(j/2,i), leftlane);
    float right_distance = findDistance(Point(j/2,i), rightlane);

    StraightLine middle_line;
    middle_line.A = Point(j/2,200);
    middle_line.B = Point(j/2,600);

    Point_<int> I_left = findIntersection(leftlane, middle_line);
    int angle_left = acos(left_distance / (i - I_left.y)) * 180 / CV_PI;

    Point_<int> I_right = findIntersection(rightlane, middle_line);
    int angle_right = acos(right_distance / (i - I_right.y)) * 180 / CV_PI;

    int angle_drive = angle_left - angle_right;

    int maxd =maxr * 180/CV_PI;
    int mind =minr * 180/CV_PI;
    if(flag1 == true && flag2 == false){
        angle_drive = (90 - mind)/2;
    }
    if(flag1 == false && flag2 == true){
        if(maxd < 90 && maxd > 0){
            angle_drive = angle_right/2;
        }
        else{
            angle_drive = -angle_right/2;
        }
    }
    //ostringstream lable5;
    //lable5 << angle_drive;
    //putText(img, lable5.str(), Point(3*j/4, i/4), FONT_HERSHEY_PLAIN, 5.0, CV_RGB(0,255,0), 5.0);

    //Create a window for display.
    //namedWindow( "Display window", CV_WINDOW_NORMAL );
    //resizeWindow("Display window", 400, 400);
    //imshow( "Display window", img );
    //imwrite("hinh.png",img);
    return angle_drive;
}

void sendData(int fd, int angle_drive)
{
    if(angle_drive <45 && angle_drive > -45){
        if(angle_drive > 30){
            angle_drive = 30;
        }
        else{
            if(angle_drive < -30){
                angle_drive = -30;
            }
        }
        char tt[10];
        sprintf(tt, "%d", angle_drive);
        serialPuts(fd, tt);
        serialPutchar(fd, '.');
    }
}
