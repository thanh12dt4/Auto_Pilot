#ifndef IMAGE_FUNCTION_H
#define IMAGE_FUNCTION_H

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

typedef struct StraightLine{
    Point_<int> A;
    Point_<int> B;
}StraightLine;

//to calculate edge and mask most of non-road area in the image
Mat calcEdgeAndROI(Mat gray);

list<float> selectRadians(vector<Vec2f> lines);

StraightLine calcX1Y1X2Y2(float rad, float rho);

float findMaxRads(list<float> rads);

float findMinRads(list<float> rads);

Point_<int> findIntersection(StraightLine l1, StraightLine l2);

float findDistance(Point I, StraightLine l);

void sendData(int fd, int angle_drive);

int findAngleDrive(Mat img);

#endif // IMAGE_FUNCTION_H
