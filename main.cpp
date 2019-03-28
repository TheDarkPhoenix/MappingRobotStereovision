#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "types.h"
#include "util.h"
#include "robot.h"
#include "mapa.h"
#include <iostream>

#define radtodeg 57.295779513082

using namespace cv;
using namespace std;

int main()
{
    HANDLE hSerial;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts={0};

    init(hSerial, dcbSerialParams, timeouts);

    sendread(hSerial, 1);
    sendread(hSerial, 125);
    sendread(hSerial, 2);
    sendread(hSerial, 125);
    sendread(hSerial, 5);
    sendread(hSerial, 6);

    StereoSGBM sgbm;
    VideoCapture cap1(1);
    VideoCapture cap2(2);
    Mat map11, map12, map21, map22, img1r, img2r, imgl, imgr, Q;
    int c = 0, numberOfDisparities = 128;

    if (!initStereovision(sgbm, cap1, cap2, map11, map12, map21, map22, Q, numberOfDisparities))
        return 1;

    Mat rob = Mat(Size(1280, 960), CV_8U, Scalar(0));
    /*namedWindow("robot",0);
    resizeWindow("robot", 640, 480);*/

    //namedWindow("objects",1);
    namedWindow("objectsbox",1);
    /*namedWindow("mapa",0);
    resizeWindow("mapa", 700, 700);*/
    namedWindow("disparity", 0);
    resizeWindow("disparity", 640, 480);
    namedWindow("submap",1);
    //resizeWindow("submap", 480, 600);

    Mat disp, dispr, disp8, disp3d;
    mapa mappin;
    robot robocik;
    int shiftx = 640, shifty = 480;
    while(true)
    {

        cap1 >> imgr;
        cap2 >> imgl;

        remap(imgl, img1r, map21, map22, INTER_LINEAR);
        remap(imgr, img2r, map11, map12, INTER_LINEAR);

        imgl = img1r;
        imgr = img2r;


        sgbm(imgr, imgl, dispr);

        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < dispr.cols; ++j)
                dispr.at<int16_t>(i, j) = 0;
        }

        for (int i = 314; i < 320; ++i)
        {
            for (int j = 0; j < dispr.rows; ++j)
                dispr.at<int16_t>(j, i) = 0;
        }

        /*Mat element = getStructuringElement( 0, Size( 5, 5 ), Point( 2, 2 ) );
        morphologyEx(dispr, disp, 3, element);*/
        disp = dispr;

        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*8.));

        reprojectImageTo3D(disp, disp3d, Q, true, CV_32F);
        setMouseCallback("imgr", CallBackFunc, &disp3d);

        mappin.run(disp, disp3d);

        //imshow("objects", mappin.getObjects());
        imshow("imgr", imgr);
        imshow("objectsbox", mappin.getObjectsbox());
        //imshow("mapa", mappin.getMap());
        imshow("disparity", disp8);
        imshow("submap", mappin.getSubMap());
        //line(rob, Point(robocik.getLastPosition().y + shifty, -robocik.getLastPosition().x + shiftx), Point(robocik.getPosition().y + shifty, -robocik.getPosition().x + shiftx), Scalar(200, 200, 200), 2);
        //imshow("robot", rob);

        c = waitKey(1);
        if((char)c==27 ) break;
        switch (c)
        {
            case 'w':
                robocik.accelerate(1, hSerial);
                break;
            case 's':
                robocik.accelerate(-1, hSerial);
                break;
            case 'd':
                robocik.turn(1, hSerial);
                break;
            case 'a':
                robocik.turn(-1, hSerial);
                break;
            case 'q':
                robocik.stop(hSerial);
                break;
        }
        //robocik.update(hSerial);
        //mappin.move(robocik.getPosition());
    }

    sendread(hSerial, 1);
    sendread(hSerial, 125);
    sendread(hSerial, 2);
    sendread(hSerial, 125);
    CloseHandle(hSerial);

    return 0;
}


