#include "mapa.h"
#include <iostream>

using namespace std;

void rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(src.cols/2, src.rows/2);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(len, len));
}

mapa::mapa()
{
    rozmiary.height = 1000;
    rozmiary.width = 1000;
    rozmiaryWycinka.height = 150;
    rozmiaryWycinka.width = 120;
    rozmiaryPoObroceniu.width = 324;
    rozmiaryPoObroceniu.height = 324;
    mMapa = Mat(rozmiary, CV_8U, Scalar(0));
    //subMap = Mat(rozmiary, CV_8U, Scalar(0));
    //rotateMap = Mat(200, 200, CV_8U, Scalar(0));
    shift.x = rozmiary.width/2;
    shift.y = rozmiary.height/2;
    /*FileStorage fs("D:/mappa.yml", CV_STORAGE_READ);
    if(fs.isOpened())
    {
        fs["V"] >> kara;
        fs.release();
        cout << kara.size() << endl;
    }
    else
        cout << "error" << endl;
    kara.convertTo(kara, CV_8U);*/
    kara = Mat(150, 120, CV_8U);

    aktualne = Mat(kara.size(), CV_8U, Scalar(freeScale));

    lastPoid = punkt(0,0);
    lastPoig = punkt(0,0);

    lastmaxg = punkt(18,0);
    lastmaxd = punkt(56, 239);

    lastming = punkt(16,0);
    lastmind = punkt(54, 239);
    rotation = 0;
}

void mapa::vdisparity()
{
    int v;
    Vec3f point;
    for (int i = 0; i < disparity.size().height; ++i)
    {
        for (int j = 0; j < disparity.size().width; ++j)
        {
            v = disparity.at<int16_t>(i,j)/16;
            if (v > 0 && v < 320)
                vDisparity.at<int8_t>(i,v) += 1;
        }
    }
    for (int i = 0; i < disparity.size().height; ++i)
    {
        for (int j = 0; j < disparity.size().width; ++j)
        {
            if (vDisparity.at<int8_t>(i,j) < 2)
                vDisparity.at<int8_t>(i,j) = 0;
        }
    }
}

void mapa::findObjects()
{
    float calcPointDisp,pointDisp;
    for (int i = 0; i < disparity.size().height; ++i)
    {
        calcPointDisp = (i-bestLine.b)/bestLine.a;
        for (int j = 0; j < disparity.size().width; ++j)
        {
            pointDisp = disparity.at<int16_t>(i,j)/16;
            if (pointDisp > (calcPointDisp+1.25)) //&& (pointDisp < calcPointDisp+5))
            {
                objects.at<int8_t>(i,j) = pointDisp;
            }
        }
    }
}

void mapa::addObjectToMap(Rect& object, Mat& objects)
{
    Point2i pu;
    Vec3f point;
    Vec3i pointmap;
    Mat subMap = Mat(rozmiaryWycinka, CV_8U, Scalar(0));
    for (int i = object.x; i < (object.x + object.width); ++i)
    {
        for (int j = object.y; j < (object.y + object.height); ++j)
        {
            point = disp3d.at<Vec3f>(j,i);
            if(fabs(point[2])< 30 && objects.at<uint8_t>(j,i) > 0)
            {
                point[0] *= 15.9;
                point[1] *= 19.2;
                point[2] *= 16;
                pointmap[0] = (point[0]) + 60;
                pointmap[2] = (-point[2])+150;
                if (pointmap[0] > 0 && pointmap[0] < 120 && pointmap[2] > 5 && pointmap[2] < 150 &&(mMapa.at<uint8_t>(pointmap[2] + shift.y, pointmap[0] + shift.x)+addMap)< 255)
                {
                    if (aktualne.at<uint8_t>(pointmap[2], pointmap[0]) == freeScale)
                    {
                        aktualne.at<uint8_t>(pointmap[2], pointmap[0]) = objScale;
                        subMap.at<uint8_t>(pointmap[2], pointmap[0]) += addMap;
                    }
                }
            }
        }
    }
    Mat rotateMap = Mat(rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height, CV_8U, Scalar(0));

    Mat srot = rotateMap(Rect(102, 12, rozmiaryWycinka.width, rozmiaryWycinka.height));
    subMap.copyTo(srot);
    rotate(rotateMap, rotation, rotateMap);
    Mat crot = mMapa(Rect((shift.x - rozmiaryPoObroceniu.width/2), (shift.y-rozmiaryPoObroceniu.height/2), rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height));
    crot += rotateMap;
}

void mapa::drawObjects()
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /*for (int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < objects.size().width; ++j)
        {
            objectsbox.at<uint8_t>(i,j) = 0;
        }
    }*/
    threshold( objectsbox, threshold_output, thres, 255, THRESH_BINARY );

    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    }

    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    drawing = objectsbox;
    aktualne = Mat(kara.size(), CV_8U, Scalar(freeScale));
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( 122,122,122 );
        //if (boundRect[i].height > 20 && boundRect[i].width > 20)
        if (boundRect[i].height * boundRect[i].width > 300)
        {
            addObjectToMap(boundRect[i],drawing);
            drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        }
    }
    objectsbox = drawing;
}

void multiply(Mat& A, Mat& B, Mat& C)
{
    for (int i = 0; i < A.cols; ++i)
        for (int j = 0; j < A.rows; ++j)
            C.at<uint8_t>(i,j) = A.at<uint8_t>(i,j) * B.at<uint8_t>(i,j);
}

void mapa::penalizemap()
{
    visibility();
    Mat wynik = Mat(kara.size(), CV_8U, Scalar(0));
    multiply(aktualne, kara, wynik);

    Mat crot = Mat(rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height, CV_8U, Scalar(0));
    Mat srot = crot(Rect(102, 12, rozmiaryWycinka.width, rozmiaryWycinka.height));
    wynik.copyTo(srot);
    rotate(crot, rotation, crot);
    Mat submap = mMapa(Rect((shift.x - rozmiaryPoObroceniu.width/2), (shift.y-rozmiaryPoObroceniu.height/2), rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height));
    submap -= crot;
}

/*void udisparity(Mat& disp, Mat& udisp)
{
    int u;
    Vec3f point;
    const double max_z = 10000;
    for (int i = 0; i < disp.size().width; ++i)
    {
        for (int j = 0; j < disp.size().height; ++j)
        {
            u = disp.at<int16_t>(j,i)/16;
            if (u > 0 && u < 240)
                udisp.at<int8_t>(u,i) += 1;
        }
    }
    for (int i = 0; i < disp.size().height; ++i)
    {
        for (int j = 0; j < disp.size().width; ++j)
        {
            if (udisp.at<int8_t>(i,j) < 20)
                udisp.at<int8_t>(i,j) = 0;
        }
    }
}

void uroad(punkt p1, punkt p2,const Mat& disp, Mat& road)
{
    float u;
    float x = (p1.y + p2.y)/2;
    for (int i = p1.x; i < p2.x; ++i)
    {
        for (int j = 0; j < road.size().height; ++j)
        {
            u = disp.at<int16_t>(j,i)/16;
            if ((u > (x-2)) && (u < (x + 2)))
                road.at<int8_t>(j,i) = 0;
        }
    }
}*/

void mapa::visibility()
{
    const double max_z = 1.0e4;
    Vec3f point;
    Vec3i pointmap;
    int val = 10;
    kara = Mat(150, 120, CV_8U);
    for (int i = 0; i < disp3d.cols; ++i)
    {
        for (int j = 0; j < disp3d.rows; ++j)
        {
            point = disp3d.at<Vec3f>(j,i);
            point[0] *= 15.9;
            point[1] *= 19.2;
            point[2] *= 16;
            pointmap[0] = (point[0]) + 60;
            pointmap[2] = (-point[2])+150;
            if (pointmap[0] > 0 && pointmap[0] < 120 && pointmap[2] > 5 && pointmap[2] < 150)
                kara.at<uint8_t>(pointmap[2], pointmap[0]) = val;
        }
    }
    Mat mapar;
    Mat element = getStructuringElement( 0, Size( 9, 9 ), Point( 2, 2 ) );
    morphologyEx(kara, mapar, 3, element);
    kara = mapar;
    imshow("kara", kara);
}

void mapa::run(Mat& disp1, Mat& disp3d1)
{
    disparity = disp1;
    disp3d = disp3d1;

    /*Mat udisp = Mat(disparity.size(), CV_8U,Scalar(0));
    udisparity(disparity,udisp);

    Mat ulines = Mat(disparity.size(), CV_8UC3, Scalar(0));
    disparity.convertTo(road, CV_8U, 255/(128*8.));
    if (threshold1 > 0)
        HoughLinesP(udisp, lines, 1, CV_PI/180, 1, 30, 15 );//50 5
    for( size_t i = 0; i < lines.size(); i++ )
    {
        lg = punkt(lines[i][0], lines[i][1]);
        ld = punkt(lines[i][2], lines[i][3]);
        line(ulines, Point(lg.x, lg.y), Point(ld.x, ld.y), 255);
        uroad(lg, ld,disparity , road);
    }
    lines.clear();
    imshow("uroad", road);*/
    //disparity  = road;

    vDisparity = Mat(disparity.size(),CV_8U,Scalar(0));
    vdisparity();

    if (threshold2 > 0)
        HoughLinesP(vDisparity, lines, 1, CV_PI/240, threshold2, minLinLength, maxLineGap );//36
    if (lines.size())
    {
        bestLine = lineab();
        maxlg = maxld = poig = poid = punkt(0,0);
        minlg = minld = punkt(300,300);
    }
    for( size_t i = 0; i < lines.size(); i++ )
    {
        lg = punkt(lines[i][0], lines[i][1]);
        ld = punkt(lines[i][2], lines[i][3]);

        if (lg.x < minlg.x)
        {
            minlg.x = lg.x;
        }
        if (lg.x > maxlg.x)
        {
            maxlg.x = lg.x;
        }
        if (ld.x < minld.x)
        {
            minld.x = ld.x;
        }
        if (ld.x > maxld.x)
        {
            maxld.x = ld.x;
        }
        if (lg.y < minlg.y)
        {
            minlg.y = lg.y;
        }
        if (lg.y > maxlg.y)
        {
            maxlg.y = lg.y;
        }
        if (ld.y < minld.y)
        {
            minld.y = ld.y;
        }
        if (ld.y > maxld.y)
        {
            maxld.y = ld.y;
        }
    }
    lines.clear();

    srlg = maxlg;
    srld = maxld;

    if (abs(lastPoig.x - srlg.x)< 2)
    {
        lastPoig.x += (srlg.x - lastPoig.x);
        lastPoig.y += (srlg.y - lastPoig.y);
        poig = lastPoig;
    }
    else
    {
        lastPoig.x += (srlg.x - lastPoig.x)/10;
        lastPoig.y += (srlg.y - lastPoig.y)/10;
        poig = lastPoig;
    }

    if (abs(lastPoid.x - srld.x) < 2)
    {
        lastPoid.x += (srld.x - lastPoid.x);
        lastPoid.y += (srld.y - lastPoid.y);
        poid = lastPoid;
    }
    else
    {
        lastPoid.x += (srld.x - lastPoid.x)/10;
        lastPoid.y += (srld.y - lastPoid.y)/10;
        poid = lastPoid;
    }


    if (poig.x != poid.x)
    {
        bestLine.a = (poig.y - poid.y)/(poig.x - poid.x);
        bestLine.b = (poig.x*poid.y - poid.x*poig.y)/(poig.x - poid.x);
    }

    objects = Mat(disparity.size(),CV_8U,Scalar(0));
    findObjects();

    penalizemap();
    objects.copyTo(objectsbox);
    drawObjects();
    //mMapa.convertTo(mMapa8U, CV_8U);
}

