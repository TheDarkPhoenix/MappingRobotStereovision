#ifndef MAPA_H
#define MAPA_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "types.h"

#define radtodeg 57.295779513082


using namespace cv;

class mapa
{
    private:
        Mat mMapa;
        Mat mMapa8U;
        punkt shift;
        Size rozmiary, rozmiaryWycinka, rozmiaryPoObroceniu;
        vector<Vec4i> lines;
        static const int thres = 20,threshold2 = 200,minLinLength = 150,maxLineGap = 10,si = 0, threshold1 = 100,minLinLength1 = 50,maxLineGap1 = 5;
        static const int shiftx = 500, shifty = 500;
        static const int objScale = 1, freeScale = 3, addMap = 70;
        punkt lg, ld, maxlg, maxld, minlg, minld, poig, poid,srlg, srld, lastPoig, lastPoid,lastmaxg, lastmaxd, lastming, lastmind;
        Mat kara, aktualne;
        Mat disparity;
        Mat vDisparity;
        Mat disp3d; // wartosci wyliczone z disparity w funkcji reprojectImageTo3D
        Mat objects; // z disparity na podstawie bestLine odrzucone pod³o¿e i pozosta³y same przeszkody
        Mat objectsbox; // objects tylko przeszkody doœæ du¿e s¹ oznaczone
        Mat road;
        lineab bestLine;
        int rotation;
        void vdisparity();
        void findObjects();
        void addObjectToMap(Rect& object, Mat& objects);
        void drawObjects();
        void penalizemap();
        void visibility();
    public:
        mapa();
        void run(Mat& disp1, Mat& disp3d1);
        void move(pos p){shift.x = shiftx + p.y;shift.y = shifty - p.x; rotation = p.o*radtodeg;};
        Mat getMap() {return mMapa;};
        Mat getSubMap() {return mMapa(Rect((shift.x - rozmiaryPoObroceniu.width/2), (shift.y-rozmiaryPoObroceniu.height/2), rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height));};
        Mat getObjects() {return objects;};
        Mat getObjectsbox() {return objectsbox;};
};

#endif // MAPA_H

