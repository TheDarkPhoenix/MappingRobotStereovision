#ifndef ROBOT_H
#define ROBOT_H

#include "types.h"
#include <windows.h>

class robot
{
    private:
        const double R = 5.9, L = 39.6, N = 36;
        static const int maxSpeed = 200, minSpeed = 50, stopSpeed = 125;
        speed predkosc;
        pos pozycja, lPozycja;
        enkoders enkodery;
        int ensuml, ensumr;
        int change;
    public:
        robot();
        void setSpeed(speed pr, HANDLE& hSerial);
        bool accelerate(int a, HANDLE& hSerial);
        bool turn(int dir, HANDLE& hSerial);
        void stop(HANDLE& hSerial);
        void update(HANDLE& hSerial);
        pos getPosition() const{return pozycja;};
        pos getLastPosition() const{return lPozycja;};
};

#endif // ROBOT_H

