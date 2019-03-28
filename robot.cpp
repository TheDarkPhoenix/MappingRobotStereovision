#include "robot.h"
#include "util.h"
#include <cmath>

robot::robot()
{
    pozycja = lPozycja = pos(0,0,0);
    enkodery = enkoders(0,0,0,0,0);
    predkosc = speed(stopSpeed, stopSpeed);
    ensuml = ensumr = 0;
    change = 1;
}

void robot::update(HANDLE& hSerial)
{
    lPozycja = pozycja;
    enkodery.l = sendread(hSerial, 5);
    enkodery.r = sendread(hSerial, 6);
    ensumr += enkodery.r;
    ensuml += enkodery.l;
    enkodery.dl = 2 * PI * R * (enkodery.l/N);
    enkodery.dr = 2 * PI * R * (enkodery.r/N);

    enkodery.dc = (enkodery.dl + enkodery.dr)/2;

    pozycja.x += enkodery.dc*cos(pozycja.o);
    pozycja.y += enkodery.dc*sin(pozycja.o);
    pozycja.o += (enkodery.dr-enkodery.dl)/L;
    pozycja.o = atan2(sin(pozycja.o), cos(pozycja.o));
}

bool robot::accelerate(int a, HANDLE& hSerial)
{
    if (a < 0)
    {
        if (predkosc.vl > minSpeed && predkosc.vr < maxSpeed)
        {
            predkosc.vl -= change;
            predkosc.vr += change;
            setSpeed(predkosc, hSerial);
        }
        return true;
    }
    else if (a > 0)
    {
        if (predkosc.vr > minSpeed && predkosc.vl < maxSpeed)
        {
            predkosc.vr -= change;
            predkosc.vl += change;
            setSpeed(predkosc, hSerial);
        }
        return true;
    }
    else
        return false;
}

bool robot::turn(int dir, HANDLE& hSerial)
{
    if (dir > 0) // w prawo
    {
        if (predkosc.vr < maxSpeed)
            predkosc.vr += change;
        if (predkosc.vl < maxSpeed)
            predkosc.vl += change;
        setSpeed(predkosc, hSerial);
        return true;
    }
    else if (dir < 0) // w lewo
    {
        if (predkosc.vr > minSpeed)
            predkosc.vr -= change;
        if (predkosc.vl > minSpeed)
            predkosc.vl -= change;
        setSpeed(predkosc, hSerial);
        return true;
    }
    else
        return false;
}

void robot::stop(HANDLE& hSerial)
{
    predkosc.vr = stopSpeed;
    predkosc.vl = stopSpeed;
    setSpeed(predkosc, hSerial);
}

void robot::setSpeed(speed pr, HANDLE& hSerial)
{
    sendread(hSerial, 1);
    sendread(hSerial, pr.vl);
    sendread(hSerial, 2);
    sendread(hSerial, pr.vr);
}

