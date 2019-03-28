#ifndef TYPES_H
#define TYPES_H

#define PI 3.14159265359

struct model
{
    model(double v1 = 0, double w1 = 0){v = v1; w = w1;};
    double v, w;
};

struct speed
{
    speed(int vr1 = 0, int vl1 = 0){vr = vr1; vl = vl1;};
    int vr, vl;
};

struct pidreg
{
    double p, i, d;
};

struct pos
{
    pos(double x1 = 0, double y1 = 0, double o1 = 0){x = x1; y = y1; o = o1;};
    double x,y,o;
};

struct enkoders
{
    enkoders (double dl1 = 0, double dr1 = 0, double dc1 = 0, int r1 = 0, int l1 = 0){dl = dl1; dr = dr1; dc = dc1; r = r1; l = l1;};
    double dl, dr, dc;
    int r, l;
};

struct lineab
{
    double a,b,len;
    lineab(){a = 0; b = 0; len = 0;};
};

struct punkt
{
    double x, y;
    punkt(float x1 = 0, float y1 = 0){x = x1; y = y1;};
};

#endif // TYPES_H


