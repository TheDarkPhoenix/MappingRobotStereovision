#include "util.h"

#define radtodeg 57.295779513082
bool aktualne[80][60];

using namespace std;

int sendread(HANDLE hSerial, uint8_t l)
{
    uint8_t* ch = &l;
    DWORD dwBytesRead = 0;
    uint8_t low;
    uint8_t high;
    uint16_t bateria, dalmierz;
    int8_t enkoder1, enkoder2;
    if(!WriteFile(hSerial, ch, 1, &dwBytesRead, NULL))
    {
        cout << "error writing" << endl;
    }
    dwBytesRead = 0;
    if (l == 3)
    {
        if(!ReadFile(hSerial, &low, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        if(!ReadFile(hSerial, &high, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        dalmierz = low | (high << 8);
        cout << "Dalmierz: " << dalmierz << endl;
    }
    else if (l == 4)
    {
        if(!ReadFile(hSerial, &low, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        if(!ReadFile(hSerial, &high, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        bateria = low | (high << 8);
        cout << "Bateria: " << bateria << endl;
    }
    else if (l == 5)
    {
        /*if(!ReadFile(hSerial, &low, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        if(!ReadFile(hSerial, &high, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        enkoder1 = low | (high << 8);
        cout << "Enkoder1: " << enkoder1 << endl;
        return enkoder1;*/
        if(!ReadFile(hSerial, &enkoder1, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        cout << "Enkoder1: " << (int)enkoder1 << endl;
        return enkoder1;
    }
    else if (l == 6)
    {
        /*if(!ReadFile(hSerial, &low, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        if(!ReadFile(hSerial, &high, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        enkoder2 = low | (high << 8);
        cout << "Enkoder2: " << enkoder2 << endl;
        return enkoder2;*/
        if(!ReadFile(hSerial, &enkoder2, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        //enkoder2 = -enkoder2;
        cout << "Enkoder2: " << (int)enkoder2 << endl;
        return enkoder2;
    }
}

void init(HANDLE& hSerial, DCB& dcbSerialParams, COMMTIMEOUTS& timeouts)
{
    hSerial = CreateFile("COM9",
                         GENERIC_READ | GENERIC_WRITE,
                         0,
                         0,
                         OPEN_EXISTING,
                         FILE_ATTRIBUTE_NORMAL,
                         0);
    if(hSerial==INVALID_HANDLE_VALUE)
    {
        if(GetLastError()==ERROR_FILE_NOT_FOUND)
        {
            cout << "serial port doesnt exist" << endl;
        }
        cout << "error" << endl;
    }

    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        cout << "error getting state" << endl;
    }
    dcbSerialParams.BaudRate=CBR_9600;
    dcbSerialParams.ByteSize=8;
    dcbSerialParams.StopBits=ONESTOPBIT;
    dcbSerialParams.Parity=NOPARITY;
    if(!SetCommState(hSerial, &dcbSerialParams))
    {
        cout << "error setting serial port state" << endl;
    }

    timeouts.ReadIntervalTimeout=50;
    timeouts.ReadTotalTimeoutConstant=50;
    timeouts.ReadTotalTimeoutMultiplier=10;
    timeouts.WriteTotalTimeoutConstant=50;
    timeouts.WriteTotalTimeoutMultiplier=10;
    if(!SetCommTimeouts(hSerial, &timeouts))
    {
        cout << "error occureed" << endl;
    }
}

bool initStereovision(StereoSGBM& sgbm, VideoCapture& cap1, VideoCapture& cap2, Mat& map11, Mat& map12, Mat& map21, Mat& map22, Mat& Q, int& numberOfDisparities)
{
    const char* intrinsic_filename = "D:/C++/Code Blocks/Programy/secondMapping/bin/Debug/intrinsics.yml";
    const char* extrinsic_filename = "D:/C++/Code Blocks/Programy/secondMapping/bin/Debug/extrinsics.yml";
    float scale = 0.5f;

    Mat imgl, imgr;

    if(!cap1.isOpened())
    {
        printf("Capture failure\n");
        return false;
    }
    cap1.set(CV_CAP_PROP_FRAME_WIDTH,320);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,240);

    if(!cap2.isOpened())
    {
        printf("Capture failure\n");
        return false;
    }
    cap2.set(CV_CAP_PROP_FRAME_WIDTH,320);
    cap2.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap1 >> imgr;
    cap2 >> imgl;
    Size img_size = imgl.size();
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", intrinsic_filename);
        return false;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= scale;
    M2 *= scale;

    fs.open(extrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsic_filename);
        return false;
    }

    Rect roi1, roi2;
    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);


    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = 1;
    int cn = imgl.channels();
    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;//8
    sgbm.P2 = 256*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;//32
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities =64;//numberOfDisparities;
    sgbm.uniquenessRatio = 0;
    sgbm.speckleWindowSize = 0;
    sgbm.speckleRange = 0;
    sgbm.disp12MaxDiff = 16;
    sgbm.fullDP = 0;

    return true;
}

void CallBackFunc(int event, int x, int y, int flags, void* vdisp)
{
     const double max_z = 1.0e4;
     if  ( event == EVENT_LBUTTONDOWN )
     {
        Mat* dis = (Mat*) vdisp;
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        Vec3f point = dis->at<Vec3f>(y,x);
        if(!(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z))
            cout << point[0]*15.9 << ' ' << point[1]*19.2 << ' ' << point[2]*16 << endl;
     }
}
