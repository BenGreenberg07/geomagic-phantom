#define _CRT_SECURE_NO_WARNINGS

#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduVector.h>
#include <HD/hd.h>

#include <GL/glut.h>
#include <fstream>
#include <iomanip>
#include <ctime>

//---------------------------------------------------------------------
//  Function declarations
//---------------------------------------------------------------------
HDCallbackCode HDCALLBACK deviceCallback(void* pUserData);
void initDevice();
void displayFunc();
void idleFunc();
void exitHandler();

//---------------------------------------------------------------------
//  Logging
//---------------------------------------------------------------------
std::ofstream logFile;
double lastLogTime = 0.0;
hduVector3Dd currentPos(0, 0, 0);
bool buttonPressed = false;

//---------------------------------------------------------------------
//  Haptic device handle
//---------------------------------------------------------------------
HHD hHD = 0;

//---------------------------------------------------------------------
//  Device callback - runs continuously inside the servo loop
//---------------------------------------------------------------------
HDCallbackCode HDCALLBACK deviceCallback(void* pUserData)
{
    hdBeginFrame(hHD);

    // Read position (mm)
    hduVector3Dd pos;
    hdGetDoublev(HD_CURRENT_POSITION, pos);
    currentPos = pos;

    // Read button state
    HDint buttons;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
    buttonPressed = (buttons & HD_DEVICE_BUTTON_1) != 0;

    hdEndFrame(hHD);

    return HD_CALLBACK_CONTINUE;
}

//---------------------------------------------------------------------
//  Initializes device + logging
//---------------------------------------------------------------------
void initDevice()
{
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);

    hdEnable(HD_FORCE_OUTPUT);

    // Start scheduler
    hdStartScheduler();

    // Start device callback
    hdScheduleAsynchronous(deviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    // Open CSV file
    logFile.open("position_log.csv");
    logFile << "timestamp,x_mm,y_mm,z_mm,button_held\n";
}

//---------------------------------------------------------------------
//  Draw a small sphere at the current haptic cursor location
//---------------------------------------------------------------------
void displayFunc()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glTranslated(currentPos[0] / 100.0, currentPos[1] / 100.0, currentPos[2] / 100.0);

    glutSolidSphere(0.02, 20, 20);   // visual cursor

    glPopMatrix();

    glutSwapBuffers();
}

//---------------------------------------------------------------------
//  Called repeatedly - logs position every 5 seconds
//---------------------------------------------------------------------
void idleFunc()
{
    double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;  // seconds

    if (t - lastLogTime >= 5.0)
    {
        // Get timestamp
        std::time_t rawtime = std::time(nullptr);
        std::tm tm;
#ifdef _WIN32
        localtime_s(&tm, &rawtime);
#else
        localtime_r(&rawtime, &tm);
#endif
        char buffer[20];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);

        // Write log
        logFile << buffer << ","
            << std::fixed << std::setprecision(3)
            << currentPos[0] << ","
            << currentPos[1] << ","
            << currentPos[2] << ","
            << (buttonPressed ? 1 : 0)
            << "\n";

        lastLogTime = t;
    }

    glutPostRedisplay();
}

//---------------------------------------------------------------------
//  Cleanup on exit
//---------------------------------------------------------------------
void exitHandler()
{
    hdStopScheduler();
    hdDisableDevice(hHD);
    if (logFile.is_open()) logFile.close();
}

//---------------------------------------------------------------------
//  Main
//---------------------------------------------------------------------
int main(int argc, char* argv[])
{
    std::atexit(exitHandler);

    // Init device
    initDevice();

    // Init graphics
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(600, 600);
    glutCreateWindow("Geomagic Position Logger");

    glEnable(GL_DEPTH_TEST);

    glutDisplayFunc(displayFunc);
    glutIdleFunc(idleFunc);

    glutMainLoop();

    return 0;
}
