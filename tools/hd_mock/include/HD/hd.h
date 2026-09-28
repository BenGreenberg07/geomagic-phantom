#pragma once
// MOCK of OpenHaptics <HD/hd.h> - NOT the real SDK.
//
// Lets the apps compile and run on any machine (a Mac/Linux laptop, CI) with a
// simulated arm: a point mass with a lazy "hand" drifting in slow circles, and
// the stylus button pressed briefly every HD_MOCK_BUTTON_PERIOD seconds
// (default 3). It implements only the subset of HD this repo uses. Names and
// signatures match the real header; numeric values do not.
//
// Build with:  cmake -S . -B build-mock -DPHANTOM_MOCK=ON && cmake --build build-mock

typedef unsigned int HHD;
typedef unsigned int HDenum;
typedef unsigned int HDerror;
typedef int HDint;
typedef double HDdouble;
typedef float HDfloat;
typedef unsigned char HDboolean;
typedef const char* HDstring;
typedef unsigned short HDushort;
typedef unsigned long HDulong;
typedef unsigned long HDSchedulerHandle;
typedef unsigned int HDCallbackCode;

#define HDCALLBACK
#define HD_CALLBACK_DONE 0
#define HD_CALLBACK_CONTINUE 1
typedef HDCallbackCode(HDCALLBACK* HDSchedulerCallback)(void* pUserData);

#define HD_DEFAULT_DEVICE ((HDstring)0)
#define HD_INVALID_HANDLE 0xFFFFFFFF

#define HD_MAX_SCHEDULER_PRIORITY 0xffff
#define HD_MIN_SCHEDULER_PRIORITY 0
#define HD_DEFAULT_SCHEDULER_PRIORITY 0x7fff

typedef struct {
    HDerror errorCode;
    int internalErrorCode;
    HHD hHD;
} HDErrorInfo;
#define HD_DEVICE_ERROR(X) (((X).errorCode) != HD_SUCCESS)

// errors
#define HD_SUCCESS 0x0000
#define HD_INVALID_ENUM 0x0100
#define HD_DEVICE_ALREADY_INITIATED 0x0200
#define HD_COMM_ERROR 0x0201
#define HD_TIMER_ERROR 0x0203
#define HD_ILLEGAL_BEGIN 0x0204
#define HD_ILLEGAL_END 0x0205
#define HD_WARM_MOTORS 0x0300
#define HD_EXCEEDED_MAX_FORCE 0x0301
#define HD_EXCEEDED_MAX_FORCE_IMPULSE 0x0302
#define HD_EXCEEDED_MAX_VELOCITY 0x0303
#define HD_FORCE_ERROR 0x0304

// get/set parameters
#define HD_CURRENT_BUTTONS 0x2000
#define HD_CURRENT_POSITION 0x2050
#define HD_CURRENT_VELOCITY 0x2051
#define HD_CURRENT_JOINT_ANGLES 0x2100
#define HD_CURRENT_GIMBAL_ANGLES 0x2150
#define HD_CURRENT_FORCE 0x2700
#define HD_DEVICE_MODEL_TYPE 0x2501
#define HD_DEVICE_VENDOR 0x2503
#define HD_DEVICE_SERIAL_NUMBER 0x2504
#define HD_NOMINAL_MAX_STIFFNESS 0x2603
#define HD_NOMINAL_MAX_DAMPING 0x2609
#define HD_NOMINAL_MAX_FORCE 0x2604
#define HD_NOMINAL_MAX_CONTINUOUS_FORCE 0x2605
#define HD_USABLE_WORKSPACE_DIMENSIONS 0x2551
#define HD_INPUT_DOF 0x2601
#define HD_OUTPUT_DOF 0x2602
#define HD_CALIBRATION_STYLE 0x2650

// enable/disable
#define HD_FORCE_OUTPUT 0x4000
#define HD_MAX_FORCE_CLAMPING 0x4001
#define HD_FORCE_RAMPING 0x4002

// buttons
#define HD_DEVICE_BUTTON_1 (1 << 0)
#define HD_DEVICE_BUTTON_2 (1 << 1)

// calibration
#define HD_CALIBRATION_OK 0x5000
#define HD_CALIBRATION_NEEDS_UPDATE 0x5001
#define HD_CALIBRATION_NEEDS_MANUAL_INPUT 0x5002
#define HD_CALIBRATION_ENCODER_RESET (1 << 0)
#define HD_CALIBRATION_AUTO (1 << 1)
#define HD_CALIBRATION_INKWELL (1 << 2)

#ifdef __cplusplus
extern "C" {
#endif

HHD hdInitDevice(HDstring pConfigName);
void hdDisableDevice(HHD hHD);
void hdMakeCurrentDevice(HHD hHD);
HHD hdGetCurrentDevice(void);

void hdBeginFrame(HHD hHD);
void hdEndFrame(HHD hHD);

void hdGetBooleanv(HDenum pname, HDboolean* params);
void hdGetIntegerv(HDenum pname, HDint* params);
void hdGetDoublev(HDenum pname, HDdouble* params);
HDstring hdGetString(HDenum pname);
void hdSetDoublev(HDenum pname, const HDdouble* params);

void hdEnable(HDenum cap);
void hdDisable(HDenum cap);
HDboolean hdIsEnabled(HDenum cap);

HDErrorInfo hdGetError(void);
HDstring hdGetErrorString(HDerror errorCode);

HDenum hdCheckCalibration(void);
void hdUpdateCalibration(HDenum style);

void hdStartScheduler(void);
void hdStopScheduler(void);
HDSchedulerHandle hdScheduleAsynchronous(HDSchedulerCallback cb, void* pUserData, HDushort nPriority);
void hdScheduleSynchronous(HDSchedulerCallback cb, void* pUserData, HDushort nPriority);
void hdUnschedule(HDSchedulerHandle h);

#ifdef __cplusplus
}
#endif
