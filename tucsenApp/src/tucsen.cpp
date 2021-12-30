/* This is a driver for the TUcsen Dhyana 900D camera. It should work for all
 * Tucsen USB3 cameras that use the TUCAM api.
 *
 * Author: David Vine
 * Date  : 28 October 2017
 *
 * Based on Mark Rivers PointGrey driver.
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>
#include <epicsExport.h>

#include <TUCamApi.h>

#include <ADDriver.h>

#define DRIVER_VERSION      1
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

#define TucsenBusString           "T_BUS"
#define TucsenProductIDString     "T_PRODUCT_ID"
#define TucsenDriverVersionString "T_DRIVER_VERSION"
#define TucsenTransferRateString  "T_TRANSFER_RATE"

#define TucsenBinningModeString   "T_BIN_MODE"
#define TucsenImageModeString     "T_IMG_MODE"
#define TucsenFanGearString       "T_FAN_GEAR"
#define TucsenAutoLString         "T_AUTO_LEVELS"
#define TucsenDPCString           "T_DEFECT_CORR"
#define TucsenHoriString          "T_HORIZONTAL"
#define TucsenTestPatternString   "T_TESTPATTERN"
#define TucsenDSNUString          "T_DSNU"
#define TucsenPRNUString          "T_PRNU"
#define TucsenTimeStampString     "T_TIMESTAMP"
#define TucsenDeviceLEDString     "T_DEVICELED"
#define TucsenConConfigString     "T_CXP_CONFIG"
#define TucsenUserSelectString    "T_USERSELECT"
#define TucsenSplitEnString       "T_FRAME_SPTEN"
#define TucsenTrigModeString      "T_ACQTRIG_MODE"
#define TucsenTrigEdgeString      "T_TRIG_EDGE"
#define TucsenTrigExpString       "T_TRIG_EXP"
#define TucsenTrgOutPortString    "T_TRIOUT_PORT"
#define TucsenTrgOutKindString    "T_TRIOUT_KIND"
#define TucsenTrgOutEdgeString    "T_TRIOUT_EDGE"
#define TucsenSensorClTypeString  "T_COOL_TYPE"
#define TucsenSensorCoolingString "T_SENSOR_COOLING"
#define TucsenAntiDewString       "T_ANTIDEW"

#define TucsenTrgOutDelayString   "T_TRIOUT_DELAY"
#define TucsenTrgOutWidthString   "T_TRIOUT_WIDTH"
#define TucsenTrigDelayString     "T_TRIG_DELAY"
#define TucsenSplitNumString      "T_FRAME_SPTNUM"
#define TucsenFrameRateString     "T_FRAME_RATE"
#define TucsenBlackLString        "T_BLACK_LEVEL"
#define TucsenGammmaString        "T_GAMMA"
#define TucsenConstString         "T_CONTRAST"
#define TucsenLeftLString         "T_LEFT_LEVELS"
#define TucsenRightLString        "T_RIGHT_LEVELS"
#define TucsenAmbientTString      "T_AMBIENT_TEMPERATURE"
#define TucsenHumidityString      "T_HUMIDITY"
#define TucsenDeviceTempString    "T_DEVICE_TEMPERATURE"
#define TucsenDeviceWTempString   "T_DEVICEWARN_TEMPERATURE"
#define TucsenDeviceWTimeString   "T_DEVICE_WORKINGTIME"

#define TucsenAcqStartString      "T_ACQUIRE_START"
#define TucsenAcqStopString       "T_ACQUIRE_STOP"
#define TucsenSoftSignalString    "T_SOFT_SIGNAL"
#define TucsenParaSaveString      "T_PARA_SAVE"
#define TucsenFactoryDefString    "T_FACTORY_DEFAULT"
#define TucsenDeviceResetString   "T_DEVICE_RESET"

static const char* driverName = "tucsen";

static int TUCAMInitialized = 0;

/* Main driver class inherited from areaDetector ADDriver class */

class tucsen : public ADDriver
{
    public:
        tucsen( const char* portName, int cameraId, int traceMask, int maxBuffers,
                size_t maxMemory, int priority, int stackSize);

        /* Virtual methods to override from ADDrive */
        virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
        virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);

        /* These should be private but must be called from C */
        void imageGrabTask();
        void shutdown();
        void tempTask();

    protected:
        int TucsenBus;
#define FIRST_TUCSEN_PARAM TucsenBus
        int TucsenProductID;
        int TucsenDriverVersion;
        int TucsenTransferRate;

        int TucsenBinMode;
        int TucsenImageMode;
        int TucsenFanGear;
        int TucsenAutoL;
        int TucsenDPC;
        int TucsenHori;
        int TucsenTestPattern;
        int TucsenDSNU;
        int TucsenPRNU;
        int TucsenTimeStamp;
        int TucsenDeviceLED;
        int TucsenConConfig;
        int TucsenUserSelect;
        int TucsenSplitEn;
        int TucsenTrigMode;
        int TucsenTrigEdge;
        int TucsenTrigExp;
        int TucsenTrgOutPort;
        int TucsenTrgOutKind;
        int TucsenTrgOutEdge;
        int TucsenSensorCoolType;
        int TucsenSensorCooling;
        int TucsenAntiDew;

        int TucsenTrgOutDelay;
        int TucsenTrgOutWidth;
        int TucsenTrigDelay;
        int TucsenSplitNum;
        int TucsenFrameRate;
        int TucsenBlackL;
        int TucsenGamma;
        int TucsenContrast;
        int TucsenLeftL;
        int TucsenRightL;
        int TucsenAmbientTemperature;
        int TucsenHumidity;
        int TucsenDeviceTemperature;
        int TucsenDeviceWarningTemperature;
        int TucsenDeviceWorkingTime;

        int TucsenAcqStart;
        int TucsenAcqStop;
        int TucsenSoftSignal;
        int TucsenParaSave;
        int TucsenFactoryDefault;
        int TucsenDeviceReset;
#define LAST_TUCSEN_PARAM TucsenDeviceReset

    private:
        /* Local methods to this class */
        asynStatus grabImage();
        asynStatus startCapture();
        asynStatus stopCapture();

        asynStatus connectCamera();
        asynStatus disconnectCamera();
        asynStatus iniCameraPara();
        asynStatus iniImgAdjustPara();

        /* camera property control functions */
        asynStatus setCamInfo(int param, int nID, int dtype);
        asynStatus setSerialNumber();
        asynStatus setWarningTemp();

        /* Data */
        int cameraId_;
        int exiting_;
        TUCAM_INIT apiHandle_;
        TUCAM_OPEN camHandle_;
        TUCAM_FRAME frameHandle_;
        TUCAM_TRIGGER_ATTR triggerHandle_;
        epicsEventId startEventId_;
        NDArray *pRaw_;
};

#define NUM_TUCSEN_PARAMS ((int)(&LAST_TUCSEN_PARAM-&FIRST_TUCSEN_PARAM+1))


/* Configuration function to configure one camera
 *
 * This function needs to be called once for each camera used by the IOC. A
 * call to this function instantiates one object of the Tucsen class.
 * \param[in] portName asyn port to assign to the camera
 * \param[in] cameraId The camera index or serial number
 * \param[in] traceMask the initial value of asynTraceMask
 *            if set to 0 or 1 then asynTraceMask will be set to
 *            ASYN_TRACE_ERROR.
 *            if set to 0x21 ( ASYN_TRACE_WARNING | ASYN_TRACE_ERROR) then each
 *            call will be traced during initialization
 * \param[in] maxBuffers Maximum number of NDArray objects (image buffers) this
 *            driver is allowed to allocate.
 *            0 = unlimited
 * \param[in] maxMemory Maximum memort (in bytes) that this driver is allowed
 *            to allocate.
 *            0=unlimited
 * \param[in] priority The epics thread priority for this driver. 0= asyn
 *            default.
 * \param[in] stackSize The size of the stack of the EPICS port thread. 0=use
 *            asyn default.
 */
extern "C" int tucsenConfig(const char *portName, int cameraId, int traceMask,
        int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new tucsen( portName, cameraId, traceMask, maxBuffers, maxMemory, priority, stackSize);
    return asynSuccess;
}

static void c_shutdown(void *arg)
{
    tucsen *t = (tucsen *)arg;
    t->shutdown();
}

static void imageGrabTaskC(void *drvPvt)
{
    tucsen *t = (tucsen *)drvPvt;
    t->imageGrabTask();
}

static void tempReadTaskC(void *drvPvt)
{
    tucsen *t = (tucsen *)drvPvt;
    t->tempTask();
}

/* Constructor for the Tucsen class */

tucsen::tucsen(const char *portName, int cameraId, int traceMask, int maxBuffers,
        size_t maxMemory, int priority, int stackSize)
    : ADDriver( portName, 1, NUM_TUCSEN_PARAMS, maxBuffers, maxMemory, asynEnumMask,
            asynEnumMask, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, stackSize),
    cameraId_(cameraId), exiting_(0), pRaw_(NULL)
{
    static const char *functionName = "tucsen";
    char versionString[20];
    asynStatus status;

    if(traceMask==0) traceMask = ASYN_TRACE_ERROR;
    pasynTrace->setTraceMask(pasynUserSelf, traceMask);

    createParam(TucsenBusString,           asynParamOctet,   &TucsenBus);
    createParam(TucsenProductIDString,     asynParamFloat64, &TucsenProductID);
    createParam(TucsenDriverVersionString, asynParamOctet,   &TucsenDriverVersion);
    createParam(TucsenTransferRateString,  asynParamFloat64, &TucsenTransferRate);

    createParam(TucsenBinningModeString,   asynParamInt32,   &TucsenBinMode);
    createParam(TucsenImageModeString,     asynParamInt32,   &TucsenImageMode);
    createParam(TucsenFanGearString,       asynParamInt32,   &TucsenFanGear);
    createParam(TucsenAutoLString,         asynParamInt32,   &TucsenAutoL);
    createParam(TucsenDPCString,           asynParamInt32,   &TucsenDPC);
    createParam(TucsenHoriString,          asynParamInt32,   &TucsenHori);
    createParam(TucsenTestPatternString,   asynParamInt32,   &TucsenTestPattern);
    createParam(TucsenDSNUString,          asynParamInt32,   &TucsenDSNU);
    createParam(TucsenPRNUString,          asynParamInt32,   &TucsenPRNU);
    createParam(TucsenTimeStampString,     asynParamInt32,   &TucsenTimeStamp);
    createParam(TucsenDeviceLEDString,     asynParamInt32,   &TucsenDeviceLED);
    createParam(TucsenConConfigString,     asynParamInt32,   &TucsenConConfig);
    createParam(TucsenUserSelectString,    asynParamInt32,   &TucsenUserSelect);
    createParam(TucsenSplitEnString,       asynParamInt32,   &TucsenSplitEn);
    createParam(TucsenTrigModeString,      asynParamInt32,   &TucsenTrigMode);
    createParam(TucsenTrigEdgeString,      asynParamInt32,   &TucsenTrigEdge);
    createParam(TucsenTrigExpString,       asynParamInt32,   &TucsenTrigExp);
    createParam(TucsenTrgOutPortString,    asynParamInt32,   &TucsenTrgOutPort);
    createParam(TucsenTrgOutKindString,    asynParamInt32,   &TucsenTrgOutKind);
    createParam(TucsenTrgOutEdgeString,    asynParamInt32,   &TucsenTrgOutEdge);
    createParam(TucsenSensorClTypeString,  asynParamInt32,   &TucsenSensorCoolType);
    createParam(TucsenSensorCoolingString, asynParamInt32,   &TucsenSensorCooling);
    createParam(TucsenAntiDewString,       asynParamInt32,   &TucsenAntiDew);

    createParam(TucsenTrgOutDelayString,   asynParamInt32,   &TucsenTrgOutDelay);
    createParam(TucsenTrgOutWidthString,   asynParamInt32,   &TucsenTrgOutWidth);
    createParam(TucsenTrigDelayString,     asynParamInt32,   &TucsenTrigDelay);
    createParam(TucsenSplitNumString,      asynParamInt32,   &TucsenSplitNum);
    createParam(TucsenFrameRateString,     asynParamFloat64, &TucsenFrameRate);
    createParam(TucsenBlackLString,        asynParamInt32,   &TucsenBlackL);
    createParam(TucsenGammmaString,        asynParamInt32,   &TucsenGamma);
    createParam(TucsenConstString,         asynParamInt32,   &TucsenContrast);
    createParam(TucsenLeftLString,         asynParamInt32,   &TucsenLeftL);
    createParam(TucsenRightLString,        asynParamInt32,   &TucsenRightL);
    createParam(TucsenAmbientTString,      asynParamFloat64, &TucsenAmbientTemperature);
    createParam(TucsenHumidityString,      asynParamFloat64, &TucsenHumidity);
    createParam(TucsenDeviceTempString,    asynParamFloat64, &TucsenDeviceTemperature);
    createParam(TucsenDeviceWTempString,   asynParamFloat64, &TucsenDeviceWarningTemperature);
    createParam(TucsenDeviceWTimeString,   asynParamInt32,   &TucsenDeviceWorkingTime);

    createParam(TucsenAcqStartString,      asynParamInt32,   &TucsenAcqStart);
    createParam(TucsenAcqStopString,       asynParamInt32,   &TucsenAcqStop);
    createParam(TucsenSoftSignalString,    asynParamInt32,   &TucsenSoftSignal);
    createParam(TucsenParaSaveString,      asynParamInt32,   &TucsenParaSave);
    createParam(TucsenFactoryDefString,    asynParamInt32,   &TucsenFactoryDefault);
    createParam(TucsenDeviceResetString,   asynParamInt32,   &TucsenDeviceReset);

    /* Set initial values for some parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
    setStringParam(ADManufacturer, "Tucsen");
    epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d",
            DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
    setStringParam(NDDriverVersion, versionString);
    setStringParam(TucsenDriverVersion, versionString);

    setIntegerParam(TucsenAcqStart, 0);
    setIntegerParam(TucsenAcqStop, 0);
    setIntegerParam(TucsenSoftSignal, 0);
    setIntegerParam(TucsenParaSave, 0);
    setIntegerParam(TucsenFactoryDefault, 0);
    setIntegerParam(TucsenDeviceReset, 0);

    status = connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: camera connection failed (%d)\n",
                driverName, functionName, status);
        setIntegerParam(ADStatus, ADStatusDisconnected);
        setStringParam(ADStatusMessage, "camera connection failed");
        report(stdout, 1);
        return;
    }

    startEventId_ = epicsEventCreate(epicsEventEmpty);

    /* Launch image read task */
    epicsThreadCreate("TucsenImageReadTask",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            imageGrabTaskC, this);

    /* Launch temp task */
    epicsThreadCreate("TucsenTempReadTask",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            tempReadTaskC, this);


    /* Launch shutdown task */
    epicsAtExit(c_shutdown, this);

    return;
}

void tucsen::shutdown(void)
{
    exiting_=1;
    if (camHandle_.hIdxTUCam != NULL){
        disconnectCamera();
    }
    TUCAMInitialized--;
    if(TUCAMInitialized==0){
        TUCAM_Api_Uninit();
    }
}

asynStatus tucsen::connectCamera()
{
    static const char* functionName = "connectCamera";
    int tucStatus;
    asynStatus status;

    // Init API
    char szPath[1024] = {0};
    getcwd(szPath, 1024);
    apiHandle_.pstrConfigPath = szPath;
    apiHandle_.uiCamCount = 0;

    tucStatus = TUCAM_Api_Init(&apiHandle_);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: TUCAM API init failed (0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    if (apiHandle_.uiCamCount<1){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: no camera detected (0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    TUCAMInitialized++;

    // Init camera
    camHandle_.hIdxTUCam = NULL;
    camHandle_.uiIdxOpen = cameraId_;
    frameHandle_.ucFormatGet = TUFRM_FMT_RAW;
    frameHandle_.uiRsdSize = 1;

    tucStatus = TUCAM_Dev_Open(&camHandle_);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: open camera device failed (0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    ///printf("TUCAM_Dev_Open = %d\n", apiHandle_.uiCamCount);
    status = setCamInfo(TucsenBus, TUIDI_BUS, 0);
    status = setCamInfo(TucsenProductID, TUIDI_PRODUCT, 1);
    status = setCamInfo(ADSDKVersion, TUIDI_VERSION_API, 0);
    status = setCamInfo(ADFirmwareVersion, TUIDI_VERSION_FRMW, 0);
    status = setCamInfo(ADModel, TUIDI_CAMERA_MODEL, 0);
    status = setSerialNumber();
    status = setWarningTemp();
    status = iniCameraPara();
    status = iniImgAdjustPara();

    return status;
}

asynStatus tucsen::iniCameraPara()
{
    static const char* functionName = "iniCameraPara";
    int tucStatus;
    int nVal = 0;
    int nSizeX = 0;
    int nSizeY = 0;
    asynStatus status = asynSuccess;
    TUCAM_ELEMENT node;

    node.pName = "AcquisitionTrigMode";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    nVal = (node.nVal > 0)? 1 : 0;
    setIntegerParam(ADTriggerMode, nVal);

    node.pName = "AcquisitionExpTime";
    TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setDoubleParam(ADAcquireTime, node.nVal/1000000.0);

    node.pName = "SetSensorTemperature";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setDoubleParam(ADTemperature, node.dbVal);

    node.pName = "OffsetX";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(ADMinX, node.nVal);

    node.pName = "OffsetY";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(ADMinY, node.nVal);

    node.pName = "Width";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(ADSizeX, node.nVal);

    node.pName = "Height";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(ADSizeY, node.nVal);

    node.pName = "SensorWidth";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    nSizeX = node.nVal;

    node.pName = "SensorHeight";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    nSizeY = node.nVal;

    setIntegerParam(ADMaxSizeX, nSizeX);
    setIntegerParam(ADMaxSizeY, nSizeY);
    setIntegerParam(NDArraySizeX, nSizeX);
    setIntegerParam(NDArraySizeY, nSizeY);
    setIntegerParam(NDArraySize, 2*nSizeX*nSizeY);

    node.pName = "Binning";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenBinMode, node.nVal);

    node.pName = "GainMode";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenImageMode, node.nVal);

    node.pName = "FanSpeed";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenFanGear, node.nVal);

    node.pName = "DefectPixelCorrection";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenDPC, node.nVal);

    node.pName = "HorizontalFlip";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenHori, node.nVal);

    node.pName = "TestPattern";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTestPattern, node.nVal);

    node.pName = "DSNU";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenDSNU, node.nVal);

    node.pName = "PRNU";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenPRNU, node.nVal);

    node.pName = "TimeStamp";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTimeStamp, node.nVal);

    node.pName = "DeviceLED";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenDeviceLED, node.nVal);

    node.pName = "ConnectionConfig";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenConConfig, node.nVal);

    node.pName = "UserSelect";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenUserSelect, node.nVal);

    node.pName = "AcquisitionFrameSplitEn";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenSplitEn, node.nVal);

    node.pName = "AcquisitionTrigMode";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrigMode, node.nVal);

    node.pName = "TrigEdge";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrigEdge, node.nVal);

    node.pName = "TrigExpType";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrigExp, node.nVal);

    node.pName = "TrigOutputPort";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrgOutPort, node.nVal);

    node.pName = "TrigOutputKind";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrgOutKind, node.nVal);

    node.pName = "TrigOutputEdge";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrgOutEdge, node.nVal);

    node.pName = "SensorCoolType";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenSensorCoolType, node.nVal);

    node.pName = "SensorCooling";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenSensorCooling, node.nVal);

    node.pName = "AntiDew";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenAntiDew, node.nVal);

    node.pName = "TrigOutputDelay";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrgOutDelay, node.nVal);

    node.pName = "TrigOutputWidth";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrgOutWidth, node.nVal);

    node.pName = "TrigDelay";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenTrigDelay, node.nVal);

    node.pName = "AcquisitionFrameSplitNum";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenSplitNum, node.nVal);

    node.pName = "AcquisitionFrameRate";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setDoubleParam(TucsenFrameRate, node.dbVal);

    node.pName = "BlackLevel";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setIntegerParam(TucsenBlackL, node.nVal);

    node.pName = "AmbientTemperature";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setDoubleParam(TucsenAmbientTemperature, node.dbVal);

    node.pName = "Humidity";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    setDoubleParam(TucsenHumidity, node.dbVal);

    return status;
}

asynStatus tucsen::iniImgAdjustPara()
{
    static const char* functionName = "iniImgAdjustPara";
    int nVal = 0;
    double dbVal = 0;
    asynStatus status = asynSuccess;
    TUCAM_CAPA_ATTR attrCapa;
    TUCAM_PROP_ATTR attrProp;

    attrCapa.idCapa = TUIDC_ATLEVELS;
    if (TUCAMRET_SUCCESS == TUCAM_Capa_GetAttr(camHandle_.hIdxTUCam, &attrCapa))
    {
        TUCAM_Capa_GetValue(camHandle_.hIdxTUCam, attrCapa.idCapa, &nVal);
        setIntegerParam(TucsenAutoL, 0 == nVal ? 0 : 1);
    }

    attrProp.idProp = TUIDP_LFTLEVELS;
    attrProp.nIdxChn= 0;
    if (TUCAMRET_SUCCESS == TUCAM_Prop_GetAttr(camHandle_.hIdxTUCam, &attrProp))
    {
        TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, attrProp.idProp, &dbVal);
        nVal = (int)dbVal;
        setIntegerParam(TucsenLeftL, nVal);
    }

    attrProp.idProp = TUIDP_RGTLEVELS;
    if (TUCAMRET_SUCCESS == TUCAM_Prop_GetAttr(camHandle_.hIdxTUCam, &attrProp))
    {
        TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, attrProp.idProp, &dbVal);
        nVal = (int)dbVal;
        setIntegerParam(TucsenRightL, nVal);
    }

    attrProp.idProp = TUIDP_GAMMA;
    if (TUCAMRET_SUCCESS == TUCAM_Prop_GetAttr(camHandle_.hIdxTUCam, &attrProp))
    {
        TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, attrProp.idProp, &dbVal);
        nVal = (int)dbVal;
        setIntegerParam(TucsenGamma, nVal);
    }

    attrProp.idProp = TUIDP_CONTRAST;
    if (TUCAMRET_SUCCESS == TUCAM_Prop_GetAttr(camHandle_.hIdxTUCam, &attrProp))
    {
        TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, attrProp.idProp, &dbVal);
        nVal = (int)dbVal;
        setIntegerParam(TucsenContrast, nVal);
    }

    return status;
}

asynStatus tucsen::disconnectCamera(void){
    static const char* functionName = "disconnectCamera";
    int tucStatus;
    int acquiring;
    asynStatus status;

    // check if acquiring
    status = getIntegerParam(ADAcquire, &acquiring);

    // if necessary stop acquiring
    if (status==asynSuccess && acquiring){
        status = stopCapture();
    }

    tucStatus = TUCAM_Dev_Close(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        status = asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable close camera (0x%x)\n",
                driverName, functionName, tucStatus);
    }
    return status;
}

void tucsen::imageGrabTask(void)
{
    static const char* functionName = "imageGrabTask";
    asynStatus status  = asynSuccess;
    int tucStatus;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int triggerMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire;

    lock();
    while(1){
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring wait for a semaphore that is given when
         * acquisition is started */
        if(!acquire){
            if (!status) {
                setIntegerParam(ADStatus, ADStatusIdle);
                setStringParam(ADStatusMessage, "Waiting for the acquire command");
                callParamCallbacks();
            }
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: waiting for acquisition to start\n",
                      driverName, functionName);
            unlock();
            tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
            epicsEventWait(startEventId_);
            lock();
            getIntegerParam(TucsenTrigMode, &triggerMode);
            tucStatus = TUCAM_Buf_Alloc(camHandle_.hIdxTUCam, &frameHandle_);
            tucStatus = TUCAM_Cap_Start(camHandle_.hIdxTUCam, triggerMode);
            if (tucStatus!=TUCAMRET_SUCCESS){
                status = asynError;
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: failed to start image capture (0x%x)\n",
                          driverName, functionName, tucStatus);
                setIntegerParam(ADAcquire, 0);
                setIntegerParam(ADStatus, ADStatusError);
                setStringParam(ADStatusMessage, "Failed to start image capture");
                callParamCallbacks();
                continue;
            }

            setIntegerParam(ADStatus, ADStatusAcquire);
            setStringParam(ADStatusMessage, "Acquiring...");
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: acquisition started\n",
                      driverName, functionName);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
            callParamCallbacks();
        }

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);

        status = grabImage();
        if (status==asynError){
            // Release the allocated NDArray
            if (pRaw_) pRaw_->release();
            pRaw_ = NULL;

            getIntegerParam(ADAcquire, &acquire);
            if (acquire == 0) {
                if (imageMode != ADImageContinuous) {
                    setIntegerParam(ADStatus, ADStatusAborted);
                    setStringParam(ADStatusMessage, "Aborted by user");
                } else {
                    status = asynSuccess;
                }
            } else {
                stopCapture();
                setIntegerParam(ADAcquire, 0);
                setIntegerParam(ADStatus, ADStatusError);
                setStringParam(ADStatusMessage, "Failed to get image");
            }
            callParamCallbacks();
            continue;
        }

        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        if(arrayCallbacks){
            doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
        }

        if (pRaw_) pRaw_->release();
        pRaw_ = NULL;

        if ((imageMode==ADImageSingle) || ((imageMode==ADImageMultiple) && (numImagesCounter>=numImages))){
            status = stopCapture();
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, ADStatusIdle);
        }
        callParamCallbacks();
    }
}

asynStatus tucsen::grabImage()
{
    static const char* functionName = "grabImage";
    asynStatus status = asynSuccess;
    int tucStatus;
    int nCols, nRows;
    int pixelFormat, channels, bitDepth;
    size_t dataSize, tDataSize;
    NDDataType_t dataType = NDUInt16;
    NDColorMode_t colorMode = NDColorModeMono;
    int numColors = 1;
    int pixelSize = 2;
    size_t dims[3] = {0};
    int nDims;
    int count;

    unlock();
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: wait for buffer\n",
            driverName, functionName);
    tucStatus = TUCAM_Buf_WaitForFrame(camHandle_.hIdxTUCam, &frameHandle_);
    lock();
    if (tucStatus!= TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failed to wait for buffer (0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    // Width & height are array dimensions
    nCols = frameHandle_.usWidth;
    nRows = frameHandle_.usHeight;

    // Image format
    pixelFormat = frameHandle_.ucFormat;
    // Number of channels
    channels = frameHandle_.ucChannels;
    // Pixel bit depth
    bitDepth = frameHandle_.ucDepth;
    // Frame image data size
    tDataSize = frameHandle_.uiImgSize;

    /* There is zero documentation on what the formats mean
     * Most of the below is gleaned through trial and error */
    if (pixelFormat==TUFRM_FMT_RAW){
        // Raw data - no filtering applied
        if (bitDepth == 1){
            dataType = NDUInt8;
            pixelSize = 1;
        } else if (bitDepth==2) {
            dataType = NDUInt16;
            pixelSize = 2;
        }
        colorMode = NDColorModeMono;
        numColors = 1;
    } else if (pixelFormat==TUFRM_FMT_USUAl){
        if (bitDepth == 1){
            dataType = NDUInt8;
            pixelSize = 1;
        } else if (bitDepth == 2) {
            dataType = NDUInt16;
            pixelSize = 2;
        }

        if (channels==1){
            colorMode = NDColorModeMono;
            numColors = 1;
        } else if (channels==3){
            colorMode = NDColorModeRGB1;
            numColors = 3;
        }
    } else if (pixelFormat==TUFRM_FMT_RGB888){
        dataType = NDUInt8;
        pixelSize = 1;
        colorMode = NDColorModeRGB1;
        numColors = 3;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Unsupported pixel format %d\n",
                driverName, functionName, pixelFormat);
        return asynError;
    }
    if (numColors==1){
        nDims = 2;
        dims[0] = nCols;
        dims[1] = nRows;
    } else {
        nDims = 3;
        dims[0] = 3;
        dims[1] = nCols;
        dims[2] = nRows;
    }

    dataSize = dims[0]*dims[1]*pixelSize;
    if (nDims==3) dataSize *= dims[2];

    if (dataSize != tDataSize){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: data size mismatch: calculated=%zu, reported=%zu\n",
                driverName, functionName, dataSize, tDataSize);
        return asynError;
    }

    setIntegerParam(NDArraySizeX, nCols);
    setIntegerParam(NDArraySizeY, nRows);
    setIntegerParam(NDArraySize, (int)dataSize);
    setIntegerParam(NDDataType, dataType);
    setIntegerParam(NDColorMode, colorMode);

    pRaw_ = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
    if(!pRaw_){
        // No valid buffer so we need to abort
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s [%s] ERROR: Serious problem: not enough buffers left. Aborting acquisition!\n",
                driverName, functionName, portName);
        return asynError;
    }
    memcpy(pRaw_->pData, frameHandle_.pBuffer+frameHandle_.usOffset, dataSize);
    getIntegerParam(NDArrayCounter, &count);
    pRaw_->uniqueId = count;
    updateTimeStamp(&pRaw_->epicsTS);
    pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch+pRaw_->epicsTS.nsec/1e9;

    getAttributes(pRaw_->pAttributeList);

    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);

    return status;
}

void tucsen::tempTask(void){
    static const char* functionName = "tempTask";
    TUCAM_VALUE_INFO valInfo;
    TUCAM_ELEMENT node;
    int tucStatus;
    int nVal;
    double dbVal;

    lock();
    while (!exiting_){

        node.pName = "SensorTemperature";
        tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
        dbVal = node.dbVal;
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: failed to read temperature (0x%x)\n",
                      driverName, functionName, tucStatus);
        } else {
            setDoubleParam(ADTemperatureActual, dbVal);
        }

        node.pName = "DeviceTemperature";
        tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
        dbVal = node.dbVal;
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: failed to read device temperature (0x%x)\n",
                      driverName, functionName, tucStatus);
        } else {
            setDoubleParam(TucsenDeviceTemperature, dbVal);
        }

        node.pName = "DeviceWorkingTime";
        tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
        nVal = node.nVal;
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: failed to read working time (0x%x)\n",
                      driverName, functionName, tucStatus);
        } else {
            setIntegerParam(TucsenDeviceWorkingTime, nVal);
        }

        valInfo.nID = TUIDI_TRANSFER_RATE;
        tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: failed to read transfer rate (0x%x)\n",
                      driverName, functionName, tucStatus);
        } else {
            setDoubleParam(TucsenTransferRate, valInfo.nValue);
        }
        callParamCallbacks();
        unlock();
        epicsThreadSleep(1);
        lock();
    }
}

/* Sets an int32 parameter */
asynStatus tucsen::writeInt32( asynUser *pasynUser, epicsInt32 value)
{
    static const char* functionName = "writeInt32";
    const char* paramName;
    asynStatus status = asynSuccess;
    bool bSetPara = true;
    int tucStatus;
    int function = pasynUser->reason;
    double dbVal = 0;
    TUCAM_ELEMENT node;

    getParamName(function, &paramName);
    status = setIntegerParam(function, value);
    printf("Write Int32 pName = %s, function = %d, value = %d\n", paramName, function, value);
    ///return status;
    if (function==ADAcquire){
        if (value){
            status = startCapture();
        } else {
            status = stopCapture();
        }
        bSetPara = false;
    } else if ((function==ADNumImages)   ||
               (function==ADNumExposures)){
        bSetPara = false;
    } else if (function==TucsenAutoL){
        bSetPara = false;
        TUCAM_Capa_SetValue(camHandle_.hIdxTUCam, TUIDC_ATLEVELS, 0 == value ? 0 : 3);
        TUCAM_Capa_SetValue(camHandle_.hIdxTUCam, TUIDC_HISTC, value);
    } else if (function==TucsenGamma){
        bSetPara = false;
        TUCAM_Prop_SetValue(camHandle_.hIdxTUCam, TUIDP_GAMMA, value);
        TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, TUIDP_GAMMA, &dbVal);
        value = (int)dbVal;
    } else if (function==TucsenContrast){
        bSetPara = false;
        TUCAM_Prop_SetValue(camHandle_.hIdxTUCam, TUIDP_CONTRAST, value);
        TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, TUIDP_CONTRAST, &dbVal);
        value = (int)dbVal;
    } else if (function==TucsenLeftL){
        bSetPara = false;
        TUCAM_Prop_SetValue(camHandle_.hIdxTUCam, TUIDP_LFTLEVELS, value);
        TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, TUIDP_LFTLEVELS, &dbVal);
        value = (int)dbVal;
    }else if (function==TucsenRightL){
        bSetPara = false;
        TUCAM_Prop_SetValue(camHandle_.hIdxTUCam, TUIDP_RGTLEVELS, value);
        TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, TUIDP_RGTLEVELS, &dbVal);
        value = (int)dbVal;
    } else if (function==ADTriggerMode){
        node.pName = "AcquisitionTrigMode";
    }else if (function==ADMinX){
        node.pName = "OffsetX";
    }else if (function==ADMinY){
        node.pName = "OffsetY";
    }else if (function==ADSizeX){
        node.pName = "Width";
    }else if (function==ADSizeY){
        node.pName = "Height";
    } else if (function==TucsenBinMode){
        node.pName = "Binning";
    } else if (function==TucsenImageMode){
        node.pName = "GainMode";
    } else if (function==TucsenFanGear){
        node.pName = "FanSpeed";
    }else if (function==TucsenDPC){
        node.pName = "DefectPixelCorrection";
    }else if (function==TucsenHori){
        node.pName = "HorizontalFlip";
    }else if (function==TucsenTestPattern){
        node.pName = "TestPattern";
    }else if (function==TucsenDSNU){
        node.pName = "DSNU";
    }else if (function==TucsenPRNU){
        node.pName = "PRNU";
    }else if (function==TucsenTimeStamp){
        node.pName = "TimeStamp";
    }else if (function==TucsenDeviceLED){
        node.pName = "DeviceLED";
    }else if (function==TucsenConConfig){
        node.pName = "ConnectionConfig";
    }else if (function==TucsenUserSelect){
        node.pName = "UserSelect";
    }else if (function==TucsenSplitEn){
        node.pName = "AcquisitionFrameSplitEn";
    }else if (function==TucsenTrigMode){
        node.pName = "AcquisitionTrigMode";
    }else if (function==TucsenTrigEdge){
        node.pName = "TrigEdge";
    }else if (function==TucsenTrigExp){
        node.pName = "TrigExpType";
    }else if (function==TucsenTrgOutPort){
        node.pName = "TrigOutputPort";
    }else if (function==TucsenTrgOutKind){
        node.pName = "TrigOutputKind";
    }else if (function==TucsenTrgOutEdge){
        node.pName = "TrigOutputEdge";
    }else if (function==TucsenSensorCoolType){
        node.pName = "SensorCoolType";
    }else if (function==TucsenSensorCooling){
        node.pName = "SensorCooling";
    }else if (function==TucsenAntiDew){
        node.pName = "AntiDew";
    }else if (function==TucsenTrgOutDelay){
        node.pName = "TrigOutputDelay";
    }else if (function==TucsenTrgOutWidth){
        node.pName = "TrigOutputWidth";
    }else if (function==TucsenTrigDelay){
        node.pName = "TrigDelay";
    }else if (function==TucsenSplitNum){
        node.pName = "AcquisitionFrameSplitNum";
    }else if (function==TucsenBlackL){
        node.pName = "BlackLevel";
    }else if (function==TucsenAcqStart){
        node.pName = "AcquisitionStart";
    }else if (function==TucsenAcqStop){
        node.pName = "AcquisitionStop";
    }else if (function==TucsenSoftSignal){
        node.pName = "TrigSoftwareSignal";
    }else if (function==TucsenParaSave){
        node.pName = "ParameterSave";
    }else if (function==TucsenFactoryDefault){
        node.pName = "FactoryDefault";
    }else if (function==TucsenDeviceReset){
        node.pName = "AntiDew";
    } else {
        if (function < FIRST_TUCSEN_PARAM){
            status = ADDriver::writeInt32(pasynUser, value);
        }
        bSetPara = false;
    }

    if(bSetPara)
    {
        tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
        if ((value >= node.nMin) || (value <= node.nMax)){
            node.nVal = value;
            tucStatus = TUCAM_GenICam_SetElementValue(camHandle_.hIdxTUCam, &node);
            tucStatus = TUCAM_GenICam_GetElementValue(camHandle_.hIdxTUCam, &node);
            value = node.nVal;
        }

        if(TU_ElemInteger == node.Type) { iniCameraPara();}
    }

    ///printf("After Write Int32 pName = %s, function = %d, value = %d\n", paramName, function, value);
    status = setIntegerParam(function, value);

    if (tucStatus != TUCAMRET_SUCCESS){
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
            "%s::%s function=%d, value=%d, status=%d, ret=0x%x\n",
            driverName, functionName, function, value, status, tucStatus);
    }
    callParamCallbacks();
    return status;
}

/* Sets an double parameter */
asynStatus tucsen::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    static const char* functionName = "writeFloat64";
    const char* paramName;
    double dbVal = 0.0;

    bool bSetPara = true;
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int tucStatus =TUCAMRET_SUCCESS;
    TUCAM_ELEMENT node;

    getParamName(function, &paramName);
    status = setDoubleParam(function, value);
    printf("Write Float pName = %s, function = %d, value=%f\n", paramName, function, value);
    ///return status;
    if (function==ADAcquireTime){
        node.pName = "AcquisitionExpTime";
    } else if (function==ADTemperature){
        node.pName = "SetSensorTemperature";
    } else if (function==TucsenFrameRate){
        node.pName = "AcquisitionFrameRate";
    } else if (function==TucsenAmbientTemperature){
        node.pName = "AmbientTemperature";
    } else if (function==TucsenHumidity){
        node.pName = "Humidity";
    } else {
        if (function < FIRST_TUCSEN_PARAM){
            status = ADDriver::writeFloat64(pasynUser, value);
        }
        bSetPara = false;
    }

    if(bSetPara)
    {
        if (function==ADAcquireTime)   ///ExposureTime
        {
            dbVal = value*1000000.0;  /// s->us
            TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
            node.nVal = dbVal;
            tucStatus=TUCAM_GenICam_SetElementValue(camHandle_.hIdxTUCam, &node);
            tucStatus=TUCAM_GenICam_GetElementValue(camHandle_.hIdxTUCam, &node);
            value = node.nVal/ 1000000.0;
        }
        else
        {
            TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
            node.dbVal = value;
            tucStatus=TUCAM_GenICam_SetElementValue(camHandle_.hIdxTUCam, &node);
            tucStatus=TUCAM_GenICam_GetElementValue(camHandle_.hIdxTUCam, &node);
            value = node.dbVal;
        }

        if(TU_ElemFloat == node.Type) { iniCameraPara();}
    }
    status = setDoubleParam(function, value);

    if (tucStatus != TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: failed to set double ret=0x%x\n",
                  driverName, functionName, tucStatus);
    }

    callParamCallbacks();
    return (asynStatus)status;
}

asynStatus tucsen::setCamInfo(int param, int nID, int dtype)
{
    static const char* functionName = "setCamInfo";

    int tucStatus;
    TUCAM_VALUE_INFO valInfo;
    char sInfo[1024] = {0};
    valInfo.pText = sInfo;
    valInfo.nTextSize = 1024;
    valInfo.nID = nID;

    tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
    if (tucStatus==TUCAMRET_SUCCESS){
        if (param==TucsenBus){
            if (valInfo.nValue==0x06){
                setStringParam(param, "CXP");
            } else if (valInfo.nValue==0x210){
                setStringParam(param, "USB2.0");
            } else {
                setStringParam(param, "USB3.0");
            }
        } else if (dtype==0){
            setStringParam(param, valInfo.pText);
        } else if (dtype==1){
            setDoubleParam(param, valInfo.nValue);
        }
        callParamCallbacks();
        return asynSuccess;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s param %d id %d (error=0x%x)\n",
                driverName, functionName, param, nID, tucStatus);
        return asynError;
    }
}

asynStatus tucsen::setSerialNumber()
{
    static const char* functionName = "setSerialNumber";
    int tucStatus;
    char cSN[TUSN_SIZE] = {0};

    TUCAM_ELEMENT node;
    node.pName = "DeviceID";
    node.pTransfer = &cSN[0];
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);

    if (tucStatus==TUCAMRET_SUCCESS){
        setStringParam(ADSerialNumber, node.pTransfer);
        return asynSuccess;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to get serial number (error=0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
}

asynStatus tucsen::setWarningTemp()
{
    static const char* functionName = "setWarningTemp";
    int tucStatus;
    double dbVal;

    TUCAM_ELEMENT node;
    node.pName = "DeviceWarningTemperature";
    tucStatus = TUCAM_GenICam_ElementAttr(camHandle_.hIdxTUCam, &node, node.pName);
    dbVal = node.dbVal;

    if (tucStatus==TUCAMRET_SUCCESS){
        setDoubleParam(TucsenDeviceWarningTemperature, dbVal);
        return asynSuccess;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to get warning temperature (error=0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
}

asynStatus tucsen::startCapture()
{
    static const char* functionName = "startCapture";

    setIntegerParam(ADNumImagesCounter, 0);
    setShutter(1);
    epicsEventSignal(startEventId_);
    return asynSuccess;
}

asynStatus tucsen::stopCapture()
{
    static const char* functionName = "stopCapture";
    int tucStatus;
    setShutter(0);

    tucStatus = TUCAM_Buf_AbortWait(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to abort wait (0x%x)\n",
                driverName, functionName, tucStatus);
    }

    tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to stop acquisition (0x%x)\n",
                driverName, functionName, tucStatus);
    }

    tucStatus = TUCAM_Buf_Release(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to release camera buffer (0x%x)\n",
                driverName, functionName, tucStatus);
    }

    return asynSuccess;
}

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"CameraId", iocshArgInt};
static const iocshArg configArg2 = {"traceMask", iocshArgInt};
static const iocshArg configArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg configArg4 = {"maxMemory", iocshArgInt};
static const iocshArg configArg5 = {"priority", iocshArgInt};
static const iocshArg configArg6 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs [] = {&configArg0,
                                               &configArg1,
                                               &configArg2,
                                               &configArg3,
                                               &configArg4,
                                               &configArg5,
                                               &configArg6};
static const iocshFuncDef configtucsen = {"tucsenConfig", 7, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    tucsenConfig(args[0].sval, args[1].ival, args[2].ival,
                 args[3].ival, args[4].ival, args[5].ival,
                 args[6].ival);
}

static void tucsenRegister(void)
{
    iocshRegister(&configtucsen, configCallFunc);
}

extern "C" {
    epicsExportRegistrar(tucsenRegister);
}

