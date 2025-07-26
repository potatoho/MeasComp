#include <iocsh.h>
#include <epicsExport.h>
#include <asynPortDriver.h>
#include <epicsThread.h>
#include <cstring>
#include "uldaq.h"

static const char *driverName = "MultiFunction";

#define analogOutValueString "ANALOG_OUT_VALUE"
#define analogInValueString  "ANALOG_IN_VALUE"
#define analogInRangeString  "ANALOG_IN_RANGE"
#define digitalDirectionString "DIGITAL_DIRECTION"
#define digitalInputString   "DIGITAL_INPUT"
#define digitalOutputString  "DIGITAL_OUTPUT"
#define counterCountsString  "COUNTER_VALUE"
#define counterResetString   "COUNTER_RESET"

#define DEFAULT_POLL_TIME 0.01
#define NUM_ANALOG_IN 16
#define NUM_ANALOG_OUT 2
#define NUM_IO_BITS 8
#define NUM_COUNTERS 2
#define MAX_SIGNALS NUM_ANALOG_IN

#define DIGITAL_PORT AUXPORT

class MultiFunction : public asynPortDriver {
public:
    MultiFunction(const char *portName, const char *uniqueID);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
    virtual asynStatus getBounds(asynUser *pasynUser, epicsInt32 *low, epicsInt32 *high);
    virtual void report(FILE *fp, int details);
    virtual void pollerThread(void);

protected:
    int analogOutValue_;
#define FIRST_MULTIFUNCTION_PARAM analogOutValue_
    int analogInValue_;
    int analogInRange_;
    int digitalDirection_;
    int digitalOutput_;
    int digitalInput_;
    int counterCounts_;
    int counterReset_;
#define LAST_MULTIFUNCTION_PARAM counterReset_

private:
    DaqDeviceHandle daqDeviceHandle_;
    double pollTime_;
    int forceCallback_;
};

#define NUM_PARAMS (&LAST_MULTIFUNCTION_PARAM - &FIRST_MULTIFUNCTION_PARAM + 1)

static void pollerThreadC(void *pPvt) {
    MultiFunction *pMF = (MultiFunction *)pPvt;
    pMF->pollerThread();
}

MultiFunction::MultiFunction(const char *portName, const char *uniqueID)
    : asynPortDriver(portName, MAX_SIGNALS, NUM_PARAMS,
                     asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask,
                     asynUInt32DigitalMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0),
      daqDeviceHandle_(0), pollTime_(DEFAULT_POLL_TIME), forceCallback_(1)
{
    createParam(analogOutValueString, asynParamInt32, &analogOutValue_);
    createParam(analogInValueString, asynParamInt32, &analogInValue_);
    createParam(analogInRangeString, asynParamInt32, &analogInRange_);
    createParam(digitalDirectionString, asynParamUInt32Digital, &digitalDirection_);
    createParam(digitalOutputString, asynParamUInt32Digital, &digitalOutput_);
    createParam(digitalInputString, asynParamUInt32Digital, &digitalInput_);
    createParam(counterCountsString, asynParamInt32, &counterCounts_);
    createParam(counterResetString, asynParamInt32, &counterReset_);

    DaqDeviceDescriptor devDescriptors[10];
    unsigned int numDevs = 10;
    UlError err = ulGetDaqDeviceInventory(ANY_IFC, devDescriptors, &numDevs);
    if (err == ERR_NO_ERROR) {
        for (unsigned int i = 0; i < numDevs; i++) {
            if (strcmp(devDescriptors[i].uniqueId, uniqueID) == 0) {
                daqDeviceHandle_ = ulCreateDaqDevice(devDescriptors[i]);
                if (daqDeviceHandle_) {
                    ulConnectDaqDevice(daqDeviceHandle_);
                    ulDConfigPort(daqDeviceHandle_, DIGITAL_PORT, DD_INPUT);
                }
                break;
            }
        }
    }

    epicsThreadCreate("MultiFunctionPoller",
                      epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)pollerThreadC, this);
}

asynStatus MultiFunction::getBounds(asynUser *pasynUser, epicsInt32 *low, epicsInt32 *high) {
    int function = pasynUser->reason;
    if (function == analogOutValue_ || function == analogInValue_) {
        *low = 0; *high = 65535;
        return asynSuccess;
    }
    return asynError;
}

asynStatus MultiFunction::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int addr, status = 0;
    int function = pasynUser->reason;
    this->getAddress(pasynUser, &addr);
    setIntegerParam(addr, function, value);

    if (function == analogOutValue_)
        status = ulAOut(daqDeviceHandle_, addr, BIP10VOLTS, AOUT_FF_NOSCALEDATA, (double)value);
    else if (function == counterReset_)
        status = ulCLoad(daqDeviceHandle_, addr, CRT_LOAD, 0);

    callParamCallbacks(addr);
    return (status == 0) ? asynSuccess : asynError;
}

asynStatus MultiFunction::readInt32(asynUser *pasynUser, epicsInt32 *value) {
    int addr;
    int function = pasynUser->reason;
    this->getAddress(pasynUser, &addr);

    if (function == analogInValue_) {
        double data;
        UlError err = ulAIn(daqDeviceHandle_, addr, AI_SINGLE_ENDED, BIP10VOLTS, AIN_FF_NOSCALEDATA, &data);
        *value = (epicsInt32)data;
        setIntegerParam(addr, analogInValue_, *value);
    } else {
        asynPortDriver::readInt32(pasynUser, value);
    }

    callParamCallbacks(addr);
    return asynSuccess;
}

asynStatus MultiFunction::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask) {
    int function = pasynUser->reason;
    setUIntDigitalParam(function, value, mask);

    if (function == digitalDirection_) {
        for (int i = 0; i < NUM_IO_BITS; i++) {
            if (mask & (1 << i)) {
                ulDConfigBit(daqDeviceHandle_, DIGITAL_PORT, i,
                    (value & (1 << i)) ? DD_OUTPUT : DD_INPUT);
            }
        }
    }
    else if (function == digitalOutput_) {
        for (int i = 0; i < NUM_IO_BITS; i++) {
            if (mask & (1 << i)) {
                ulDBitOut(daqDeviceHandle_, DIGITAL_PORT, i, (value & (1 << i)) ? 1 : 0);
            }
        }
    }

    callParamCallbacks();
    return asynSuccess;
}

void MultiFunction::pollerThread() {
    unsigned long long newValue = 0, countVal = 0;
    while (1) {
        lock();
        if (daqDeviceHandle_) {
            ulDIn(daqDeviceHandle_, DIGITAL_PORT, &newValue);
            setUIntDigitalParam(digitalInput_, (epicsUInt32)newValue, 0xFFFFFFFF);

            for (int i = 0; i < NUM_COUNTERS; i++) {
                ulCIn(daqDeviceHandle_, i, &countVal);
                setIntegerParam(i, counterCounts_, (epicsInt32)countVal);
            }
        }
        for (int i = 0; i < MAX_SIGNALS; i++) callParamCallbacks(i);
        unlock();
        epicsThreadSleep(pollTime_);
    }
}

void MultiFunction::report(FILE *fp, int details) {
    fprintf(fp, "MultiFunction ULDAQ on port %s\n", this->portName);
    asynPortDriver::report(fp, details);
}

extern "C" int MultiFunctionConfig(const char *portName, const char *uniqueID) {
    new MultiFunction(portName, uniqueID);
    return asynSuccess;
}

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"Unique ID", iocshArgString};
static const iocshArg *const configArgs[] = {&configArg0, &configArg1};
static const iocshFuncDef configFuncDef = {"MultiFunctionConfig", 2, configArgs};
static void configCallFunc(const iocshArgBuf *args) {
    MultiFunctionConfig(args[0].sval, args[1].sval);
}

void drvMultiFunctionRegister(void) {
    iocshRegister(&configFuncDef, configCallFunc);
}

extern "C" {
    epicsExportRegistrar(drvMultiFunctionRegister);
}
