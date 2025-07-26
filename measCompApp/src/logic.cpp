#include <asynPortClient.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <stdio.h>

class LogicController {
public:
    LogicController(const char* portName);
    void logicThread();

private:
    asynInt32Client *ai1Client_;
    asynUInt32DigitalClient *bo1Client_;
    asynInt32Client *counterClient_;
    asynUInt32DigitalClient *bo2Client_;
    asynUInt32DigitalClient *bo3Client_;
    asynInt32Client *ao2Client_;
    double pollTime_;
};

LogicController::LogicController(const char* portName)
    : pollTime_(0.1)
{
    ai1Client_      = new asynInt32Client(portName, 0, "ANALOG_IN_VALUE");
    bo1Client_      = new asynUInt32DigitalClient(portName, 0, "DIGITAL_OUTPUT");
    counterClient_  = new asynInt32Client(portName, 0, "COUNTER_VALUE");
    bo2Client_      = new asynUInt32DigitalClient(portName, 0, "DIGITAL_OUTPUT"); 
    bo3Client_      = new asynUInt32DigitalClient(portName, 0, "DIGITAL_OUTPUT"); 
    ao2Client_      = new asynInt32Client(portName, 1, "ANALOG_OUT_VALUE");

    epicsThreadCreate("LogicThread",
                      epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)([](void *pPvt) {
                          ((LogicController*)pPvt)->logicThread();
                      }),
                      this);
}

void LogicController::logicThread() {
    epicsInt32 prevBo1 = -1;
    epicsInt32 prevBo2 = -1;
    epicsInt32 prevBo3 = -1;
    epicsInt32 prevAo2 = -1;

    printf("[Logic] Thread started successfully.\n");

    while (true) {
        //Ai1 5V 이상일 때  Bo1 on
        epicsInt32 ai1Val = 0;
        if (ai1Client_->read(&ai1Val) == asynSuccess) {
            epicsInt32 bo1Val = (ai1Val >= 49151) ? 1 : 0;
            if (bo1Val != prevBo1) {
                bo1Client_->write(bo1Val ? 0x01 : 0x00, 0x01);
                prevBo1 = bo1Val;
                printf("[Logic] Bo1 updated: %d (Ai1=%d)\n", bo1Val, ai1Val);
            }
        }

        //Counter %3 
        epicsInt32 countVal = 0;
        if (counterClient_->read(&countVal) == asynSuccess) {
            // BO2/BO3 값 계산
            epicsInt32 bo2Val = (countVal % 3 == 0) ? 1 : 0;
            epicsInt32 bo3Val = (countVal % 3 == 0) ? 0 : 1;

            // BO2 제어 (비트1)
            if (bo2Val != prevBo2) {
                bo2Client_->write(bo2Val ? 0x02 : 0x00, 0x02); // mask=0x02
                prevBo2 = bo2Val;
            }

            // BO3 제어 (비트2)
            if (bo3Val != prevBo3) {
                bo3Client_->write(bo3Val ? 0x04 : 0x00, 0x04); // mask=0x04
                prevBo3 = bo3Val;
            }

            // AO2 출력 (카운터 기반)
            double ao2Volt = countVal * 0.1;
            if (ao2Volt > 10.0) ao2Volt = 10.0;
            if (ao2Volt < 0.0) ao2Volt = 0.0;

            epicsInt32 ao2Raw = (epicsInt32)(((ao2Volt + 10.0) / 20.0) * 65535.0);
            if (ao2Raw != prevAo2) {
                ao2Client_->write(ao2Raw);
                prevAo2 = ao2Raw;
                printf("AO2 updated: RAW=%d (Volt=%.2f, Counter=%d)\n",
                       ao2Raw, ao2Volt, countVal);
            }
        }

        epicsThreadSleep(pollTime_);
    }
}

extern "C" int LogicConfig(const char* portName) {
    new LogicController(portName);
    return asynSuccess;
}

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg *const configArgs[] = {&configArg0};
static const iocshFuncDef configFuncDef = {"LogicConfig", 1, configArgs};
static void configCallFunc(const iocshArgBuf *args) {
    LogicConfig(args[0].sval);
}

void drvLogicRegister(void) {
    iocshRegister(&configFuncDef, configCallFunc);
}

extern "C" {
    epicsExportRegistrar(drvLogicRegister);
}
