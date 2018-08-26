#include "../common.h"
#include "../adrobot_control/adrobot_control.h"

/* dataExchange.cpp */
extern void SvoReadFromServo(SVO *data);
extern void SvoWriteFromServo(SVO *data);
extern void SvoReadFromGui(SVO *data);
extern void SvoWriteFromGui(SVO *data);
extern void SvoReadFromDis(SVO *data);
extern void ChangePathData(PATH*path);
extern void ChangeHandData(PATH*path);
extern void PosOriServo(int*posoriservoflag);
extern void SetPosOriSvo(SVO*data);
extern void SetJntSvo(SVO *data);

/* dataSave.cpp*/
extern void ExpDataSave(SVO_SAVE* data);
extern void SaveDataReset();
extern void ExpDataWrite();

