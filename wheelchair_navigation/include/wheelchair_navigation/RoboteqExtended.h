#ifndef __RoboteqExtended_H_
#define __RoboteqExtended_H_

#include "RoboteqDevice.h"

class RoboteqExtended : public RoboteqDevice
{
public:
    static const int waitms = 1;
    static const int max_trial = 3;

    int GetTelemetry(string &telemetry);

    int FastSetCommand(int commandItem, int index, int value, int waitms = waitms, int max_trial = max_trial);
    int FastSetCommand(int commandItem, int value);
    int FastSetCommand(int commandItem);

    int FastSetConfig(int configItem, int index, int value, int waitms = waitms, int max_trial = max_trial);
};


#endif