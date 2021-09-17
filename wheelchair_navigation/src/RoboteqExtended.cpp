#include <string>
#include <cstdio>

#include "wheelchair_navigation/RoboteqExtended.h"
#include "wheelchair_navigation/Constants.h"
#include "wheelchair_navigation/ErrorCodes.h"

#define MISSING_VALUE -1024

int RoboteqExtended::GetTelemetry(string &telemetry)
{
    int status;

    status = FastSetCommand(_VAR, 1, 127);
    if(status != RQ_SUCCESS)
        return status;

    sleepms(10);

    string response;
	status = ReadAll(response);
	if(status != RQ_SUCCESS)
		return status;

    if(response.length() < 2)
        return RQ_INVALID_RESPONSE;

	string::size_type pos_begin = response.rfind("{");
	if(pos_begin == string::npos)
		return RQ_INVALID_RESPONSE;

	string::size_type pos_end = response.find("}", pos_begin);
	if(pos_end == string::npos)
		return RQ_INVALID_RESPONSE;

	telemetry = response.substr(pos_begin, pos_end - pos_begin + 1);

	return RQ_SUCCESS;
}

int RoboteqExtended::FastSetCommand(int commandItem, int index, int value, int waitms, int max_trial)
{
	char command[10];
	char args[50];

	if(commandItem < 0 || commandItem > 255)
		return RQ_INVALID_COMMAND_ITEM;

	sprintf(command, "$%02X", commandItem);
	sprintf(args, "%i %i", index, value);
	if(index == MISSING_VALUE)
	{
		if(value != MISSING_VALUE)
			sprintf(args, "%i", value);
		index = 0;
	}

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;
    
    string response;
    int status;
    for (int trial = 1; trial <= max_trial; trial++) {
    	status = IssueCommand("!", command, args, waitms, response, true);
        if (status != RQ_SUCCESS)
            continue;
        else if (response != "+")
            continue;
        else
            break;
    }
	if(response != "+")
		return RQ_SET_COMMAND_FAILED;

	return status;
}
int RoboteqExtended::FastSetCommand(int commandItem, int value)
{
    return FastSetCommand(commandItem, MISSING_VALUE, value, waitms, max_trial);
}
int RoboteqExtended::FastSetCommand(int commandItem)
{
    return FastSetCommand(commandItem, MISSING_VALUE, MISSING_VALUE, waitms, max_trial);
}


int RoboteqExtended::FastSetConfig(int configItem, int index, int value, int waitms, int max_trial)
{
	char command[10];
	char args[50];

	if(configItem < 0 || configItem > 255)
		return RQ_INVALID_CONFIG_ITEM;

	sprintf(command, "$%02X", configItem);
	sprintf(args, "%i %i", index, value);
	if(index == MISSING_VALUE)
	{
		sprintf(args, "%i", value);
		index = 0;
	}

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

    string response;
    int status;
    for (int trial = 1; trial <= max_trial; trial++) {
    	status = IssueCommand("^", command, args, waitms, response, true);
        if (status != RQ_SUCCESS)
            continue;
        else if (response != "+")
            continue;
        else
            break;
    }
	if(response != "+")
		return RQ_SET_CONFIG_FAILED;

	return status;
}