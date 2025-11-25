
#ifndef _COMMUNICATE_H
#define _COMMUNICATE_H


typedef enum _PIDType{AnglePID, SpeedPID,}PIDType;


extern char StatusFlag;

void Parse(char *dataInput);
void ResponseIMU(void);
void ResponsePID(PIDType type);
void ResponseStatus(void);


#endif

