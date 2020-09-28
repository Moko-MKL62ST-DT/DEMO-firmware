#ifndef USERAPP_SHT30_H
#define USERAPP_SHT30_H
#include "stdbool.h"
#endif

#ifdef	USERAPP_SHT30_C


#else /*LIB_IO***********************************************************************/

extern void ReadTemHumTaskInit(void);

//��ȡ��ʪ��ֵ
extern void Read_tempHum_valu(void);

extern void ReadTemHumTask(void);

//���sht30��ʪ��ֵ
extern void Clr_sht30TemHumValue(void);

extern int Get_sht30HumValu(void);

extern int Get_sht30TemValu(void);

extern void sht30TimeCntCallBack(void);

//��ȡ��ʪ�ȳ�ʼ����ɱ�־
extern bool getReadTempHumResult(void);

#endif

