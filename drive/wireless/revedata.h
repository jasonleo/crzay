#ifndef _ReceiveData_H_
#define _ReceiveData_H_
#include "stm32f10x.h"




void ReceiveDataFormNRF(void);
void ReceiveDataFormUART(void);

extern int  Rool_error_init;     //����ɻ���ɳ���ƫ��Rool_error_init�����������޸�;����ƫ��Rool_error_init�����������޸�
extern int  Pitch_error_init;     //����ɻ���ɳ�ǰƫ��Pitch_error_init�����������޸�;����ƫ��Pitch_error_init�����������޸�



#endif

