#ifndef __MOTOR_H
#define __MOTOR_H

#include "user_config.h"


extern uint16_t motor(uint32_t count, uint8_t * err_type);	//ιʳ����
extern char feed_run(FEED_INFO_t * info);
//extern void feed_scan(FEED_INFO_t * info);
extern int feed_scan(tuya_ble_time_struct_data_t *time, FEED_INFO_t *feed_t);


extern uint8_t feed_error;				//ιʳ���� 1(����) 2���豸�쳣�� 4��������
//extern uint8_t feed_required;			//ִ��ιʳ����
//extern FEED_INFO_t  feed_info_func;		//׼��ιʳ��ιʳ����
//extern uint8_t feed_detect_again;		//�ٴ�ιʳ��־
//extern uint8_t check_feed_flag;			//���ιʳ�ƻ���־
//extern uint8_t feed_one_flag;			//����ιʳ��־

//extern char food_count;					//��������ʳ�������
//extern uint8_t door_open;				//���Ŵ����־��=2 ���ų���ֹͣ���ţ�

extern uint8_t feed_status;				//ιʳ״̬����






#endif

