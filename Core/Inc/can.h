#ifndef CAN_H
#define CAN_H


#include<stdint.h>

typedef enum
{
	CAN_STATUS_OK=0,
	CAN_STATUS_NO_DATA,
	CAN_STATUS_ERROR,
	CAN_STATUS_BAD_ARGUMENT
}CAN_Status;

typedef struct
{
    int16_t current;
    int16_t speed;
    uint16_t mechanical_angle;
    uint8_t temperature;
} Receive_t;





//(配置过滤器并且启动CAN1
    CAN_Status CAN_AppInit(void);



//发送一帧标准数据帧
    CAN_Status CAN_SendStd(uint16_t std_id,const uint8_t*data,uint8_t length);
 //从FIFO0读取一帧标准帧数据
CAN_Status CAN_ReadStd(uint32_t *std_id,
                       uint8_t *data,
                       uint8_t *length);

#endif/*CAN_H*/
