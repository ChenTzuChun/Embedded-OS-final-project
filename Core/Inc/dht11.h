#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f4xx_hal.h"

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

// 讀取狀態回傳值
#define DHT11_OK      1
#define DHT11_FAIL    0

// 結構體封裝溫濕度資料
typedef struct {
    uint8_t humidity_int;
    uint8_t humidity_dec;
    uint8_t temperature_int;
    uint8_t temperature_dec;
    uint8_t SUM;
    uint8_t checksum;
} DHT11_DataTypedef;

// 初始化與讀取
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);
uint8_t DHT11_Read_Data(DHT11_DataTypedef *data);

void delay(uint16_t time);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif
