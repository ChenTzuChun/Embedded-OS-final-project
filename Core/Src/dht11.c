#include "dht11.h"

extern TIM_HandleTypeDef htim6;

void delay(uint16_t time);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void DHT11_Start(void)
{
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    delay(18000);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay(20);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t DHT11_Check_Response(void)
{
    uint8_t Response = 0;
    delay(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
    {
        delay(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) Response = 1;
        else Response = 0;
    }
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
    return Response;
}

uint8_t DHT11_Read(void)
{
    uint8_t i = 0, j;
    for (j = 0; j < 8; j++)
    {
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
        delay(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) i &= ~(1 << (7 - j));
        else i |= (1 << (7 - j));
        while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
    }
    return i;
}

// 讀取完整資料（5 bytes），解析後放到結構體
uint8_t DHT11_Read_Data(DHT11_DataTypedef *data)
{
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;

    DHT11_Start();
    if (!DHT11_Check_Response())
        return DHT11_FAIL;

    Rh_byte1 = DHT11_Read();
    Rh_byte2 = DHT11_Read();
    Temp_byte1 = DHT11_Read();
    Temp_byte2 = DHT11_Read();
    SUM = DHT11_Read();

    if (SUM == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2))
    {
        data->humidity_int = Rh_byte1;
        data->humidity_dec = Rh_byte2;
        data->temperature_int = Temp_byte1;
        data->temperature_dec = Temp_byte2;
        data->SUM = SUM;
        data->checksum = Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2;
        return DHT11_OK;
    }
    else
    {
        data->SUM = SUM;
        data->checksum = Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2;
        return DHT11_FAIL;
    }
}


void delay(uint16_t time)
{
    if (time == 0) return;
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while ((__HAL_TIM_GET_COUNTER(&htim6)) < time);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
