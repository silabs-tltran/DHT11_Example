/**
  ******************************************************************************
  * @file    dht11.c
  * @brief   Temperature - humidity DHT11 sensor module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of DHT11 module:
  *          + Start and initialize DHT11 sensor
  *          + Read data from DHT11 sensor
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "dht11.h"

//static void DHT_TimerInit(void)
//{
//    TIM_HandleTypeDef htim4;
//    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//    TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//    htim4.Instance = TIM4;
//    htim4.Init.Prescaler = 0;
//    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim4.Init.Period = 65535;
//    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
//        Error_Handler();
//    }
//    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
//        Error_Handler();
//    }
//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
//        Error_Handler();
//    }
//}

static void DHT_DelayInit(DHT_Name_t* dht)
{
    HAL_TIM_Base_Start(dht->htimer);
}

static void DHT_DelayUs(DHT_Name_t* dht, uint16_t time)
{
    __HAL_TIM_SET_COUNTER(dht->htimer, 0);
    while(__HAL_TIM_GET_COUNTER(dht->htimer) < time) {
    }
}

static void DHT_SetPinOut(DHT_Name_t* dht)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = dht->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
}

static void DHT_SetPinIn(DHT_Name_t* dht)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = dht->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
}

static void DHT_WritePin(DHT_Name_t* dht, uint8_t value)
{
    HAL_GPIO_WritePin(dht->port, dht->pin, value);
}

static uint8_t DHT_ReadPin(DHT_Name_t* dht)
{
    uint8_t value;
    value =  HAL_GPIO_ReadPin(dht->port, dht->pin);

    return value;
}

static uint8_t DHT_Start(DHT_Name_t* dht)
{
    uint8_t response = 0;
    DHT_SetPinOut(dht);
    DHT_WritePin(dht, 0);
    DHT_DelayUs(dht, DHT11_STARTTIME);
    DHT_SetPinIn(dht);
    DHT_DelayUs(dht, 40);
    if (DHT_ReadPin(dht) == 0) {
        DHT_DelayUs(dht, 40);
        if (DHT_ReadPin(dht) == 1) {
            response = 1;
        }
        else response = 0;
    }
    while (DHT_ReadPin(dht) == 1);

    return response;
}

static uint8_t DHT_Read(DHT_Name_t* dht)
{
    uint8_t value = 0;
    uint8_t i = 0;

    DHT_SetPinIn(dht);
    for (i = 0; i<8; i++) {
        while (DHT_ReadPin(dht) == 0);
        DHT_DelayUs(dht, 40);
        if(DHT_ReadPin(dht) == 0) {
            value &= ~(1 << (7 - i));
        }
        else {
            value |= 1 << (7 - i);
        }
        while (DHT_ReadPin(dht) == 1);
    }

    return value;
}

void DHT_Init(DHT_Name_t* dht, TIM_HandleTypeDef* htimer, GPIO_TypeDef* dht_port, uint16_t dht_pin)
{
    dht->port = dht_port;
    dht->pin = dht_pin;
    dht->htimer = htimer;
    DHT_DelayInit(dht);
}

uint8_t DHT_ReadData(DHT_Name_t* dht)
{
    uint8_t temp1, temp2, rh1, rh2;
    uint16_t temp, humid, sum = 0;
    DHT_Start(dht);
    rh1 = DHT_Read(dht);
    rh2 = DHT_Read(dht);
    temp1 = DHT_Read(dht);
    temp2 = DHT_Read(dht);
    sum = DHT_Read(dht);
    temp = (temp1 << 8) | temp2;
    humid = (rh1 << 8) | rh2;
    dht->temp = (float) (temp / 10.0);
    dht->humid = (float) (humid / 10.0);

    return sum;
}
