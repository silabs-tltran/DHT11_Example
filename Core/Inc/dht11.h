/**
  ******************************************************************************
  * @file    dht11.h
  * @brief   Header file of temperature - humidity DHT11 sensor module.
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

#ifndef _DHT11_H_
#define _DHT11_H_

#include "stm32f4xx_hal.h"
#define DHT11_STARTTIME 18000
typedef struct
{
    TIM_HandleTypeDef* htimer;
    uint16_t pin;
    GPIO_TypeDef* port;
    float temp;
    float humid;
} DHT_Name_t;

void DHT_Init(DHT_Name_t* dht, TIM_HandleTypeDef* htimer, GPIO_TypeDef* dht_port, uint16_t dht_pin);
uint8_t DHT_ReadData(DHT_Name_t* dht);
#endif
