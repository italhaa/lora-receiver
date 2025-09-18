/*
 * lora_base.h
 *
 *  Created on: Feb 27, 2025
 *  Author: b.jamin
 *  
 *  LoRa Hub Firmware Header - Hub-only implementation
 */

#ifndef LORA_BASE_H
#define LORA_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

/* System includes */

#include "../../Drivers/lr11xx_driver/src/lr11xx_hal.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_system.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_radio.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_regmem.h"

/* Project includes */

/* Unit structs */

/* Unit public functions */

void lora_system_init(void);
void lora_system_process(void);
uint8_t lora_system_transmit_power(uint8_t boost);

#ifdef __cplusplus
}
#endif

#endif  // LORA_BASE_H

