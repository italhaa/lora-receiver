# LoRa Hub Firmware - STM32_LR1121

This repository contains the hub-only firmware for the LoRa communication system using STM32 microcontroller with LR1121 LoRa transceiver.

## Overview

This firmware is specifically designed to operate as a LoRa hub that:
- Receives data packets from remote LoRa nodes
- Processes and validates incoming data
- Sends acknowledgment responses back to nodes
- Manages multiple node communications

## Hardware

- STM32L476RG microcontroller
- LR1121 LoRa transceiver
- Custom PCB design

## Note

This is the **hub-only** version of the firmware. For node firmware, use the separate node repository.