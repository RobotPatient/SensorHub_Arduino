#ifndef ADS_DEFINITIONS_H
#define ADS_DEFINITIONS_H

// Table 10. ADS7138 Registers
// Based on datasheet https://www.ti.com/lit/ds/symlink/ads7138.pdf?ts=1689706366614
// J.A. Korten, July 19, 2023

#define SYSTEM_STATUS   0x00
#define GENERAL_CFG     0x01
#define DATA_CFG        0x02
#define OSR_CFG         0x03
#define OPMODE_CFG      0x04
#define PIN_CFG         0x05
#define GPIO_CFG        0x07
#define GPO_DRIVE_CFG   0x09
#define GPO_VALUE       0x0B
#define GPI_VALUE       0x0D
#define SEQUENCE_CFG    0x10
#define CHANNEL_SEL     0x11
#define AUTO_SEQ_CH_SEL 0x12
#define ALERT_CH_SEL    0x14
#define ALERT_MAP       0x16
#define ALERT_PIN_CFG   0x17
#define EVENT_FLAG      0x18
#define EVENT_HIGH_FLAG 0x1A
#define EVENT_LOW_FLAG  0x1C
#define EVENT_RGN       0x1E

#define HYSTERESIS_CH0  0x20
#define HIGH_TH_CH0     0x21
#define EVENT_COUNT_CH0 0x22
#define LOW_TH_CH0      0x23
#define HYSTERESIS_CH1  0x24
#define HIGH_TH_CH1     0x25
#define EVENT_COUNT_CH1 0x26
#define LOW_TH_CH1      0x27
#define HYSTERESIS_CH2  0x28
#define HIGH_TH_CH2     0x29
#define EVENT_COUNT_CH2 0x2A
#define LOW_TH_CH2      0x2B
#define HYSTERESIS_CH3  0x2C
#define HIGH_TH_CH3     0x2D
#define EVENT_COUNT_CH3 0x2E
#define LOW_TH_CH3      0x2F
#define HYSTERESIS_CH4  0x30
#define HIGH_TH_CH4     0x31
#define EVENT_COUNT_CH4 0x32
#define LOW_TH_CH4      0x33
#define HYSTERESIS_CH5  0x34
#define HIGH_TH_CH5     0x35
#define EVENT_COUNT_CH5 0x36
#define LOW_TH_CH5      0x37
#define HYSTERESIS_CH6  0x38
#define HIGH_TH_CH6     0x39
#define EVENT_COUNT_CH6 0x3A
#define LOW_TH_CH6      0x3B
#define HYSTERESIS_CH7  0x3C
#define HIGH_TH_CH7     0x3D
#define EVENT_COUNT_CH7 0x3E
#define LOW_TH_CH7      0x3F

#define MAX_CH0_LSB     0x60
#define MAX_CH0_MSB     0x61
#define MAX_CH1_LSB     0x62
#define MAX_CH1_MSB     0x63
#define MAX_CH2_LSB     0x64
#define MAX_CH2_MSB     0x65
#define MAX_CH3_LSB     0x66
#define MAX_CH3_MSB     0x67
#define MAX_CH4_LSB     0x68
#define MAX_CH4_MSB     0x69
#define MAX_CH5_LSB     0x6A
#define MAX_CH5_MSB     0x6B
#define MAX_CH6_LSB     0x6C
#define MAX_CH6_MSB     0x6D
#define MAX_CH7_LSB     0x6E
#define MAX_CH7_MSB     0x6F

#define MIN_CH0_LSB     0x80
#define MIN_CH0_MSB     0x81
#define MIN_CH1_LSB     0x82
#define MIN_CH1_MSB     0x83
#define MIN_CH2_LSB     0x84
#define MIN_CH2_MSB     0x85
#define MIN_CH3_LSB     0x86
#define MIN_CH3_MSB     0x87
#define MIN_CH4_LSB     0x88
#define MIN_CH4_MSB     0x89
#define MIN_CH5_LSB     0x8A
#define MIN_CH5_MSB     0x8B
#define MIN_CH6_LSB     0x8C
#define MIN_CH6_MSB     0x8D
#define MIN_CH7_LSB     0x8E
#define MIN_CH7_MSB     0x8F

#define RECENT_CH0_LSB  0xA0
#define RECENT_CH0_MSB  0xA1
#define RECENT_CH1_LSB  0xA2
#define RECENT_CH1_MSB  0xA3
#define RECENT_CH2_LSB  0xA4
#define RECENT_CH2_MSB  0xA5
#define RECENT_CH3_LSB  0xA6
#define RECENT_CH3_MSB  0xA7
#define RECENT_CH4_LSB  0xA8
#define RECENT_CH4_MSB  0xA9
#define RECENT_CH5_LSB  0xAA
#define RECENT_CH5_MSB  0xAB
#define RECENT_CH6_LSB  0xAC
#define RECENT_CH6_MSB  0xAD
#define RECENT_CH7_LSB  0xAE
#define RECENT_CH7_MSB  0xAF

#define GPO0_TRIG_EVENT_SEL 0xC3
#define GPO1_TRIG_EVENT_SEL 0xC5
#define GPO2_TRIG_EVENT_SEL 0xC7
#define GPO3_TRIG_EVENT_SEL 0xC9
#define GPO4_TRIG_EVENT_SEL 0xCB
#define GPO5_TRIG_EVENT_SEL 0xCD
#define GPO6_TRIG_EVENT_SEL 0xCF
#define GPO7_TRIG_EVENT_SEL 0xD1

#define GPO_TRIGGER_CFG 0xE9
#define GPO_VALUE_TRIG  0xEB

// Commands Datasheet 8.5.1.1

#define SINGLE_READ     0x10  // Single Register Read
#define SINGLE_WRITE    0x08  // Single Register Write
#define SET_BIT         0x18  // Set Bit
#define CLEAR_BIT       0x20  // Clear Bit
#define READ_BLOCK      0x30  // Reading a continuous block of registers
#define WRITE_BLOCK     0x28  // Writing a continuous block of registers

// Convenient commands:

#define RESET_SYS_STAT 0xB


#endif
