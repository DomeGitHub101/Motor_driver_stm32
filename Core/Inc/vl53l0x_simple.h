#ifndef VL53L0X_SIMPLE_H
#define VL53L0X_SIMPLE_H

#include "main.h"
#include <stdint.h>

/* 8-bit I2C address ตอนเปิดเครื่อง (0x29 << 1) */
#define TOF0_DEFAULT_ADDR 0x52

typedef struct {
	uint8_t addr;           /* 8-bit address ที่ใช้คุยกับตัวนี้ */
	uint8_t stop_variable;  /* ใช้ภายใน driver */
} tof0_t;

/* คืน 0 = สำเร็จ, ค่าอื่น = error (10 อ่าน I2C ไม่ได้, 11 ID ไม่ใช่ 0xEE ...) */
int tof0_init(tof0_t *d);

/* เปลี่ยน address (ทำก่อน tof0_init, ตอนเปิดทีละตัว) */
int tof0_set_address(tof0_t *d, uint8_t new_addr8);

/* อ่านระยะแบบ single-shot (มม.) คืน 0 = สำเร็จ
 * ค่า >= 8190 หมายถึงไม่เจอวัตถุ/อยู่นอกระยะ */
int tof0_read_mm(tof0_t *d, uint16_t *mm);

/* Start back-to-back continuous ranging (call after tof0_init), 0 = OK */
int tof0_start_continuous(tof0_t *d);

/* Non-blocking: 0 = new sample in *mm, -1 = not ready yet, > 0 = I2C error */
int tof0_poll_mm(tof0_t *d, uint16_t *mm);

/* Read one register (debug), 0 = OK */
int tof0_read_reg(tof0_t *d, uint8_t reg, uint8_t *v);

#endif
