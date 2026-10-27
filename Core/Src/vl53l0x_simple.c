/* Minimal VL53L0X driver for STM32 HAL (I2C 8-bit register index).
 * ลำดับ init อ้างอิงจากไลบรารี VL53L0X ของ Pololu (MIT license). */
#include "vl53l0x_simple.h"
#include "i2c.h"

#define TO_MS 10   /* short: a noisy transfer must not stall the main loop */
#define WAIT_MS 100   /* register poll limit (calibration, SPAD info): bounds a
                         failed re-init, which blocks the main loop */

static uint8_t cur_addr = TOF0_DEFAULT_ADDR;   /* address ของตัวที่กำลังคุยอยู่ */

static int wr8(uint8_t reg, uint8_t v) {
	return HAL_I2C_Mem_Write(&hi2c1, cur_addr, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, TO_MS) != HAL_OK;
}
static int rd8(uint8_t reg, uint8_t *v) {
	return HAL_I2C_Mem_Read(&hi2c1, cur_addr, reg, I2C_MEMADD_SIZE_8BIT, v, 1, TO_MS) != HAL_OK;
}
static int wr16(uint8_t reg, uint16_t v) {
	uint8_t b[2] = { v >> 8, v & 0xFF };
	return HAL_I2C_Mem_Write(&hi2c1, cur_addr, reg, I2C_MEMADD_SIZE_8BIT, b, 2, TO_MS) != HAL_OK;
}
static int rd16(uint8_t reg, uint16_t *v) {
	uint8_t b[2];
	if (HAL_I2C_Mem_Read(&hi2c1, cur_addr, reg, I2C_MEMADD_SIZE_8BIT, b, 2, TO_MS) != HAL_OK)
		return 1;
	*v = ((uint16_t) b[0] << 8) | b[1];
	return 0;
}
static int wrN(uint8_t reg, uint8_t *p, uint16_t n) {
	return HAL_I2C_Mem_Write(&hi2c1, cur_addr, reg, I2C_MEMADD_SIZE_8BIT, p, n, TO_MS) != HAL_OK;
}
static int rdN(uint8_t reg, uint8_t *p, uint16_t n) {
	return HAL_I2C_Mem_Read(&hi2c1, cur_addr, reg, I2C_MEMADD_SIZE_8BIT, p, n, TO_MS) != HAL_OK;
}

/* รอจน (reg & mask) == want  หรือ timeout 500 ms */
static int wait_reg(uint8_t reg, uint8_t mask, uint8_t want) {
	uint8_t v;
	uint32_t t0 = HAL_GetTick();
	for (;;) {
		if (rd8(reg, &v)) return 1;
		if ((v & mask) == want) return 0;
		if (HAL_GetTick() - t0 > WAIT_MS) return 2;
	}
}
static int wait_reg_nonzero(uint8_t reg, uint8_t mask) {
	uint8_t v;
	uint32_t t0 = HAL_GetTick();
	for (;;) {
		if (rd8(reg, &v)) return 1;
		if (v & mask) return 0;
		if (HAL_GetTick() - t0 > WAIT_MS) return 2;
	}
}

static int get_spad_info(uint8_t *count, uint8_t *is_aperture) {
	uint8_t tmp;
	int e = 0;
	e |= wr8(0x80, 0x01); e |= wr8(0xFF, 0x01); e |= wr8(0x00, 0x00);
	e |= wr8(0xFF, 0x06);
	e |= rd8(0x83, &tmp); e |= wr8(0x83, tmp | 0x04);
	e |= wr8(0xFF, 0x07); e |= wr8(0x81, 0x01); e |= wr8(0x80, 0x01);
	e |= wr8(0x94, 0x6B); e |= wr8(0x83, 0x00);
	if (e) return 1;
	if (wait_reg_nonzero(0x83, 0xFF)) return 2;
	e |= wr8(0x83, 0x01);
	e |= rd8(0x92, &tmp);
	*count = tmp & 0x7F;
	*is_aperture = (tmp >> 7) & 0x01;
	e |= wr8(0x81, 0x00); e |= wr8(0xFF, 0x06);
	e |= rd8(0x83, &tmp); e |= wr8(0x83, tmp & ~0x04);
	e |= wr8(0xFF, 0x01); e |= wr8(0x00, 0x01);
	e |= wr8(0xFF, 0x00); e |= wr8(0x80, 0x00);
	return e ? 3 : 0;
}

static int ref_calibration(uint8_t vhv_init_byte) {
	if (wr8(0x00, 0x01 | vhv_init_byte)) return 1;
	if (wait_reg_nonzero(0x13, 0x07)) return 2;
	if (wr8(0x0B, 0x01)) return 1;
	if (wr8(0x00, 0x00)) return 1;
	return 0;
}

int tof0_init(tof0_t *d) {
	uint8_t v, id = 0;
	int e = 0;
	cur_addr = d->addr;

	/* ตรวจ ID: reg 0xC0 ต้องเป็น 0xEE */
	if (rd8(0xC0, &id)) return 10;
	if (id != 0xEE) return 11;

	/* ตั้ง I/O เป็น 2.8V mode */
	e |= rd8(0x89, &v); e |= wr8(0x89, v | 0x01);

	/* I2C standard mode */
	e |= wr8(0x88, 0x00);
	e |= wr8(0x80, 0x01); e |= wr8(0xFF, 0x01); e |= wr8(0x00, 0x00);
	e |= rd8(0x91, &d->stop_variable);
	e |= wr8(0x00, 0x01); e |= wr8(0xFF, 0x00); e |= wr8(0x80, 0x00);

	/* ปิด signal rate limit check บางตัว */
	e |= rd8(0x60, &v); e |= wr8(0x60, v | 0x12);
	/* final range signal rate limit = 0.25 MCPS (0.25 * 128 = 32) */
	e |= wr16(0x44, 32);
	e |= wr8(0x01, 0xFF);
	if (e) return 12;

	/* SPAD */
	uint8_t spad_count, spad_aperture;
	if (get_spad_info(&spad_count, &spad_aperture)) return 13;

	uint8_t ref_spad_map[6];
	e |= rdN(0xB0, ref_spad_map, 6);
	e |= wr8(0xFF, 0x01); e |= wr8(0x4F, 0x00); e |= wr8(0x4E, 0x2C);
	e |= wr8(0xFF, 0x00); e |= wr8(0xB6, 0xB4);

	uint8_t first = spad_aperture ? 12 : 0;
	uint8_t enabled = 0;
	for (uint8_t i = 0; i < 48; i++) {
		if (i < first || enabled == spad_count) {
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		} else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
			enabled++;
		}
	}
	e |= wrN(0xB0, ref_spad_map, 6);
	if (e) return 14;

	/* Default tuning settings */
	static const uint8_t tune[][2] = {
		{0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
		{0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
		{0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
		{0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
		{0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
		{0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
		{0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
		{0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
		{0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
		{0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
		{0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
		{0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
		{0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
		{0xFF,0x00},{0x80,0x00}
	};
	for (unsigned i = 0; i < sizeof(tune) / sizeof(tune[0]); i++)
		e |= wr8(tune[i][0], tune[i][1]);
	if (e) return 15;

	/* GPIO config: new sample ready, active low */
	e |= wr8(0x0A, 0x04);
	e |= rd8(0x84, &v); e |= wr8(0x84, v & ~0x10);
	e |= wr8(0x0B, 0x01);
	e |= wr8(0x01, 0xE8);
	if (e) return 16;

	/* calibration */
	e |= wr8(0x01, 0x01);
	if (e || ref_calibration(0x40)) return 17;
	e |= wr8(0x01, 0x02);
	if (e || ref_calibration(0x00)) return 18;
	if (wr8(0x01, 0xE8)) return 19;

	return 0;
}

int tof0_read_mm(tof0_t *d, uint16_t *mm) {
	int e = 0;
	cur_addr = d->addr;
	e |= wr8(0x80, 0x01); e |= wr8(0xFF, 0x01); e |= wr8(0x00, 0x00);
	e |= wr8(0x91, d->stop_variable);
	e |= wr8(0x00, 0x01); e |= wr8(0xFF, 0x00); e |= wr8(0x80, 0x00);
	e |= wr8(0x00, 0x01);               /* SYSRANGE_START: single shot */
	if (e) return 1;
	if (wait_reg(0x00, 0x01, 0x00)) return 2;      /* start bit cleared */
	if (wait_reg_nonzero(0x13, 0x07)) return 3;    /* result ready */
	if (rd16(0x14 + 10, mm)) return 4;
	if (wr8(0x0B, 0x01)) return 5;                 /* clear interrupt */
	return 0;
}

int tof0_start_continuous(tof0_t *d) {
	int e = 0;
	cur_addr = d->addr;
	e |= wr8(0x80, 0x01); e |= wr8(0xFF, 0x01); e |= wr8(0x00, 0x00);
	e |= wr8(0x91, d->stop_variable);
	e |= wr8(0x00, 0x01); e |= wr8(0xFF, 0x00); e |= wr8(0x80, 0x00);
	e |= wr8(0x00, 0x02);               /* SYSRANGE_START: back-to-back */
	return e ? 1 : 0;
}

int tof0_poll_mm(tof0_t *d, uint16_t *mm) {
	uint8_t st;
	cur_addr = d->addr;
	if (rd8(0x13, &st)) return 1;                 /* RESULT_INTERRUPT_STATUS */
	if ((st & 0x07) == 0) return -1;              /* no new sample yet */
	if (rd16(0x14 + 10, mm)) return 2;
	if (wr8(0x0B, 0x01)) return 3;                /* clear interrupt */
	return 0;
}

int tof0_read_reg(tof0_t *d, uint8_t reg, uint8_t *v) {
	cur_addr = d->addr;
	return rd8(reg, v);
}

/* เปลี่ยน I2C address ของเซนเซอร์ (8-bit address เช่น 0x54)
 * ค่านี้หายเมื่อปิดไฟหรือดึง XSHUT ต่ำ */
int tof0_set_address(tof0_t *d, uint8_t new_addr8) {
	cur_addr = d->addr;
	if (wr8(0x8A, (new_addr8 >> 1) & 0x7F)) return 1;
	d->addr = new_addr8;
	return 0;
}
