/* 
This code is in the Public Domain (or CC0 licensed, at your option.)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <driver/i2c.h>
#include "esp_log.h"
#include "esp_err.h"
#include "bme280.h"

static const char* TAG = "BME280";


struct bme280_t {
	i2c_port_t port;
	int addr;
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
	int t_fine;
};


#define REG_HUM_LSB 0xFE
#define REG_HUM_MSB 0xFD
#define REG_TEMP_XLSB 0xFC
#define REG_TEMP_LSB 0xFB
#define REG_TEMP_MSB 0xFA
#define REG_PRESS_XLSB 0xF9
#define REG_PRESS_LSB 0xF8
#define REG_PRESS_MSB 0xF7
#define REG_CONFIG 0xF5
#define REG_CTRL_MEAS 0xF4
#define REG_STATUS 0xF3
#define REG_CTRL_HUM 0xF2
#define REG_RESET 0xED
#define REG_ID 0xD0
#define REG_CALIB(x) (x<26?(0x88+x):(0xE1+x))


#define RESET_MAGIC 0x6B
#define ID_VAL 0x60


#define ACK_CHECK_EN 1
#define ACK_VAL    0x0
#define NACK_VAL   0x1



static int writereg(bme280_t *d, int reg, int val) {
	esp_err_t ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, d->addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	if (val!=-1) {
		i2c_master_write_byte(cmd, val, ACK_CHECK_EN);
	}
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(d->port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return (ret != ESP_FAIL);
}

static int readreg(bme280_t *d, int reg, int *retval) {
	esp_err_t ret;
	uint8_t ret_b;
	if (!writereg(d, reg, -1)) return false;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, d->addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &ret_b, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(d->port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	*retval=ret_b;
	return (ret != ESP_FAIL);
}

//Reads REG_PRESS_MSB to REG_HUM_LSB
static int readsensorregs(bme280_t *d, uint8_t *regs) {
	if (!writereg(d, REG_PRESS_MSB, -1)) return false;
	esp_err_t ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, d->addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	for (int i=REG_PRESS_MSB; i<=REG_HUM_LSB; i++) {
		i2c_master_read_byte(cmd, &regs[i-REG_PRESS_MSB], (i!=REG_HUM_LSB)?ACK_VAL:NACK_VAL);
	}
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(d->port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return (ret != ESP_FAIL);
}

typedef struct {
	int offset;
	int reglo;
	int reghi;
} caldata_map_t;

const caldata_map_t caldata[]={
	{offsetof(bme280_t, dig_T1), 0x88, 0x89},
	{offsetof(bme280_t, dig_T2), 0x8A, 0x8B},
	{offsetof(bme280_t, dig_T3), 0x8C, 0x8D},
	{offsetof(bme280_t, dig_P1), 0x8E, 0x8F},
	{offsetof(bme280_t, dig_P2), 0x90, 0x91},
	{offsetof(bme280_t, dig_P3), 0x92, 0x93},
	{offsetof(bme280_t, dig_P4), 0x94, 0x95},
	{offsetof(bme280_t, dig_P5), 0x96, 0x97},
	{offsetof(bme280_t, dig_P6), 0x98, 0x99},
	{offsetof(bme280_t, dig_P7), 0x9A, 0x9B},
	{offsetof(bme280_t, dig_P8), 0x9C, 0x9D},
	{offsetof(bme280_t, dig_P9), 0x9E, 0x9F},
	{offsetof(bme280_t, dig_H1), 0xA1, -1},
	{offsetof(bme280_t, dig_H2), 0xE1, 0xE2},
	{offsetof(bme280_t, dig_H3), 0xE3, -1},
	{offsetof(bme280_t, dig_H4), 0xE5, 0xE4},  //de-mangled later
	{offsetof(bme280_t, dig_H5), 0xE5, 0xE6},  //de-mangled later
	{offsetof(bme280_t, dig_H6), 0xE7, -1},
	{-1, -1, -1}
};

static int read_cal(bme280_t *d) {
	int i=0;
	int r;
	uint8_t *dp=(uint8_t*)d;
	while (caldata[i].offset>0) {
		int rlo, rhi;
		r=readreg(d, caldata[i].reglo, &rlo);
		if (!r) goto err;
		if (caldata[i].reghi>0) {
			uint16_t *dps=(uint16_t*)(dp+caldata[i].offset);
			r=readreg(d, caldata[i].reghi, &rhi);
			if (!r) goto err;
			*dps=(rhi<<8)|rlo;
			ESP_LOGI(TAG, "Cal: Read short %d (0x%X)", (int)*dps, (int)*dps);
		} else {
			dp[caldata[i].offset]=rlo;
			ESP_LOGI(TAG, "Cal: Read char %d (0x%X)", (int)dp[caldata[i].offset], (int)dp[caldata[i].offset]);
		}
		i++;
	}

	//H4 and H5 share bits. De-mangle.
	d->dig_H4=((d->dig_H4&0xFF00)>>4)|(d->dig_H4&0xF);
	d->dig_H5=((d->dig_H5&0xFF00)>>4)|(d->dig_H4>>4);

	ESP_LOGI(TAG, "T1: %d T2: %d T3: %d", d->dig_T1, d->dig_T2, d->dig_T3);

	return true;
err:
	ESP_LOGE(TAG, "I2C error reading cal data from sensor\n");
	return false;
}



// From the datasheet.
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of 5123 equals 51.23 DegC.
// _t_fine carries fine temperature as "global" value.
static int32_t compensate_temperature(bme280_t *d, int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)d->dig_T1<<1))) * ((int32_t)d->dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)d->dig_T1)) * ((adc_T>>4) - ((int32_t)d->dig_T1))) >> 12) * ((int32_t)d->dig_T3)) >> 14;
	d->t_fine = var1 + var2;
	T = (d->t_fine * 5 + 128) >> 8;
	return T;
}


// From the datasheet.
// Returns pressure in Pa as unsigned 32 bit integer. Output value of 96386 equals 96386 Pa = 963.86 hPa
static uint32_t compensate_pressure(bme280_t *d, int32_t adc_P) {
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)d->t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)d->dig_P6);
	var2 = var2 + ((var1*((int32_t)d->dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)d->dig_P4)<<16);
	var1 = (((d->dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)d->dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)d->dig_P1))>>15);
	if (var1 == 0) return 0; // avoid exception caused by division by zero
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000){
		 p = (p << 1) / ((uint32_t)var1);
	} else {
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)d->dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)d->dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + d->dig_P7) >> 4));
	return p;
}


// From the datasheet.
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of 47445 represents 47445/1024 = 46.333 %RH
static int32_t compensate_humidity(bme280_t *d, int32_t adc_H) {
	int32_t v_x1_u32r;
	v_x1_u32r = (d->t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)d->dig_H4) << 20) - (((int32_t)d->dig_H5) * v_x1_u32r)) +
			((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)d->dig_H6)) >> 10) * (((v_x1_u32r *
			((int32_t)d->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
			((int32_t)d->dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)d->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}


int bme280_read(bme280_t *d, int32_t *temp, uint32_t *press, int32_t *hum) {
	uint8_t reg[8];
	int r=readsensorregs(d, reg);
	if (!r) goto err;
	
	int unc_temp, unc_press, unc_hum;
	
	unc_press=(reg[0]<<12)|(reg[1]<<4)|(reg[2]>>4);
	unc_temp=(reg[3]<<12)|(reg[4]<<4)|(reg[5]>>4);
	unc_hum=(reg[6]<<8)|(reg[7]);
	
	//ESP_LOGI(TAG, "Raw values: temp %d press %d hum %d", unc_temp, unc_press, unc_hum);
	
	int32_t rtemp=compensate_temperature(d, unc_temp);
	if (temp) *temp=rtemp;
	if (press) *press=compensate_pressure(d, unc_press);
	if (hum) *hum=compensate_humidity(d, unc_hum);
	
	return true;
err:
	ESP_LOGE(TAG, "I2C communication error");
	return false;
}



bme280_t *bme280_init(i2c_port_t port, int addr) {
	bme280_t *ret=malloc(sizeof(bme280_t));
	ret->port=port;
	ret->addr=addr;

	int id;
	int r=readreg(ret, REG_ID, &id);
	if (!r) {
		ESP_LOGE(TAG, "Read ID reg error for addr 0x%X", addr);
		goto err;
	}
	if (id!=ID_VAL) {
		ESP_LOGE(TAG, "Read ID: didn't receive 0x60 but 0x%X", id);
		goto err;
	}

	//Reset chip
	r=writereg(ret, REG_RESET, RESET_MAGIC);
	if (!r) {
		ESP_LOGE(TAG, "Chip reset reg error for addr 0x%X", addr);
		goto err;
	}

	if (!read_cal(ret)) goto err;


	r=writereg(ret, REG_CTRL_HUM, 0x5);
	r&=writereg(ret, REG_CTRL_MEAS, 0xB7);
	r&=writereg(ret, REG_CONFIG, 0);
	if (!r) {
		ESP_LOGE(TAG, "Chip config err");
		goto err;
	}



	ESP_LOGI(TAG, "Device at 0x%X initialized, ID=%X.", addr, id);

	return ret;
err:
	free(ret);
	return NULL;
}


void bme280_free(bme280_t *d) {
	free(d);
}

