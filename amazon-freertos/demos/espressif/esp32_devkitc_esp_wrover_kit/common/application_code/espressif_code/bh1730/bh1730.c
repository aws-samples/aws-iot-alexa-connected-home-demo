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
#include "bh1730.h"

static const char* TAG = "BH1730";


struct bh1730_t {
	i2c_port_t port;
	int addr;
	int itime;
	int gain;
};


#define BH1730_CMD_MAGIC 0x80
#define BH1730_CMD_SETADDR 0x0
#define BH1730_CMD_SPECCMD 0x60

#define BH1730_SPECCMD_INTRESET 0x1
#define BH1730_SPECCMD_STOPMEAS 0x2
#define BH1730_SPECCMD_STARTMEAS 0x3
#define BH1730_SPECCMD_RESET 0x4

#define BH1730_ADDR_CONTROL 0x00
#define BH1730_ADDR_TIMING 0x01
#define BH1730_ADDR_INTERRUPT 0x02
#define BH1730_ADDR_THLLOW 0x03
#define BH1730_ADDR_THLHIGH 0x04
#define BH1730_ADDR_THHLOW 0x05
#define BH1730_ADDR_THHIGH 0x06
#define BH1730_ADDR_GAIN 0x07
#define BH1730_ADDR_ID 0x12
#define BH1730_ADDR_DATA0LOW 0x14
#define BH1730_ADDR_DATA0HIGH 0x15
#define BH1730_ADDR_DATA1LOW 0x16
#define BH1730_ADDR_DATA1HIGH 0x17


#define BH1730_CTL_ADC_INTR 0x20
#define BH1730_CTL_ADC_VALID 0x10
#define BH1730_CTL_ONE_TIME 0x08
#define BH1730_CTL_DATA_SEL 0x04
#define BH1730_CTL_ADC_EN 0x02
#define BH1730_CTL_POWER 0x01


#define ACK_CHECK_EN 1
#define ACK_VAL    0x0
#define NACK_VAL   0x1


static int sendcmd(bh1730_t *d, int cmdbyte, int val) {
	esp_err_t ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, d->addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, cmdbyte, ACK_CHECK_EN);
	if (val!=-1) {
		i2c_master_write_byte(cmd, val, ACK_CHECK_EN);
	}
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(d->port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return (ret != ESP_FAIL);
}

static int readreg(bh1730_t *d, int reg, int *retval) {
	if (!sendcmd(d, reg|BH1730_CMD_MAGIC|BH1730_CMD_SETADDR, -1)) return false;

	esp_err_t ret;
	uint8_t ret_b;
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

#define T_INT_US 2.8 //uS, typical, from datasheet

float bh1730_read_lux(bh1730_t *d) {
	int r=sendcmd(d, BH1730_CMD_MAGIC|BH1730_CMD_SETADDR|BH1730_ADDR_CONTROL, BH1730_CTL_POWER|BH1730_CTL_ADC_EN|BH1730_CTL_ONE_TIME);
	if (!r) goto err;

	int d0h, d0l, d1h, d1l, s;
	int tmo=100;
	do {
		r=readreg(d, BH1730_ADDR_CONTROL, &s);
		if (!r) goto err;
		vTaskDelay(20 / portTICK_RATE_MS);
		if ((tmo--)==0) goto err;
	} while ((s&BH1730_CTL_ADC_VALID)==0);

	r=readreg(d, BH1730_ADDR_DATA0LOW, &d0l);
	r&=readreg(d, BH1730_ADDR_DATA0HIGH, &d0h);
	r&=readreg(d, BH1730_ADDR_DATA1LOW, &d1l);
	r&=readreg(d, BH1730_ADDR_DATA1HIGH, &d1h);
	if (!r) goto err;

	float data0=(d0h<<8)+d0l;
	float data1=(d1h<<8)+d1l;
	float itime_ms=(T_INT_US*964*(256-d->itime))/1000.0;

	float lux=0;
	if (data0!=0) {
		if (data1/data0<0.26) {
			lux=(1.290*data0-2.733*data1)/d->gain*102.6/itime_ms;
		} else if (data1/data0<0.55) {
			lux=(0.795*data0-0.859*data1)/d->gain*102.6/itime_ms;
		} else if (data1/data0<1.09) {
			lux=(0.510*data0-0.345*data1)/d->gain*102.6/itime_ms;
		} else if (data1/data0<2.13) {
			lux=(0.276*data0-0.130*data1)/d->gain*102.6/itime_ms;
		}
	}
	return lux;
err:
	ESP_LOGE(TAG, "I2C communications error");
	return -1;
}


bh1730_t *bh1730_init(i2c_port_t port, int addr) {
	bh1730_t *ret=malloc(sizeof(bh1730_t));
	ret->port=port;
	ret->addr=addr;
	ret->itime=0xDA; //reset val
	ret->gain=1; //reset val

	int id;
	int r=readreg(ret, BH1730_ADDR_ID, &id);
	if (!r) {
		ESP_LOGE(TAG, "Read ID reg error for addr 0x%X", addr);
		goto err;
	}
	if ((id&0xF0)!=0x70) {
		ESP_LOGE(TAG, "Read ID: didn't receive 0x7x but 0x%X", id);
		goto err;
	}

	//Reset chip
	r=sendcmd(ret, BH1730_CMD_MAGIC|BH1730_CMD_SPECCMD|BH1730_SPECCMD_RESET, -1);
	if (!r) {
		ESP_LOGE(TAG, "Chip reset reg error for addr 0x%X", addr);
		goto err;
	}

	ESP_LOGI(TAG, "Device at 0x%X initialized, ID=%X.", addr, id);

	return ret;
err:
	free(ret);
	return NULL;
}


void bh1730_free(bh1730_t *d) {
	free(d);
}

