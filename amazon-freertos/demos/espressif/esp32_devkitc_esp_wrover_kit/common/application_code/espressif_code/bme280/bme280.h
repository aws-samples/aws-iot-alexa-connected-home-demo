/* 
This code is in the Public Domain (or CC0 licensed, at your option.)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef BME280_H
#define BME280_H

typedef struct bme280_t bme280_t;

bme280_t *bme280_init(i2c_port_t port, int addr);
int bme280_read(bme280_t *d, int32_t *temp, uint32_t *press, int32_t *hum);
void bme280_free(bme280_t *d);

#endif
