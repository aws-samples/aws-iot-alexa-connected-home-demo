/* 
This code is in the Public Domain (or CC0 licensed, at your option.)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef BH1730_H
#define BH1730_H

typedef struct bh1730_t bh1730_t;

bh1730_t *bh1730_init(i2c_port_t port, int addr);
float bh1730_read_lux(bh1730_t *d);
void bh1730_free(bh1730_t *d);

#endif
