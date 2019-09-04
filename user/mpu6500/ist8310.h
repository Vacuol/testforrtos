
#ifndef __IST8310__
#define __IST8310__

#include "main.h"

#define IST8310_IIC_ADDRESS 0x0E  //IST8310��IIC��ַ
#define IST8310_IIC_READ_MSB 0x80 //IST8310��SPI��ȡ���͵�һ��bitΪ1

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  float mag[3];
} ist8310_real_data_t;

extern ist8310_real_data_t ist8310_data;

extern void ist8310_com_init(void);  //ist8310��ͨѶ��ʼ��
extern void ist8310_auto_com_by_mpu6500(void);
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void); //��λIO �ø�
extern void ist8310_RST_L(void); //��λIO �õ� �õػ�����ist8310����

extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
extern void ist8310_read_mag(ist8310_real_data_t *ist8310_real_data);


#endif
