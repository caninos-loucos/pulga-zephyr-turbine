#include <zephyr/logging/log.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>


// change log level in debug.conf
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include "bmi160.h"
//#include "bmx280_params.h"
//#include "bmx280.h"

/* Macros for frames to be read */

#define ACC_FRAMES 50 /* 40 Frames are available every 25ms @ 1600 Hz */
/* 50 frames containing a 1 byte header, 6 bytes of accelerometer,
 * This results in 7 bytes per frame*/
#define FIFO_SIZE 350

/* Variable declarations */
struct bmi160_dev bmi;

uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_sensor_data accel_data[ACC_FRAMES];

int8_t rslt;

typedef struct
{
    int16_t X_axis;
    int16_t Y_axis;
    int16_t Z_axis;
} leitura;

#define MAX_READINGS 1600

leitura readings_buffer[MAX_READINGS];
int write_index;

int8_t init_bmiSensor(void);
void direct_read(void);
int8_t acquire_ACC_Values(void);

static const struct device *dev;

union bmi160_bus {
	struct i2c_dt_spec i2c;
};

typedef bool (*bmi160_bus_ready_fn)(const struct device *dev);
typedef int (*bmi160_reg_read_fn)(const struct device *dev,
				  uint8_t reg_addr, void *data, uint8_t len);
typedef int (*bmi160_reg_write_fn)(const struct device *dev,
				   uint8_t reg_addr, void *data, uint8_t len);

struct bmi160_bus_io {
	bmi160_bus_ready_fn ready;
	bmi160_reg_read_fn read;
	bmi160_reg_write_fn write;
};

struct bmi160_cfg_i2c {
	union bmi160_bus bus;
	const struct bmi160_bus_io *bus_io;
};

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // LOG_DBG("i2c_read_regs(%d, %x, %x, ...)\n", dev, dev_addr, reg_addr);
    const struct bmi160_cfg_i2c *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->bus.i2c, reg_addr, data, len);
}

int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // LOG_DBG("i2c_write_regs(%d, %x, %x, ...)\n", dev, dev_addr, reg_addr);
    const struct bmi160_cfg_i2c *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->bus.i2c, reg_addr, data, len);
}

void user_delay(uint32_t period)
{
    k_sleep(K_MSEC(period));
}

/* accel params and conversion constants */
// #define AC (float)2048.0 // for 16G
#define AC (float)8192.0 // for 4G
// #define AC (float)16384.0 // for 2G

#define STR_ANSWER_BUFFER_SIZE 4096

uint32_t t1, t2;

int main(void)
{
    k_sleep(K_MSEC(1000));
    
    rslt = init_bmiSensor();

    if (rslt != BMI160_OK)
        {
            LOG_DBG("Error init BMI160 - %d\n \r", rslt);
            goto out_error;
        }
    
    k_sleep(K_MSEC(500));

    while (1)
    {
        write_index = 0;
        rslt = bmi160_set_fifo_flush(&bmi);
        if (rslt != BMI160_OK)
        {
            LOG_DBG("Error flushing BMI160 FIFO - %d\n \r", rslt);
            goto out_error;
        }
        LOG_DBG("flush ok");
        t1 = k_uptime_get();
        rslt = acquire_ACC_Values();
        t2 = k_uptime_get();
        direct_read();
        printf("time: %d \n\r", t2 - t1);
    }
out_error:
    while (1)
    {
    LOG_DBG("Error \n \r");
    k_sleep(K_MSEC(500));
    }
}

int8_t init_bmiSensor(void)
{
    LOG_DBG("Initializing BMI160");
    dev = DEVICE_DT_GET_ANY(bosch_bmi160);

    if (!dev)
    {
        LOG_ERR("bmi160 not declared at device tree");
        return -ENODEV;
    }
    else if (!device_is_ready(dev))
    {
        LOG_ERR("device \"%s\" is not ready", dev->name);
        return -EAGAIN;
    }


    k_sleep(K_MSEC(500));

    /* Initialize I2C as the host interface to the BMI160 */

    bmi.id = BMI160_I2C_ADDR;
    bmi.read = user_i2c_read;
    bmi.write = user_i2c_write;
    bmi.delay_ms = user_delay;
    bmi.intf = BMI160_I2C_INTF;

    rslt = bmi160_init_i2c(&bmi);
    if (rslt == BMI160_OK)
    {
        LOG_DBG("Success initializing BMI160 - Chip ID 0x%X\n \r", bmi.chip_id);
    }
    else if (rslt == BMI160_E_DEV_NOT_FOUND)
    {
        LOG_DBG("Error initializing BMI160: device not found\n \r");
        return rslt;
    }
    else
    {
        LOG_DBG("Error initializing BMI160 - %d\n \r", rslt);
        return rslt;
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    // bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;

    /* Normal power mode for accelerometer */
    bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Suspend power mode for Gyroscope */
    bmi.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi);
    if (rslt != BMI160_OK)
    {
        LOG_DBG("Error configuring BMI160 - %d\n \r", rslt);
        return rslt;
    }
    
    LOG_DBG("config \n \r");
    
    /* Link the FIFO memory location */
    fifo_frame.data = fifo_buff;
    fifo_frame.length = FIFO_SIZE;
    bmi.fifo = &fifo_frame;

    /* Clear all existing FIFO configurations */
    rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK, BMI160_DISABLE, &bmi);
    if (rslt != BMI160_OK)
    {
        LOG_DBG("Error clearing fifo - %d\n \r", rslt);
        return rslt;
    }
    LOG_DBG("clear \n \r");
    uint8_t fifo_config = BMI160_FIFO_HEADER | BMI160_FIFO_ACCEL;
    rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
    if (rslt != BMI160_OK)
    {
        LOG_DBG("Error enabling fifo - %d\n \r", rslt);
        return rslt;
    }
    LOG_DBG("enable \n \r");

    return BMI160_OK;
}

int8_t acquire_ACC_Values(void)
{
    while(write_index < MAX_READINGS){
        /* It is VERY important to reload the length of the FIFO memory as after the
         * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
         * number of bytes read from the FIFO */
        bmi.fifo->length = FIFO_SIZE;
        
        rslt = bmi160_get_fifo_data(&bmi);
        if (rslt != BMI160_OK)
        {
            LOG_DBG("Error getting fifo data - %d\n \r", rslt);
            return rslt;
        }

        uint8_t acc_inst = ACC_FRAMES;
        rslt = bmi160_extract_accel(accel_data, &acc_inst, &bmi);
        if (rslt != BMI160_OK)
        {
            LOG_DBG("Error extracting accel data - %d\n \r", rslt);
            return rslt;
        }
        
        
        for (int j = 0; j < acc_inst && write_index < MAX_READINGS; j++)
        {
            readings_buffer[write_index].X_axis = accel_data[j].x;
            readings_buffer[write_index].Y_axis = accel_data[j].y;
            readings_buffer[write_index].Z_axis = accel_data[j].z;
            write_index++; // increment write index
        }
    }

    return BMI160_OK;
}

void direct_read(void)
{
    for (int i = 0; i < MAX_READINGS; i++)
    {
        printf("%2.6f %2.6f %2.6f \n\r", ((float)readings_buffer[i].X_axis) / AC
                                     , ((float)readings_buffer[i].Y_axis) / AC
                                     , ((float)readings_buffer[i].Z_axis) / AC);
    }
}