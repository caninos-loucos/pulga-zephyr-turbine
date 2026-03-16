#include <zephyr/logging/log.h>

#include <zephyr/drivers/sensor.h>

//#include <zephyr/types.h>
//#include <stddef.h>
//#include <string.h>
//#include <errno.h>
//#include <stdio.h>
//#include <stdlib.h>

#include <zephyr/rtio/rtio.h>
//#include <zephyr/sys/util.h>
//#include <zephyr/sys/printk.h>
//#include <zephyr/sys/byteorder.h>
//#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>

#include <../../../../zephyr/drivers/sensor/bosch/bmi323/bmi323.h>
#include <../../../../zephyr/drivers/sensor/bosch/bmi323/bmi323_i2c.h>

// change log level in debug.conf
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct device *bmi323;

#define BMI323_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bmi323)

/* cria automaticamente o RTIO iodev */
SENSOR_DT_READ_IODEV(bmi323_iodev, BMI323_NODE, SENSOR_CHAN_ACCEL_XYZ);

RTIO_DEFINE(rtio_ctx, 4, 4);

/* === BUFFERS === */
#define FIFO_BUFFER_SIZE 1024 //1024
static uint8_t fifo_buffer[FIFO_BUFFER_SIZE];

uint8_t read_buffer[2048];

typedef struct
{
    struct sensor_value acceleration[3];
    struct sensor_value rotation[3];
    uint32_t timestamp;
} SensorModelBMI323;

typedef struct
{
    int16_t X_axis;
    int16_t Y_axis;
    int16_t Z_axis;
} leitura;

#define MAX_READINGS 6400 //6400

leitura readings_buffer[MAX_READINGS];
int write_index;

int64_t time_aux_main;

static int init_bmi323()
{
    LOG_DBG("Initializing BMI323");
    bmi323 = DEVICE_DT_GET_ANY(bosch_bmi323);

    if (!bmi323)
    {
        LOG_ERR("bmi323 not declared at device tree");
        return -ENODEV;
    }
    else if (!device_is_ready(bmi323))
    {
        LOG_ERR("device \"%s\" is not ready", bmi323->name);
        return -EAGAIN;
    }

   k_usleep(100);

    bosch_bmi323_set_fifo_acc(bmi323, 160);
    k_usleep(100);

    return 0;
}

static int config_bmi323(void)
{
    int ret;
    struct sensor_value v;
    
    printf("Config BMI323\n");
    //k_msleep(50);

    /* =========================
     * ACCELERÔMETRO
     * ========================= */

	/* Disnable ACC */
	v.val1 = 0;				//bmi_dev
	ret = sensor_attr_set(bmi323,
			      SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_FEATURE_MASK,
			      &v);
	if (ret) return ret;

    /* 1. ODR = 100 Hz */
    v.val1 = 6400;
    v.val2 = 0;
    ret = sensor_attr_set(bmi323,
                          SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY,
                          &v);
    if (ret) {
        LOG_ERR("ACC ODR failed: %d", ret);
        return ret;
    }

    /* 2. Range = ±4g */
    v.val1 = 4;
    v.val2 = 0;
    ret = sensor_attr_set(bmi323,
                          SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_FULL_SCALE,
                          &v);
    if (ret) {
        LOG_ERR("ACC range failed: %d", ret);
        return ret;
    }

    /* 3. ENABLE ACC (isso tira do DIS!) */
    v.val1 = 1;   /* NÃO use 1 */
    v.val2 = 0;
    ret = sensor_attr_set(bmi323,
                          SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_FEATURE_MASK,
                          &v);
    if (ret) {
        LOG_ERR("ACC enable failed: %d", ret);
        return ret;
    }

    // /* FIFO watermark */
    // v.val1 = 20;   /* 20 frames */
    // v.val2 = 0;    /* não parar quando cheia */

    // sensor_attr_set(bmi323,
    //                 SENSOR_CHAN_ACCEL_XYZ,
    //                 SENSOR_ATTR_BMI323_FIFO_WATERMARK,
    //                 &v);

    // printf("FIFO configurada\n");

    bmi323_fifo_enable_acc(bmi323);

    k_msleep(50);

    uint16_t verify;
    //int ret2;

    ret = bmi323_read_reg(bmi323,
                        IMU_BOSCH_BMI323_REG_ACC_CONF,
                        &verify);

    if (!ret) {
        printf("ACC_ODR_CONF = 0x%04X\n", verify);
    } else {
        printf("Read error: %d\n", ret);
    }

    /* =========================
     * STARTUP DELAY
     * ========================= */

    /* Aguarda pelo menos 2 períodos de ODR (100 Hz -> 20 ms) */
    k_msleep(30);

    return 0;
}

int main(void)
{
    printf("BMI323 FIFO TEST\n");

    if (init_bmi323()) {
        printf("Init failed\n");
        return 0;
    }

    if (config_bmi323()) {
        printf("Config failed\n");
        return 0;
    }

    //printf("Flush FIFO\n");

    bosch_bmi323_fifo_flush(bmi323);

    // /* pegar decoder uma vez */
    // const struct sensor_decoder_api *decoder;

    // if (sensor_get_decoder(bmi323, &decoder) != 0) {
    //     printk("Decoder error\n");
    //     return 0;
    // }

    uint32_t sample_count = 0;
    int64_t start_time = k_uptime_get();

     while (1) {

        printf("Loop principal\n");

        uint16_t bytes_lidos = 0;

        /* =========================
        Ler FIFO via driver
        ========================= */

        int ret = bosch_bmi323_fifo_read_acc(
                        bmi323,
                        fifo_buffer,
                        FIFO_BUFFER_SIZE,
                        &bytes_lidos);

        if (ret < 0) {
            printf("Erro lendo FIFO\n");
            k_msleep(10);
            continue;
        }

        if (bytes_lidos == 0) {
            printf("FIFO vazia\n");
            k_msleep(10);
            continue;
        }

        printf("FIFO lida: %u bytes\n", bytes_lidos);

        /* =========================
        Interpretar ACC
        ========================= */

        for (int i = 0; i < bytes_lidos; i += 6) {

            int16_t x = sys_get_le16(&fifo_buffer[i]);
            int16_t y = sys_get_le16(&fifo_buffer[i + 2]);
            int16_t z = sys_get_le16(&fifo_buffer[i + 4]);

            printf("ACC X=%d Y=%d Z=%d\n", x, y, z);

            sample_count++;
        }
        //k_msleep(500);
        k_usleep(100);
    }
}