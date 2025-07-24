#include <zephyr/logging/log.h>

#include <sensors/sensors_interface.h>
#include <integration/data_abstraction/abstraction_service.h>
#include <communication/comm_interface.h>
#include <sensors/bmi160/bmi160_service.h>

// change log level in debug.conf
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct device *bmi160;

typedef struct
{
    int16_t X_axis;
    int16_t Y_axis;
    int16_t Z_axis;
} leitura;

#define MAX_READINGS 100 //1600

leitura readings_buffer[MAX_READINGS];
int write_index;

int64_t time_aux_main;

static int init_bmi160()
{
    LOG_DBG("Initializing BMI160");
    bmi160 = DEVICE_DT_GET_ANY(bosch_bmi160);

    if (!bmi160)
    {
        LOG_ERR("bmi160 not declared at device tree");
        return -ENODEV;
    }
    else if (!device_is_ready(bmi160))
    {
        LOG_ERR("device \"%s\" is not ready", bmi160->name);
        return -EAGAIN;
    }
    return 0;
}

int main(void)
{
    if(init_bmi160()){
        while(1){
            k_busy_wait(1000);
            LOG_ERR("deu ruim init");
    
        }
    }
    /*struct sensor_value bmi160_sample_rate;
    sensor_attr_get(bmi160, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &bmi160_sample_rate);
    LOG_DBG("BMI160 SR %d.%d", bmi160_sample_rate.val1, bmi160_sample_rate.val2);
    bmi160_sample_rate.val1 = 1600; bmi160_sample_rate.val2 = 0;
    if(sensor_attr_set(bmi160, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &bmi160_sample_rate)){
        while(1){
            k_busy_wait(1000);
            LOG_ERR("deu ruim odr");
    
        }
    }
    sensor_attr_get(bmi160, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &bmi160_sample_rate);
    LOG_DBG("BMI160 SR %d.%d", bmi160_sample_rate.val1, bmi160_sample_rate.val2);*/

    LOG_DBG("Reading BMI160");

    SensorModelBMI160 bmi160_model;

    write_index = 0;
    time_aux_main = k_uptime_get();
    while(1){
    int error = 0;

sample_fetch:
    error = sensor_sample_fetch(bmi160);
    if (!error)
    {
        sensor_channel_get(bmi160, SENSOR_CHAN_ACCEL_XYZ,
                           bmi160_model.acceleration);
    }
    else if (error == -EAGAIN)
    {
        LOG_WRN("fetch sample from \"%s\" failed: %d, trying again",
                bmi160->name, error);
        goto sample_fetch;
    }
    else
    {
        LOG_ERR("fetch sample from \"%s\" failed: %d",
                bmi160->name, error);
        goto sample_fetch;
    }

    readings_buffer[write_index].X_axis = bmi160_model.acceleration[0].val1 * 1e6 + bmi160_model.acceleration[0].val2;
    readings_buffer[write_index].Y_axis = bmi160_model.acceleration[1].val1 * 1e6 + bmi160_model.acceleration[1].val2;
    readings_buffer[write_index].Z_axis = bmi160_model.acceleration[2].val1 * 1e6 + bmi160_model.acceleration[2].val2;

    write_index++;

    if(write_index >= MAX_READINGS){
        write_index = 0;
        LOG_DBG("sample: %lld", (100000/(k_uptime_get()-time_aux_main)));
        time_aux_main = k_uptime_get();
    }

    /*LOG_DBG("fetch sample %d.%d a %d.%d a %d.%d",
		bmi160_model.acceleration[0].val1,
		bmi160_model.acceleration[0].val2 / 10000,
		bmi160_model.acceleration[1].val1,
		bmi160_model.acceleration[1].val2 / 10000,
		bmi160_model.acceleration[2].val1,
		bmi160_model.acceleration[2].val2 / 10000 );*/
}

while(1){
        k_busy_wait(1000);
        LOG_ERR("deu ruim read");

    }


}
