#include <zephyr/logging/log.h>

#include <sensors/sensors_interface.h>
#include <integration/data_abstraction/abstraction_service.h>
#include <communication/comm_interface.h>
#include <sensors/bmi160/bmi160_service.h>

// change log level in debug.conf
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct device *bmi160;

int main(void)
{
    LOG_DBG("Initializing BMI160");
    bmi160 = DEVICE_DT_GET_ANY(bosch_bmi160);

    k_busy_wait(1000000);
	// Removes sensor API from registered APIs if cannot start sensor
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
    while(1){
        k_busy_wait(1000);
        LOG_ERR("deu ruim init");

    }

    LOG_DBG("Reading BMI160");

    SensorModelBMI160 bmi160_model;
    uint32_t bmi160_data[MAX_32_WORDS];
    while(1){
    int error = 0;

sample_fetch:
    error = sensor_sample_fetch(bmi160);
    if (!error)
    {
        sensor_channel_get(bmi160, SENSOR_CHAN_ACCEL_XYZ,
                           bmi160_model.acceleration);
        sensor_channel_get(bmi160, SENSOR_CHAN_GYRO_XYZ,
                           bmi160_model.rotation);
        memcpy(&bmi160_data, &bmi160_model, sizeof(SensorModelBMI160));
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
    }
    LOG_DBG("fetch sample %d.%d a %d.%d a %d.%d",
		bmi160_model.acceleration[0].val1,
		bmi160_model.acceleration[0].val2 / 10000,
		bmi160_model.acceleration[1].val1,
		bmi160_model.acceleration[1].val2 / 10000,
		bmi160_model.acceleration[2].val1,
		bmi160_model.acceleration[2].val2 / 10000 );
}

while(1){
        k_busy_wait(1000);
        LOG_ERR("deu ruim read");

    }


}
