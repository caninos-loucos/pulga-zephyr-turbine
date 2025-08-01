#include <zephyr/logging/log.h>

#include <zephyr/drivers/sensor.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
//#include <stdlib.h>

//#include <zephyr/rtio/rtio.h>
//#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>


// change log level in debug.conf
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct device *bmi160;

typedef struct
{
    struct sensor_value acceleration[3];
    struct sensor_value rotation[3];
    uint32_t timestamp;
} SensorModelBMI160;

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

static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static void htmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	simulate_htm = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params, uint8_t err)
{
	printk("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	printk("Indication complete\n");
	indicating = 0U;
}

/* Health Thermometer Service Declaration */
BT_GATT_SERVICE_DEFINE(hts_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_MEASUREMENT, BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(htmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void hts_indicate(void)
{
	if (simulate_htm) {
		static uint8_t htm[5];
		static double temperature = 20U;
		uint32_t mantissa;
		uint8_t exponent;
		int r;

		if (indicating) {
			return;
		}

		if (!temp_dev) {
			temperature++;
			if (temperature == 30U) {
				temperature = 20U;
			}

			goto gatt_indicate;
		}

		r = sensor_sample_fetch(temp_dev);
		if (r) {
			printk("sensor_sample_fetch failed return: %d\n", r);
		}

		r = sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP,
				       &temp_value);
		if (r) {
			printk("sensor_channel_get failed return: %d\n", r);
		}

		temperature = sensor_value_to_double(&temp_value);

gatt_indicate:
		printf("temperature is %gC\n", temperature);

		mantissa = (uint32_t)(temperature * 100);
		exponent = (uint8_t)-2;

		htm[0] = 0; /* temperature in celsius */
		sys_put_le24(mantissa, (uint8_t *)&htm[1]);
		htm[4] = exponent;

		ind_params.attr = &hts_svc.attrs[2];
		ind_params.func = indicate_cb;
		ind_params.destroy = indicate_destroy;
		ind_params.data = &htm;
		ind_params.len = sizeof(htm);

		if (bt_gatt_indicate(NULL, &ind_params) == 0) {
			indicating = 1U;
		}
	}
}



static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HTS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

int main(void)
{
    if(init_bmi160()){
        while(1){
            k_busy_wait(1e6);
            LOG_ERR("deu ruim init bmi160");
        }
    }

    int err = bt_enable(NULL);
    if (err) {
		while(1){
            k_busy_wait(1e6);
            LOG_ERR("Bluetooth init failed (err %d)\n", err);
        }
	}

    bt_ready();

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

            hts_indicate();

            /* Battery level simulation */
            bas_notify();
        }*/

        /*for (int i = 0; i < MAX_READINGS; i++)
        {
            LOG_DBG("fetch sample %d a %d a %d",
            readings_buffer[i].X_axis,
            readings_buffer[i].X_axis,
            readings_buffer[i].X_axis
        }*/
    }

while(1){
        k_busy_wait(1e6);
        LOG_ERR("deu ruim read");

    }


}