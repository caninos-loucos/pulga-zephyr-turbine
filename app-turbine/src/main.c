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

static uint8_t indicate_htm;
static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;

#define READINGS_PER_INDICATE 4

int indicate_index;

static void htmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	indicate_htm = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
    LOG_DBG("ccc_indicate %s\n", indicate_htm == 0U ? "disabled" : "enabled");
}

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params, uint8_t err)
{
	LOG_DBG("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	LOG_DBG("Indication complete\n");
	indicating = 0U;
    indicate_index++;
}

#define TURBINE_SVC_UUID                 BT_UUID_128_ENCODE(0x02b454b7, 0xc19f, 0x4d1c, 0xa2c0, 0xb7fc10f8a8a3)
#define TURBINE_CHR_IND_ACCEL_UUID      BT_UUID_128_ENCODE(0x682d75cf, 0x44fc, 0x4df4, 0xac81, 0x00f02aa9b98a)

static const struct bt_uuid_128 turb_svc = BT_UUID_INIT_128(TURBINE_SVC_UUID);
static const struct bt_uuid_128 turb_char_ind_accel_uuid = BT_UUID_INIT_128(TURBINE_CHR_IND_ACCEL_UUID);


/* Health Thermometer Service Declaration */
BT_GATT_SERVICE_DEFINE(hts_svc,
	BT_GATT_PRIMARY_SERVICE(&turb_svc),
	BT_GATT_CHARACTERISTIC(&turb_char_ind_accel_uuid, BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(htmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void hts_indicate(void)
{
    if(indicate_htm){
        while(indicate_index < MAX_READINGS/READINGS_PER_INDICATE){
            
            if (indicating) {
                k_sleep(K_MSEC(60));;
            }

            static uint8_t indicate_payload[1+6*READINGS_PER_INDICATE];

            indicate_payload[0] = indicate_index;
            
            for(int i = 0; i < READINGS_PER_INDICATE; i++){
                indicate_payload[1 + 6*i] = readings_buffer[indicate_index*READINGS_PER_INDICATE + i].X_axis;
                indicate_payload[3 + 6*i] = readings_buffer[indicate_index*READINGS_PER_INDICATE + i].Y_axis;
                indicate_payload[5 + 6*i] = readings_buffer[indicate_index*READINGS_PER_INDICATE + i].Z_axis;
            }

            ind_params.attr = &hts_svc.attrs[2];
            ind_params.func = indicate_cb;
            ind_params.destroy = indicate_destroy;
            ind_params.data = &indicate_payload;
            ind_params.len = sizeof(indicate_payload);

            if (bt_gatt_indicate(NULL, &ind_params) == 0) {
                indicating = 1U;
            }
        }
        indicate_index = 0;
    }
}



static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HTS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		LOG_DBG("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_DBG("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	LOG_DBG("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

	LOG_DBG("Advertising successfully started\n");
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

    //LOG_DBG("BLE MTU: %d", BLE_ATT_MTU_MAX);

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

        readings_buffer[write_index].X_axis = bmi160_model.acceleration[0].val1 * 1e3 + bmi160_model.acceleration[0].val2 / 1e3;
        readings_buffer[write_index].Y_axis = bmi160_model.acceleration[1].val1 * 1e3 + bmi160_model.acceleration[1].val2 / 1e3;
        readings_buffer[write_index].Z_axis = bmi160_model.acceleration[2].val1 * 1e3 + bmi160_model.acceleration[2].val2 / 1e3;

        write_index++;

        if(write_index >= MAX_READINGS){
            write_index = 0;
            LOG_DBG("sample: %lld", (100000/(k_uptime_get()-time_aux_main)));
            
            time_aux_main = k_uptime_get();

            hts_indicate();
        }

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