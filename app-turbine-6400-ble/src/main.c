#include <zephyr/logging/log.h>

#include <zephyr/drivers/sensor.h>

//#include <zephyr/types.h>
#include <zephyr/sys/util.h>
//#include <stddef.h>
//#include <string.h>
//#include <errno.h>

//#include <assert.h>
//#include <stdio.h>
//#include <stdlib.h>

//#include <zephyr/sys/printk.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
//#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>

//#include <zephyr/drivers/sensor/bosch/bmi323/bmi323.h>
//#include <zephyr/drivers/sensor/bosch/bmi323/bmi323_i2c.h>
#include <../../../zephyr/drivers/sensor/bosch/bmi323/bmi323.h>
#include <../../../zephyr/drivers/sensor/bosch/bmi323/bmi323_i2c.h>


// change log level in debug.conf
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* Variable declarations */
static const struct device *bmi323;

#define BMI323_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bmi323)

int8_t rslt;

typedef struct
{
    int16_t X_axis;
    int16_t Y_axis;
    int16_t Z_axis;
} bmi_readings;

#define MAX_READINGS 6400 //6400

bmi_readings readings_buffer[MAX_READINGS];
int write_index;

/* accel params and conversion constants */
// #define AC 2048.0F // for 16G
// #define AC 4096.0fF // for 8G
#define AC 8192.0F // for 4G
// #define AC 16384.0F // for 2G

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

   k_msleep(100);

    bosch_bmi323_set_fifo_acc(bmi323, 160);
    k_msleep(100);

    return 0;
}

static int config_bmi323(void)
{
    int ret;
    struct sensor_value v;
    
    LOG_DBG("Config BMI323\n");
    k_msleep(50);

	v.val1 = 0;
	ret = sensor_attr_set(bmi323,
			      SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_FEATURE_MASK,
			      &v);
	if (ret) {
        LOG_ERR("ACC disable (before config) failed: %d", ret);
        return ret;
    }

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

    v.val1 = 1;
    v.val2 = 0;
    ret = sensor_attr_set(bmi323,
                          SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_FEATURE_MASK,
                          &v);
    if (ret) {
        LOG_ERR("ACC enable failed: %d", ret);
        return ret;
    }

    bmi323_fifo_enable_acc(bmi323);

    k_msleep(50);

    uint16_t verify;
    //int ret2;

    ret = bmi323_read_reg(bmi323,
                        IMU_BOSCH_BMI323_REG_ACC_CONF,
                        &verify);

    if (!ret) {
        LOG_DBG("ACC_ODR_CONF = 0x%04X\n", verify);
    } else {
        LOG_ERR("Read error for bmi323 acc conf reg: %d\n", ret);
    }

    // startup delay, at least 2 ODR periods (6400 Hz -> 0.3125 ms) */
    k_msleep(10);

    return 0;
}

static uint8_t bt_connected, bt_advertising;
static uint8_t indicating, indicate_htm;
static struct bt_gatt_indicate_params ind_params;

#define READINGS_PER_INDICATE 40

uint16_t indicate_index;

uint32_t t1, t2, t3;

static void htmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	indicate_htm = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
    LOG_DBG("ccc_indicate %s\n", indicate_htm == 0U ? "disabled" : "enabled");
}

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params, uint8_t err)
{
	if(err != 0U) LOG_ERR("Indication fail\n\r");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	//LOG_DBG("Indication complete\n");
	indicating = 0U;
    indicate_index++;
}

#define TURBINE_SVC_UUID                 BT_UUID_128_ENCODE(0x02b454b7, 0xc19f, 0x4d1c, 0xa2c0, 0xb7fc10f8a8a3)
#define TURBINE_CHR_IND_ACCEL_UUID      BT_UUID_128_ENCODE(0x682d75cf, 0x44fc, 0x4df4, 0xac81, 0x00f02aa9b98a)

static const struct bt_uuid_128 turb_svc = BT_UUID_INIT_128(TURBINE_SVC_UUID);
static const struct bt_uuid_128 turb_char_ind_accel_uuid = BT_UUID_INIT_128(TURBINE_CHR_IND_ACCEL_UUID);


/* BLE Service Declaration */
BT_GATT_SERVICE_DEFINE(hts_svc,
	BT_GATT_PRIMARY_SERVICE(&turb_svc),
	BT_GATT_CHARACTERISTIC(&turb_char_ind_accel_uuid.uuid, BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(htmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void hts_indicate(void)
{
    while(bt_connected && indicate_htm && indicate_index < MAX_READINGS/READINGS_PER_INDICATE){

        //LOG_DBG("Indication: %d\n", indicate_index);

        static uint8_t indicate_payload[1+6*READINGS_PER_INDICATE];

        indicate_payload[0] = indicate_index;
        
        memcpy(indicate_payload+1, &readings_buffer[indicate_index*READINGS_PER_INDICATE], 6*READINGS_PER_INDICATE ); // 2 bytes x 3 axis x readings

        ind_params.attr = &hts_svc.attrs[2];
        ind_params.func = indicate_cb;
        ind_params.destroy = indicate_destroy;
        ind_params.data = &indicate_payload;
        ind_params.len = sizeof(indicate_payload);

        if (bt_gatt_indicate(NULL, &ind_params) == 0) {
            indicating = 1U;
        }
        else LOG_ERR("Indication fail\n\r");

        while(indicating) {
            k_sleep(K_USEC(100));
        }
    }
    indicate_index = 0;
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	LOG_DBG("%s: MTU exchange %s (%u)\n", __func__, err == 0U ? "successful" : "failed", bt_gatt_get_mtu(conn));
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
	.func = mtu_exchange_cb
};

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
        bt_le_adv_stop();
	} else {
		LOG_DBG("Connected\n");

        bt_connected = 1;
        
        LOG_DBG("%s: Current MTU = %u\n", __func__, bt_gatt_get_mtu(conn));
        
        LOG_DBG("%s: Will try to exchange MTU...\n", __func__);
	    err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	    if (err) {
		    LOG_ERR("%s: MTU exchange failed (err %d)", __func__, err);
	    }
    }
    bt_advertising = 0;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    bt_connected = 0;
	LOG_DBG("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

    bt_advertising = 1;

	LOG_DBG("Advertising successfully started\n");
}

int8_t acquire_ACC_Values(void)
{
    uint16_t words_read;
    while(write_index < MAX_READINGS){
        
        words_read = 0;

        // Read FIFO via driver

        int ret = bosch_bmi323_fifo_read_acc(
                        bmi323,
                        (uint8_t *)&readings_buffer[write_index],
                        3*(MAX_READINGS-write_index),
                        &words_read);

        if (ret < 0) {
            LOG_ERR("Error reading FIFO\n");
            k_msleep(200);
            return ret;
        }

        if (words_read == 0) {
            LOG_DBG("FIFO empty\n");
            k_msleep(100);
            continue;
        }

        write_index += words_read/3;
    }
    return 0;
}

void direct_read(void)
{
    for (int i = 0; i < MAX_READINGS; i++)
    {
        LOG_DBG("%2.6f %2.6f %2.6f \n\r", (readings_buffer[i].X_axis / AC)
                                     , (readings_buffer[i].Y_axis / AC)
                                     , (readings_buffer[i].Z_axis / AC));
    }
}

int main(void)
{
    k_sleep(K_MSEC(1000));

    LOG_DBG("BMI323 FIFO TEST\n");

    rslt = init_bmi323();
    if (rslt) {
        LOG_ERR("Error init BMI160 - %d\n \r", rslt);
        goto out_error;
    }

    rslt = config_bmi323();
    if (rslt) {
        LOG_ERR("Error config BMI160 - %d\n \r", rslt);
        goto out_error;
    }

    rslt = bt_enable(NULL);

    if (rslt)
        {
            LOG_ERR("Bluetooth init failed (err %d)\n \r", rslt);
            goto out_error;
        }

    LOG_DBG("Bluetooth initialized\n");

    bt_ready();
    
    k_sleep(K_MSEC(500));

    LOG_DBG("Reading BMI160 \n \r");

    while (1)
    {
        write_index = 0;
        rslt = bosch_bmi323_fifo_flush(bmi323);
        if (rslt)
        {
            LOG_ERR("Error flushing BMI160 FIFO - %d\n \r", rslt);
            goto out_error;
        }
        //LOG_DBG("flush ok");
        t1 = k_uptime_get();
        rslt = acquire_ACC_Values();
        t2 = k_uptime_get();
        //direct_read();
        hts_indicate();
        t3 = k_uptime_get();

        LOG_DBG("time acq: %d \n\r", t2 - t1);
        LOG_DBG("sample rate acq: %d \n\r", MAX_READINGS*1000/(t2 - t1));
        LOG_DBG("time indicate: %d \n\r", t3 - t2);

        if(!bt_connected && !bt_advertising)
        {
            bt_ready(); //re-enable advertising after disconnect
        }
    }
out_error:
    while (1)
    {
    LOG_ERR("System error, will reset pulga \n \r");
    k_sleep(K_MSEC(5000));
    sys_reboot(1);
    }
}