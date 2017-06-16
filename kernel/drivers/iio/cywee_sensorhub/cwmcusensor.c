#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h> /* BUS_I2C */
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>

#include "cwmcusensor.h"
#include "linux/iio/buffer.h"
#include "linux/iio/iio.h"
#include "linux/iio/sysfs.h"
#include "linux/iio/trigger.h"
#include "linux/iio/trigger_consumer.h"
#include "linux/iio/kfifo_buf.h"
#include <linux/irq_work.h>


#define MAIN_VERSION  					"3.00_1512"	
#define SENSOR_HUB_TAG                  "CWM:"
//#define DEBUG                           1
#if defined(DEBUG)
#define SH_FUN(f)                       printk(KERN_INFO SENSOR_HUB_TAG"%s\n", __FUNCTION__)
#define SH_ERR(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_LOG(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_DBG(fmt, args...)            printk(KERN_INFO SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#else
#define SH_FUN(f)
#define SH_ERR(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_LOG(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_DBG(fmt, args...)
#endif

/* for N5 define */
#if 1
#define GPIO_CW_MCU_WAKE_UP		67
#define GPIO_CW_MCU_BOOT		139
#define GPIO_CW_MCU_RESET		140
#define GPIO_CW_MCU_INTERRUPT	73
#else
/* for N5 NFC define */
#define GPIO_CW_MCU_WAKE_UP		92
#define GPIO_CW_MCU_BOOT		32
#define GPIO_CW_MCU_RESET		94
#define GPIO_CW_MCU_INTERRUPT	59
#endif

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000
#define FT_SENSORS_VTG_MIN_UV	2850000
#define FT_SENSORS_VTG_MAX_UV	2850000

/* GPIO for MCU control */
#define QueueSystemInfoMsgSize		30
#define QueueSystemInfoMsgBuffSize      QueueSystemInfoMsgSize*5

#define ACK			0x79
#define NACK		0x1F

#define DPS_MAX			(1 << (16 - 1))

/* Input poll interval in milliseconds */

#define CWMCU_POLL_MAX      2000

#define CFG_MAX_TOUCH_POINTS	5
#define ST_PRESS		0x7F
#define ST_CFG_X_RESOLUTION	1080
#define ST_CFG_Y_RESOLUTION	1920

/* turn on gpio interrupt if defined */
#define CWM_USE_IRQ_WORK
//#define CWM_USE_DELAY_WORK
#define CWM_USE_ERROR_HANDLE_WORK

#ifdef CWM_USE_ERROR_HANDLE_WORK
#define CWM_LEVEL0_MCU_NON    0
#define CWM_LEVEL1_MCU_RESET    1
#define CWM_LEVEL2_MCU_RESET    2
#define CWM_LEVEL3_MCU_STOP    3
#define CWM_MAX_ERROR_WORK_TIME    10000
#endif

#define CWMCU_CALIB_SAVE_IN_FLASH

#define SUPPORT_HALL_SENSOR
#define SUPPORT_INPUT

struct CWMCU_T {
	struct i2c_client *client;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
   struct regulator *vcc_sensors;
	struct workqueue_struct *driver_wq;
#ifdef CWM_USE_IRQ_WORK
	struct work_struct work;
#endif
#ifdef CWM_USE_DELAY_WORK
	struct delayed_work	delay_work;
#endif
#ifdef CWM_USE_ERROR_HANDLE_WORK
	struct delayed_work error_handle_work;
#endif
	struct input_dev *input;
	struct CWMCU_SENSORS_INFO sensors_info[HANDLE_ID_END][SENSORS_ID_END];
	SensorsInit_T	hw_info[DRIVER_ID_END];
	RegInformation *pReadRegInfo;
	RegInformation *pWriteRegInfo;
	u8 m_cReadRegCount;
	u8 m_cWriteRegCount;

	int mcu_mode;
	uint8_t kernel_status;

	/* enable & batch list */
	uint32_t enabled_list[HANDLE_ID_END];
	uint32_t interrupt_status;
	uint8_t calibratordata[DRIVER_ID_END][30];
	uint8_t calibratorUpdate[DRIVER_ID_END];
	uint8_t CalibratorStatus[DRIVER_ID_END];
	uint8_t SelfTestStatus[DRIVER_ID_END];

	/* Mcu site enable list*/

	/* power status */
	volatile	uint32_t power_on_list;

	/* Calibrator status */
	int cal_cmd;
	int cal_type;
	int cal_id;

	/* gpio */
	int irq_gpio;
	int wakeup_gpio;
	int reset_gpio;
	int boot_gpio;
	uint32_t debug_log;

	int cmd;
	uint32_t addr;
	int len;
	int mcu_slave_addr;
	int firmware_update_status;
	int cw_i2c_rw;	/* r = 0 , w = 1 */
	int cw_i2c_len;
	uint8_t cw_i2c_data[300];
#ifdef CWM_USE_ERROR_HANDLE_WORK
	int i2c_error_count;
	int i2c_error_state;
#endif

	s32 iio_data[6];
	struct iio_dev *indio_dev;
	struct irq_work iio_irq_work;
	struct iio_trigger  *trig;
	atomic_t pseudo_irq_enable;

	struct class *sensor_class;
	struct device *sensor_dev;
	atomic_t delay;

	int wq_polling_time;

	struct mutex mutex_lock;

	unsigned char loge_buff[QueueSystemInfoMsgSize*2];
	unsigned char loge_buff_count;

	uint8_t logcat_cmd;

	/*Data for GPS and Health Info*/
	uint8_t GPSData[40];
	uint8_t HEALTHData[30];

	int mcu_status;
	int mcu_init_count;
};


#define QUALCOMM_PLATFORM
//#define MTK_PLATFORM
//#define SAMSUNG_PLATFORM
#define CWM_GPIO_NON            0
#define CWM_GPIO_IRQ            1
#define CWM_GPIO_INPUT          2
#define CWM_GPIO_OUTPUT     3

static int cwstm_power_on(struct CWMCU_T *sensor,bool on);
static void cwm_gpio_control(int gpio, int state)
{
	if(gpio <=0){
		SH_DBG("GPIO[%d]",gpio);
		return;
	}
	gpio_set_value(gpio, state);
}
static int cwm_gpio_read(int gpio)
{
	if(gpio <=0){
		SH_DBG("GPIO[%d]",gpio);
		return 0;
	}
	return gpio_get_value(gpio);
}

static int cwm_gpio_setup(int gpio, const unsigned char *buf, bool config, int dir, int state)
{
#ifdef QUALCOMM_PLATFORM
	int retval = 0;
	if(gpio<0){
		retval = -1;
		SH_ERR("GPIO[%d] - [%s] Set wrong index error[%d]",gpio, buf,retval);
		return retval;
	}
	if(config) {
		if(dir == CWM_GPIO_INPUT){
			retval = gpio_request(gpio, buf);
			if(retval) {
				SH_ERR("GPIO[%d] Request Fail[%d]",gpio,retval);
				return retval;
			}
			retval = gpio_direction_input(gpio);
			if(retval) {
				SH_ERR("GPIO[%d] Set Direction Fail[%d]",gpio,retval);
				return retval;
			}
		}else if(dir == CWM_GPIO_OUTPUT){
			retval = gpio_request(gpio, buf);
			if(retval) {
				SH_ERR("GPIO[%d] Request Fail[%d]",gpio,retval);
				return retval;
			}
			retval = gpio_direction_output(gpio, state);
			if(retval) {
				SH_ERR("GPIO[%d] Set Direction Fail[%d]",gpio,retval);
				return retval;
			}
		}else{
			SH_ERR("GPIO[%d] Set Wrong DIR[%d]",gpio,dir);
		}
	}else {
		if(gpio >0)
			gpio_free(gpio);
	}
	return 0;

#elif defined(SAMSUNG_PLATFORM)
#elif defined(MTK_PLATFORM)
#else
#error Missing definition:Platform define
#endif
}

static int cwm_irq_gpio_setup(struct CWMCU_T *sensor,bool config, int dir, int state)
{
	int retval = 0;
	int gpio = sensor->irq_gpio;

	if(gpio <=0){
		gpio = GPIO_CW_MCU_INTERRUPT;
		SH_ERR("GPIO[%d] Failed to configure, We used default Kernel define gpio pin nmb",gpio);
	}
	if(dir == CWM_GPIO_IRQ){
		retval = gpio_request(gpio, "cwstm,irq-gpio");
		if(retval) {
			SH_ERR("GPIO[%d] Request Fail[%d]",sensor->irq_gpio,retval);
			goto err_gpio;
		}
		sensor->client->irq = gpio_to_irq(sensor->irq_gpio);
	}else{
		retval = cwm_gpio_setup(gpio, "cwstm,irq-gpio",config,dir,state);
		if (retval < 0) {
			SH_ERR("GPIO[%d] Failed to configure, Err[%d]",gpio,retval);
			goto err_gpio;
		}
	}

	return 0;
err_gpio:
	cwm_gpio_setup(gpio, "",0,0,0);
	return retval;
}

static int cwm_wakeup_gpio_setup(struct CWMCU_T *sensor,bool config, int dir, int state)
{
	int retval = 0;
	int gpio = sensor->wakeup_gpio;

	if(gpio <=0){
		gpio = GPIO_CW_MCU_WAKE_UP;
		SH_ERR("GPIO[%d] Failed to configure, We used default Kernel define gpio pin nmb",gpio);
	}

	retval = cwm_gpio_setup(gpio, "cwstm,wakeup-gpio",config,dir,state);
	if (retval < 0) {
		SH_ERR("GPIO[%d] Failed to configure, Err[%d]",gpio,retval);
		goto err_gpio;
	}
	return 0;
err_gpio:
	cwm_gpio_setup(gpio, "",0,0,0);
	return retval;
}

static int cwm_reset_gpio_setup(struct CWMCU_T *sensor,bool config, int dir, int state)
{
	int retval = 0;
	int gpio = sensor->reset_gpio;

	if(gpio <=0){
		gpio = GPIO_CW_MCU_RESET;
		SH_ERR("GPIO[%d] Failed to configure, We used default Kernel define gpio pin nmb",gpio);
	}

	retval = cwm_gpio_setup(gpio, "cwstm,reset-gpio",config,dir,state);
	if (retval < 0) {
		SH_ERR("GPIO[%d] Failed to configure, Err[%d]",gpio,retval);
		goto err_gpio;
	}
	return 0;
err_gpio:
	cwm_gpio_setup(gpio, "",0,0,0);
	return retval;
}

static int cwm_boot_gpio_setup(struct CWMCU_T *sensor,bool config, int dir, int state)
{
	int retval = 0;
	int gpio = sensor->boot_gpio;

	if(gpio <=0){
		gpio = GPIO_CW_MCU_BOOT;
		SH_ERR("GPIO[%d] Failed to configure, We used default Kernel define gpio pin nmb",gpio);
	}

	retval = cwm_gpio_setup(gpio, "cwstm,boot-gpio",config,dir,state);
	if (retval < 0) {
		SH_ERR("GPIO[%d] Failed to configure, Err[%d]",gpio,retval);
		goto err_gpio;
	}
	return 0;
err_gpio:
	cwm_gpio_setup(gpio, "",0,0,0);
	return retval;
}

//for geater than 32 bytes read
static int CWMCU_Object_read(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr	= sensor->client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg_addr,  
		},
		{
			.addr	= sensor->client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= data,  
		},
	};

	mutex_lock(&sensor->mutex_lock);
	ret = i2c_transfer(sensor->client->adapter, msgs, 2);  
	mutex_unlock(&sensor->mutex_lock);

	return (ret == 2) ? len : ret;
}
static void wakeup_pin_reset(struct CWMCU_T *sensor)
{
	cwm_gpio_control(sensor->wakeup_gpio,0);
	usleep_range(100, 100);
	cwm_gpio_control(sensor->wakeup_gpio,1);
	usleep_range(100, 100);
}
static int CWM_I2C_ERROR_CHECK(struct CWMCU_T *sensor,int err)
{
#ifdef CWM_USE_ERROR_HANDLE_WORK
	if(err<0){
		sensor->i2c_error_count ++;
		if(sensor->i2c_error_count ==5){
			sensor->i2c_error_state = CWM_LEVEL1_MCU_RESET;
			queue_delayed_work(sensor->driver_wq, &sensor->error_handle_work, 0);
		}else if(sensor->i2c_error_count ==10){
			sensor->i2c_error_state = CWM_LEVEL2_MCU_RESET;
			queue_delayed_work(sensor->driver_wq, &sensor->error_handle_work, 0);
		}else if(sensor->i2c_error_count ==15){
			sensor->i2c_error_state = CWM_LEVEL3_MCU_STOP;
			queue_delayed_work(sensor->driver_wq, &sensor->error_handle_work, 0);
		}
	}else{
		sensor->i2c_error_count = 0;
		sensor->i2c_error_state = CWM_LEVEL0_MCU_NON;
	}
#endif
	return 0;
}

static int CWMCU_reg_read(struct CWMCU_T *sensor, u8 reg_addr, u8 *data)
{
	RegInformation *pReadRegInfoInx = sensor->pReadRegInfo;
	int i;
	u8 cReadRegCount = sensor->m_cReadRegCount;
	int wRetVal = 0;

	if(pReadRegInfoInx == NULL || cReadRegCount == 0)
	{
		SH_ERR("pReadRegInfoInx==NULL or cReadRegCount==0\n");
		wRetVal = -1;
		return wRetVal;
	}

	for(i = 0; i < cReadRegCount; i++)
	{
		if(pReadRegInfoInx[i].cIndex == reg_addr)
			break;
	}

	if(i >= cReadRegCount)
	{
		wRetVal = -1;
	}
	else
	{
		if(pReadRegInfoInx[i].cObjLen != 0)
			wRetVal = CWMCU_Object_read(sensor, pReadRegInfoInx[i].cIndex, data, pReadRegInfoInx[i].cObjLen);
	}
	return wRetVal;
}

static int CWMCU_I2C_R(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int rty = 0;
	int err = 0;

	mutex_lock(&sensor->mutex_lock);

retry:
	err = i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len, data);
	if(err <0){
		if(rty<3)
		{
			SH_ERR("I2c Read[%x] RTY[%d]\n",reg_addr ,err);
			rty++; 
			wakeup_pin_reset(sensor);
			goto retry;
		}
		else
		{
			SH_ERR("I2c Read[%x] Error[%d]\n",reg_addr ,err);
		}
	}
	CWM_I2C_ERROR_CHECK(sensor, err);

	mutex_unlock(&sensor->mutex_lock);

	return err;
}

// write format    1.slave address  2.data[0]  3.data[1] 4.data[2]
static int CWMCU_I2C_W(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int rty = 0;
	int err = 0;

	mutex_lock(&sensor->mutex_lock);

retry:
	err = i2c_smbus_write_i2c_block_data(sensor->client, reg_addr, len, data);
	if(err <0){
		if(rty<3)
		{
			SH_ERR("I2c Write[%x] RTY[%d]\n",reg_addr ,err);
			rty++; 
			wakeup_pin_reset(sensor);
			goto retry;
		}
		else
		{
			SH_ERR("I2c Write[%x] Error[%d]\n",reg_addr ,err);
		}
	}
	CWM_I2C_ERROR_CHECK(sensor, err);

	mutex_unlock(&sensor->mutex_lock);

	return err;
}


static int CWMCU_I2C_W_SERIAL(struct CWMCU_T *sensor,u8 *data, int len)
{
	int dummy;
	dummy = i2c_master_send(sensor->client, data, len);
	if (dummy < 0) {
		SH_ERR("I2c Write[%x] Error[%d]\n",data[0] ,dummy);
		return dummy;
	}
	return 0;
}

static int CWMCU_I2C_R_SERIAL(struct CWMCU_T *sensor,u8 *data, int len)
{
	int dummy;
	dummy = i2c_master_recv(sensor->client, data, len);
	if (dummy < 0) {
		SH_ERR("I2c Read[%x] Error[%d]\n",data[0] ,dummy);
		return dummy;
	}
	return 0;
}
#ifdef TOUCH
static int cw_send_touch_event(struct CWMCU_T *sensor, u8 handle, u8 id, u8 *data)
{
#ifdef SUPPORT_INPUT
	uint8_t uT_EventType = data[0];
	uint8_t uT_TouchId = data[1];
	uint16_t uT_PositionX = (((uint16_t)data[3])<<8)|((uint16_t)data[2]);
	uint16_t uT_PositionY = (((uint16_t)data[5])<<8)|((uint16_t)data[4]);
	uint8_t uT_Fingers = data[6];
	uint16_t pressure = 0;

	if(uT_EventType ==3 || uT_EventType == 5){
		pressure = ST_PRESS;
		input_report_abs(sensor->input, ABS_MT_POSITION_X, uT_PositionX);
		input_report_abs(sensor->input, ABS_MT_POSITION_Y, uT_PositionY);
		input_report_abs(sensor->input, ABS_MT_PRESSURE, pressure);
		input_report_abs(sensor->input, ABS_MT_TRACKING_ID, uT_TouchId);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, pressure);
		input_mt_sync(sensor->input);
	}else if(uT_EventType ==4){
		pressure = 0;
		input_report_abs(sensor->input, ABS_MT_PRESSURE, pressure);
		input_report_abs(sensor->input, ABS_MT_TRACKING_ID, uT_TouchId);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, pressure);
		input_mt_sync(sensor->input);
	}

	input_report_key(sensor->input, BTN_TOUCH, !!uT_Fingers);
	input_sync(sensor->input);
#endif    

	return 0;
}
#endif
static int cw_send_event(struct CWMCU_T *sensor, u8 handle, u8 id, u8 *data)
{
	u8 event[21];/* Sensor HAL uses fixed 21 bytes */

	if (id == CWMCU_NODATA)
		return FAIL;
#ifdef TOUCH
	if (id == TOUCH)
		return cw_send_touch_event(sensor,handle,id,data);
#endif

	event[0] = handle;
	event[1] = id;
	memcpy(&event[2], data, 19);

	if (sensor->debug_log & (1<<D_IIO_DATA)) 
		SH_DBG("%s: id%d,data:%d,%d,%d\n",
				__func__,id,data[0],data[1],data[2]);
	if (sensor->indio_dev->active_scan_mask &&
			(!bitmap_empty(sensor->indio_dev->active_scan_mask,
				       sensor->indio_dev->masklength))) {
		iio_push_to_buffers(sensor->indio_dev, event);
		return 0;
	}
	else if (NULL == sensor->indio_dev->active_scan_mask)
	{
		SH_ERR("active_scan_mask = NULL, event might be missing\n");
	}

	return -EIO;
}

static void power_pin_sw(struct CWMCU_T *sensor,SWITCH_POWER_ID id, int onoff)
{
	int value = 0;

	mutex_lock(&sensor->mutex_lock);

	value = gpio_get_value(sensor->wakeup_gpio);
	if (onoff) {
		sensor->power_on_list |= ((uint32_t)(1) << id);
		if(value ==0){
			gpio_set_value(sensor->wakeup_gpio, onoff);
			usleep_range(100, 100);
		}   
	} else {
		sensor->power_on_list &= ~(1 << id);
		if (sensor->power_on_list == 0 && value == 1) {
			gpio_set_value(sensor->wakeup_gpio, onoff);
			usleep_range(100, 100);
		}
	}

	mutex_unlock(&sensor->mutex_lock);

}

static void cwm_set_kernel_status(struct CWMCU_T *sensor,uint8_t status)
{
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_DBG("mcu_mode == CW_BOOT!\n");
		return ;
	}
	sensor->kernel_status = status;
	if (CWMCU_I2C_W(sensor, RegMapW_SetHostStatus, &sensor->kernel_status, 1) < 0)
	{
		SH_ERR("Write SetHostStatus Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);
	}
}

static int check_enable_list(struct CWMCU_T *sensor)
{
	int i = 0,j=0;
	int count = 0;
	int handle = 0;
	uint8_t data[10] = {0};
	int error_msg = 0;
	uint32_t enabled_list[HANDLE_ID_END] = {0};
	uint32_t enabled_list_temp = 0;

	if (sensor->mcu_mode == CW_BOOT) {
		SH_DBG("mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
		return 0;
	}

	if( (CWMCU_I2C_R(sensor, RegMapR_GetHostEnableList, data, 8)) < 0)
	{
		SH_ERR("Read GetHostEnableList Fail [I2C] , func: %s , li: %d\n",__func__,__LINE__);
		return FAIL;
	}

	enabled_list[NonWakeUpHandle] = (uint32_t)data[3]<<24 |(uint32_t)data[2]<<16 |(uint32_t)data[1]<<8 |(uint32_t)data[0];
	enabled_list[WakeUpHandle] = (uint32_t)data[7]<<24 |(uint32_t)data[6]<<16 |(uint32_t)data[5]<<8 |(uint32_t)data[4];
	enabled_list[InternalHandle] = 0;

	if((enabled_list[NonWakeUpHandle] != sensor->enabled_list[NonWakeUpHandle]) 
			|| (enabled_list[WakeUpHandle] != sensor->enabled_list[WakeUpHandle]))
	{
		SH_DBG("Enable List Check AP0:%d,MCU0:%d;AP1:%d,MCU1:%d\n",
				sensor->enabled_list[NonWakeUpHandle],enabled_list[NonWakeUpHandle],
				sensor->enabled_list[WakeUpHandle],enabled_list[WakeUpHandle]);

		for(j = 0; j < InternalHandle; j++)
		{
			handle = j;
			enabled_list_temp = sensor->enabled_list[handle]^enabled_list[handle];
			for (i = 0; i < SENSORS_ID_END; i++)
			{
				if (enabled_list_temp & (1<<i))
				{
					data[0] = handle;
					data[1] = i;
					if (sensor->sensors_info[handle][i].en)
					{
						sensor->sensors_info[handle][i].rate = (sensor->sensors_info[handle][i].rate ==0)?200:sensor->sensors_info[handle][i].rate;
						data[2] = sensor->sensors_info[handle][i].rate;
						data[3] = (uint8_t)sensor->sensors_info[handle][i].timeout;
						data[4] = (uint8_t)(sensor->sensors_info[handle][i].timeout >>8);
						error_msg = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
					}
					else
					{
						data[2] = 0;
						data[3] = 0;
						data[4] = 0;
						error_msg = CWMCU_I2C_W(sensor, RegMapW_SetDisable, data, 5);
					}
					if (error_msg < 0)
						SH_ERR("I2c Write Fail;%d,%d\n", handle, i);

					count++;
					if(count >15)
					{
						count = 0;
						msleep(20);
					}
				}
			}
		}
	}
	return 0;
}

static int cwmcu_read_buff(struct CWMCU_T *sensor , u8 handle)
{       
	uint8_t count_reg;
	uint8_t data_reg;
	uint8_t data[24] = {0};
	uint16_t count = 0;
	int i = 0;

	if (sensor->mcu_mode == CW_BOOT) {
		SH_DBG("-CWMCU- mcu_mode = boot \n");
		return -1;
	}

	if (handle == NonWakeUpHandle)
	{
		count_reg = RegMapR_StreamCount;
		data_reg = RegMapR_StreamEvent;
	}
	else if (handle == WakeUpHandle)
	{
		count_reg = RegMapR_BatchCount;
		data_reg = RegMapR_BatchEvent;
	}
	else
	{
		return FAIL;
	}

	if (CWMCU_I2C_R(sensor, count_reg, data, 2) >= 0) 
	{
		count = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
	}
	else 
	{
		SH_ERR("check count failed)\n");
		return FAIL;
	}
	if((data[0] ==0xFF) && (data[1] ==0xFF))
		return NO_ERROR;

	for (i = 0; i < count; i++)
	{
		if (CWMCU_I2C_R(sensor, data_reg, data, 9) >= 0)
		{
			cw_send_event(sensor,handle,data[0],&data[1]);
		}
		else
		{
			SH_ERR("Read stream buffer fail [I2C]\n");
		}
	}
	return NO_ERROR;
}

static int cwmcu_read_gesture(struct CWMCU_T *sensor )
{
	uint8_t data[24] = {0};
	uint8_t count = 0;
	int i = 0;

	if (sensor->mcu_mode == CW_BOOT) {
		SH_DBG("mcu_mode = boot \n");
		return 0;
	}

	if (CWMCU_I2C_R(sensor, RegMapR_GestureCount, &count, 1) < 0)
	{
		SH_ERR("check count failed)\n");
		return FAIL;
	}

	if(count ==0xFF)
		return NO_ERROR;        

	for (i = 0; i < count; i++)
	{
		if (CWMCU_I2C_R(sensor, RegMapR_GestureEvent, data, 9) >= 0)
		{
			cw_send_event(sensor,NonWakeUpHandle,data[0],&data[1]);
		}else{
			SH_ERR("Read GestureEvent fail [I2C] \n");
		}
	}
	return NO_ERROR;
}

static void parser_mcu_info(char *data)
{
	static unsigned char loge_bufftemp[QueueSystemInfoMsgBuffSize];
	static int buff_counttemp = 0;
	int i;

	for (i = 0; i < QueueSystemInfoMsgSize; i++)
	{
		loge_bufftemp[buff_counttemp] = (unsigned char)data[i];
		buff_counttemp++;
		if (data[i] == '\n' || (buff_counttemp >=QueueSystemInfoMsgBuffSize))
		{
			SH_LOG("%s:%s", "MSG",loge_bufftemp);
			memset(loge_bufftemp,0x00,QueueSystemInfoMsgBuffSize);
			buff_counttemp = 0;
		}
	}
}

static void read_mcu_info(struct CWMCU_T *sensor)
{
	uint8_t data[40];
	uint16_t count = 0;
	int i = 0;

	SH_FUN();
	if (sensor->mcu_mode == CW_BOOT) {
		SH_DBG("mcu_mode = BOOT \n");
		return ;
	}

	if (CWMCU_I2C_R(sensor, RegMapR_SystemInfoMsgCount, data, 1) >= 0)
	{
		count = (uint16_t)data[0];
	}
	else
	{
		SH_ERR("Read SystemInfoMsgCount Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
		return ;
	}
	if(count ==0xFF){
		SH_DBG("Count = 0xFF , func: %s , li: %d\n",__func__,__LINE__);
		return ;
	}

	for (i = 0; i < count; i++)
	{
		if (CWMCU_I2C_R(sensor, RegMapR_SystemInfoMsgEvent, data, 30) >= 0)
		{
			parser_mcu_info(data);
		}else{	
			SH_ERR("Read SystemInfoMsgEvent Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
		}
	}
}

static int cwmcu_find_mindelay(struct CWMCU_T *sensor, int handle)
{
	int i;
	int min_delay = 30;
	for (i = 0; i < SENSORS_ID_END; i++)
	{	
		if(sensor->sensors_info[handle][i].en
				&& (sensor->sensors_info[handle][i].rate >= 10)
				&& (sensor->sensors_info[handle][i].rate < min_delay)
		  )
		{
			min_delay = sensor->sensors_info[handle][i].rate;
		}
	}
	min_delay = (min_delay<=10)? 10: min_delay;
	return min_delay;
}

static ssize_t active_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int enabled = 0;
	int id = 0;
	int handle = 0;
	int error_msg = 0;
	uint8_t data[10];

	sscanf(buf, "%d %d %d\n", &handle, &id, &enabled);

	sensor->sensors_info[handle][id].en = enabled;
	data[0] = handle;
	data[1] = id;
	if (enabled)
	{
		sensor->enabled_list[handle] |= 1<<id;
		data[2] = (sensor->sensors_info[handle][id].rate ==0)?200:sensor->sensors_info[handle][id].rate;
		data[3] = (uint8_t)sensor->sensors_info[handle][id].timeout;
		data[4] = (uint8_t)(sensor->sensors_info[handle][id].timeout >>8);
	}
	else
	{
		sensor->enabled_list[handle] &= ~(1<<id);
		sensor->sensors_info[handle][id].rate = 0;
		sensor->sensors_info[handle][id].timeout = 0;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0;
	}

	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode = CW_BOOT \n");
		return count;
	}
	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);
	if (enabled)
	{
		error_msg = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
	}
	else
	{
		error_msg = CWMCU_I2C_W(sensor, RegMapW_SetDisable, data, 5);
	}

	if (error_msg < 0)
		SH_ERR("Write SetEn/Disable Fail [I2C] \n");

	msleep(10);
	check_enable_list(sensor);
	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);
	if (NonWakeUpHandle == handle)
	{
		sensor->wq_polling_time = cwmcu_find_mindelay(sensor,NonWakeUpHandle);
		if (sensor->wq_polling_time != atomic_read(&sensor->delay))
		{
			if (sensor->debug_log & (1<<D_EN)) 
				SH_DBG("sensor->wq_polling_time != atomic_read(&sensor->delay)\n");
#ifdef CWM_USE_DELAY_WORK
			cancel_delayed_work(&sensor->delay_work);
#endif
			if(sensor->enabled_list[NonWakeUpHandle])
			{
				if (sensor->debug_log & (1<<D_EN)) 
					SH_DBG("sensor->enabled_list[NonWakeUpHandle] == 1\n");
				atomic_set(&sensor->delay, sensor->wq_polling_time);
#ifdef CWM_USE_DELAY_WORK
				queue_delayed_work(sensor->driver_wq, &sensor->delay_work,
						msecs_to_jiffies(atomic_read(&sensor->delay)));
#endif
			}
			else
			{
				atomic_set(&sensor->delay, CWMCU_POLL_MAX);
			}
		}
	}
	if (sensor->debug_log & (1<<D_EN)) 
		SH_LOG("%d,%d,%d,%d,%d\n", handle, id, enabled, (int)sensor->sensors_info[handle][id].rate, (int)sensor->sensors_info[handle][id].timeout);

	return count;
}

static ssize_t active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t data[10] = {0};
	uint32_t enabled_list[2] ={0, 0};
	int err = 0;

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode = CW_BOOT\n");
		return sprintf(buf, "In Boot Mode\n");
	}

	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);
	err = CWMCU_I2C_R(sensor, RegMapR_GetHostEnableList, data, 8);
	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);
	if (err >= 0)
	{
		enabled_list[NonWakeUpHandle] = (uint32_t)data[3]<<24 |(uint32_t)data[2]<<16 |(uint32_t)data[1]<<8 |(uint32_t)data[0];
		enabled_list[WakeUpHandle] = (uint32_t)data[7]<<24 |(uint32_t)data[6]<<16 |(uint32_t)data[5]<<8 |(uint32_t)data[4];
		if (sensor->debug_log & (1<<D_EN)) 
			SH_LOG("MCU En Status:%d,%d\n", enabled_list[NonWakeUpHandle], enabled_list[WakeUpHandle]);

		return sprintf(buf, "%d %d %d %d\n", sensor->enabled_list[NonWakeUpHandle],
				sensor->enabled_list[WakeUpHandle], enabled_list[NonWakeUpHandle], enabled_list[WakeUpHandle]);
	}
	else
	{
		SH_ERR("Read GetHostEnableList Fail [I2C]\n");
		return sprintf(buf, "read RegMapR_GetHostEnableList failed!\n");
	}
}

static ssize_t batch_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint32_t id = 0;
	uint32_t handle = 0;	
	uint32_t mode = -1;
	uint32_t rate = 0;
	uint32_t timeout = 0;
	uint8_t data[5];
	int err = 0;

	sscanf(buf, "%d %d %d %d %d\n", &handle, &id, &mode, &rate, &timeout);
	if (0 == mode)
	{

		if (sensor->debug_log & (1<<D_EN))
			SH_LOG("%d %d %d %d %d\n", handle, id, mode, rate, timeout);
		sensor->sensors_info[handle][id].rate = (uint8_t)rate;
		sensor->sensors_info[handle][id].timeout = (uint16_t)timeout;
		data[0] = handle;
		data[1] = id;
		data[2] = sensor->sensors_info[handle][id].rate;
		data[3] = (uint8_t)sensor->sensors_info[handle][id].timeout;
		data[4] = (uint8_t)(sensor->sensors_info[handle][id].timeout >>8);

		if (CW_BOOT == sensor->mcu_mode)
		{
			SH_ERR("mcu_mode = CW_BOOT\n");
			return count;
		}

		if(sensor->sensors_info[handle][id].en)
		{
			power_pin_sw(sensor,SWITCH_POWER_BATCH, 1);
			err = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
			power_pin_sw(sensor,SWITCH_POWER_BATCH, 0);
			if (err < 0)
			{
				SH_ERR("Write Fail:id:%d, mode:%d, rate:%d, timeout:%d)\n", id, mode, rate, timeout);
			}
		}

		if (sensor->debug_log & (1<<D_EN)) 
			SH_LOG("id:%d, mode:%d, rate:%d, timeout:%d\n", id, mode, rate, timeout);
	}

	return count;
}

static ssize_t flush_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int id = 0;
	int handle = 0;	
	uint8_t data[2];
	int err = 0;

	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode == CW_BOOT\n");
		return count;
	}
	sscanf(buf, "%d %d\n", &handle, &id);
	if (sensor->debug_log & (1<<D_EN))
		SH_LOG("flush:id:%d\n", id);
	data[0] = (uint8_t)handle;
	data[1] = (uint8_t)id;
	power_pin_sw(sensor,SWITCH_POWER_FLUSH, 1);
	err = CWMCU_I2C_W(sensor, RegMapW_SetFlush, data, 2);
	power_pin_sw(sensor,SWITCH_POWER_FLUSH, 0);
	if (err < 0)
		SH_ERR("Write SetFlush Fail [I2C], func: %s , ln: %d\n",__func__,__LINE__);

	return count;
}

static int set_calib_cmd(struct CWMCU_T *sensor, uint8_t cmd, uint8_t id, uint8_t type)
{
	uint8_t data[4];
	int err;

	if (CW_BOOT == sensor->mcu_mode) 
	{
		SH_ERR("mcu_mode == CW_BOOT\n");
		return -1;
	}

	data[0] = cmd;
	data[1] = id;
	data[2] = type;
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
	err = CWMCU_I2C_W(sensor, RegMapW_CalibratorCmd, data, 4);
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if(err)
		SH_ERR("Write CalibratorCmd Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);

	return err;
}

/*
   sensors default calibrator flow:
   sensors_calib_start(sensors, id);
   do{
   sensors_calib_status(sensors, id,&status);
   }while(status ==CALIB_STATUS_INPROCESS)
   if(status ==CALIB_STATUS_PASS)
   sensors_calib_data_read(sensors, id,data);
   save data    
   */
static int sensors_calib_start(struct CWMCU_T *sensor, uint8_t id)
{
	int err;
	err = set_calib_cmd(sensor, CALIB_EN, id, CALIB_TYPE_DEFAULT);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	err = set_calib_cmd(sensor, CALIB_CHECK_STATUS, id, CALIB_TYPE_DEFAULT);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	return err;
}

static int sensors_calib_data_read(struct CWMCU_T *sensor, uint8_t id, uint8_t *data)
{
	int err;

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode == CW_BOOT\n");
		return FAIL;
	}

	err = set_calib_cmd(sensor, CALIB_DATA_READ, id, CALIB_TYPE_NON);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
	err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, data, 30);
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if (err < 0)
	{
		SH_ERR("Read CalibratorData Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);	

		return err;
	}

	return NO_ERROR;
}

#ifndef CWMCU_CALIB_SAVE_IN_FLASH
static int sensors_calib_data_write(struct CWMCU_T *sensor, uint8_t id, uint8_t *data)
{
	int err;

	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode == CW_BOOT\n");
		return FAIL;
	}

	err = set_calib_cmd(sensor, CALIB_DATA_WRITE, id, CALIB_TYPE_NON);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
	err = CWMCU_I2C_W(sensor, RegMapW_CalibratorData, data, 30);
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if (err < 0)
	{
		SH_ERR("Write CalibratorData fail [I2C], func: %s , li: %d\n",__func__,__LINE__);	
		return err;
	}

	return NO_ERROR;
}
#endif

static int sensors_calib_status(struct CWMCU_T *sensor, uint8_t id, int *status)
{
	int err;
	uint8_t i2c_data[31] = {0};

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode == CW_BOOT\n");
		return FAIL;
	}

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
	err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, i2c_data, 30);
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if (err < 0)
	{
		SH_ERR("Read CalibratorData Fail [I2C]\n");
		return I2C_FAIL;
	}
	status[0] = (int)((int8_t)i2c_data[0]);

	if(status[0] == CALIB_STATUS_PASS)
		sensors_calib_data_read(sensor, id,i2c_data);

	return NO_ERROR;
}

static int sensors_self_test_start(struct CWMCU_T *sensor, uint8_t id)
{
	int err;
	err = set_calib_cmd(sensor, CALIB_EN, id, CALIB_TYPE_SELFTEST);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	err = set_calib_cmd(sensor, CALIB_CHECK_STATUS, id, CALIB_TYPE_SELFTEST);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	return err;
}

static int get_all_calib_status(struct CWMCU_T *sensor, uint8_t *status)
{
	int err;
	uint8_t i2c_data[31] = {0};

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode == CW_BOOT\n");
		return FAIL;
	}

	err = set_calib_cmd(sensor, CALIB_GET_ALL_CALIB_STATUS, 0, 0);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
	err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, i2c_data, 30);
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if (err < 0)
	{
		SH_ERR("Read Data Fail [I2C]\n");
		return I2C_FAIL;
	}
	memcpy(status, i2c_data, sizeof(uint8_t)*DRIVER_ID_END);

	return NO_ERROR;
}

static int get_all_selftest_status(struct CWMCU_T *sensor, uint8_t *status)
{
	int err;
	uint8_t i2c_data[31] = {0};

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode == CW_BOOT\n");
		return FAIL;
	}

	err = set_calib_cmd(sensor, CALIB_GET_ALL_SELFTEST_STATUS, 0, 0);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
	err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, i2c_data, 30);
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if (err < 0)
	{
		SH_ERR("Read Data Fail [I2C]\n");
		return I2C_FAIL;
	}
	memcpy(status, i2c_data, sizeof(uint8_t)*DRIVER_ID_END);

	return NO_ERROR;
}

static int get_mcu_sensors_info(struct CWMCU_T *sensor, int id, uint8_t *status)
{
	int err;
	uint8_t i2c_data[31] = {0};

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode == CW_BOOT\n");
		return FAIL;
	}

	err = set_calib_cmd(sensor, CALIB_CHECK_STATUS, id, CALIB_TYPE_CHIP_ID);
	if (err < 0)
	{
		SH_ERR("I2c Write Fail!\n");
		return err;
	}

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
	err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, i2c_data, 30);
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if (err < 0)
	{
		SH_ERR("Read Data Fail [I2C]\n");
		return I2C_FAIL;
	}
	memcpy(status, i2c_data, sizeof(uint8_t)*DRIVER_ID_END);

	return NO_ERROR;
}

static void cwm_reset_mcu(struct CWMCU_T *sensor)
{
	int status = 0;
	status = cwm_reset_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, 1);
	if (status < 0) {
		SH_ERR("GPIO Failed to configure, Err[%d]",status);
	}
	status = cwm_boot_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, 0);
	if (status < 0) {
		SH_ERR("GPIO Failed to configure, Err[%d]",status);
	}

	msleep(100);
	cwm_gpio_control(sensor->reset_gpio,0);

	msleep(100);
	cwm_gpio_control(sensor->reset_gpio,1);

	msleep(100);
	cwm_gpio_control(sensor->reset_gpio,0);

	msleep(100);
	cwm_gpio_control(sensor->reset_gpio,1);
}

static ssize_t set_firmware_update_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	u8 data[300] = {0};
	int i = 0;
	int status = 0;

	sscanf(buf, "%d %d %d\n", &sensor->cmd, &sensor->addr, &sensor->len);

	SH_DBG("cmd=%d addr=%d len=%d\n", sensor->cmd, sensor->addr, sensor->len);

	power_pin_sw(sensor,SWITCH_POWER_FIRMWARE_COMMAND, 1);

	switch (sensor->cmd) {
		case CHANGE_TO_BOOTLOADER_MODE:
			SH_DBG("Start (1 0 0)(CHANGE_TO_BOOTLOADER_MODE)\n");

			sensor->firmware_update_status = 0; 
			sensor->mcu_mode = CW_BOOT;
			sensor->mcu_slave_addr = sensor->client->addr;
			sensor->client->addr = 0x39;
#if 0
			status = cwm_reset_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, 1);
			if (status < 0) {
				SH_ERR("GPIO Failed to configure, Err[%d]",status);
			}
			status = cwm_boot_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, 1);
			if (status < 0) {
				SH_ERR("GPIO Failed to configure, Err[%d]",status);
			}
			msleep(100);
#endif
			cwm_gpio_control(sensor->boot_gpio, 1);

			msleep(100);
			cwm_gpio_control(sensor->reset_gpio,1);
			msleep(100);
			cwm_gpio_control(sensor->reset_gpio,0);
			msleep(100);
			cwm_gpio_control(sensor->reset_gpio,1);
			SH_DBG("End(1 0 0)(CHANGE_TO_BOOTLOADER_MODE)\n");
			break;

		case CHANGE_TO_NORMAL_MODE:
			SH_DBG(" Start(5 0 0)(CHANGE_TO_NORMAL_MODE)\n");

			sensor->firmware_update_status = 0;
			sensor->client->addr = 0x3a;
			sensor->mcu_mode = CW_NORMAL;
			sensor->firmware_update_status = 2;
#if 0 
			status = cwm_reset_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, 1);
			if (status < 0) {
				SH_ERR("GPIO Failed to configure, Err[%d]",status);
			}
			status = cwm_boot_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, 0);
			if (status < 0) {
				SH_ERR("GPIO Failed to configure, Err[%d]",status);
			}
			msleep(100);
#endif
			cwm_gpio_control(sensor->boot_gpio,0);
			cwm_gpio_control(sensor->reset_gpio,0);

			msleep(100);
			cwm_gpio_control(sensor->reset_gpio,1);

			msleep(100);
#if 0            
			status = cwm_reset_gpio_setup( sensor, 1, CWM_GPIO_INPUT, 1);
			if (status < 0) {
				SH_ERR("GPIO Failed to configure, Err[%d]",status);
			}
#endif         
			power_pin_sw(sensor,SWITCH_POWER_SHUTDOWN, 1);
			cwm_set_kernel_status(sensor,KERNEL_SHUTDOWN);
			power_pin_sw(sensor,SWITCH_POWER_SHUTDOWN, 0);
			SH_DBG(" Start(5 0 0)(CHANGE_TO_NORMAL_MODE)\n");
			break;

		case CHECK_FIRMWAVE_VERSION:
			if (CWMCU_I2C_R(sensor, RegMapR_GetFWVersion, data, 4) >= 0) {
				printk("%s:%s:(CHECK_FIRMWAVE_VERSION:%u,%u,%u,%u)\n",LOG_TAG_KERNEL ,__FUNCTION__, data[0],data[1],data[2],data[3]);
			}
			break;

		case GET_FWPROJECT_ID:
			if (CWMCU_reg_read(sensor, RegMapR_GetProjectID, data) >= 0) {
				printk("%s:%s:(PROJECT ID:%s) \n",LOG_TAG_KERNEL ,__FUNCTION__, data);
			}
			break;
		case SET_DEBUG_LOG:
			if(sensor->len)
				sensor->debug_log  |= (1<< sensor->addr);
			else
				sensor->debug_log  &= ~(1<< sensor->addr);
			printk("%s:%s:(SET_DEBUG_LOG%u)\n",LOG_TAG_KERNEL ,__FUNCTION__,sensor->debug_log);
			break;
		case SET_SYSTEM_COMMAND:
			data[0] = sensor->addr;
			data[1] = sensor->len;
			if (CWMCU_I2C_W(sensor, RegMapW_SetSystemCommand, data, 2) < 0)
			{
				SH_ERR(" Write SetSystemCommand Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);
			}else{
				printk("%s:%s:(SET_SYSTEM_COMMAND)\n",LOG_TAG_KERNEL ,__FUNCTION__);
			}
			break;
		case SET_SENSORS_POSITION:
			data[0] = sensor->addr;
			data[1] = sensor->len;
			if (CWMCU_I2C_W(sensor, RegMapW_SetSensorAxisReference, data, 2) < 0 )
			{
				SH_ERR(" Write SetSensorAxisReference Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);
			}
			break;
		case MCU_SET_GPIO_INPUT:
			/*[1]:GPIO id*/
			if(sensor->addr ==0){
				status = cwm_irq_gpio_setup( sensor, 1, CWM_GPIO_INPUT, sensor->len);
			}else if(sensor->addr ==1){
				status = cwm_wakeup_gpio_setup( sensor, 1, CWM_GPIO_INPUT, sensor->len);
			}else if(sensor->addr ==2){
				status = cwm_reset_gpio_setup( sensor, 1, CWM_GPIO_INPUT, sensor->len);
			}else if(sensor->addr ==3){
				status = cwm_boot_gpio_setup( sensor, 1, CWM_GPIO_INPUT, sensor->len);
			}

			if (status < 0) {
				SH_ERR("GPIO[%d] Failed to configure, Err[%d]",i,status);
			}
			break;

		case MCU_SET_GPIO_OUTPUT:
			/*[1]:GPIO id [2]:Output High/Low*/
			if(sensor->addr ==0){
				status = cwm_irq_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, sensor->len);
			}else if(sensor->addr ==1){
				status = cwm_wakeup_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, sensor->len);
			}else if(sensor->addr ==2){
				status = cwm_reset_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, sensor->len);
			}else if(sensor->addr ==3){
				status = cwm_boot_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, sensor->len);
			}

			if (status < 0) {
				SH_ERR("GPIO[%d] Failed to configure, Err[%d]",i,status);
			}
			break;

		case MCU_SET_GPIO_CONTROL:
			/*[1]:GPIO id [2]:Output High/Low*/
			if(sensor->addr ==0){
				i = sensor->irq_gpio;
			}else if(sensor->addr ==1){
				i = sensor->wakeup_gpio;
			}else if(sensor->addr ==2){
				i = sensor->reset_gpio;
			}else if(sensor->addr ==3){
				i = sensor->boot_gpio;            
			}
			cwm_gpio_control(i, sensor->len);
			break;

		case MCU_SET_GPIO_CLEAN:
			/*[1]:GPIO id [2]:Output High/Low*/

			if(sensor->addr ==0){
				status = cwm_irq_gpio_setup( sensor, 0, CWM_GPIO_NON, sensor->len);
			}else if(sensor->addr ==1){
				status = cwm_wakeup_gpio_setup( sensor, 0, CWM_GPIO_NON, sensor->len);
			}else if(sensor->addr ==2){
				status = cwm_reset_gpio_setup( sensor, 0, CWM_GPIO_NON, sensor->len);
			}else if(sensor->addr ==3){
				status = cwm_boot_gpio_setup( sensor, 0, CWM_GPIO_NON, sensor->len);
			}

			if (status < 0) {
				SH_ERR("GPIO[%d] Failed to configure, Err[%d]",i,status);
			}
			break;

		case CMD_CALIBRATOR:
			sensors_calib_start(sensor,sensor->addr);
			break;
		case CMD_SELF_TEST:
			sensors_self_test_start(sensor,sensor->addr);
			break;
		case MCU_RESET: //20 0 0 mcu_reset
			sensor->firmware_update_status = 0;
			cwm_reset_mcu(sensor);
			break;
	}
	power_pin_sw(sensor,SWITCH_POWER_FIRMWARE_COMMAND, 0);
	return count;
}

static ssize_t get_firmware_update_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	SH_DBG("firmware_update_status = %d\n", sensor->firmware_update_status);
	return sprintf(buf, "%d\n", sensor->firmware_update_status);
}

static ssize_t set_firmware_update_i2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int intsize = sizeof(int);

	SH_FUN();
	memcpy(&sensor->cw_i2c_rw, buf, intsize);
	memcpy(&sensor->cw_i2c_len, &buf[4], intsize);
	memcpy(sensor->cw_i2c_data, &buf[8], sensor->cw_i2c_len);
	return count;
}

static ssize_t get_firmware_update_i2(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int status = 0;

	SH_FUN();
	if (sensor->cw_i2c_rw)
	{
		if (CWMCU_I2C_W_SERIAL(sensor,sensor->cw_i2c_data, sensor->cw_i2c_len) < 0)
		{
			status = -1;
		}
		memcpy(buf, &status, sizeof(int));
		return 4;
	}
	else
	{
		if (CWMCU_I2C_R_SERIAL(sensor,sensor->cw_i2c_data, sensor->cw_i2c_len) < 0)
		{
			status = -1;
			memcpy(buf, &status, sizeof(int));
			return 4;
		}
		memcpy(buf, &status, sizeof(int));
		memcpy(&buf[4], sensor->cw_i2c_data, sensor->cw_i2c_len);
		return 4+sensor->cw_i2c_len;
	}
	return  0;
}

static ssize_t mcu_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->mcu_mode);
}

static ssize_t mcu_model_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int mode = 0;
	sscanf(buf, "%d\n", &mode);
	sensor->mcu_mode = mode;
	return count;
}

static ssize_t set_calibrator_cmd(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int err;

	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_ERR("mcu_mode == CW_BOOT!\n");
		return count;
	}

	sscanf(buf, "%d %d %d\n", &sensor->cal_cmd, &sensor->cal_id, &sensor->cal_type);
	err = set_calib_cmd(sensor, sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
	if (sensor->debug_log & (1<<D_CALIB)) 
		SH_LOG("cmd:%d,id:%d,type:%d\n", sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
	if (err < 0)
		SH_ERR("I2c Write Fail!\n");

	return count;
}

static ssize_t get_calibrator_cmd(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "Cmd:%d,Id:%d,Type:%d\n", sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
}

static ssize_t get_calibrator_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t Cal_data[31] = {0};
	int err;

	if (sensor->mcu_mode == CW_BOOT)
	{
		SH_ERR("mcu_mode == CW_BOOT!\n");
		return 0;
	}

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);

	err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, Cal_data, 30);
	if (err < 0 )
		SH_ERR(" Read CalibratorData Fail [I2C]\n");

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if(sensor->cal_cmd == CALIB_DATA_READ && err >=0){
		memcpy(sensor->calibratordata[sensor->cal_id],Cal_data,30);
		sensor->calibratorUpdate[sensor->cal_id] = 1;
	}
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", 
			err,
			Cal_data[0], Cal_data[1], Cal_data[2],
			Cal_data[3], Cal_data[4], Cal_data[5], Cal_data[6], Cal_data[7], Cal_data[8], Cal_data[9], Cal_data[10], Cal_data[11], Cal_data[12],
			Cal_data[13], Cal_data[14], Cal_data[15], Cal_data[16], Cal_data[17], Cal_data[18], Cal_data[19], Cal_data[20], Cal_data[21], Cal_data[22],
			Cal_data[23], Cal_data[24], Cal_data[25], Cal_data[26], Cal_data[27], Cal_data[28], Cal_data[29]);
}

static ssize_t set_calibrator_data(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t data[30];
	int temp[33] = {0};
	int i,err;

	if (sensor->mcu_mode == CW_BOOT) 
	{
		SH_ERR("mcu_mode == CW_BOOT!\n");
		return count;
	}

	sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
			&temp[0], &temp[1], &temp[2],
			&temp[3], &temp[4], &temp[5], &temp[6], &temp[7], &temp[8], &temp[9], &temp[10], &temp[11], &temp[12],
			&temp[13], &temp[14], &temp[15], &temp[16], &temp[17], &temp[18], &temp[19], &temp[20], &temp[21], &temp[22],
			&temp[23], &temp[24], &temp[25], &temp[26], &temp[27], &temp[28], &temp[29]);

	for (i = 0 ; i < 30; i++)
		data[i] = (uint8_t)temp[i];

	if(sensor->cal_cmd == CALIB_DATA_WRITE){
		memcpy(sensor->calibratordata[sensor->cal_id],data,30);
		sensor->calibratorUpdate[sensor->cal_id] = 1;
	}

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
	err = CWMCU_I2C_W(sensor, RegMapW_CalibratorData, data, 30);
	if (err < 0)
		SH_ERR(" Write CalibratorData Fail [I2C]\n");

	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t data[4];
	int16_t version = -1;

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_DBG("mcu_mode == CW_BOOT!\n");
		return FAIL;
	}

	power_pin_sw(sensor,SWITCH_POWER_VERSION, 1);
	if (CWMCU_I2C_R(sensor, RegMapR_GetFWVersion, data, 4) >= 0)
	{
		version = (int16_t)( ((uint16_t)data[1])<<8 | (uint16_t)data[0]);
		SH_LOG(" Check FW Version[3-0]: (M:%u,D:%u,V:%u,SV:%u)\n", data[3], data[2], data[1], data[0]);
	}
	else
	{
		SH_ERR("Read Get FW Version Fail [I2C], func: %s ,ln: %d\n",__func__,__LINE__);
		data[0] = 1;
		data[1] = 0;
	}
	if (CWMCU_I2C_R(sensor, RegMapR_ChipId, data, 2) >= 0)
	{
		SH_LOG(" Check Chip Id %X,%X\n", data[0], data[1]);
	}

	power_pin_sw(sensor,SWITCH_POWER_VERSION, 0);
	return sprintf(buf, "%d\n", version);
}

static ssize_t library_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t data[4] = {0, 0, 0, 0};

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_DBG("mcu_mode == CW_BOOT!\n");
		return FAIL;
	}

	power_pin_sw(sensor,SWITCH_POWER_VERSION, 1);
	if (CWMCU_I2C_R(sensor, RegMapR_GetLibVersion, data, 4) >= 0)
	{
		SH_LOG("check_library_version:%u,%u,%u,%u\n", data[3], data[2], data[1], data[0]);
	}
	else
	{
		SH_ERR("i2c read fail)\n");
	}
	power_pin_sw(sensor,SWITCH_POWER_VERSION, 0);
	return sprintf(buf, "%d %d %d %d\n", data[3], data[2], data[1], data[0]);
}

static ssize_t timestamp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t data[4];
	uint32_t *ptr;
	int err;
	ptr = (uint32_t *)data;

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_DBG("mcu_mode == CW_BOOT!\n");
		return FAIL;
	}
	power_pin_sw(sensor,SWITCH_POWER_TIME, 1);
	err = CWMCU_I2C_R(sensor, RegMapR_GetSystemTimestamp, data, 4);
	power_pin_sw(sensor,SWITCH_POWER_TIME, 0);
	return sprintf(buf, "%d %u\n", err, ptr[0]);
}

static ssize_t set_sys_cmd(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t data[8];
	int temp[8] = {0};
	int i,err;

	if (sensor->mcu_mode == CW_BOOT) 
	{
		SH_ERR("mcu_mode == CW_BOOT!\n");
		return count;
	}

	sscanf(buf, "%d %d %d %d %d %d %d %d\n",
			&temp[0], &temp[1], &temp[2],
			&temp[3], &temp[4], &temp[5], &temp[6], &temp[7]);

	for (i = 0 ; i < 8; i++)
		data[i] = (uint8_t)temp[i];

	power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
	err = CWMCU_I2C_W(sensor, RegMapW_SetSystemCommand, data, 8);
	if (err < 0)
		SH_ERR("I2c Write Fail!\n");
	power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
	return count;
}

static int read_sensors_rawdata(struct CWMCU_T *sensor,int id, int *rawdata)
{
	uint8_t data[12];
	int *ptr;
	int err = 0;
	ptr = (int *)data;

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_DBG("mcu_mode == CW_BOOT!\n");
		return FAIL;
	}

	power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
	if(CWMCU_I2C_R(sensor, RegMapR_GetAccelerationRawData+id, data, 12)<0){
		SH_ERR("read RawData[%d] failed!\n",RegMapR_GetAccelerationRawData+id);
		err = -1;
	}
	power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
	memcpy(rawdata,ptr,sizeof(int)*3);
	return err;
}

static ssize_t sensorhub_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t fw_data[4];
	int16_t version = -1;
	uint8_t data[10] = {0};
	uint32_t mcu_enabled_list[2] ={0};
	int len = 0;
	int i = 0;
	uint8_t status[31] = {0};
	int rawdata[3] = {0};

	if (sensor->mcu_mode == CW_BOOT)
		return 0;

	len += sprintf(buf+len , "=================================================================\n");
	/* version show*/
	power_pin_sw(sensor,SWITCH_POWER_VERSION, 1);
	if (CWMCU_I2C_R(sensor, RegMapR_GetFWVersion, fw_data, 4) >= 0) {
		version = (int16_t)( ((uint16_t)fw_data[1])<<8 | (uint16_t)fw_data[0]);
		printk("%s:%s:(CHECK_FIRMWAVE_VERSION : M:%u,D:%u,V:%u,SV:%u)\n",LOG_TAG_HAL ,__FUNCTION__, fw_data[3], fw_data[2], fw_data[1], fw_data[0]);
	}else{
		SH_ERR("Read GetFWVersion Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);
		data[0] = 1;
		data[1] = 0;
	}
	power_pin_sw(sensor,SWITCH_POWER_VERSION, 0);

	len += sprintf(buf+len , "Kernel version : %s \nFirmware version : %d.%d\n",MAIN_VERSION, fw_data[1],fw_data[0]);

	len += sprintf(buf+len , "Power control flag:%d\n",sensor->power_on_list);
	len += sprintf(buf+len , "GPIO Status:Irq[%d:%d]:WakeUp[%d:%d]:Reset[%d:%d]:Boot[%d:%d]\n",
			sensor->irq_gpio,cwm_gpio_read(sensor->irq_gpio),sensor->wakeup_gpio,cwm_gpio_read(sensor->wakeup_gpio),
			sensor->reset_gpio,cwm_gpio_read(sensor->reset_gpio),sensor->boot_gpio,cwm_gpio_read(sensor->boot_gpio));

	/* flag show */

	len += sprintf(buf+len , "mcu_init_count: %d\nmcu_status: %d\nmcu_mode: %d\n", sensor->mcu_init_count,sensor->mcu_status,sensor->mcu_mode);

	/* active show (sensor_hub info) */
	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);
	if (CWMCU_I2C_R(sensor, RegMapR_GetHostEnableList, data, 8) >= 0)
	{
		mcu_enabled_list[NonWakeUpHandle] = (uint32_t)data[3]<<24 |(uint32_t)data[2]<<16 |(uint32_t)data[1]<<8 |(uint32_t)data[0];
		mcu_enabled_list[WakeUpHandle] = (uint32_t)data[7]<<24 |(uint32_t)data[6]<<16 |(uint32_t)data[5]<<8 |(uint32_t)data[4];
		if (sensor->debug_log & (1<<D_EN)) 
			SH_LOG("MCU En Status:%d,%d\n", mcu_enabled_list[NonWakeUpHandle], mcu_enabled_list[WakeUpHandle]);
	}
	else
	{
		SH_ERR(" Read GetHostEnableList Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
	}
	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);

	len += sprintf(buf+len , "mcu(Nonwp , wp): %d , %d | AP(Nonwp , wp): %d , %d\n",mcu_enabled_list[NonWakeUpHandle],
			mcu_enabled_list[WakeUpHandle], sensor->enabled_list[NonWakeUpHandle],sensor->enabled_list[WakeUpHandle]);

	/* sensor status include : nonwakeup_sensor*/

	len += sprintf(buf+len , "=================================================================\n");
	for(i = 0; i < SENSORS_ID_END; i++){
		len += sprintf(buf+len ,"id: %2d , en[0]: %1d , rate: %4d , timeout: %6d , en[1]: %1d , rate: %4d , timeout: %6d \n",i
				,sensor->sensors_info[NonWakeUpHandle][i].en , sensor->sensors_info[NonWakeUpHandle][i].rate, sensor->sensors_info[NonWakeUpHandle][i].timeout
				,sensor->sensors_info[WakeUpHandle][i].en , sensor->sensors_info[WakeUpHandle][i].rate, sensor->sensors_info[WakeUpHandle][i].timeout
			      );
	}

	get_all_calib_status( sensor, sensor->CalibratorStatus);
	get_all_selftest_status( sensor, sensor->SelfTestStatus);
	for(i = 0; i < DRIVER_ID_END; i++){
		get_mcu_sensors_info(sensor,i,status);
		len += sprintf(buf+len ,"[%2d]:ChipId:[%2x] Addres:[%2x] HwId:[%2x] HwType:[%2x] Calib[%2x] SelfTest[%2x]\n",
				i,status[0],status[1],status[2],status[3],sensor->CalibratorStatus[i],sensor->SelfTestStatus[i]);
		if(read_sensors_rawdata(sensor,i,rawdata) <0){
			SH_ERR("read RawData[%d] failed!\n",i);
			len += sprintf(buf+len , "[%2d]:Read Error\n",i);
		}else{
			len += sprintf(buf+len , "[%2d]:%d\t%d\t%d\n", i,rawdata[0], rawdata[1], rawdata[2]);
		}
	}
	len += sprintf(buf+len , "=================================================================\n");

	return len;
}

static void read_calib_info(struct CWMCU_T *sensor)
{
	uint8_t data[24] = {0};
	int status = 0;    
	uint16_t *ptr;
	ptr = (uint16_t *)data;

	if (sensor->mcu_mode == CW_BOOT) {
		SH_DBG("mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
		return;
	}

	if(set_calib_cmd(sensor, CALIB_CHECK_STATUS, sensor->cal_id, sensor->cal_type)){
		SH_ERR("I2c Write Fail!\n");
		return;
	}

	if(sensors_calib_status(sensor,  sensor->cal_id, &status) >=0){
		SH_DBG("Calib id:%d:status:%d\n", sensor->cal_id , status);
		if(status ==CALIB_STATUS_PASS){
			ptr[0] =  (uint16_t)sensor->cal_id;
			cw_send_event(sensor,NonWakeUpHandle,CALIBRATOR_UPDATE,data);
		}
	}
	return ;
}

#ifdef SUPPORT_HALL_SENSOR

static void read_hall_sensor(struct CWMCU_T *sensor)
{
	uint8_t data[4] = {0};
	int8_t *ptr;
	int err;
	ptr = (int8_t *)data;
	err = CWMCU_I2C_R(sensor, RegMapR_HallStatus, data, 4);
	if (err < 0)
		SH_ERR("I2c Read Fail!\n");

	SH_DBG("%d,%d,%d,%d)\n", ptr[0], ptr[1], ptr[2], ptr[3]);
	return ;
}
#endif

static void read_error_code(struct CWMCU_T *sensor)
{
	uint8_t data[4] = {0};
	int8_t *ptr;
	int err;
	ptr = (int8_t *)data;
	err = CWMCU_I2C_R(sensor, RegMapR_ErrorCode, data, 4);
	if (err < 0)
		SH_ERR("I2c Write Fail!\n");
	if (ptr[0] == ERR_TASK_BLOCK)
	{
		SH_DBG("ERR_TASK_BLOCK\n");
	}
	SH_DBG("%s:%d,%d,%d,%d)\n",__FUNCTION__ , ptr[0], ptr[1], ptr[2], ptr[3]);
}
static ssize_t get_raw_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int i;
	int data[3];
	int len = 0;

	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_DBG("mcu_mode == CW_BOOT!\n");
		return FAIL;
	}
	for(i=0;i<DRIVER_ID_END;i++){
		if(read_sensors_rawdata(sensor,i,data) <0){
			SH_ERR("read RawData[%d] failed!\n",i);
			len += sprintf(buf+len , "[%d]:Read Error\n",i);
		}else{
			len += sprintf(buf+len , "[%d]:%d\t%d\t%d\n", i,data[0], data[1], data[2]);
		}

	}
	return len;
}

static ssize_t get_mag_special_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t data[64];
	uint16_t *ptr;
	int err;
	ptr = (uint16_t *)data;
	SH_FUN();
	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_DBG("mcu_mode == CW_BOOT!\n");
		return FAIL;
	}
	power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
	err = CWMCU_Object_read(sensor, RegMapR_MagSpecialData, data, 64);
	power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
	if (err < 0) 
	{
		SH_ERR("read RegMapR_MagSpecialData failed!\n");
		return err;
	}
	memcpy(buf,data,64);
	return 64;
}

static ssize_t set_sys_msg(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t data[40] = {0};
	int temp[40] = {0};
	int i,err = 0;

	if (sensor->mcu_mode == CW_BOOT) 
	{
		SH_ERR("mcu_mode == CW_BOOT!\n");
		return count;
	}

	sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
			&temp[0], &temp[1], &temp[2],
			&temp[3], &temp[4], &temp[5], &temp[6], &temp[7], &temp[8], &temp[9], &temp[10], &temp[11], &temp[12],
			&temp[13], &temp[14], &temp[15], &temp[16], &temp[17], &temp[18], &temp[19], &temp[20], &temp[21], &temp[22],
			&temp[23], &temp[24], &temp[25], &temp[26], &temp[27], &temp[28], &temp[29], &temp[30], &temp[31]);

	for (i = 0 ; i < 40; i++)
		data[i] = (uint8_t)temp[i];

	if(data[0] == GPSINFO)
	{
		memset(sensor->GPSData, 0 ,sizeof(uint8_t)*40);
		memcpy(sensor->GPSData, data, sizeof(uint8_t)*40);
	}    
	if(data[0] == HEALTHINFO)
	{
		memset(sensor->HEALTHData, 0 ,sizeof(uint8_t)*30);
		memcpy(sensor->HEALTHData, data, sizeof(uint8_t)*30);
	}

	power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
	err = CWMCU_I2C_W(sensor, RegMapW_SetSystemMsg, data, 40);
	power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
	if (err < 0) 
		SH_ERR("I2c Write Fail!\n");

	return count;
}

static ssize_t get_sys_msg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t extrahub_data[30] = {0};
	int err;

	if (sensor->mcu_mode == CW_BOOT)
	{
		SH_ERR("mcu_mode == CW_BOOT!\n");
		return 0;
	}

	power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
	err = CWMCU_I2C_R(sensor, RegMapR_GetSystemMsg, extrahub_data, 30);
	power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", 
			err,
			extrahub_data[0], extrahub_data[1], extrahub_data[2],extrahub_data[3], extrahub_data[4], extrahub_data[5], extrahub_data[6], extrahub_data[7], 
			extrahub_data[8], extrahub_data[9], extrahub_data[10], extrahub_data[11], extrahub_data[12], extrahub_data[13], extrahub_data[14], extrahub_data[15], 
			extrahub_data[16], extrahub_data[17], extrahub_data[18], extrahub_data[19], extrahub_data[20], extrahub_data[21], extrahub_data[22],
			extrahub_data[23], extrahub_data[24], extrahub_data[25], extrahub_data[26], extrahub_data[27], extrahub_data[28], extrahub_data[29]);
}

#ifdef SUPPORT_INPUT
void report_input(struct CWMCU_T *sensor)
{
	input_report_key(sensor->input, KEY_POWER, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_POWER, 0);
	input_sync(sensor->input);
	return;
}
#endif

#ifndef CWMCU_CALIB_SAVE_IN_FLASH
static void reload_calib_data(struct CWMCU_T *sensor)
{
	int i;
	for(i = 0;i < DRIVER_ID_END ; i ++)
	{
		if(sensor->calibratorUpdate[i])
		{
			sensors_calib_data_write(sensor, i, sensor->calibratordata[i]);
			msleep(10);
		}
	}
}
#endif

int CWM_Restore_GPSINFO(struct CWMCU_T *sensor)
{
	int err = 0;
	if (sensor->mcu_mode == CW_BOOT) 
	{
		SH_ERR("mcu_mode == CW_BOOT!\n");
		return 0;
	}

	power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
	err = CWMCU_I2C_W(sensor, RegMapW_SetSystemMsg, sensor->GPSData, 40);
	power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
	if (err < 0) 
	{
		SH_ERR("I2c Write Fail!\n");
		return 0;
	}
	return 1;
}

int CWM_Restore_HealthINFO(struct CWMCU_T *sensor)
{
	int err = 0;
	if (sensor->mcu_mode == CW_BOOT) 
	{
		SH_ERR("mcu_mode == CW_BOOT!\n");
		return 0;
	}

	power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
	err = CWMCU_I2C_W(sensor, RegMapW_SetSystemMsg, sensor->HEALTHData, 30);
	power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
	if (err < 0) 
	{
		SH_ERR("I2c Write Fail!\n");
		return 0;
	}
	return 1;
}

static void cwmcu_reinit(struct CWMCU_T *sensor)
{
	sensor->mcu_init_count++;
#ifndef CWMCU_CALIB_SAVE_IN_FLASH
	reload_calib_data(sensor);
#endif
	check_enable_list(sensor);
	cwm_set_kernel_status(sensor,KERNEL_RESUME);

	CWM_Restore_GPSINFO(sensor);
	CWM_Restore_HealthINFO(sensor);
}

static struct device_attribute attributes[] = {
	__ATTR(enable, S_IRUGO,  active_show, active_set),
	__ATTR(batch, S_IWUGO, NULL, batch_set),
	__ATTR(flush, S_IWUGO, NULL, flush_set),
	__ATTR(mcu_mode, S_IRUGO | S_IWUGO, mcu_mode_show, mcu_model_set),
	__ATTR(calibrator_cmd, S_IRUGO,  get_calibrator_cmd, set_calibrator_cmd),
	__ATTR(calibrator_data, S_IRUGO, get_calibrator_data, set_calibrator_data),
	__ATTR(firmware_update_i2c, S_IRUGO, get_firmware_update_i2, set_firmware_update_i2),
	__ATTR(firmware_update_cmd, S_IWUGO, NULL, set_firmware_update_cmd),
	__ATTR(firmware_update_status, S_IRUGO, get_firmware_update_status, NULL),
	__ATTR(version, S_IRUGO,  version_show, NULL),
	__ATTR(library_version, S_IRUGO,  library_version_show, NULL),
	__ATTR(timestamp, S_IRUGO, timestamp_show, NULL),			
	__ATTR(sys_cmd, S_IWUGO,  NULL, set_sys_cmd),
	__ATTR(sys_msg, S_IWUGO,  get_sys_msg, set_sys_msg),
	__ATTR(raw_data, S_IRUGO, get_raw_data, NULL),
	__ATTR(mag_special_data, S_IRUGO, get_mag_special_data, NULL),
	__ATTR(ssh_info, S_IRUGO , sensorhub_info_show, NULL),
};

static void CWMCU_IRQ(struct CWMCU_T *sensor)
{
	uint8_t temp[2] = {0};
	uint8_t data_event[24] = {0};

	if (sensor->mcu_mode == CW_BOOT) {
		SH_DBG("mcu_mode = BOOT\n");
		return;
	}

	sensor->interrupt_status = 0;
	if (CWMCU_I2C_R(sensor, RegMapR_InterruptStatus, temp, 2) >= 0)
	{		
		sensor->interrupt_status = (u32)temp[1] << 8 | (u32)temp[0];
		if (sensor->debug_log & (1<<D_IRQ)){
			SH_LOG("interrupt_status:%x ,temp[0-1]: %x %x\n ",
					sensor->interrupt_status, (u32)temp[0],(u32)temp[1]);
		}
		if( sensor->interrupt_status >= (1<<IRQ_MAX_SIZE)){
			sensor->interrupt_status = 0;
			SH_DBG("interrupt_status > IRQ_MAX_SIZE\n");
		}
	}
	else
	{
		SH_ERR(" Read interrupt_status Fail [I2C]\n");
		sensor->interrupt_status = 0;
	}

	if (sensor->interrupt_status & (1<<IRQ_INIT))
	{
		cwmcu_reinit(sensor);
		cw_send_event(sensor, NonWakeUpHandle, MCU_REINITIAL, data_event);
	}

	if (sensor->interrupt_status & (1<<IRQ_GESTURE))
	{
		cwmcu_read_gesture(sensor);
	}

	if ((sensor->interrupt_status & (1<<IRQ_BATCH_TIMEOUT)) ||(sensor->interrupt_status & (1<<IRQ_BATCH_FULL)) ) 
	{
		cwmcu_read_buff(sensor,WakeUpHandle);
	}

	if (sensor->interrupt_status & (1<<IRQ_DATA_READY))
	{
		cwmcu_read_buff(sensor, NonWakeUpHandle);
	}

#ifdef SUPPORT_INPUT
	if (sensor->interrupt_status & (1<<IRQ_INPUT))
	{
		report_input(sensor);
	}
#endif
#ifdef SUPPORT_HALL_SENSOR
	if (sensor->interrupt_status & (1<<IRQ_HALL)) 
	{
		read_hall_sensor(sensor);
	}
#endif
	if (sensor->interrupt_status & (1<<IRQ_INFO))
	{
		read_mcu_info(sensor);
	}
	if (sensor->interrupt_status & (1<<IRQ_CALIB)) 
	{
		read_calib_info(sensor);
	}
	if (sensor->interrupt_status & (1<<IRQ_ERROR)) 
	{
		read_error_code(sensor);
	}

}

#ifdef CWM_USE_DELAY_WORK
static int CWMCU_POLLING(struct CWMCU_T *sensor)
{
	power_pin_sw(sensor,SWITCH_POWER_POLLING, 1);
	CWMCU_IRQ(sensor);
	power_pin_sw(sensor,SWITCH_POWER_POLLING, 0);
	return 0;
}
#endif

static int CWMCU_suspend(struct device *dev)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int error;
	SH_DBG(" CWMCU_suspend. sensor enabled list wkup= %d, nonwkup=%d\n", sensor->enabled_list[WakeUpHandle],sensor->enabled_list[NonWakeUpHandle]);

#ifdef CWM_USE_DELAY_WORK
	cancel_delayed_work_sync(&sensor->delay_work);
#endif
	power_pin_sw(sensor,SWITCH_POWER_PROBE, 1);
	cwm_set_kernel_status(sensor,KERNEL_SUSPEND);
	power_pin_sw(sensor,SWITCH_POWER_PROBE, 0);
	if(!(sensor->enabled_list[NonWakeUpHandle] ||sensor->enabled_list[WakeUpHandle]) ){
		error = cwstm_power_on(sensor,false);
		if (error) {
			SH_ERR("power on failed");
		}
	}

	mutex_lock(&sensor->mutex_lock);

	return 0;
}

static int CWMCU_resume(struct device *dev)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int error;

	SH_DBG(" CWMCU_resume.\n");

	mutex_unlock(&sensor->mutex_lock);
	if(!(sensor->enabled_list[NonWakeUpHandle] ||sensor->enabled_list[WakeUpHandle]) ){
		error = cwstm_power_on(sensor,true);
		if (error) {
			SH_ERR("power on failed");
		}
	}

	power_pin_sw(sensor,SWITCH_POWER_PROBE, 1);
	cwm_set_kernel_status(sensor,KERNEL_RESUME);
	power_pin_sw(sensor,SWITCH_POWER_PROBE, 0);
#ifdef CWM_USE_IRQ_WORK
	queue_work(sensor->driver_wq, &sensor->work);
#endif
#ifdef CWM_USE_DELAY_WORK
	queue_delayed_work(sensor->driver_wq, &sensor->delay_work,
			msecs_to_jiffies(atomic_read(&sensor->delay)));
#endif

	return 0;
}

/*=======iio device reg=========*/
static void iio_trigger_work(struct irq_work *work)
{
	struct CWMCU_T *mcu_data = container_of((struct irq_work *)work, struct CWMCU_T, iio_irq_work);
	iio_trigger_poll(mcu_data->trig, iio_get_time_ns());
}

static irqreturn_t cw_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct CWMCU_T *mcu_data = iio_priv(indio_dev);

	mutex_lock(&mcu_data->mutex_lock);
	iio_trigger_notify_done(mcu_data->indio_dev->trig);
	mutex_unlock(&mcu_data->mutex_lock);

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops cw_buffer_setup_ops = {
	.preenable = &iio_sw_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static int cw_pseudo_irq_enable(struct iio_dev *indio_dev)
{
	struct CWMCU_T *mcu_data = iio_priv(indio_dev);

	if (!atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 0, 1))
	{
		SH_FUN();
#ifdef CWM_USE_IRQ_WORK
		cancel_work_sync(&mcu_data->work);
		queue_work(mcu_data->driver_wq, &mcu_data->work);
#endif
#ifdef CWM_USE_DELAY_WORK
		cancel_delayed_work_sync(&mcu_data->delay_work);
		queue_delayed_work(mcu_data->driver_wq, &mcu_data->delay_work, 0);
#endif
	}

	return 0;
}

static int cw_pseudo_irq_disable(struct iio_dev *indio_dev)
{
	struct CWMCU_T *mcu_data = iio_priv(indio_dev);

	if (atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 1, 0))
	{
#ifdef CWM_USE_IRQ_WORK
		cancel_work_sync(&mcu_data->work);
#endif
#ifdef CWM_USE_DELAY_WORK
		cancel_delayed_work_sync(&mcu_data->delay_work);
#endif
		SH_FUN();
	}
	return 0;
}

static int cw_set_pseudo_irq(struct iio_dev *indio_dev, int enable)
{
	if (enable)
		cw_pseudo_irq_enable(indio_dev);
	else
		cw_pseudo_irq_disable(indio_dev);
	return 0;
}

static int cw_data_rdy_trigger_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = (struct iio_dev *)iio_trigger_get_drvdata(trig);
	struct CWMCU_T *mcu_data = iio_priv(indio_dev);

	mutex_lock(&mcu_data->mutex_lock);
	cw_set_pseudo_irq(indio_dev, state);
	mutex_unlock(&mcu_data->mutex_lock);

	return 0;
}

static const struct iio_trigger_ops cw_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &cw_data_rdy_trigger_set_state,
};

static int cw_probe_trigger(struct iio_dev *iio_dev)
{
	struct CWMCU_T *mcu_data = iio_priv(iio_dev);
	int ret;

	iio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time, &cw_trigger_handler, 
			IRQF_ONESHOT, iio_dev, "%s_consumer%d", iio_dev->name, iio_dev->id);
	if (NULL == iio_dev->pollfunc)
	{
		ret = -ENOMEM;
		goto error_ret;
	}

	mcu_data->trig = iio_trigger_alloc("%s-dev%d",
			iio_dev->name,
			iio_dev->id);
	if (!mcu_data->trig) {
		ret = -ENOMEM;
		goto error_dealloc_pollfunc;
	}

	mcu_data->trig->dev.parent = &mcu_data->client->dev;
	mcu_data->trig->ops = &cw_trigger_ops;
	iio_trigger_set_drvdata(mcu_data->trig, iio_dev);

	ret = iio_trigger_register(mcu_data->trig);
	if (ret)
		goto error_free_trig;

	return 0;

error_free_trig:
	iio_trigger_free(mcu_data->trig);
error_dealloc_pollfunc:
	iio_dealloc_pollfunc(iio_dev->pollfunc);
error_ret:
	return ret;
}

static int cw_probe_buffer(struct iio_dev *iio_dev)
{
	int ret;
	struct iio_buffer *buffer;

	buffer = iio_kfifo_allocate(iio_dev);
	if (!buffer)
	{
		ret = -ENOMEM;
		goto error_ret;
	}

	buffer->scan_timestamp = true;
	iio_dev->buffer = buffer;
	iio_dev->setup_ops = &cw_buffer_setup_ops;
	iio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_buffer_register(iio_dev, iio_dev->channels,
			iio_dev->num_channels);
	if (ret)
		goto error_free_buf;

	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_ID);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_X);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Y);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Z);
	return 0;

error_free_buf:
	iio_kfifo_free(iio_dev->buffer);
error_ret:
	return ret;
}

static int cw_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{
	struct CWMCU_T *mcu_data = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return ret;

	mutex_lock(&mcu_data->mutex_lock);

	switch (mask)
	{
		case 0:
			*val = mcu_data->iio_data[chan->channel2 - IIO_MOD_X];
			ret = IIO_VAL_INT;
			break;

		case IIO_CHAN_INFO_SCALE:
			/* Gain : counts / uT = 1000 [nT] */
			/* Scaling factor : 1000000 / Gain = 1000 */
			*val = 0;
			*val2 = 1000;
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;

		default:
			break;
	}

	mutex_unlock(&mcu_data->mutex_lock);
	return ret;
}

#define CW_CHANNEL(axis)			\
{						\
	.type = IIO_ACCEL,			\
	.modified = 1,				\
	.channel2 = axis+1,			\
	.info_mask = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = axis,			\
	.scan_type = IIO_ST('u', 32, 32, 0)	\
}

static const struct iio_chan_spec cw_channels[] = {
	CW_CHANNEL(CW_SCAN_ID),
	CW_CHANNEL(CW_SCAN_X),
	CW_CHANNEL(CW_SCAN_Y),
	CW_CHANNEL(CW_SCAN_Z),
	IIO_CHAN_SOFT_TIMESTAMP(CW_SCAN_TIMESTAMP)
};

static const struct iio_info cw_info = {
	.read_raw = &cw_read_raw,
	.driver_module = THIS_MODULE,
};

static int create_sysfs_interfaces(struct CWMCU_T *mcu_data)
{
	int i;
	int res;

	mcu_data->sensor_class = class_create(THIS_MODULE, "cywee_sensorhub");
	if (IS_ERR(mcu_data->sensor_class))
		return PTR_ERR(mcu_data->sensor_class);

	mcu_data->sensor_dev = device_create(mcu_data->sensor_class, NULL, 0, "%s", "sensor_hub");
	if (IS_ERR(mcu_data->sensor_dev))
	{
		res = PTR_ERR(mcu_data->sensor_dev);
		goto err_device_create;
	}

	res = dev_set_drvdata(mcu_data->sensor_dev, mcu_data);
	if (res)
		goto err_set_drvdata;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(mcu_data->sensor_dev, attributes + i))
			goto error;

	res = sysfs_create_link(&mcu_data->sensor_dev->kobj, &mcu_data->indio_dev->dev.kobj, "iio");
	if (res < 0)
		goto error;

	return 0;

error:
	while (--i >= 0)
		device_remove_file(mcu_data->sensor_dev, attributes + i);
err_set_drvdata:
	put_device(mcu_data->sensor_dev);
	device_unregister(mcu_data->sensor_dev);
err_device_create:
	class_destroy(mcu_data->sensor_class);
	return res;
}

#ifdef SUPPORT_INPUT
static int cwmcu_input_init(struct input_dev **input)
{
	int err;

	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	set_bit(EV_KEY, (*input)->evbit);

	input_set_capability(*input, EV_KEY, 116);
	input_set_capability(*input, EV_KEY, 102);
	input_set_capability(*input, EV_KEY, KEY_POWER);

#ifdef TOUCH
	__set_bit(EV_KEY, (*input)->evbit);
	__set_bit(EV_ABS, (*input)->evbit);
	__set_bit(BTN_TOUCH, (*input)->keybit);
	__set_bit(KEY_POWER, (*input)->keybit);
	__set_bit(INPUT_PROP_DIRECT, (*input)->propbit);

	input_set_abs_params((*input), ABS_MT_POSITION_X, 0,
			ST_CFG_X_RESOLUTION, 0, 0);
	input_set_abs_params((*input), ABS_MT_POSITION_Y, 0,
			ST_CFG_Y_RESOLUTION, 0, 0);
	input_set_abs_params((*input), ABS_MT_TRACKING_ID, 0,
			CFG_MAX_TOUCH_POINTS, 0, 0);
	input_set_abs_params((*input), ABS_MT_TOUCH_MAJOR, 0, ST_PRESS, 0, 0);
	input_set_abs_params((*input), ABS_MT_PRESSURE, 0, ST_PRESS, 0, 0);
#endif

	(*input)->name = CWMCU_I2C_NAME;

	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}
#endif

#ifdef CWM_USE_IRQ_WORK
static irqreturn_t CWMCU_interrupt_thread(int irq, void *data)
{
	struct CWMCU_T *sensor = data;
	if (sensor->mcu_mode == CW_BOOT) {
		SH_DBG("sensor->mcu_mode = CW_BOOT\n");
		return IRQ_HANDLED;
	}
	schedule_work(&sensor->work);

	return IRQ_HANDLED;
}

static void cwmcu_work_report(struct work_struct *work)
{

	struct CWMCU_T *sensor = container_of((struct work_struct *)work,
			struct CWMCU_T, work);

	if (sensor->mcu_mode == CW_BOOT) {
		SH_DBG("sensor->mcu_mode = CW_BOOT\n");
		return ;
	}

	power_pin_sw(sensor,SWITCH_POWER_INTERRUPT, 1);
	CWMCU_IRQ(sensor);
	power_pin_sw(sensor,SWITCH_POWER_INTERRUPT, 0);
}
#endif

#ifdef CWM_USE_DELAY_WORK
static void cwmcu_delwork_report(struct work_struct *work)
{
	struct CWMCU_T *sensor = container_of((struct delayed_work *)work,
			struct CWMCU_T, delay_work);

	if (CW_BOOT == sensor->mcu_mode)
	{
		SH_DBG("sensor->mcu_mode = CW_BOOT\n");
	}
	CWMCU_POLLING(sensor);
	SH_DBG(" POLLING\n");
	queue_delayed_work(sensor->driver_wq, &sensor->delay_work,msecs_to_jiffies(atomic_read(&sensor->delay)));
}
#endif

#ifdef CWM_USE_ERROR_HANDLE_WORK
static void cwmcu_error_handle_report(struct work_struct *work)
{
	struct CWMCU_T *sensor = container_of((struct delayed_work *)work,
			struct CWMCU_T, error_handle_work);
	uint8_t data[2];
	/*When firmware_update_status != 2, Firmware update daemon is in process, we can't do any thing*/
	if(sensor->firmware_update_status != 2){
		SH_LOG("Mcu not response, we will keep mcu to boot mode[%d]\n",sensor->mcu_mode);
		queue_delayed_work(sensor->driver_wq, &sensor->error_handle_work, msecs_to_jiffies(CWM_MAX_ERROR_WORK_TIME));
		return;
	}
	SH_ERR("In: Error count:%d; Error state:%d\n",sensor->i2c_error_count,sensor->i2c_error_state);
	if(sensor->i2c_error_state == CWM_LEVEL1_MCU_RESET){
		mutex_lock(&sensor->mutex_lock);
		msleep(100);
		cwm_reset_mcu(sensor);
		msleep(100);
		mutex_unlock(&sensor->mutex_lock);
	}else if(sensor->i2c_error_state == CWM_LEVEL2_MCU_RESET){
		mutex_lock(&sensor->mutex_lock);
		msleep(100);
		cwm_reset_mcu(sensor);
		msleep(100);
		mutex_unlock(&sensor->mutex_lock);    
	}else if(sensor->i2c_error_state == CWM_LEVEL3_MCU_STOP){
		mutex_lock(&sensor->mutex_lock);
		msleep(100);
		cwm_reset_mcu(sensor);
		msleep(300);
		mutex_unlock(&sensor->mutex_lock);
		sensor->mcu_mode = CW_NORMAL;
		if (CWMCU_I2C_R(sensor, RegMapR_ChipId, data, 2) >= 0)
		{
			SH_LOG(" Check Chip Id %X,%X\n", data[0], data[1]);
			sensor->i2c_error_count = 0;
			sensor->i2c_error_state = CWM_LEVEL0_MCU_NON;
		}else{
			sensor->mcu_mode = CW_BOOT;
			SH_LOG("Mcu not response, we will keep mcu to boot mode[%d]\n",sensor->mcu_mode);
			queue_delayed_work(sensor->driver_wq, &sensor->error_handle_work, msecs_to_jiffies(CWM_MAX_ERROR_WORK_TIME));
		}
	}
	SH_ERR("Out: Error count:%d; Error state:%d\n",sensor->i2c_error_count,sensor->i2c_error_state);
}
#endif

static int cwstm_parse_dt(struct device *dev,
		struct CWMCU_T *sensor)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = of_get_named_gpio(np, "cwstm,irq-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"cwstm,irq_gpio\"\n");
		goto err;
	}
	sensor->irq_gpio = ret;
	ret = of_get_named_gpio(np, "cwstm,wakeup-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"wakeup\"\n");
		goto err;
	}
	sensor->wakeup_gpio = ret;

	ret = of_get_named_gpio(np, "cwstm,reset-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"cwstm,reset-gpio\"\n");
		goto err;
	}
	sensor->reset_gpio = ret;

	ret = of_get_named_gpio(np, "cwstm,boot-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"cwstm,boot-gpio\"\n");
		goto err;
	}
	sensor->boot_gpio = ret;
	pr_err("irq_gpio = %d, wakeup_gpio = %d, reset_gpio = %d, boot_gpio = %d\n", sensor->irq_gpio, sensor->wakeup_gpio, sensor->reset_gpio, sensor->boot_gpio);

err:
	return ret;
}


static int cwm_platform_gpio_init(struct CWMCU_T *sensor)
{
#ifdef QUALCOMM_PLATFORM

	int retval = 0;

	retval = cwm_wakeup_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT,0);
	if (retval < 0) {
		goto err_gpio_wakeup;
	}
	retval = cwm_boot_gpio_setup(sensor, 1, CWM_GPIO_OUTPUT, 0);
	if (retval < 0) {
		goto err_gpio_boot;
	}
	retval = cwm_reset_gpio_setup( sensor, 1, CWM_GPIO_OUTPUT, 1);
	if (retval < 0) {
		goto err_gpio_reset;
	}

	return 0;

err_gpio_reset:
	cwm_reset_gpio_setup(sensor,0,0,0);
err_gpio_boot:
	cwm_boot_gpio_setup(sensor,0,0,0);
err_gpio_wakeup:
	cwm_wakeup_gpio_setup(sensor,0,0,0);
	return retval;

#elif defined(SAMSUNG_PLATFORM)
#elif defined(MTK_PLATFORM)
#else
#error Missing definition:Platform define
#endif
}

static int cwm_platform_irq_init(struct CWMCU_T *sensor){
#ifdef QUALCOMM_PLATFORM
	int retval = 0;
#ifdef CWM_USE_IRQ_WORK

	retval = cwm_irq_gpio_setup(sensor , 1, CWM_GPIO_IRQ, 1);
	if (retval < 0) {
		SH_LOG("Request GPIO IRQ %d Fail\n", retval);
		goto err_gpio_irq;
	}

	if(sensor->client->irq <=0)
		goto err_gpio_irq;

	retval = request_threaded_irq(sensor->client->irq, NULL,
			CWMCU_interrupt_thread,
			IRQF_TRIGGER_FALLING |  IRQF_ONESHOT,
			"cwmcu", sensor);
	if (retval < 0) {
		SH_LOG("Request IRQ %d Fail\n", sensor->client->irq);
		goto err_gpio_irq;
	}
	INIT_WORK(&sensor->work, cwmcu_work_report);

	if (enable_irq_wake(sensor->client->irq) < 0)
		SH_LOG("Could not enable irq as wakeup source\n");

	return retval;
err_gpio_irq:
#endif
	return retval;
#elif defined(SAMSUNG_PLATFORM)
#elif defined(MTK_PLATFORM)
#else
#error Missing definition:Platform define
#endif

}


static void cwmcu_remove_trigger(struct iio_dev *indio_dev)
{
	struct CWMCU_T *mcu_data = iio_priv(indio_dev);

	iio_trigger_unregister(mcu_data->trig);
	iio_trigger_free(mcu_data->trig);
	iio_dealloc_pollfunc(indio_dev->pollfunc);
}

static void cwmcu_remove_buffer(struct iio_dev *indio_dev)
{
	iio_buffer_unregister(indio_dev);
	iio_kfifo_free(indio_dev->buffer);
}


static int cwstm_power_on(struct CWMCU_T *sensor,bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(sensor->vdd);
	if (rc)
	{
		SH_ERR("Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(sensor->vcc_i2c);
	if (rc)
	{
		SH_ERR("Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(sensor->vdd);
	}

	rc = regulator_enable(sensor->vcc_sensors);
	if (rc)
	{
		SH_ERR("Regulator vcc_sensors enable failed rc=%d\n", rc);
		regulator_disable(sensor->vdd);
		regulator_disable(sensor->vcc_i2c);
	}

	return rc;

power_off:
	rc = regulator_disable(sensor->vdd);
	if (rc)
	{
		SH_ERR("Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(sensor->vcc_i2c);
	if (rc)
	{
		SH_ERR("Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(sensor->vdd);
	}

	rc = regulator_disable(sensor->vcc_sensors);
	if (rc)
	{
		SH_ERR("Regulator vcc_sensors disable failed rc=%d\n", rc);
		rc = regulator_enable(sensor->vdd);
		rc = regulator_enable(sensor->vcc_i2c);
	}

	return rc;
}

static int cwstm_power_init(struct CWMCU_T *sensor,bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	sensor->vdd = regulator_get(&sensor->client->dev, "cwstm,vdd_ana");
	if (IS_ERR(sensor->vdd)) {
		rc = PTR_ERR(sensor->vdd);
		dev_err(&sensor->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(sensor->vdd) > 0) {
		rc = regulator_set_voltage(sensor->vdd, FT_VTG_MIN_UV,
				FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&sensor->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	sensor->vcc_i2c = regulator_get(&sensor->client->dev, "cwstm,vcc_i2c");
	if (IS_ERR(sensor->vcc_i2c)) {
		rc = PTR_ERR(sensor->vcc_i2c);
		dev_err(&sensor->client->dev,
				"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(sensor->vcc_i2c) > 0) {
		rc = regulator_set_voltage(sensor->vcc_i2c, FT_I2C_VTG_MIN_UV,
				FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&sensor->client->dev,
					"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	sensor->vcc_sensors = regulator_get(&sensor->client->dev, "cwstm,vcc_sensors");
	if (IS_ERR(sensor->vcc_sensors)) {
		rc = PTR_ERR(sensor->vcc_sensors);
		dev_err(&sensor->client->dev,
				"Regulator get failed vcc_sensors rc=%d\n", rc);
		goto reg_vcc_sensors_put;
	}

	if (regulator_count_voltages(sensor->vcc_sensors) > 0) {
		rc = regulator_set_voltage(sensor->vcc_sensors, FT_SENSORS_VTG_MIN_UV,
				FT_SENSORS_VTG_MAX_UV);
		if (rc) {
			dev_err(&sensor->client->dev,
					"Regulator set_vtg failed vcc_sensors rc=%d\n", rc);
			goto reg_vcc_sensors_put;
		}
	}

	return 0;

reg_vcc_sensors_put:
	regulator_put(sensor->vcc_sensors);
reg_vcc_i2c_put:
	regulator_put(sensor->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(sensor->vdd) > 0)
		regulator_set_voltage(sensor->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(sensor->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(sensor->vdd) > 0)
		regulator_set_voltage(sensor->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(sensor->vdd);

	if (regulator_count_voltages(sensor->vcc_i2c) > 0)
		regulator_set_voltage(sensor->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(sensor->vcc_i2c);

   if (regulator_count_voltages(sensor->vcc_sensors) > 0)
		regulator_set_voltage(sensor->vcc_sensors, 0, FT_SENSORS_VTG_MAX_UV);

   regulator_put(sensor->vcc_sensors);

	return 0;
}

static int cwm_delay_work_queue_init(struct CWMCU_T *sensor)
{
#ifdef CWM_USE_DELAY_WORK
	INIT_DELAYED_WORK(&sensor->delay_work, cwmcu_delwork_report);
#endif
	return 0;
}

static int cwm_error_handle_work_queue_init(struct CWMCU_T *sensor)
{
#ifdef CWM_USE_ERROR_HANDLE_WORK
	INIT_DELAYED_WORK(&sensor->error_handle_work, cwmcu_error_handle_report);
#endif
	return 0;
}

static int cwm_mutex_init(struct CWMCU_T *sensor)
{
	mutex_init(&sensor->mutex_lock);
	return 0;
}

static void cwmcu_hw_config_init(struct CWMCU_T *sensor)
{
	int i = 0;
	int j = 0;

	atomic_set(&sensor->delay, 20);
	for(i = 0; i < HANDLE_ID_END; i++)
	{
		sensor->enabled_list[i] = 0;
		for(j = 0;j<SENSORS_ID_END;j++)
		{
			sensor->sensors_info[i][j].en = 0;
			sensor->sensors_info[i][j].mode= 0;
			sensor->sensors_info[i][j].rate = 0;
			sensor->sensors_info[i][j].timeout= 0;
		}
	}

	sensor->mcu_init_count = 0;
	sensor->interrupt_status = 0;
	sensor->power_on_list = 0;
	sensor->cal_cmd = 0;
	sensor->cal_type = 0;
	sensor->cal_id = 0;
	sensor->debug_log = 0;

	sensor->irq_gpio = 0;
	sensor->wakeup_gpio = 0;
	sensor->reset_gpio = 0;
	sensor->boot_gpio = 0;
	sensor->i2c_error_count = 0;
	sensor->i2c_error_state = CWM_LEVEL0_MCU_NON;
	sensor->firmware_update_status = 2;

	for(i = 0;i<DRIVER_ID_END;i++)
	{
		sensor->hw_info[i].hw_id=0;
		sensor->calibratorUpdate[i]=0;
		for(j = 0;j<30;j++)
		{
			sensor->calibratordata[i][j]=0;
		}
		sensor->CalibratorStatus[i]=0;
		sensor->SelfTestStatus[i]=0;
	}

}

static int CWMCU_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct CWMCU_T *mcu;
	struct iio_dev *indio_dev;
	int error;

	SH_LOG(" MAIN_VERSION : %s \n", MAIN_VERSION);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "-CWMCU- i2c_check_functionality error\n");
		return -EIO;
	}

	indio_dev = iio_device_alloc(sizeof(*mcu));
	if (!indio_dev) {
		SH_ERR(" %s: iio_device_alloc failed\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, indio_dev);

	indio_dev->name = CWMCU_I2C_NAME;
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &cw_info;
	indio_dev->channels = cw_channels;
	indio_dev->num_channels = ARRAY_SIZE(cw_channels);
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	mcu = iio_priv(indio_dev);
	mcu->client = client;
	mcu->indio_dev = indio_dev;

	mcu->mcu_mode = CW_BOOT;

	/*Driver parameter Init*/
	cwmcu_hw_config_init(mcu);

	/*paser platform config*/
	error = cwstm_parse_dt(&client->dev, mcu);
	if (error < 0) {
		SH_ERR("failed to parse device tree: %d\n", error);
		goto err_parse_dt;
	}

	/*Gpio initial*/
	error = cwm_platform_gpio_init(mcu);
	if (error < 0) {
		SH_ERR("Failed to gpio initial: %d\n", error);
		goto err_gpio_init;
	}

	/*Power initial*/
	error = cwstm_power_init(mcu,true);
	if (error) {
		SH_ERR("power init failed");
	}
	/*Power on*/
	error = cwstm_power_on(mcu,true);
	if (error) {
		SH_ERR("power on failed");
	}

	/*Mcu reset*/
	cwm_reset_mcu(mcu);

	/*Mutex Init*/
	error = cwm_mutex_init(mcu);
	if (error < 0) {
		SH_ERR("Failed to initial mutex: %d\n", error);
	}

	/*IIO Init*/
	error = cw_probe_buffer(indio_dev);
	if (error) {
		printk("%s: iio yas_probe_buffer failed\n", __func__);
		goto error_free_dev;
	}
	error = cw_probe_trigger(indio_dev);
	if (error) {
		printk("%s: iio yas_probe_trigger failed\n", __func__);
		goto error_remove_buffer;
	}
	error = iio_device_register(indio_dev);
	if (error) {
		printk("%s: iio iio_device_register failed\n", __func__);
		goto error_remove_trigger;
	}

	/*Sysfs interfaces Init*/
	error = create_sysfs_interfaces(mcu);
	if (error)
		goto err_free_mem;

	init_irq_work(&mcu->iio_irq_work, iio_trigger_work);

	mcu->driver_wq = create_singlethread_workqueue("cywee_mcu");
	i2c_set_clientdata(client, mcu);
	pm_runtime_enable(&client->dev);

	/*Delay work queue Init*/
	error = cwm_delay_work_queue_init(mcu);
	if (error < 0) {
		SH_ERR("Failed to initial delay work queue: %d\n", error);
	}

	/*Error handle work queue Init*/
	error = cwm_error_handle_work_queue_init(mcu);
	if (error < 0) {
		SH_ERR("Failed to initial error handle work queue: %d\n", error);
	}

	/*Irq Init*/
	error = cwm_platform_irq_init(mcu);
	if (error) {
		SH_ERR("Failed to initial IRQ: %d\n", error);
		goto error_irq_fail;
	}

#ifdef SUPPORT_INPUT
	/*input device Init*/
	error = cwmcu_input_init(&mcu->input);
	if (error) {
		SH_ERR("%s: input_dev register failed\n",  __func__);
		goto error_input_dev;
	}
	input_set_drvdata(mcu->input, mcu);
#endif

	mcu->mcu_mode = CW_NORMAL;
	power_pin_sw(mcu,SWITCH_POWER_PROBE, 1);
	mcu->kernel_status = KERNEL_PROBE;
	error = CWMCU_I2C_W(mcu, RegMapW_SetHostStatus, &mcu->kernel_status, 1);
	power_pin_sw(mcu,SWITCH_POWER_PROBE, 0);
	if (error< 0)
	{
		SH_ERR("Write Kernel Status Fail");
		mcu->mcu_mode = CW_BOOT; //Wait for firmware update;
#ifdef CWM_USE_ERROR_HANDLE_WORK
		mcu->i2c_error_count +=20;
		mcu->i2c_error_state = CWM_LEVEL3_MCU_STOP;
		queue_delayed_work(mcu->driver_wq, &mcu->error_handle_work, msecs_to_jiffies(CWM_MAX_ERROR_WORK_TIME));
#endif
	}

	SH_LOG(" Sensor HUB Probe success. %s:%s:\n",LOG_TAG_KERNEL ,__FUNCTION__);
	return 0;

#ifdef SUPPORT_INPUT
error_input_dev:
#endif
error_irq_fail:
err_free_mem:
	iio_device_unregister(indio_dev);
error_remove_trigger:
	cwmcu_remove_trigger(indio_dev);
error_remove_buffer:
	cwmcu_remove_buffer(indio_dev);
error_free_dev:
err_gpio_init:
err_parse_dt:
	iio_device_free(indio_dev);
	i2c_set_clientdata(client, NULL);
	SH_ERR(" sensorhub probe fail.\n");
	return error;
}

static void CWMCU_shutdown(struct i2c_client *client)
{
	struct CWMCU_T *sensor = i2c_get_clientdata(client);
	power_pin_sw(sensor,SWITCH_POWER_SHUTDOWN, 1);
	cwm_set_kernel_status(sensor,KERNEL_SHUTDOWN);
	power_pin_sw(sensor,SWITCH_POWER_SHUTDOWN, 0);
}

static int CWMCU_i2c_remove(struct i2c_client *client)
{
	struct CWMCU_T *sensor = i2c_get_clientdata(client);
	kfree(sensor);
	return 0;
}

static struct of_device_id cwstm_match_table[] = {
	{ .compatible = "cwstm,cwstm32",},
	{ },
};

static const struct dev_pm_ops CWMCU_pm_ops = {
	.suspend = CWMCU_suspend,
	.resume = CWMCU_resume
};

static const struct i2c_device_id CWMCU_id[] = {
	{ CWMCU_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, CWMCU_id);

static struct i2c_driver CWMCU_driver = {
	.driver = {
		.name = CWMCU_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &CWMCU_pm_ops,
		.of_match_table = cwstm_match_table,
	},
	.probe    = CWMCU_i2c_probe,
	.shutdown = CWMCU_shutdown,
	.remove   = CWMCU_i2c_remove,
	.id_table = CWMCU_id,
};

static int __init CWMCU_i2c_init(void){
	printk("%s:%s:(init)\n",LOG_TAG_KERNEL ,__FUNCTION__);
	return i2c_add_driver(&CWMCU_driver);
}

static void __exit CWMCU_i2c_exit(void){
	i2c_del_driver(&CWMCU_driver);
}

module_init(CWMCU_i2c_init);
module_exit(CWMCU_i2c_exit);

MODULE_DESCRIPTION("CWMCU I2C Bus Driver");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
