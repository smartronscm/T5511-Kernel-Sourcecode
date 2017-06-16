#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/irqreturn.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
static struct i2c_client *g_i2c_client ;

struct usbswitch_chip {
struct device			*dev;
struct i2c_client       *i2client;
//struct regulator        *vdd;
//struct regulator        *vdd18;
//struct work_struct      intb_work;
int                     gpio_intpin;
int                     irq;
int                     resume_completed;
};

static int usbswitch_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= g_i2c_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(g_i2c_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int usbswitch_write_reg(u8 addr, u8* para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = 0x00;
    buf[1] = para[0];
	//buf[2] = para[1];
    ret = usbswitch_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
	pr_err("usbswitch_write_reg %x %d", buf[1], ret);
    return 0;
}


static int usbswitch_read_reg(u8 addr, u8 *pdata , struct i2c_client *client)
{
	int ret;
	//u8 buf[5] = {1,0,0,0,0};
	
	struct i2c_msg msgs[] = {
	/*	{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},*/
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 5,
			.buf	= pdata,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
    pr_err("usbswitch_read_reg buf :%x %x %x %x %x ret :%d\n",pdata[0],pdata[1],pdata[2],pdata[3],pdata[4],ret);
	//*pdata = buf[2];
	return ret;
}
static int read_vendorID(char *pvendor,struct i2c_client *client){
    int ret = 0 ;
    struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= pvendor,
		},
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	
	if( ret < 0 ){
	   pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}
	return ret;
}
static u8 rddata[4] = {0};
static int irq_ret = 0 ;
static u8 PI5USB30216Reg4;
static u8 PI5USB30216Reg3;
static u8 OldReg4;
static u8 OldReg3;
static u8 Audio_accessory_flag;
static u8 Debug_accessory_flag;
static u8 RepeatTimes;
static u8 WtBuf[2]={0};

static void intb_handler(struct usbswitch_chip *chip){

    if(!chip->resume_completed){
	    return ;
	};
    msleep(50);
	pr_err("usbswitch_intb_irq\n");
	irq_ret = usbswitch_read_reg(0x00,rddata,g_i2c_client);
	if(irq_ret < 0) {
	   pr_err("usbswitch_intb_irq read data failed \n");
	}
	WtBuf[1]=0x05;
	usbswitch_write_reg(0x00,&WtBuf[1]);
	
	OldReg4=PI5USB30216Reg4;								//保存上一次寄存器数据
   	OldReg3=PI5USB30216Reg3;
   	PI5USB30216Reg4=rddata[3];								//保存寄存器数据
   	PI5USB30216Reg3=rddata[2];
	
	/*if(RepeatTimes>0x03)
	{
		RepeatTimes=0x00;
	}*/
	switch(PI5USB30216Reg3)				//Plug-in Detach	//判定0x03寄存器数据
	{
		case 0x01:											//设备插入 Attach
		{
			switch(PI5USB30216Reg4)
			{
				case 0x00:			   						//没有有效器件接入
					RepeatTimes=0x00;						//清除重复计数寄存器 
					////NULL;										//VBUS控制口关闭
					Audio_accessory_flag=0x00;				//清除Audio设备标志
					Debug_accessory_flag=0x00;				//清除Debug设备标志
					break;							
				case 0x05:									//检测到Device-otg设备，可能是误报，做重新检测
				case 0x06:
					if(RepeatTimes>=0x02)					//重新检测次数2次，检测次数到，确认为Device-otg设备
					{
						RepeatTimes=0x00;					//清除重复计数寄存器
					    msleep(50);
						////NULL;										//确认为Device-otg设备，开启VBUS供电
						//Device Plug in					//传递消息给主程序
					}
					/*else
					{
						RepeatTimes++;						//重新检测程序,延时240ms，写01h到0x02寄存器，延时100ms，写04h到0x02寄存器
						//mdelay(240);
						
						WtBuf[1]=0x01;
						usbswitch_write_reg(0x00,&WtBuf[1]);
						msleep(100);
						
						WtBuf[1]=0x04;
						usbswitch_write_reg(0x00,&WtBuf[1]);						
					}*/
					break;								
				case 0x13:									//检测到Debug accessory，CC1或CC2可能会有抖动，重新检测1次确认
					if(RepeatTimes>=0x01)
					{
						RepeatTimes=0x00;					//清除重复计数寄存器
						msleep(50);
						//NULL;								//确认为Debug accessory且无外部VBUS供电，开启VBUS供电
						Debug_accessory_flag=0x01;			//Debug accessory 标志设为1			
						//Device Plug in					//传递消息给主程序
					}
					/*else
					{
						RepeatTimes++;						//重复计数寄存器加1
						//NULL;							//关闭VBUS控制		  
						Debug_accessory_flag=0x00;			//Debug accessory 标志设为1
						msleep(240);						//重新检测程序,延时240ms，写01h到0x02寄存器，延时100ms，写04h到0x02寄存器
						
						WtBuf[1]=0x01;
						usbswitch_write_reg(0x00,&WtBuf[1]);
						msleep(100);
						
						WtBuf[1]=0x04;
						usbswitch_write_reg(0x00,&WtBuf[1]);					
					}*/
					break;		
				case 0xa8:								 	//检测到异常情况，外部VBUS接入，但CC没有连接，不做处理，等待出现97h
					//if((Debug_accessory_flag==0x01)|(Audio_accessory_flag==0x01))
					{
						//NULL; 							//关闭VBUS控制
					}
					break;
				case 0x0f:									//检测到Audio accessory
					Audio_accessory_flag=0x01;				//设置Audio accessory标志为1
					//Audio accessory plugin				//传递消息给CPU
					//mdelay(50);
					//NULL;									//开启VBUS供电													
					break;
				case 0x93:									//检测到Debug accessory
					if(Debug_accessory_flag==0x01)			//如果Debug_accessory_flag=1，表示是不带供电的Debug accessory
					{
					//NULL;									//继续VBUS输出					
					}
					else									//否则为自带供电的Debug accessory
					{
					//NULL; 								//关闭VBUS控制				
					}
					break;
				case 0x8f:									//检测到Audio accessory
					if(Audio_accessory_flag==0x01)			//如果Audio_accessory_flag=1，表示是不带供电的Audio accessory
					{
					//NULL;									//继续VBUS输出					
					}
					else
					{
					//NULL;								//否则为自带供电的Audio accessory
					}
					break;
				case 0xa9:								//Host Plugin, CC1连接，电流模式为default							
				case 0xaa:								//Host Plugin, CC2连接，电流模式为default	
				case 0xc9:								//Host Plugin, CC1连接，电流模式为1.5A
				case 0xca:								//Host Plugin, CC2连接，电流模式为1.5A
				case 0xe9:								//Host Plugin, CC1连接，电流模式为3A
				case 0xea:								//Host Plugin, CC2连接，电流模式为3A
					//Host plug in					//检测到Host设备接入，传递消息给CPU
					break;
				default:
					break;
				}		
			}
			break;
		case 0x00:									   	   //异常情况处理
			switch(PI5USB30216Reg4)						   //判定0x04寄存器
			{
				case 0x00:		   						//没有有效器件接入
					RepeatTimes=0x00;						//清除重复计数寄存器 
					//NULL;								//VBUS控制口关闭
					Audio_accessory_flag=0x00;				//清除Audio设备标志
					Debug_accessory_flag=0x00;				//清除Debug设备标志
					break;										
				case 0x97:								   //出现97有两种可能
					if((Debug_accessory_flag==0x01)|(Audio_accessory_flag==0x01))		//如果Debug accessory 和 Audio accessory标志为1，标志设备拔出
					{
					msleep(100);						   //延时100ms消抖
					////NULL;							   //关闭VBUS控制
					}
					else if(RepeatTimes>=0x03)			   //否则重复检测3次
					{
						RepeatTimes=0x00;				   //重测次数到，则表示仍然检测出错
						//Debug_accessory_flag=0x01;	   //传递检测错误信息给CPU		
						//VBUS in, no CC
					}
					/*else
					{
						RepeatTimes++;					   //重新检测程序
						//Debug_accessory_flag=0x00;
						msleep(240);
						
						WtBuf[1]=0x01;
						usbswitch_write_reg(0x00,&WtBuf[1]);
						msleep(100);
						
						WtBuf[1]=0x04;
						usbswitch_write_reg(0x00,&WtBuf[1]);					
					}*/
					break;
				case 0x93:
					if(Debug_accessory_flag==0x01)
					{
					//Debug accessory plugin
					}
					else
					{
					}
					break;
				case 0x8f:
					if(Audio_accessory_flag==0x01)
					{
					//Audio accessory plugin
					}
					else
					{
					}
					break;
				case 0xa9:								//Host Plugin, CC1连接，电流模式为default							
				case 0xaa:								//Host Plugin, CC2连接，电流模式为default	
				case 0xc9:								//Host Plugin, CC1连接，电流模式为1.5A
				case 0xca:								//Host Plugin, CC2连接，电流模式为1.5A
				case 0xe9:								//Host Plugin, CC1连接，电流模式为3A
				case 0xea:								//Host Plugin, CC2连接，电流模式为3A
					//Host plug in					//检测到Host设备接入，传递消息给CPU
					break;
				case 0x04:
				case 0x05:
				case 0x06:
				default:
					break;					
				}
				break;
		case 0x02:				//Plugout			   //检测到设备拔出，清除标志，清除VBUS供电控制，清除重复次数
		        pr_err("usbswitch type c plug out\n");
				Audio_accessory_flag=0x00;
				Debug_accessory_flag=0x00;
				RepeatTimes=0x00;
				break;
 		default:
			Audio_accessory_flag=0x00;
			Debug_accessory_flag=0x00;
			RepeatTimes=0x00;
			break;
	}		

}
static irqreturn_t usbswitch_intb_irq(int irq, void *data)
{
    struct usbswitch_chip *chip = data;
	//disable_irq(irq);
	intb_handler(chip);
    //schedule_work(&chip->intb_work);
	//enable_irq(irq);
	return IRQ_HANDLED;
}
static ssize_t smb1360_icinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    char data[5] = {0};
	int ret = 0 ;
	ret = usbswitch_read_reg(0x00,data,g_i2c_client);
/*	data[1]=data[1]|0x01;
	usbswitch_write_reg(0x00,&data[1]);
	usbswitch_read_reg(0x00,data,g_i2c_client);
	data[1]=data[1]&0xFE;
	usbswitch_write_reg(0x00,&data[1]);
	usbswitch_read_reg(0x00,data,g_i2c_client);*/
//pr_err("usbswitch_read_reg buf :%x %x %x %x %x ret :%d\n",data[0],data[1],data[2],data[3],data[4],ret);
/*	data[1]=data[1]|0x01;
	data[2]=data[2]|0x01;
	ret = usbswitch_write_reg(0x00,&data[1]);
	ret = usbswitch_read_reg(0x00,data,g_i2c_client);
	pr_err("usbswitch_read_reg buf :%x %x %x %x %x ret :%d\n",data[0],data[1],data[2],data[3],data[4],ret);
	data[1]=data[1]&0xFE;
	data[2]=data[2]&0xFE;
	ret = usbswitch_write_reg(0x00,&data[1]);
	ret = usbswitch_read_reg(0x00,data,g_i2c_client);
	pr_err("usbswitch_read_reg buf :%x %x %x %x %x ret :%d\n",data[0],data[1],data[2],data[3],data[4],ret);*/
	return snprintf(buf, 50,"%s %d ret:%d %d %d %d %d\n","usbswitch",data[0],data[1],data[2],data[3],data[4],ret);
}

DEVICE_ATTR(usbswitch, 0444, smb1360_icinfo_show, NULL);

/*static ssize_t usbswitch_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    char data[5] = {0};
	int ret = 0 ;
	data[1]=0x01;
	ret = usbswitch_write_reg(0x00,&buf[1]);
	ret = usbswitch_read_reg(0x00,data,g_i2c_client);
	pr_err("usbswitch_read_reg buf :%x %x %x %x %x ret :%d\n",data[0],data[1],data[2],data[3],data[4],ret);
	buf[1]=buf[1]&0xFE;
	buf[2]=buf[2]&0xFE;
	ret = usbswitch_write_reg(0x00,&buf[1]);
	ret = usbswitch_read_reg(0x00,data,g_i2c_client);
	pr_err("usbswitch_read_reg buf :%x %x %x %x %x ret :%d\n",buf[0],buf[1],buf[2],buf[3],buf[4],ret);
	return snprintf(buf, 30,"%s %d ret:%d\n","usbswitch",data[2],ret);
}*/
static int usbswitch_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
    char buf[5]={0};
	int ret = 0;
	int int_irq = 0;
	struct power_supply *usb_psy;
	struct usbswitch_chip *chip;
	ret = read_vendorID(&buf[0],client);
	if ( ret < 0)
    {
	    pr_err("usbswitch_probe probe EPROBE_DEFER!");
	    return -EPROBE_DEFER;
	}
	pr_err("usbswitch_probe probe start vendorID : %x!",buf[0]);
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}
	chip = devm_kzalloc(&client->dev, sizeof(struct usbswitch_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	chip->i2client = client;
	chip->dev = &client->dev;
	g_i2c_client = client;
	i2c_set_clientdata(client, chip);
	/*
	chip->vdd = devm_regulator_get(chip->dev, "vdd");
	if (IS_ERR(chip->vdd)) {
		dev_err(chip->dev, "unable to get vdd supply\n");
		return PTR_ERR(chip->vdd);
	}
	ret = regulator_enable(chip->vdd);
    if (ret) {
	dev_err(&client->dev, "Unable to enable VDD\n");
	}
	chip->vdd18 = devm_regulator_get(chip->dev, "vdda18");
	if (IS_ERR(chip->vdd18)) {
		dev_err(chip->dev, "unable to get vdda18 supply\n");
		return PTR_ERR(chip->vdd18);
	}
	ret = regulator_enable(chip->vdd18);
    if (ret) {
	dev_err(&client->dev, "Unable to enable vdd18\n");
	}
	*/
	pr_err("usbswitch_probe\n");
	buf[1] = 0x01;
	usbswitch_write_reg(0x01,&buf[1]);
	msleep(30);
	buf[1] = 0x04;
	usbswitch_write_reg(0x01,&buf[1]);
    usbswitch_read_reg(0x01,buf,client);
	//pr_err("usbswitch_probe buf:%d\n",buf);
	usbswitch_read_reg(0x01,buf,client);
	//INIT_WORK(&chip->intb_work,intb_handler);
	chip->gpio_intpin = of_get_named_gpio(chip->dev->of_node, "qcom,intpin", 0);
	if (chip->gpio_intpin < 0)
		pr_err("gpio_intpin is not available\n");
	
    if (gpio_is_valid(chip->gpio_intpin)) {

			ret = gpio_request(chip->gpio_intpin,
							"USBSW-intpin");
			if (ret < 0) {
				pr_err("gpio req failed for id\n");
				chip->gpio_intpin = 0;
			}

			int_irq = gpio_to_irq(chip->gpio_intpin);
		    if (int_irq) {
			    pr_err("usbswitch request irq irqnu:%d \n",int_irq);
		        chip->irq = int_irq;
                //IRQF_TRIGGER_RISING | 
		        ret = devm_request_threaded_irq(&client->dev, int_irq, NULL,
				    usbswitch_intb_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				    "usbswitch_stat_irq", chip);
			   if (ret < 0 ) {
				   pr_err("request irq failed for ID irqnu:%d \n",int_irq);
			   }
	        }
			/*ret = request_irq(int_irq,
					  usbswitch_intb_irq,
					  IRQF_TRIGGER_RISING |
					  IRQF_TRIGGER_FALLING,
					  "usbswitch", chip);
			if (ret) {
				pr_err("request irq failed for ID\n");

			}*/
	} else {
			ret = -ENODEV;
			pr_err("ID IRQ doesn't exist\n");
    }
	device_create_file(&client->dev,&dev_attr_usbswitch);//wubo add create icinfo node.
	
	return 0;
}

static int usbswitch_remove(struct i2c_client *client)
{

	return 0;
}

static int usbswitch_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
	struct usbswitch_chip *chip = i2c_get_clientdata(client);
	chip->resume_completed = 0 ;
	return 0;
}

static int usbswitch_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
	struct usbswitch_chip *chip = i2c_get_clientdata(client);
	chip->resume_completed = 1 ;
	return 0;
}

static const struct dev_pm_ops usbswitch_pm_ops = {
	.suspend = usbswitch_suspend,
	.resume = usbswitch_resume,
};

static const struct i2c_device_id usbswitch_led_id[] = {
	{"usbswitch", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, usbswitch_led_id);

static struct of_device_id usbswitch_match_table[] = {
	{ .compatible = "PI5usb30216,usb30216",},
	{ },
};

static struct i2c_driver usbswitch_driver = {
	.probe = usbswitch_probe,
	.remove = usbswitch_remove,
	.driver = {
		.name = "usbswitch",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(usbswitch_match_table),
#ifdef CONFIG_PM
		.pm = &usbswitch_pm_ops,
#endif
	},
	.id_table = usbswitch_led_id,
};

static int __init usbswitch_init(void)
{
	return i2c_add_driver(&usbswitch_driver);
}

static void __exit usbswitch_exit(void)
{
	return i2c_del_driver(&usbswitch_driver);
}
module_init(usbswitch_init);
module_exit(usbswitch_exit);
MODULE_AUTHOR("bo.wu@ck-telecom.com");
MODULE_DESCRIPTION("PI5usb2013a usb switch");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("usb type c switch");
