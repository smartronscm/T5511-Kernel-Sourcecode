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
	
	OldReg4=PI5USB30216Reg4;								//������һ�μĴ�������
   	OldReg3=PI5USB30216Reg3;
   	PI5USB30216Reg4=rddata[3];								//����Ĵ�������
   	PI5USB30216Reg3=rddata[2];
	
	/*if(RepeatTimes>0x03)
	{
		RepeatTimes=0x00;
	}*/
	switch(PI5USB30216Reg3)				//Plug-in Detach	//�ж�0x03�Ĵ�������
	{
		case 0x01:											//�豸���� Attach
		{
			switch(PI5USB30216Reg4)
			{
				case 0x00:			   						//û����Ч��������
					RepeatTimes=0x00;						//����ظ������Ĵ��� 
					////NULL;										//VBUS���ƿڹر�
					Audio_accessory_flag=0x00;				//���Audio�豸��־
					Debug_accessory_flag=0x00;				//���Debug�豸��־
					break;							
				case 0x05:									//��⵽Device-otg�豸���������󱨣������¼��
				case 0x06:
					if(RepeatTimes>=0x02)					//���¼�����2�Σ�����������ȷ��ΪDevice-otg�豸
					{
						RepeatTimes=0x00;					//����ظ������Ĵ���
					    msleep(50);
						////NULL;										//ȷ��ΪDevice-otg�豸������VBUS����
						//Device Plug in					//������Ϣ��������
					}
					/*else
					{
						RepeatTimes++;						//���¼�����,��ʱ240ms��д01h��0x02�Ĵ�������ʱ100ms��д04h��0x02�Ĵ���
						//mdelay(240);
						
						WtBuf[1]=0x01;
						usbswitch_write_reg(0x00,&WtBuf[1]);
						msleep(100);
						
						WtBuf[1]=0x04;
						usbswitch_write_reg(0x00,&WtBuf[1]);						
					}*/
					break;								
				case 0x13:									//��⵽Debug accessory��CC1��CC2���ܻ��ж��������¼��1��ȷ��
					if(RepeatTimes>=0x01)
					{
						RepeatTimes=0x00;					//����ظ������Ĵ���
						msleep(50);
						//NULL;								//ȷ��ΪDebug accessory�����ⲿVBUS���磬����VBUS����
						Debug_accessory_flag=0x01;			//Debug accessory ��־��Ϊ1			
						//Device Plug in					//������Ϣ��������
					}
					/*else
					{
						RepeatTimes++;						//�ظ������Ĵ�����1
						//NULL;							//�ر�VBUS����		  
						Debug_accessory_flag=0x00;			//Debug accessory ��־��Ϊ1
						msleep(240);						//���¼�����,��ʱ240ms��д01h��0x02�Ĵ�������ʱ100ms��д04h��0x02�Ĵ���
						
						WtBuf[1]=0x01;
						usbswitch_write_reg(0x00,&WtBuf[1]);
						msleep(100);
						
						WtBuf[1]=0x04;
						usbswitch_write_reg(0x00,&WtBuf[1]);					
					}*/
					break;		
				case 0xa8:								 	//��⵽�쳣������ⲿVBUS���룬��CCû�����ӣ����������ȴ�����97h
					//if((Debug_accessory_flag==0x01)|(Audio_accessory_flag==0x01))
					{
						//NULL; 							//�ر�VBUS����
					}
					break;
				case 0x0f:									//��⵽Audio accessory
					Audio_accessory_flag=0x01;				//����Audio accessory��־Ϊ1
					//Audio accessory plugin				//������Ϣ��CPU
					//mdelay(50);
					//NULL;									//����VBUS����													
					break;
				case 0x93:									//��⵽Debug accessory
					if(Debug_accessory_flag==0x01)			//���Debug_accessory_flag=1����ʾ�ǲ��������Debug accessory
					{
					//NULL;									//����VBUS���					
					}
					else									//����Ϊ�Դ������Debug accessory
					{
					//NULL; 								//�ر�VBUS����				
					}
					break;
				case 0x8f:									//��⵽Audio accessory
					if(Audio_accessory_flag==0x01)			//���Audio_accessory_flag=1����ʾ�ǲ��������Audio accessory
					{
					//NULL;									//����VBUS���					
					}
					else
					{
					//NULL;								//����Ϊ�Դ������Audio accessory
					}
					break;
				case 0xa9:								//Host Plugin, CC1���ӣ�����ģʽΪdefault							
				case 0xaa:								//Host Plugin, CC2���ӣ�����ģʽΪdefault	
				case 0xc9:								//Host Plugin, CC1���ӣ�����ģʽΪ1.5A
				case 0xca:								//Host Plugin, CC2���ӣ�����ģʽΪ1.5A
				case 0xe9:								//Host Plugin, CC1���ӣ�����ģʽΪ3A
				case 0xea:								//Host Plugin, CC2���ӣ�����ģʽΪ3A
					//Host plug in					//��⵽Host�豸���룬������Ϣ��CPU
					break;
				default:
					break;
				}		
			}
			break;
		case 0x00:									   	   //�쳣�������
			switch(PI5USB30216Reg4)						   //�ж�0x04�Ĵ���
			{
				case 0x00:		   						//û����Ч��������
					RepeatTimes=0x00;						//����ظ������Ĵ��� 
					//NULL;								//VBUS���ƿڹر�
					Audio_accessory_flag=0x00;				//���Audio�豸��־
					Debug_accessory_flag=0x00;				//���Debug�豸��־
					break;										
				case 0x97:								   //����97�����ֿ���
					if((Debug_accessory_flag==0x01)|(Audio_accessory_flag==0x01))		//���Debug accessory �� Audio accessory��־Ϊ1����־�豸�γ�
					{
					msleep(100);						   //��ʱ100ms����
					////NULL;							   //�ر�VBUS����
					}
					else if(RepeatTimes>=0x03)			   //�����ظ����3��
					{
						RepeatTimes=0x00;				   //�ز�����������ʾ��Ȼ������
						//Debug_accessory_flag=0x01;	   //���ݼ�������Ϣ��CPU		
						//VBUS in, no CC
					}
					/*else
					{
						RepeatTimes++;					   //���¼�����
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
				case 0xa9:								//Host Plugin, CC1���ӣ�����ģʽΪdefault							
				case 0xaa:								//Host Plugin, CC2���ӣ�����ģʽΪdefault	
				case 0xc9:								//Host Plugin, CC1���ӣ�����ģʽΪ1.5A
				case 0xca:								//Host Plugin, CC2���ӣ�����ģʽΪ1.5A
				case 0xe9:								//Host Plugin, CC1���ӣ�����ģʽΪ3A
				case 0xea:								//Host Plugin, CC2���ӣ�����ģʽΪ3A
					//Host plug in					//��⵽Host�豸���룬������Ϣ��CPU
					break;
				case 0x04:
				case 0x05:
				case 0x06:
				default:
					break;					
				}
				break;
		case 0x02:				//Plugout			   //��⵽�豸�γ��������־�����VBUS������ƣ�����ظ�����
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
