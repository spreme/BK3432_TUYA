#include "gpio.h"
#include "user_config.h"
#include "i2c.h" 
#include "PCF8563.h" 

#ifdef RTC_TIME
int time_zone_RTC_rec = 8 * 3600;

#define SCLK 	0x02//I2C_SCL
#define SDAT 	0x03//I2C_SDA

#if 0
/******************************************************************************
* Function Name --> IIC_GPIO_Init
* Description   --> GPIO初始化
* Input         --> none
* Output        --> none
* Reaturn       --> none 
******************************************************************************/
void IIC_GPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(IIC_SCL_PORT_RCC|IIC_SDA_PORT_RCC,ENABLE);
    
	GPIO_InitStructure.GPIO_Pin=IIC_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(IIC_SCL_PORT,&GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin=IIC_SDA_PIN;
	GPIO_Init(IIC_SDA_PORT,&GPIO_InitStructure);
	SDA0_HIGH();  //置IIC总线空闲
	SCL0_HIGH();
}
/******************************************************************************
* Function Name --> SDA_OUT
* Description   --> SDA输出配置	
* Input         --> none
* Output        --> none
* Reaturn       --> none 
******************************************************************************/
void IIC_SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin=IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(IIC_SDA_PORT,&GPIO_InitStructure);
}
/******************************************************************************
* Function Name --> SDA_IN
* Description   --> SDA输入配置
* Input         --> none
* Output        --> none
* Reaturn       --> none 
******************************************************************************/
void IIC_SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin=IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(IIC_SDA_PORT,&GPIO_InitStructure);
}
#endif

void i2cs_init()
{
    gpio_config(SCLK,OUTPUT,PULL_NONE);
    gpio_config(SDAT,OUTPUT,PULL_NONE);
}

unsigned char BCD2HEX(unsigned char bcd_data) //BCD转为HEX子程序 
{
    unsigned char temp; 
    temp=(bcd_data/16*10 + bcd_data%16); 
    return temp; 
} 
 
unsigned char HEX2BCD(unsigned char hex_data) //HEX转为BCD子程序 
{ 
    unsigned char temp; 
    temp=(hex_data/10*16 + hex_data%10); 
    return temp; 
}

/******************************************************************************
* Function Name --> IIC启动
* Description   --> SCL高电平期间，SDA由高电平突变到低电平时启动总线
*                   SCL: __________
*                                  \__________
*                   SDA: _____
*                             \_______________
* Input         --> none
* Output        --> none
* Reaturn       --> none 
******************************************************************************/
void IIC_Start(void)
{
	SDA0_SetOutput();	//设置成输出
	SDA0_HIGH();	//为SDA下降启动做准备
	SCL0_HIGH();	//在SCL高电平时，SDA为下降沿时候总线启动
	
#if _USER_DELAY_CLK==1  /* 定义了则使用延时函数来改变通讯频率 */
	
	IIC_Delay();
	SDA0_LOW();	//突变，总线启动
	IIC_Delay();
	SCL0_LOW();
	IIC_Delay();
#else  /* 否则不使用延时函数改变通讯频率 */
	
	SDA0_LOW();	//突变，总线启动
	SCL0_LOW();
#endif  /* end __USER_DELAY_CLK */
}
/******************************************************************************
* Function Name --> IIC停止
* Description   --> SCL高电平期间，SDA由低电平突变到高电平时停止总线
*                   SCL: ____________________
*                                  __________
*                   SDA: _________/
* Input         --> none
* Output        --> none
* Reaturn       --> none 
******************************************************************************/
void IIC_Stop(void)
{
	SDA0_SetOutput();	//设置成输出
	SDA0_LOW();	//为SDA上升做准备
	
#if _USER_DELAY_CLK==1  /* 定义了则使用延时函数来改变通讯频率 */
	IIC_Delay();
	SCL0_HIGH();	//在SCL高电平时，SDA为上升沿时候总线停止
	IIC_Delay();
	SDA0_HIGH();	//突变，总线停止
	IIC_Delay();
#else  /* 否则不使用延时函数改变通讯频率 */
	SCL0_HIGH();	//在SCL高电平时，SDA为上升沿时候总线停止
	SDA0_HIGH();	//突变，总线停止
#endif  /* end __USER_DELAY_CLK */
}
/******************************************************************************
* Function Name --> 主机向从机发送应答信号
* Description   --> none
* Input         --> a：应答信号
*                      0：应答信号
*                      1：非应答信号
* Output        --> none
* Reaturn       --> none 
******************************************************************************/
void IIC_Ack(u8 a)
{
	SDA0_SetOutput();	//设置成输出
	if(a)	SDA0_HIGH();	//放上应答信号电平
	else	SDA0_LOW();
	
#if _USER_DELAY_CLK==1  /* 定义了则使用延时函数来改变通讯频率 */
	IIC_Delay();
	SCL0_HIGH();	//为SCL下降做准备
	IIC_Delay();
	SCL0_LOW();	//突变，将应答信号发送过去
	IIC_Delay();
#else  /* 否则不使用延时函数改变通讯频率 */
	SCL0_HIGH();	//为SCL下降做准备
	SCL0_LOW();	//突变，将应答信号发送过去
#endif  /* end __USER_DELAY_CLK */
}
/******************************************************************************
* Function Name --> 向IIC总线发送一个字节数据
* Description   --> none
* Input         --> dat：要发送的数据
* Output        --> none
* Reaturn       --> ack：返回应答信号
******************************************************************************/
u8 IIC_Write_Byte(u8 dat)
{
	u8 i;
	u8 iic_ack=0;	//iic应答标志
	SDA0_SetOutput();	//设置成输出
	for(i = 0;i < 8;i++)
	{
		if(dat & 0x80)	SDA0_HIGH();	//判断发送位，先发送高位
		else	SDA0_LOW();
#if _USER_DELAY_CLK==1  /* 定义了则使用延时函数来改变通讯频率 */
			
		IIC_Delay();
		SCL0_HIGH();	//为SCL下降做准备
		IIC_Delay();
		SCL0_LOW();	//突变，将数据位发送过去
		dat<<=1;	//数据左移一位
	}	//字节发送完成，开始接收应答信号
	SDA0_HIGH();	//释放数据线
	SDA0_SetInput();	//设置成输入
	IIC_Delay();
	SCL0_HIGH();	//为SCL下降做准备
	IIC_Delay();
#else  /* 否则不使用延时函数改变通讯频率 */
		SCL0_HIGH();	//为SCL下降做准备
		SCL0_LOW();	//突变，将数据位发送过去
		dat<<=1;	//数据左移一位
	}	//字节发送完成，开始接收应答信号
	SDA0_HIGH();	//释放数据线
	SDA0_SetInput();	//设置成输入
	SCL0_HIGH();	//为SCL下降做准备
#endif  /* end __USER_DELAY_CLK */
	
	iic_ack = SDA0_READ();	//读入应答位
	SCL0_LOW();
	return (iic_ack);	//返回应答信号
}
/******************************************************************************
* Function Name --> 从IIC总线上读取一个字节数据
* Description   --> none
* Input         --> none
* Output        --> none
* Reaturn       --> x：读取到的数据
******************************************************************************/
u8 IIC_Read_Byte(void)
{
	u8 i;
	u8 x=0;
	SDA0_HIGH();	//首先置数据线为高电平
	SDA0_SetInput();	//设置成输入
	for(i = 0;i < 8;i++)
	{
		x <<= 1;	//读入数据，高位在前
#if _USER_DELAY_CLK==1  /* 定义了则使用延时函数来改变通讯频率 */
		IIC_Delay();
		SCL0_HIGH();	//突变
		IIC_Delay();
		
		if(SDA0_READ())	x |= 0x01;	//收到高电平
		SCL0_LOW();
		IIC_Delay();
	}	//数据接收完成
#else  /* 否则不使用延时函数改变通讯频率 */
		SCL0_HIGH();	//突变
		
		if(SDA0_READ())	x |= 0x01;	//收到高电平
		SCL0_LOW();
	}	//数据接收完成
#endif  /* end __USER_DELAY_CLK */
	SCL0_LOW();
	return x;	//返回读取到的数据
}



void PCF8563_Write_Byte(unsigned char REG_ADD, unsigned char dat)
{
	IIC_Start();                          //启动IIC
	if((IIC_Write_Byte(PCF8563_Write)) == 0)  //发送写命令并检查应答位
	{
		if(IIC_Write_Byte(REG_ADD))          //写入地址
		{
			UART_PRINTF("Write_Byte error 2 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		}
		if(IIC_Write_Byte(dat))              //发送数据
		{
			UART_PRINTF("Write_Byte error 3 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		}
	}
	else
		UART_PRINTF("Write_Byte error 1 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	
	IIC_Stop();                           //关闭IIC
} 

unsigned char PCF8563_Read_Byte(unsigned char REG_ADD)
{
	unsigned char ReData;                 //定义变量接收数据
	
	IIC_Start();                          //启动IIC
	if((IIC_Write_Byte(PCF8563_Write)) == 0)  //发送写命令并检查应答位
	{
		if(IIC_Write_Byte(REG_ADD))          //确定要操作的寄存器
		{
			UART_PRINTF("Read_Byte error 2 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		}
		IIC_Start();                      	//重启总线
		if(IIC_Write_Byte(PCF8563_Read))    	 	//发送读取命令
		{
			UART_PRINTF("Read_Byte error 3 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		}
		ReData = IIC_Read_Byte();        	//读取数据
		IIC_Ack(1);                       	//发送非应答信号结束数据传送
	}
	else
		UART_PRINTF("Read_Byte error 1 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		
	IIC_Stop();                           //关闭IIC
	return ReData;
}

void PCF8563_Write_nByte(unsigned char REG_ADD, unsigned char num, unsigned char *pBuff)
{
	unsigned char i = 0;                  //定义i辅助循环发送
		 	
	IIC_Start();                          //启动IIC
	if((IIC_Write_Byte(PCF8563_Write)) == 0)  //发送写命令并检查应答位
	{
		if(IIC_Write_Byte(REG_ADD))          //定位起始寄存器地址
		{
			UART_PRINTF("Write_nByte error 5 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		}
		for(i = 0;i < num;i++)            //循环num次，达到多个数据写入
		{
			IIC_Write_Byte(*pBuff);       //写入数据
			pBuff++;                      //数据缓存地址增加
		}
	}
	else
		UART_PRINTF("Write_nByte error 4 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	
	IIC_Stop();                           //关闭IIC
}

void PCF8563_Read_nByte(unsigned char REG_ADD, unsigned char num, unsigned char *pBuff)
{
	unsigned char i = 0;                  //定义变量i辅助多次读取
	
	IIC_Start();                          //启动IIC
	if((IIC_Write_Byte(PCF8563_Write)) == 0)  //发送写命令并检查应答位
	{
		if(IIC_Write_Byte(REG_ADD))          //定位起始寄存器地址
		{
			UART_PRINTF("Read_Byte error 5 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		}
		IIC_Start();                      //重启总线
		if(IIC_Write_Byte(PCF8563_Read))     //发送读取命令
		{
			UART_PRINTF("Read_Byte error 6 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		}
		for(i = 0;i < num;i++)            //循环num次完成多次读取
		{
			*pBuff = IIC_Read_Byte();     //读取数据
			if(i == (num - 1)) IIC_Ack(1);//发送非应答信号，知道完成数据量
			else IIC_Ack(0);              //发送应答信号
			pBuff++;                      //读取缓冲区地址增加
		}
	}
	else
		UART_PRINTF("Read_Byte error 4 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	
	IIC_Stop();	                          //关闭IIC
}

unsigned char PCF8563_Check(void)
{
	unsigned char test_value = 0;
	unsigned char Time_Count = 0;          //定时器倒计时数据缓存
 
	if(PCF8563_Read_Byte(PCF8563_Address_Timer) & 0x80)             //如果打开了定时器，则先关闭
	{
		PCF8563_Write_Byte(PCF8563_Address_Timer, PCF_Timer_Close); //先关闭定时器
		Time_Count = PCF8563_Read_Byte(PCF8563_Address_Timer_VAL);  //先保存计数值
	}
 
	PCF8563_Write_Byte(PCF8563_Address_Timer_VAL, PCF8563_Check_Data);  //写入检测值
	for(test_value = 0;test_value < 50;test_value++)  {}  //延时一定时间再读取
	test_value = PCF8563_Read_Byte(PCF8563_Address_Timer_VAL);  //再读取回来
 
	if(Time_Count != 0)  //启动了定时器功能，则恢复
	{
		PCF8563_Write_Byte(PCF8563_Address_Timer_VAL, Time_Count);  //恢复现场
		PCF8563_Write_Byte(PCF8563_Address_Timer, PCF_Timer_Open);  //启动定时器
	}
 
	if(test_value != PCF8563_Check_Data)  return 1;  //器件错误或者损坏
	
	return 0;  //正常
}



unsigned char buffer[4];
unsigned char buffer1[4];

void PCF8563_Start(void)
{
	unsigned char temp = 0;
	
	temp = PCF8563_Read_Byte(PCF8563_Address_Control_Status_1);  //读取控制/状态寄存器1
	UART_PRINTF("PCF8563_Start:%d @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@", temp);
	if (temp & PCF_Control_ChipStop)
	{
		temp &= PCF_Control_ChipRuns;  //运行芯片
	}
	if ((temp & (1<<7)) == 0)  //普通模式
	{
		temp &= PCF_Control_TestcClose;  //电源复位模式失效
	}
	PCF8563_Write_Byte(PCF8563_Address_Control_Status_1, temp);  //再写入数值
}

void PCF8563_Stop(void)
{
	unsigned char temp = 0;
	
	temp = PCF8563_Read_Byte(PCF8563_Address_Control_Status_1);  //读取控制/状态寄存器1
	temp |= PCF_Control_ChipStop;  //停止运行
	PCF8563_Write_Byte(PCF8563_Address_Control_Status_1, temp);  //再写入数值
}

void PCF8563_SetMode(unsigned char Mode)
{
	unsigned char temp = 0;
	
	temp = PCF8563_Read_Byte(PCF8563_Address_Control_Status_1);  //读取寄存器值
	if (Mode == PCF_Mode_EXT_CLK)  //EXT_CLK测试模式
	{
		temp |= PCF_Control_Status_EXT_CLKMode;
	}
	else if (Mode == PCF_Mode_Normal)
	{
		temp &= PCF_Control_Status_NormalMode;
		temp &= ~(1<<3);  //电源复位功能失效
	}
	PCF8563_Write_Byte(PCF8563_Address_Control_Status_1, temp);
}

void PCF8563_SetPowerReset(unsigned char NewState)
{
	unsigned char TestC = 0;
	
	TestC = PCF8563_Read_Byte(PCF8563_Address_Control_Status_1);  //获取寄存器值
	TestC &= ~(1<<3);  //清除之前设置
	if (NewState == PCF8563_PowerResetEnable)  //复位功能有效
	{
		TestC |= PCF8563_PowerResetEnable;
	}
	else if (NewState == PCF8563_PowerResetDisable)
	{
		TestC &= ~PCF8563_PowerResetEnable;  //失效，普通模式是值逻辑0，即失效
	}
	PCF8563_Write_Byte(PCF8563_Address_Control_Status_1, TestC);  //写入数值
}

void PCF8563_SetCLKOUT(_PCF8563_CLKOUT_Typedef* PCF_CLKOUTStruct)
{
	unsigned char tmp = 0;
	
	tmp = PCF8563_Read_Byte(PCF8563_Address_CLKOUT);  //读取寄存器值
	tmp &= 0x7c;  //清除之前设置
	if (PCF_CLKOUTStruct->CLKOUT_NewState == PCF_CLKOUT_Open)
	{
		tmp |= PCF_CLKOUT_Open;
	}
	else
	{
		tmp &= PCF_CLKOUT_Close;
	}
	tmp |= PCF_CLKOUTStruct->CLKOUT_Frequency;
	
	PCF8563_Write_Byte(PCF8563_Address_CLKOUT, tmp);
}


void PCF8563_SetTimer(_PCF8563_Timer_Typedef* PCF_TimerStruct)
{
	unsigned char Timer_Ctrl = 0;
	unsigned char Timer_Value = 0;
	
	Timer_Ctrl = PCF8563_Read_Byte(PCF8563_Address_Timer);  //获的控制寄存器值
	Timer_Value = PCF8563_Read_Byte(PCF8563_Address_Timer_VAL);  //获取倒计时数值
	//
	//先停止定时器
	//
	Timer_Ctrl &= PCF_Timer_Close;
	PCF8563_Write_Byte(PCF8563_Address_Timer, Timer_Ctrl);
	
	Timer_Ctrl &= 0x7c;  //清除定时器之前设置
	
	if (PCF_TimerStruct->RTC_Timer_NewState == PCF_Timer_Open)  //开启
	{
		Timer_Ctrl |= PCF_Timer_Open;
		Timer_Ctrl |= PCF_TimerStruct->RTC_Timer_Frequency;  //填上新的工作频率
		if (PCF_TimerStruct->RTC_Timer_Value)  //需要填上新的计数值
		{
			Timer_Value = PCF_TimerStruct->RTC_Timer_Value;  //填上新的计数值
		}
	}
	else
	{
		Timer_Ctrl &= PCF_Timer_Close;
	}
	PCF8563_Write_Byte(PCF8563_Address_Timer_VAL, Timer_Value);  //写入倒计时数值
	
	if (PCF_TimerStruct->RTC_Timer_Interrupt == PCF_Time_INT_Open)  //开启了中断输出
	{
		Timer_Value = PCF8563_Read_Byte(PCF8563_Address_Control_Status_2);  //获取控制/状态寄存器2数值
		Timer_Value &= PCF_Time_INT_Close;  //清除定时器中断使能
		Timer_Value &= ~(1<<2);  //清除定时器中断标志
		Timer_Value &= ~(1<<4);  //当 TF 有效时 INT 有效 (取决于 TIE 的状态) 
		Timer_Value |= PCF_Time_INT_Open;  //开启定时器中断输出
		PCF8563_Write_Byte(PCF8563_Address_Control_Status_2, Timer_Value);
	}
	else
	{
		Timer_Value = PCF8563_Read_Byte(PCF8563_Address_Control_Status_2);  //获取控制/状态寄存器2数值
		Timer_Value &= PCF_Time_INT_Close;  //清除定时器中断使能
		Timer_Value |= PCF_Time_INT_Open;  //开启定时器中断输出
		PCF8563_Write_Byte(PCF8563_Address_Control_Status_2, Timer_Value);
	}
	
	PCF8563_Write_Byte(PCF8563_Address_Timer, Timer_Ctrl);  //设置定时器控制寄存器
}


void PCF8563_Set_Times(unsigned char PCF_Format,\
                       unsigned char PCF_Century,\
	                   unsigned char Year, unsigned char Month, unsigned char Date, unsigned char Week,\
                       unsigned char Hour, unsigned char Minute)
{
	_PCF8563_Time_Typedef Time_InitStructure;
	_PCF8563_Date_Typedef Date_InitStructure;
	
	if (PCF_Format == PCF_Format_BIN)
	{
		//
		//判断数据是否符合范围
		//
		if (Year > 99)   Year  = 0;  //恢复00年
		if (Month > 12)  Month = 1;  //恢复1月
		if (Date > 31)   Date  = 1;  //恢复1日
		if (Week > 6)    Week  = 1;  //恢复星期一
		
		if (Hour > 23)    Hour   = 0;  //恢复0小时
		if (Minute > 59)  Minute = 0;  //恢复0分钟
		
		//
		//转换一下
		//
		Date_InitStructure.RTC_Years    = HEX2BCD(Year);
		Date_InitStructure.RTC_Months   = HEX2BCD(Month);
		Date_InitStructure.RTC_Days     = HEX2BCD(Date);
		Date_InitStructure.RTC_WeekDays = HEX2BCD(Week);
		
		Time_InitStructure.RTC_Hours    = HEX2BCD(Hour);
		Time_InitStructure.RTC_Minutes  = HEX2BCD(Minute);
	}
	Time_InitStructure.RTC_Seconds = 0x00;  //恢复0秒
	Time_InitStructure.RTC_Seconds &= PCF_Accuracy_ClockYes;  //保证准确的时间
	//
	//判断世纪位
	//
	if (PCF_Century == PCF_Century_19xx)
	{
		Date_InitStructure.RTC_Months |= PCF_Century_SetBitC;
	}
	else
	{
		Date_InitStructure.RTC_Months &= ~PCF_Century_SetBitC;
	}
	//
	//写入信息到寄存器
	//
	buffer[0] = Time_InitStructure.RTC_Seconds;
	buffer[1] = Time_InitStructure.RTC_Minutes;
	buffer[2] = Time_InitStructure.RTC_Hours;
	PCF8563_Write_nByte(PCF8563_Address_Seconds, 3, buffer);  //写入时间信息
	
	buffer[0] = Date_InitStructure.RTC_Days;
	buffer[1] = Date_InitStructure.RTC_WeekDays;
	buffer[2] = Date_InitStructure.RTC_Months;
	buffer[3] = Date_InitStructure.RTC_Years;
	PCF8563_Write_nByte(PCF8563_Address_Days,    4, buffer);  //写入日期信息
}


void PCF8563_SetTime(unsigned char PCF_Format, tuya_ble_time_struct_data_t* PCF_DataStruct)
{
	if (PCF_Format == PCF_Format_BIN)  //十进制格式，需要转换一下
	{
		//
		//判断数值是否在范围之内
		//
		if (PCF_DataStruct->nHour > 23)    PCF_DataStruct->nHour   = 0;  //恢复0小时
		if (PCF_DataStruct->nMin > 59)  PCF_DataStruct->nMin = 0;  //恢复0分钟
		if (PCF_DataStruct->nSec > 59)  PCF_DataStruct->nSec = 0;  //恢复0秒
		//
		//需要转换一下
		//
		PCF_DataStruct->nHour   = HEX2BCD(PCF_DataStruct->nHour);
		PCF_DataStruct->nMin = HEX2BCD(PCF_DataStruct->nMin);
		PCF_DataStruct->nSec = HEX2BCD(PCF_DataStruct->nSec);
	}
	//
	//拷贝数据
	//
	buffer[0] = PCF_DataStruct->nSec;
	buffer[1] = PCF_DataStruct->nMin;
	buffer[2] = PCF_DataStruct->nHour;
	//
	//写入数据到寄存器
	//
	PCF8563_Write_nByte(PCF8563_Address_Seconds, 3, buffer);
}


void PCF8563_GetTime(unsigned char PCF_Format, tuya_ble_time_struct_data_t* PCF_DataStruct)
{
	/* 读取寄存器数值 */
	PCF8563_Read_nByte(PCF8563_Address_Seconds, 3, buffer);     //调用读多为函数，起始地址为秒，长度3，存入buffer数组
	
	/* 屏蔽无效位,将时分秒寄存器的值分别放入数组 */
	buffer[0] &= PCF8563_Shield_Seconds;
	buffer[1] &= PCF8563_Shield_Minutes;
	buffer[2] &= PCF8563_Shield_Hours;
	
//	UART_PRINTF("############# get-time to rtc: %02x:%02x:%02x ",buffer[2], buffer[1], buffer[0]);
	
	//判断需要的数据格式，
	if (PCF_Format == PCF_Format_BIN)
	{
		PCF_DataStruct->nHour   = BCD2HEX(buffer[2]);
		PCF_DataStruct->nMin = BCD2HEX(buffer[1]);
		PCF_DataStruct->nSec = BCD2HEX(buffer[0]);
	}
	else if (PCF_Format == PCF_Format_BCD)
	{
		//
		//拷贝数据
		//
		PCF_DataStruct->nHour   = buffer[2];
		PCF_DataStruct->nMin = buffer[1];
		PCF_DataStruct->nSec = buffer[0];
	}
}

void PCF8563_SetDate(unsigned char PCF_Format, unsigned char PCF_Century, tuya_ble_time_struct_data_t* PCF_DataStruct)
{
	u8 year_s = 0;
	
	year_s = PCF_DataStruct->nYear - 2000;
	if (PCF_Format == PCF_Format_BIN)  //十进制格式，需要转换一下
	{
		//
		//判断数值是否在范围之内
		//
		if (year_s > 99)    				year_s    = 0;  //恢复00年
		if (PCF_DataStruct->nMonth > 12)   	PCF_DataStruct->nMonth   = 1;  //恢复1月
		if (PCF_DataStruct->nDay > 31)     	PCF_DataStruct->nDay     = 1;  //恢复1日
		if (PCF_DataStruct->DayIndex > 6) 	PCF_DataStruct->DayIndex = 1;  //恢复星期一
		//
		//需要转换一下
		//
		year_s    = HEX2BCD(year_s);
		PCF_DataStruct->nMonth   = HEX2BCD(PCF_DataStruct->nMonth);
		PCF_DataStruct->nDay     = HEX2BCD(PCF_DataStruct->nDay);
		PCF_DataStruct->DayIndex = HEX2BCD(PCF_DataStruct->DayIndex);
	}
	//
	//判断世纪位
	//
	if (PCF_Century == PCF_Century_19xx)
	{
		PCF_DataStruct->nMonth |= PCF_Century_SetBitC;
	}
	else
	{
		PCF_DataStruct->nMonth &= ~PCF_Century_SetBitC;
	}
	//
	//数据拷贝
	//
	buffer1[0] = PCF_DataStruct->nDay;
	buffer1[1] = PCF_DataStruct->DayIndex;
	buffer1[2] = PCF_DataStruct->nMonth;
	buffer1[3] = year_s;
	//
	//写入数据到寄存器
	//
	PCF8563_Write_nByte(PCF8563_Address_Days, 4, buffer1);
}

void PCF8563_GetDate(unsigned char PCF_Format, unsigned char *PCF_Century, tuya_ble_time_struct_data_t* PCF_DataStruct)
{
	//
	//读取全部寄存器数值
	//
	PCF8563_Read_nByte(PCF8563_Address_Days, 4, buffer1);
	//
	//判断世纪位数值
	//
	if (buffer1[2] & PCF_Century_SetBitC)
	{
		*PCF_Century = 1;
	}
	else
	{
		*PCF_Century = 0;
	}
	//
	//屏蔽无效位
	//
	buffer1[0] &= PCF8563_Shield_Days;
	buffer1[1] &= PCF8563_Shield_WeekDays;
	buffer1[2] &= PCF8563_Shield_Months_Century;
	buffer1[3] &= PCF8563_Shield_Years;
	
//	UART_PRINTF("############# get-time to rtc: %04x-%02x-%02x weekday:%02x",
//		buffer1[3], buffer1[2], buffer1[0], buffer1[1]);

	//
	//判断需要的数据格式
	//
	if (PCF_Format == PCF_Format_BIN)
	{
		PCF_DataStruct->nYear    = BCD2HEX(buffer1[3]) + 2000;
		PCF_DataStruct->nMonth   = BCD2HEX(buffer1[2]);
		PCF_DataStruct->nDay     = BCD2HEX(buffer1[0]);
		PCF_DataStruct->DayIndex = BCD2HEX(buffer1[1]);
	}
	else if (PCF_Format == PCF_Format_BCD)
	{
		//
		//拷贝数据
		//
		PCF_DataStruct->nYear    = buffer1[3] + 0x2000;
		PCF_DataStruct->nMonth   = buffer1[2];
		PCF_DataStruct->nDay     = buffer1[0];
		PCF_DataStruct->DayIndex = buffer1[1];
	}
}

#endif
