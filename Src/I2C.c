#include "I2C.h"//里面包含iic.h


void IIC_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOF_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;//SDA SCL WP
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD ; // 
	GPIO_InitStruct.Pull= GPIO_NOPULL;//上拉
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); //
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
}

void IIC_SDA_IN(void)//SDA用作输入：从机返回数据，从机应答等
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT ; //
	GPIO_InitStruct.Pull = GPIO_NOPULL;//上拉
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 	
}

void IIC_SDA_OUT(void)//SDA用作输出：写地址，写数据等
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; //   
	GPIO_InitStruct.Pull = GPIO_NOPULL;//上拉
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 	
}


void IIC_Start(void)//开始信号
{
	IIC_SDA_OUT(); //SDA定义为输出
	IIC_SDA(GPIO_PIN_SET); 
    
	IIC_SCL(GPIO_PIN_SET);
    delay_us(4);
	IIC_SDA(GPIO_PIN_RESET); 
	delay_us(4);
	IIC_SCL(GPIO_PIN_RESET);
} 

#if 1
void IIC_Stop(void)//停止信号
{
	IIC_SDA_OUT(); //SDA定义为输出
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA(GPIO_PIN_RESET); 
	delay_us(4);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(4);
	IIC_SDA(GPIO_PIN_SET); //发送 I2C 总线结束信号
	delay_us(4); 
}

uint8_t IIC_Wait_Ack(void)//等待应答信号：0-应答；1-非应答
{
	uint8_t ack = 0;
	IIC_SDA(GPIO_PIN_SET);
	IIC_SDA_IN(); //SDA定义为输入 
	delay_us(4); 
	IIC_SCL(GPIO_PIN_SET);
	delay_us(1);
	ack = READ_SDA;
	delay_us(4);
	IIC_SCL(GPIO_PIN_RESET);
	return (ack); 
} 


void IIC_Ack(void)//产生 ACK 应答
{ 
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA_OUT();
	IIC_SDA(GPIO_PIN_RESET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET);
}
 
void IIC_NAck(void)//产生 NACK 非应答
{ 
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA_OUT();
	IIC_SDA(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET);
} 

void IIC_Write_Byte(uint8_t txd)//IIC 发送一个字节
{ 
	uint8_t t; 
	IIC_SDA_OUT(); 
	IIC_SCL(GPIO_PIN_RESET);//拉低时钟开始数据传输
	for(t = 0;t < 8;t++)
	{ 
		IIC_SDA ( (txd & 0x80)>>7); //数据高位在前，低位在后，所以先发送高位
		txd <<= 1; 
		delay_us(4); 
		IIC_SCL(GPIO_PIN_SET);
		delay_us(4); 
		IIC_SCL(GPIO_PIN_RESET);
		delay_us(2);
	}
} 

uint8_t IIC_Read_Byte(void)//读一个字节
{ 
	uint8_t i,receive = 0;
	
	IIC_SDA_IN(); //SDA定义为输入
	for(i = 0;i < 8;i++ )
	{  
		delay_us(4);
		IIC_SCL(GPIO_PIN_SET);
		receive <<= 1;
		if(READ_SDA == 1)
		{
			receive ++;
		}
		delay_us(4); 
		IIC_SCL(GPIO_PIN_RESET);
	}
	return receive;
} 

uint8_t Write_Device_Addr(uint8_t addr)//只写地址
{
	uint8_t Read_ACK = 0;
	IIC_Start();
	IIC_Write_Byte(addr);
	Read_ACK = IIC_Wait_Ack();
	IIC_Stop();
	
	return(Read_ACK);
}

void Search_Device_Addr(void)//在总线上搜寻挂载的器件地址
{
	uint8_t result = 0;
	uint8_t j = 0;
	for(j = 0;j < 128; j++)
	{
		if((j % 16) == 0)
		{
			printf("\r\n");
		}
		result = Write_Device_Addr(j << 1);
		if(result == 0)
		{
			printf(" %X ",j << 1);//%X 十六进制输出，大写；%x 小写
		}
		else
		{
			printf(" -- ");
		}
	}
}

uint8_t Device_Read_One_Byte(uint8_t IC_Addr,uint8_t Addr)//读指定器件的指定位置中的一个字节
{
	uint8_t dat;

	IIC_Start();
	IIC_Write_Byte(IC_Addr);//写地址,7位地址左移，低位补0
	IIC_Wait_Ack();//等待应答
    
	IIC_Write_Byte(Addr);//写位置
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Write_Byte((IC_Addr) | 0x01);//读写位改为读
	IIC_Wait_Ack();

	dat = IIC_Read_Byte();
	IIC_NAck();//读一个字节结束
	IIC_Stop();
	
	return dat;
}

void Device_Write_One_Byte(uint8_t IC_Addr,uint8_t Addr,uint8_t data)//写指定器件的指定位置中的一个字节
{
	IIC_Start();
	IIC_Write_Byte(IC_Addr);//写地址,7位地址左移，低位补0
	IIC_Wait_Ack();//等待应答

	IIC_Write_Byte(Addr);//写位置
	IIC_Wait_Ack();

	IIC_Write_Byte(data);//写数据
	IIC_Wait_Ack();

	IIC_Stop();//发送完结束信号后，器件才会把数据进行擦写操作，搬运到非易失区，这段时间器件不再响应主机操作
}

void Device_Read_Bytes(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len)//连续读指定器件的指定位置中的多个字节
{
	IIC_Start();
	IIC_Write_Byte(IC_Addr);//写地址,7位地址左移，低位补0
	IIC_Wait_Ack();//等待应答

	IIC_Write_Byte(Addr);//写位置
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Write_Byte((IC_Addr) | 0x01);//读写位改为读
	IIC_Wait_Ack();

	while(len > 1)
	{
		*buf++ = IIC_Read_Byte();
		IIC_Ack();
		len--;
	}
	*buf = IIC_Read_Byte();//循环体结束指针已经指向最后一个字节存放位置
	IIC_NAck();//读一个字节结束
	IIC_Stop();	
}

void Device_Write_Bytes(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len)//连续写指定器件的指定位置中的多个字节
{
	while(len > 0)
	{
		IIC_Start();
		IIC_Write_Byte(IC_Addr);//写地址,7位地址左移，低位补0
		IIC_Wait_Ack();//等待应答

		IIC_Write_Byte(Addr++);//写位置
		IIC_Wait_Ack();

		IIC_Write_Byte(*buf++);//写数据
		IIC_Wait_Ack();

		IIC_Stop();//发送完结束信号后，器件才会把数据进行擦写操作，搬运到非易失区，这段时间器件不再响应主机操作
		osDelay(10);//eeprom连续写时必须加，否则下个字节写入失败
		len--;
	}
}

void Device_Page_Write(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len)//按页写指定器件的指定位置
{
	uint8_t i = 0;
	while(len > 0)
	{
		IIC_Start();
		IIC_Write_Byte(IC_Addr);//写地址,7位地址左移，低位补0
		IIC_Wait_Ack();//等待应答

		IIC_Write_Byte(Addr);//写位置
		IIC_Wait_Ack();
		while(len > 0)
		{
			IIC_Write_Byte(*buf++);//写数据
			IIC_Wait_Ack();
			len--;
			Addr++;//防止写入字节数比一页多，自动累加至下一页起始地址
			i++;//判断写入字节数
			if(i == Page_Size)
			{
				break;
			}
		}
		IIC_Stop();//发送完结束信号后，器件才会把数据进行擦写操作，搬运到非易失区，这段时间器件不再响应主机操作
		osDelay(10);//eeprom连续写时必须加，否则下个字节写入失败
		i = 0;
	}
}
#endif


//在此函数中加入初始化sysytick定时器步骤
//参照正点原子例程进行修改
void delay_us(uint32_t nus)
{
       uint32_t ticks;
       uint32_t told,tnow,reload,tcnt=0;
       if((0x0001&(SysTick->CTRL)) ==0)    //定时器未工作
              vPortSetupTimerInterrupt();  //初始化定时器
 
       reload = SysTick->LOAD;                     //获取重装载寄存器值
       ticks = nus * (SystemCoreClock / 1000000);  //计数时间值
       told=SysTick->VAL;                          //获取当前数值寄存器值（开始时数值）
 
       while(1)
       {
              tnow=SysTick->VAL;          //获取当前数值寄存器值
              if(tnow!=told)              //当前值不等于开始值说明已在计数
              {         
 
                     if(tnow<told)             //当前值小于开始数值，说明未计到0
                          tcnt+=told-tnow;     //计数值=开始值-当前值
 
                     else                  //当前值大于开始数值，说明已计到0并重新计数
                            tcnt+=reload-tnow+told;   //计数值=重装载值-当前值+开始值  （已
                                                      //从开始值计到0） 
 
                     told=tnow;                //更新开始值
                     if(tcnt>=ticks)break;     //时间超过/等于要延迟的时间,则退出.
              } 
       }     
}
 
//SystemCoreClock为系统时钟(system_stmf4xx.c中)，通常选择该时钟作为
//systick定时器时钟，根据具体情况更改

