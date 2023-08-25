#include "I2C.h"//�������iic.h


void IIC_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOF_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;//SDA SCL WP
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD ; // 
	GPIO_InitStruct.Pull= GPIO_NOPULL;//����
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); //
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
}

void IIC_SDA_IN(void)//SDA�������룺�ӻ��������ݣ��ӻ�Ӧ���
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT ; //
	GPIO_InitStruct.Pull = GPIO_NOPULL;//����
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 	
}

void IIC_SDA_OUT(void)//SDA���������д��ַ��д���ݵ�
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; //   
	GPIO_InitStruct.Pull = GPIO_NOPULL;//����
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 	
}


void IIC_Start(void)//��ʼ�ź�
{
	IIC_SDA_OUT(); //SDA����Ϊ���
	IIC_SDA(GPIO_PIN_SET); 
    
	IIC_SCL(GPIO_PIN_SET);
    delay_us(4);
	IIC_SDA(GPIO_PIN_RESET); 
	delay_us(4);
	IIC_SCL(GPIO_PIN_RESET);
} 

#if 1
void IIC_Stop(void)//ֹͣ�ź�
{
	IIC_SDA_OUT(); //SDA����Ϊ���
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA(GPIO_PIN_RESET); 
	delay_us(4);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(4);
	IIC_SDA(GPIO_PIN_SET); //���� I2C ���߽����ź�
	delay_us(4); 
}

uint8_t IIC_Wait_Ack(void)//�ȴ�Ӧ���źţ�0-Ӧ��1-��Ӧ��
{
	uint8_t ack = 0;
	IIC_SDA(GPIO_PIN_SET);
	IIC_SDA_IN(); //SDA����Ϊ���� 
	delay_us(4); 
	IIC_SCL(GPIO_PIN_SET);
	delay_us(1);
	ack = READ_SDA;
	delay_us(4);
	IIC_SCL(GPIO_PIN_RESET);
	return (ack); 
} 


void IIC_Ack(void)//���� ACK Ӧ��
{ 
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA_OUT();
	IIC_SDA(GPIO_PIN_RESET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET);
}
 
void IIC_NAck(void)//���� NACK ��Ӧ��
{ 
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA_OUT();
	IIC_SDA(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET);
} 

void IIC_Write_Byte(uint8_t txd)//IIC ����һ���ֽ�
{ 
	uint8_t t; 
	IIC_SDA_OUT(); 
	IIC_SCL(GPIO_PIN_RESET);//����ʱ�ӿ�ʼ���ݴ���
	for(t = 0;t < 8;t++)
	{ 
		IIC_SDA ( (txd & 0x80)>>7); //���ݸ�λ��ǰ����λ�ں������ȷ��͸�λ
		txd <<= 1; 
		delay_us(4); 
		IIC_SCL(GPIO_PIN_SET);
		delay_us(4); 
		IIC_SCL(GPIO_PIN_RESET);
		delay_us(2);
	}
} 

uint8_t IIC_Read_Byte(void)//��һ���ֽ�
{ 
	uint8_t i,receive = 0;
	
	IIC_SDA_IN(); //SDA����Ϊ����
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

uint8_t Write_Device_Addr(uint8_t addr)//ֻд��ַ
{
	uint8_t Read_ACK = 0;
	IIC_Start();
	IIC_Write_Byte(addr);
	Read_ACK = IIC_Wait_Ack();
	IIC_Stop();
	
	return(Read_ACK);
}

void Search_Device_Addr(void)//����������Ѱ���ص�������ַ
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
			printf(" %X ",j << 1);//%X ʮ�������������д��%x Сд
		}
		else
		{
			printf(" -- ");
		}
	}
}

uint8_t Device_Read_One_Byte(uint8_t IC_Addr,uint8_t Addr)//��ָ��������ָ��λ���е�һ���ֽ�
{
	uint8_t dat;

	IIC_Start();
	IIC_Write_Byte(IC_Addr);//д��ַ,7λ��ַ���ƣ���λ��0
	IIC_Wait_Ack();//�ȴ�Ӧ��
    
	IIC_Write_Byte(Addr);//дλ��
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Write_Byte((IC_Addr) | 0x01);//��дλ��Ϊ��
	IIC_Wait_Ack();

	dat = IIC_Read_Byte();
	IIC_NAck();//��һ���ֽڽ���
	IIC_Stop();
	
	return dat;
}

void Device_Write_One_Byte(uint8_t IC_Addr,uint8_t Addr,uint8_t data)//дָ��������ָ��λ���е�һ���ֽ�
{
	IIC_Start();
	IIC_Write_Byte(IC_Addr);//д��ַ,7λ��ַ���ƣ���λ��0
	IIC_Wait_Ack();//�ȴ�Ӧ��

	IIC_Write_Byte(Addr);//дλ��
	IIC_Wait_Ack();

	IIC_Write_Byte(data);//д����
	IIC_Wait_Ack();

	IIC_Stop();//����������źź������Ż�����ݽ��в�д���������˵�����ʧ�������ʱ������������Ӧ��������
}

void Device_Read_Bytes(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len)//������ָ��������ָ��λ���еĶ���ֽ�
{
	IIC_Start();
	IIC_Write_Byte(IC_Addr);//д��ַ,7λ��ַ���ƣ���λ��0
	IIC_Wait_Ack();//�ȴ�Ӧ��

	IIC_Write_Byte(Addr);//дλ��
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Write_Byte((IC_Addr) | 0x01);//��дλ��Ϊ��
	IIC_Wait_Ack();

	while(len > 1)
	{
		*buf++ = IIC_Read_Byte();
		IIC_Ack();
		len--;
	}
	*buf = IIC_Read_Byte();//ѭ�������ָ���Ѿ�ָ�����һ���ֽڴ��λ��
	IIC_NAck();//��һ���ֽڽ���
	IIC_Stop();	
}

void Device_Write_Bytes(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len)//����дָ��������ָ��λ���еĶ���ֽ�
{
	while(len > 0)
	{
		IIC_Start();
		IIC_Write_Byte(IC_Addr);//д��ַ,7λ��ַ���ƣ���λ��0
		IIC_Wait_Ack();//�ȴ�Ӧ��

		IIC_Write_Byte(Addr++);//дλ��
		IIC_Wait_Ack();

		IIC_Write_Byte(*buf++);//д����
		IIC_Wait_Ack();

		IIC_Stop();//����������źź������Ż�����ݽ��в�д���������˵�����ʧ�������ʱ������������Ӧ��������
		osDelay(10);//eeprom����дʱ����ӣ������¸��ֽ�д��ʧ��
		len--;
	}
}

void Device_Page_Write(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len)//��ҳдָ��������ָ��λ��
{
	uint8_t i = 0;
	while(len > 0)
	{
		IIC_Start();
		IIC_Write_Byte(IC_Addr);//д��ַ,7λ��ַ���ƣ���λ��0
		IIC_Wait_Ack();//�ȴ�Ӧ��

		IIC_Write_Byte(Addr);//дλ��
		IIC_Wait_Ack();
		while(len > 0)
		{
			IIC_Write_Byte(*buf++);//д����
			IIC_Wait_Ack();
			len--;
			Addr++;//��ֹд���ֽ�����һҳ�࣬�Զ��ۼ�����һҳ��ʼ��ַ
			i++;//�ж�д���ֽ���
			if(i == Page_Size)
			{
				break;
			}
		}
		IIC_Stop();//����������źź������Ż�����ݽ��в�д���������˵�����ʧ�������ʱ������������Ӧ��������
		osDelay(10);//eeprom����дʱ����ӣ������¸��ֽ�д��ʧ��
		i = 0;
	}
}
#endif


//�ڴ˺����м����ʼ��sysytick��ʱ������
//��������ԭ�����̽����޸�
void delay_us(uint32_t nus)
{
       uint32_t ticks;
       uint32_t told,tnow,reload,tcnt=0;
       if((0x0001&(SysTick->CTRL)) ==0)    //��ʱ��δ����
              vPortSetupTimerInterrupt();  //��ʼ����ʱ��
 
       reload = SysTick->LOAD;                     //��ȡ��װ�ؼĴ���ֵ
       ticks = nus * (SystemCoreClock / 1000000);  //����ʱ��ֵ
       told=SysTick->VAL;                          //��ȡ��ǰ��ֵ�Ĵ���ֵ����ʼʱ��ֵ��
 
       while(1)
       {
              tnow=SysTick->VAL;          //��ȡ��ǰ��ֵ�Ĵ���ֵ
              if(tnow!=told)              //��ǰֵ�����ڿ�ʼֵ˵�����ڼ���
              {         
 
                     if(tnow<told)             //��ǰֵС�ڿ�ʼ��ֵ��˵��δ�Ƶ�0
                          tcnt+=told-tnow;     //����ֵ=��ʼֵ-��ǰֵ
 
                     else                  //��ǰֵ���ڿ�ʼ��ֵ��˵���ѼƵ�0�����¼���
                            tcnt+=reload-tnow+told;   //����ֵ=��װ��ֵ-��ǰֵ+��ʼֵ  ����
                                                      //�ӿ�ʼֵ�Ƶ�0�� 
 
                     told=tnow;                //���¿�ʼֵ
                     if(tcnt>=ticks)break;     //ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
              } 
       }     
}
 
//SystemCoreClockΪϵͳʱ��(system_stmf4xx.c��)��ͨ��ѡ���ʱ����Ϊ
//systick��ʱ��ʱ�ӣ����ݾ����������

