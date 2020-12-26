//SISTEME DAHIL EDILMIS GEREKLI KÜTÜPHANELER
#include "stm32f4xx.h"                  
#include "stm32f4xx_adc.h"             
#include "stm32f4xx_dma.h"              
#include "stm32f4xx_gpio.h"             
#include "stm32f4xx_pwr.h"             
#include "stm32f4xx_rcc.h"             
#include "stm32f4xx_syscfg.h"           
#include "stm32f4xx_tim.h"             
#include "stm32f4xx_usart.h"            
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_gpio.h"
#include "defines.h"
#include "attributes.h"
#include "stdlib.h"
#include <stdio.h>
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_delay.h"
#include <math.h>
#include "tm_stm32f4_ds18b20.h"
#include "tm_stm32f4_onewire.h"
#include "tm_stm32f4_timer_properties.h"
//TIMER AYARLAMALARI
#define TM_DELAY_TIM            TIM2
#define TM_DELAY_TIM_IRQ        TIM2_IRQn
#define TM_DELAY_TIM_IRQ_HANDLER    TIM2_IRQHandler
//SISTEME TANIMLI SICAKLIK SENSÖRÜ SAYISI
#define EXPECTING_SENSORS    3
//SICAKLIK VERILERI IÇIN GEREKLI DEGISKENLER
char buf[40];
uint8_t devices, y, k, count, alarm_count;
uint8_t device[EXPECTING_SENSORS][8];
uint8_t alarm_device[EXPECTING_SENSORS][8];
float temps[EXPECTING_SENSORS];
float serialTemps[3];
float serialBuffer;
//HAM ANALOG-DIJITAL ÇEVRIM VERILERININ TUTULDUGU DIZILER
uint16_t ADC1_ValArray[8];
uint16_t ADC2_ValArray[6];
uint16_t ADC3_ValArray[5];
//FILTRELENMIS ÇEVRIM VERISININ TUTULDUGU DIZI
float filteredADC[18];
//BINDE BIR VE YÜZDE BIR HASSASIYETTE GERILIM DEGERLERININ TUTULDUGU DIZILER
float VoltageValue[18];
float VoltageRounded[18];
//FILTREYE YOLLANICAK GERILIM DEGERLERI IÇIN TAMPON DIZILER
float 
voltageBuffer1[20],voltageBuffer2[20],voltageBuffer3[20],voltageBuffer4[20],voltageBuffer5[20],voltageBuffer1[20],
voltageBuffer6[20],voltageBuffer7[20],voltageBuffer8[20],voltageBuffer9[20],voltageBuffer10[20],
voltageBuffer11[20],voltageBuffer12[20],voltageBuffer13[20],voltageBuffer14[20],voltageBuffer15[20],
voltageBuffer16[20],voltageBuffer17[20],voltageBuffer18[20];
//GERILIM FILTRESI IÇIN GEREKLI SAYAÇLAR
int c1=0,c2=0,c3=0,c4=0,c5=0,c6=0,c7=0,c8=0,c9=0,c10=0,c11=0,c12=0,c13=0,c14=0,c15=0,c16=0,c17=0,c18=0;
//MINIMUM GERILIMIN TUTULDUGU DEGISKEN	
float minimum;
//MINIMUM GERILIMIN DIZIDEKI INDISI
int lokasyon;
//HAREKETLI ORTAMALA FONKSIYONU IÇIN GEREKLI ÇESITLI DEGISKENLER
int counter=0;
int counterMedian=0;
int m,d;
float avg,val1,val2,val3,val4,val5,val6,val7,val8,val9,val10,val11,val12,val13,val14,val15,val16,val17,val18,val19;
//AKIM DEGISKENLERI
int akimAdcBuffer;
float akim;
float finalCurrent;
int ccc = 0;
float stabilAkim;
float akmMaxGerilim;
float wastedPower = 0;
float batteryAh;
char akimUsartBuffer[10];

/***********************************************************
* Function Name  : GPIOE_Disable_OUTPUT
* Description    : E pinlerini deaktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOE_Disable_OUTPUT(){

	
	GPIO_InitTypeDef GPIO_InitStructureOutput;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,DISABLE);
	
	GPIO_InitStructureOutput.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructureOutput.GPIO_Pin=GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_15 | GPIO_Pin_14 ;
	GPIO_InitStructureOutput.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureOutput.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructureOutput.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructureOutput);
	
	
}

void akimKesme(){
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 8399;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 9999;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 50;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_Cmd(TIM3,ENABLE);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//1Sn'lik çekilen Akim.
void TIM3_IRQHandler(){
	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update != RESET)){
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		
		
		akimAdcBuffer = stabilAkim - 2814;
		akim = akimAdcBuffer*0.065;
		//Akim (-) ise (+) Ya Çevir
		if(akim < 0){
			
			akim = -1 * akim;
			
		}
		//Saniyede Harcanan Güç Hesabi
		wastedPower += (akim * akmMaxGerilim) / 3600;
		//Saatte Çekilen Akim Miktari
		batteryAh += akim / 3600;
		//AKS Ye Yollama Kismi
		ccc++;
		
	}
	
	
}






/***********************************************************
* Function Name  : harekatliOrtalama
* Description    : gönderilen veri tekrarli olarak önceki 9 ve sonraki 9
degeri ile toplanip bölünür.
* Input          : sayaç ve deger
* Return         : float ortalama deger
***********************************************************/
float hareketliOrtalama(int counter,float deger){
	
	avg = 0;
	m = 0;
	counter ++;
	if(counter == 1){
		val1 = deger;
		avg = val1;
	}
	if(counter == 2){
		val2 = deger;
		avg = val2;
	}
	if(counter == 3){
		val3= deger;
		avg = val3;
	}
	if(counter == 4){
		val4 = deger;
		avg = val4;
	}
	if(counter == 5){
		val5 =deger;
		avg = val5;
	}
	if(counter == 6){
		val6 = deger;
		avg = val6;
	}
	if(counter == 7){
		val7 = deger;
		avg = val7;
	}
	if(counter == 8){
		val8 = deger;
		avg = val8;
	}
	if(counter == 9){
		val9 = deger;
		avg = val9;
	}
	if(counter == 10){
		val10 = deger;
		avg = val10;
	}
	if(counter == 11){
		val11 = deger;
		avg = val11;
	}
	if(counter == 12){
		val12 = deger;
		avg = val12;
	}
	if(counter == 13){
		val13 = deger;
		avg = val13;
	}
	if(counter == 14){
		val14 = deger;
		avg = val14;
	}
	if(counter == 15){
		val15 = deger;
		avg = val15;
	}
	if(counter == 16){
		val16 = deger;
		avg = val16;
	}
	if(counter == 17){
		val17 = deger;
		avg = val17;
	}
	if(counter == 18){
		val18 = deger;
		avg = val18;
	}
	if(counter == 19){
		val19 = deger;
		avg = val19;
	}
	
	else if(counter > 19){
		counter = 20;
		
		if(val1 == 0){
			m = m+1;
		}
		if(val2 == 0)
	{
		m = m+1;
	}
	if(val3 == 0){
		m = m+1;
	}
	if(val4 == 0){
		m = m+1;
	}
	if(val5 == 0){
		m = m+1;
	}
	if(val6 == 0){
		m = m+1;
	}
	if(val7 == 0){
		m = m+1;
	}
	if(val8 == 0){
		m = m+1;
	}
	if(val9 == 0){
		m = m+1;
	}
	if(val10 == 0){
		m = m+1;
	}
	if(val11 == 0){
		m = m+1;
	}
	if(val12 == 0){
		m = m+1;
	}
	if(val13 == 0){
		m = m+1;
	}
	if(val14 == 0){
		m = m+1;
	}
	if(val15 == 0){
		m = m+1;
	}
	if(val16 == 0){
		m = m+1;
	}
	if(val17 == 0){
		m = m+1;
	}
	if(val18 == 0){
		m = m+1;
	}
	if(val19 == 0){
		m = m+1;
	}
	
	if(deger == 0){
		m = m+1;
	}
	
	d = 20-m;
	
	if(d==0){
		avg = deger;
		counter = 1;
	}
	else{
		avg = (val1+val2+val3+val4+val5+val6+val7+val8+val9+val10+val11+val12+val13+val14+val15+val16+val17+val18+val19+deger)/d;
	}
	val1 = val2;
	val2 = val3;
	val3 = val4;
	val4 = val5;
	val5 = val6;
	val6 = val7;
	val7 = val8;
	val8 = val9;
	val9 = val10;
	val10 = val11;
	val11 = val12;
	val12 = val13;
	val13 = val14;
	val14 = val15;
	val15 = val16;
	val16 = val17;
	val17 = val18;
	val18 = val19;
	val19 = deger;
		
	}
	
	return avg;
	
}




/***********************************************************
* Function Name  : gerilimHesabi
* Description    : Çevrimden gelen veriyi gerilim degerine dönüstürür
* Input          : ADC kanal numarasi
* Return         : None
***********************************************************/
void gerilimHesabi(uint8_t kanal){
	
	if(kanal==0)
		
		VoltageValue[0] = (filteredADC[0]) * 0.0178745154;
	if(kanal==1)
		VoltageValue[1] = ( filteredADC[1]  *( 0.0180407124/2));
	if(kanal==2)
		VoltageValue[2] = ( filteredADC[2] * (0.0182020548/3));
	if(kanal==3)
		VoltageValue[3] =  ( filteredADC[3] * (0.0179266751/4 ));
	if(kanal==4)
		VoltageValue[4] =  ( filteredADC[4] * (0.0182680412/5));
	if(kanal==5)
		VoltageValue[5] =  ( filteredADC[5] * (0.0182262211 /6));
	if(kanal ==6)
		VoltageValue[6] = (filteredADC[6] * (0.018215859/7));
	if(kanal ==7)
		VoltageValue[7] = (filteredADC[7] * (0.0182614295/8));
	if(kanal ==8)
		VoltageValue[8] = (filteredADC[8] * (0.0182551487/9));
	if(kanal ==9)
		VoltageValue[9] = VoltageValue[8];
	if(kanal ==10)
		VoltageValue[10] = (filteredADC[10] * (0.0190895542/11)); 
	if(kanal ==11)
		VoltageValue[11] = (filteredADC[11] * (0.0182731959/12));
	if(kanal ==12)
		VoltageValue[12] = VoltageValue[11]; 
	if(kanal ==13)
		VoltageValue[13] = (filteredADC[13] * (0.0382140108/14));
	if(kanal ==14)
		VoltageValue[14] = (filteredADC[14] * (0.0165288999/15));
	if(kanal ==15)
		VoltageValue[15] = (filteredADC[15] * (0.0381952862/16));
	if(kanal ==16)
		VoltageValue[16] = (filteredADC[16] * (0.0382181357/17));
	if(kanal ==17)
		VoltageValue[17] = (filteredADC[17] * (0.0296882271/18)); 
	
}


/***********************************************************
* Function Name  : GPIOB_Init_OUTPUT
* Description    : B pinlerini çikis verecek sekilde aktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOB_Init_OUTPUT(){

	
	GPIO_InitTypeDef GPIO_InitStructureOutput;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	GPIO_InitStructureOutput.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructureOutput.GPIO_Pin=GPIO_Pin_2 | GPIO_Pin_11 ;
	GPIO_InitStructureOutput.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureOutput.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructureOutput.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructureOutput);
	
	
	
	
	
	
}

/***********************************************************
* Function Name  : GPIOF_Init_OUTPUT
* Description    : F pinlerini çikis verecek sekilde aktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOF_Init_OUTPUT(){

	
	GPIO_InitTypeDef GPIO_InitStructureOutput;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_InitStructureOutput.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructureOutput.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_14 |GPIO_Pin_13 | GPIO_Pin_15 ;
	GPIO_InitStructureOutput.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureOutput.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructureOutput.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructureOutput);

}

/***********************************************************
* Function Name  : GPIOG_Init_OUTPUT
* Description    : G pinlerini çikis verecek sekilde aktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOG_Init_OUTPUT(){

	
	GPIO_InitTypeDef GPIO_InitStructureOutput;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	GPIO_InitStructureOutput.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructureOutput.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStructureOutput.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureOutput.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructureOutput.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructureOutput);
	
	
	
	
	
	
}

/***********************************************************
* Function Name  : GPIOE_Init_OUTPUT
* Description    : E pinlerini çikis verecek sekilde aktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOE_Init_OUTPUT(){

	
	GPIO_InitTypeDef GPIO_InitStructureOutput;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructureOutput.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructureOutput.GPIO_Pin=GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_15 | GPIO_Pin_14 ;
	GPIO_InitStructureOutput.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureOutput.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructureOutput.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructureOutput);
	
	
}

/***********************************************************
* Function Name  : GPIOC_Init_OUTPUT
* Description    : C pinlerini çikis verecek sekilde aktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOC_Init_OUTPUT(){

	
	GPIO_InitTypeDef GPIO_InitStructureOutput;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitStructureOutput.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructureOutput.GPIO_Pin= GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructureOutput.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureOutput.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructureOutput.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructureOutput);
	
	
}




/***********************************************************
* Function Name  : GPIOF_Init_AIN
* Description    : F pinlerini analog okuma yapacak sekilde aktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOF_Init_AIN(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9 | GPIO_Pin_10| GPIO_Pin_4;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}	

/***********************************************************
* Function Name  : GPIOC_Init_AIN
* Description    : C pinlerini analog okuma yapacak sekilde aktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOC_Init_AIN(){
	
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 |GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/***********************************************************
* Function Name  : GPIOA_Init_AIN
* Description    : A pinlerini analog okuma yapacak sekilde aktif hale getirir.
* Input          : None
* Return         : None
***********************************************************/
void GPIOA_Init_AIN(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 |GPIO_Pin_3 | GPIO_Pin_4 |GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/***********************************************************
* Function Name  : Init_ADC1
* Description    : ADC1 kanalina bagli olan pinlerden çevrim yapilacagini
bildirir ve gerekli yarlamalari yapar.
* Input          : None
* Return         : None
***********************************************************/
void Init_ADC1(void){
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay= ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//ADC ÇÖZÜNÜRLÜGÜ 12-BIT(0X000000000000) 2^12 hassasiyete ayarlandi.
	ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode					= ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode		= ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign							= ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion 			= 8;
	
	//ÇEVRIM SILARANI BELIRLENDI.ÇEVRIM SIRASI ILE ESITLENEN ADRESE ATAMA YAPAR.
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 7, ADC_SampleTime_480Cycles);  // PC5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14,8, ADC_SampleTime_480Cycles);   // PC4 Akim
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_480Cycles);  // PA2 ADC13
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_480Cycles);  // PA3 ADC12
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_480Cycles);  // PA4 ADC15                   //4 5 6 7
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_480Cycles);  // PA5 ADC14
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, ADC_SampleTime_480Cycles);  // PA6 ADC17
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 5, ADC_SampleTime_480Cycles);  // PA7	ADC16

	ADC_Init(ADC1,&ADC_InitStructure);
	  
	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);
	
	ADC_DMACmd(ADC1,ENABLE);
  
  ADC_Cmd(ADC1,ENABLE);


	
}
/***********************************************************
* Function Name  : Init_ADC2
* Description    : ADC2 kanalina bagli olan pinlerden çevrim yapilacagini
bildirir ve gerekli yarlamalari yapar.
* Input          : None
* Return         : None
***********************************************************/
void Init_ADC2(void){
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay= ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	//ADC ÇÖZÜNÜRLÜGÜ 12-BIT(0X000000000000) 2^12 hassasiyete ayarlandi.
	ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode					= ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode		= ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign							= ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion 			= 6;
	
	
	
	
		//ÇEVRIM SILARANI BELIRLENDI.ÇEVRIM SIRASI ILE ESITLENEN ADRESE ATAMA YAPAR.
		ADC_RegularChannelConfig(ADC2, ADC_Channel_10,  2, ADC_SampleTime_480Cycles);   // PC0 ADC7
		ADC_RegularChannelConfig(ADC2, ADC_Channel_11,  1, ADC_SampleTime_480Cycles);   // PC1 	//ADC6
		ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 4, ADC_SampleTime_480Cycles);   // PC2		ADC9
		ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 3, ADC_SampleTime_480Cycles);   // PC3		ADC8
		ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 5, ADC_SampleTime_480Cycles);   // PA1		ADC10
		ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 6, ADC_SampleTime_480Cycles);   // PA0 ADC11	
		
	 
	  	ADC_Init(ADC2,&ADC_InitStructure);
			 ADC_Cmd(ADC2,ENABLE);
	  	ADC_DMARequestAfterLastTransferCmd(ADC2,ENABLE);

	
	ADC_DMACmd(ADC2,ENABLE);
  
}

/***********************************************************
* Function Name  : Init_ADC1
* Description    : ADC1 kanalina bagli olan pinlerden çevrim yapilacagini
bildirir ve gerekli yarlamalari yapar.
* Input          : None
* Return         : None
***********************************************************/
void Init_ADC3(void){
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay= ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//ADC ÇÖZÜNÜRLÜGÜ 12-BIT(0X000000000000) 2^12 hassasiyete ayarlandi.
	ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode					= ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode		= ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign							= ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion 			= 5;
	
	
	//ÇEVRIM SILARANI BELIRLENDI.ÇEVRIM SIRASI ILE ESITLENEN ADRESE ATAMA YAPAR.
	  ADC_RegularChannelConfig(ADC3, ADC_Channel_14,4, ADC_SampleTime_480Cycles);    // PF4
 	  ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);   /// PF6
		ADC_RegularChannelConfig(ADC3, ADC_Channel_6, 2, ADC_SampleTime_480Cycles);   //   PF8
	  ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 3, ADC_SampleTime_480Cycles);   //   PF7
		ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 5, ADC_SampleTime_480Cycles);    // PF9
		
	  
		ADC_Init(ADC3,&ADC_InitStructure);
	  
	ADC_DMARequestAfterLastTransferCmd(ADC3,ENABLE);
	
	ADC_DMACmd(ADC3,ENABLE);
  
  ADC_Cmd(ADC3,ENABLE);
}

	
	
	
	
	
	/***********************************************************
* Function Name  : Init_DMA2_CH0_ADC1
* Description    : ADC1 için DMA2 biriminin 0 inci kanalini açar,adresteki veriyi
hangi diziye atacagini belirler ve diger gerekli ayarlari yapar.
* Input          : None
* Return         : None
***********************************************************/
void Init_DMA2_CH0_ADC1(void){  ///////////// FOR ADC1
	
	DMA_InitTypeDef DMA_InitStructure;
	
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	DMA_InitStructure.DMA_Channel            = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)0x4001204C);      /////////////////   ((uint32_t)0x4001224C)
	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t) ADC1_ValArray;
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize         = 8;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);
	
	
}

/***********************************************************
* Function Name  : Init_DMA2_CH1_ADC2
* Description    : ADC2 için DMA2 biriminin 1 inci kanalini açar,adresteki veriyi
hangi diziye atacagini belirler ve diger gerekli ayarlari yapar.
* Input          : None
* Return         : None
***********************************************************/
void Init_DMA2_CH1_ADC2(void){  ///////////// FOR ADC2
	
	DMA_InitTypeDef DMA_InitStructure;
	
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	DMA_InitStructure.DMA_Channel            = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) 0x4001214C;     
	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t) ADC2_ValArray;
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize         = 6;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  
	
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	
  
}

/***********************************************************
* Function Name  : Init_DMA2_CH2_ADC3
* Description    : ADC3 için DMA2 biriminin 2 inci kanalini açar,adresteki veriyi
hangi diziye atacagini belirler ve diger gerekli ayarlari yapar.
* Input          : None
* Return         : None
***********************************************************/
void Init_DMA2_CH2_ADC3(void){  ///////////// FOR ADC3
	
	DMA_InitTypeDef  DMA_InitStructure;
	
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	DMA_InitStructure.DMA_Channel            = DMA_Channel_2;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) 0x4001224C;     
	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t) ADC3_ValArray;
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize         = 5;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

  
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	
	
}

/***********************************************************
* Function Name  : gerilimFiltre
* Description    : Tampon diziden sirasiyla okunan 20 gerilim degerini alir
küçükten büyüge siralar ortadaki 5 elemanini baska bir tampon diziye atar 
tampon dizinin ortalamasini filtre edilmis gerilim olarak geri döndürür.
* Input          : float sirali okunmus gerilimler
* Return         : float filtre edilmis gerilim
***********************************************************/
float gerilimFiltre(float datas[]){
	
	float geciciData[5];
	float voltageBuffer;
	for(int i=0;i<19;i++){
		for(int j=0;j<19;j++){
			if(datas[i]<datas[j+1]){
				voltageBuffer = datas[j];
				datas[i]=datas[j+1];
				datas[j+1]=voltageBuffer;
			}
		}
	}
	
	geciciData[0] = datas[8];
	geciciData[1] = datas[9];
	geciciData[2] = datas[10];
	geciciData[3] = datas[11];
	geciciData[4] = datas[12];
	
	return (geciciData[0] + geciciData[1] + geciciData[2] + geciciData[3] + geciciData[4])/5;
	
}

/***********************************************************
* Function Name  : roundFloat
* Description    : x.xxx hassasiyette okunan veriyi x.x hassasiyete indirger.
* Input          : float indirgenecek veri
* Return         : float indirgenmis veri
***********************************************************/
float roundFloat(float x) {
  return ((float)((int)(x * 10))) / 10;
}




int main(){
	//Degistirilmis saat hizlarini sisteme belirtir.Frekansi 180Mhz olarak ayarlar
	RCC_HSEConfig(RCC_HSE_ON);
	while(!RCC_WaitForHSEStartUp());
	TM_OneWire_t OneWire1;
	SystemInit();
	akimKesme();
	TM_DELAY_Init();
	//Fonksiyonlarin aktif hale getirilmesi
	TM_OneWire_Init(&OneWire1,GPIOD,GPIO_Pin_10);
	
	GPIOA_Init_AIN();
	GPIOC_Init_AIN();
	GPIOF_Init_AIN();
	GPIOB_Init_OUTPUT();
	GPIOC_Init_OUTPUT();
	GPIOF_Init_OUTPUT();
	GPIOG_Init_OUTPUT();
	Init_ADC1();
	Init_ADC2();
	Init_ADC3();
	
	Init_DMA2_CH0_ADC1();
	Init_DMA2_CH1_ADC2();
	Init_DMA2_CH2_ADC3();
	//Analog-Dijital Çevrimin baslatilmasi
  ADC_SoftwareStartConv(ADC1);
	ADC_SoftwareStartConv(ADC2);
	ADC_SoftwareStartConv(ADC3);
	//Araç Genel Devresinin Açilmasi
	GPIO_SetBits(GPIOC,GPIO_Pin_7);
	
	
	//SICAKLIK//////////////////////////
	
	  count = 0;
    devices = TM_OneWire_First(&OneWire1);
    while (devices) {
        /* Increase counter */
        count++;
        
        /* Get full ROM value, 8 bytes, give location of first byte where to save */
        TM_OneWire_GetFullROM(&OneWire1, device[count - 1]);
        
        /* Get next device */
        devices = TM_OneWire_Next(&OneWire1);
    }
    
    /* If any devices on 1wire */
    if (count > 0) {
        sprintf(buf, "Devices found on 1-wire: %d\n", count);
        TM_USART_Puts(USART1, buf);
        /* Display 64bit rom code for each device */
        for (k = 0; k < count; k++) {
            for (y = 0; y < 8; y++) {
                sprintf(buf, "0x%02X ", device[k][y]);
                TM_USART_Puts(USART1, buf);
            }
            TM_USART_Puts(USART1, "\n");
        }
    } else {
        TM_USART_Puts(USART1, "No devices on OneWire.\n");
    }
    
    /* Go through all connected devices and set resolution to 12bits */
    for (y = 0; y < count; y++) {
        /* Set resolution to 12bits */
        TM_DS18B20_SetResolution(&OneWire1, device[y], TM_DS18B20_Resolution_12bits);
    }
    
    
   
	
	while(1){
		//TM_USART_Puts(USART1,"asdas");
		//SICAKLIK/////////////////////////////
		
		 /* Start temperature conversion on all devices on one bus */
        TM_DS18B20_StartAll(&OneWire1);
        
        /* Wait until all are done on one onewire port */
        while(!TM_DS18B20_AllDone(&OneWire1));
        
        /* Read temperature from each device separatelly */
        for (y = 0; y < count; y++) {
            /* Read temperature from ROM address and store it to temps variable */
            if (TM_DS18B20_Read(&OneWire1, device[y], &temps[y])) {
                /* Print temperature */
                sprintf(buf, "Temp %d: %3.5f; \n", y, temps[y]);
                TM_USART_Puts(USART1, buf);
            } else {
                /* Reading error */
                TM_USART_Puts(USART1, "Reading error;\n");
            }
        }
        
       //Sicakliklarin tampon diziye kopyalanmasi
				for(int i=0;i<3;i++){
					serialTemps[i] = temps[i];
				}
				//Tampon dizinin küçükten büyüge siralanmasi sonuncu indir en büyük eleman.
				for(int i=0;i<3;i++){
					for(int j=i;j<3;j++){
						if(serialTemps[i]<serialTemps[j+1]){
							serialBuffer = serialTemps[j];
							serialTemps[i] = serialTemps[j+1];
							
							serialTemps[j+1] = serialBuffer;
						}
					}
				}
				//sonuncu elemanin deger kontrolüne göre flasörün yakilmasi yada sistemin kapatilmasi.	
				if(serialTemps[0] > 55){
					GPIO_SetBits(GPIOC,GPIO_Pin_8);
				}else{
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
				}
			
				if(serialTemps[0] > 70){
					GPIO_ResetBits(GPIOC,GPIO_Pin_7);	
				}else{
					GPIO_SetBits(GPIOC,GPIO_Pin_7);
				}
				
				
				
				
		//SICAKLIK/////////////////////////////////////////////////////////////
		
		//ADC Verilerinin dogru okunabilmesi için GPIOE Hattinin kesilmesi
		GPIOE_Disable_OUTPUT();
			
		//Etkilenmemis ADC Datasi stabilAkim degiskenine atandi.				
		stabilAkim = ADC1_ValArray[7];
				
				
		//ADC Verilerinin filtre edilmesi
		voltageBuffer1[c1] = ADC3_ValArray[0];
		c1++;
		if(c1 == 20){
			c1 = 0;
			filteredADC[0] = gerilimFiltre(voltageBuffer1);
		}
		//Delayms(5);
		
		
	
		
		voltageBuffer2[c2] = ADC3_ValArray[1];
		c2++;
		if(c2 == 20){
			c2 = 0;
			filteredADC[1] = gerilimFiltre(voltageBuffer2);
		}
		//Delayms(5);
		
		voltageBuffer3[c3] = ADC3_ValArray[2];
		c3++;
		if(c3 == 20){
			c3 = 0;
			filteredADC[2] = gerilimFiltre(voltageBuffer3);
		}
		//Delayms(5);
		
		voltageBuffer4[c4] = ADC3_ValArray[3];
		c4++;
		if(c4 == 20){
			c4 = 0;
			filteredADC[3] = gerilimFiltre(voltageBuffer4);
		}
		//Delayms(5);
		
		voltageBuffer5[c5] = ADC3_ValArray[4];
		c5++;
		if(c5 == 20){
			c5 = 0;
			filteredADC[4] = gerilimFiltre(voltageBuffer5);
		}
		//Delayms(5);
		
		voltageBuffer6[c6] = ADC2_ValArray[0];
		c6++;
		if(c6 == 20){
			c6 = 0;
			filteredADC[5] = gerilimFiltre(voltageBuffer6);
		}
		//Delayms(5);
		//////////////////////
		voltageBuffer7[c7] = ADC2_ValArray[1];
		c7++;
		if(c7 == 20){
			c7 = 0;
			filteredADC[6] = gerilimFiltre(voltageBuffer7);
		}
		//Delayms(5);
		
		voltageBuffer8[c8] = ADC2_ValArray[2];
		c8++;
		if(c8 == 20){
			c8 = 0;
			filteredADC[7] = gerilimFiltre(voltageBuffer8);
		}
		//Delayms(5);
		
		voltageBuffer9[c9] = ADC2_ValArray[3];
		c9++;
		if(c9 == 20){
			c9 = 0;
			filteredADC[8] = gerilimFiltre(voltageBuffer9);
		}
		//Delayms(5);
		voltageBuffer10[c10] = ADC2_ValArray[4];
		c10++;
		if(c10 == 20){
			c10 = 0;
			filteredADC[9] = gerilimFiltre(voltageBuffer10);
		}
		//Delayms(5);
		voltageBuffer11[c11] = ADC2_ValArray[5];
		c11++;
		if(c11 == 20){
			c11 = 0;
			filteredADC[10] = gerilimFiltre(voltageBuffer11);
		}
		//Delayms(5);
		voltageBuffer12[c12] = ADC1_ValArray[0];
		c12++;
		if(c12 == 20){
			c12 = 0;
			filteredADC[11] = gerilimFiltre(voltageBuffer12);
		}
		//Delayms(5);
		voltageBuffer13[c13] = ADC1_ValArray[1];
		c13++;
		if(c13 == 20){
			c13 = 0;
			filteredADC[12] = gerilimFiltre(voltageBuffer13);
		}
		//Delayms(5);
		voltageBuffer14[c14] = ADC1_ValArray[2];
		c14++;
		if(c14 == 20){
			c14 = 0;
			filteredADC[13] = gerilimFiltre(voltageBuffer14);
		}
		//Delayms(5);
		voltageBuffer15[c15] = ADC1_ValArray[3];
		c15++;
		if(c15 == 20){
			c15 = 0;
			filteredADC[14] = gerilimFiltre(voltageBuffer15);
		}
		//Delayms(5);
		voltageBuffer16[c16] = ADC1_ValArray[4];
		c16++;
		if(c16 == 20){
			c16 = 0;
			filteredADC[15] = gerilimFiltre(voltageBuffer16);
		}
		//Delayms(5);
		voltageBuffer17[c17] = ADC1_ValArray[5];
		c17++;
		if(c17 == 20){
			c17 = 0;
			filteredADC[16] = gerilimFiltre(voltageBuffer17);
		}
		//Delayms(5);
		voltageBuffer18[c18] = ADC1_ValArray[6];
		c18++;
		if(c18 == 20){
			c18 = 0;
			filteredADC[17] = gerilimFiltre(voltageBuffer18);
			
			//Tüm kanallar için gerilim hesaplarinin yapilmasi
			for(int i=0;i<18;i++){
			
			gerilimHesabi(i);
				
				
				
				
			
		}
		//Gerilimlerin hassasiyetinin düsürülmesi	
		for(int i=0;i<18;i++){
			
			VoltageRounded[i] = roundFloat(VoltageValue[i]);
			
		}
		//Toplam Pil Gerilimi Akim Saymak için Gerekli Degiskene Aktarildi
		akmMaxGerilim = (filteredADC[17] * 0.0296882271);
		
			//Röleleri tetiklemek için GPIOE Hattinin tekrardan açilmasi.
			GPIOE_Init_OUTPUT();
		
		
		//En düsük gerilimin ve dizide bulundugu adresin bulunmasi
		minimum = VoltageRounded[0];
		lokasyon = 0;
		for(int i=0;i<18;i++){
			if(minimum>VoltageRounded[i]){
				minimum = VoltageRounded[i];
				lokasyon = i;
				
			}
			
		}
			
		//Tüm gerilimlerin en düsük degerle karsilastirilmasi ve duruma göre rölelerin tetiklenmesi
		/*
		for(int i =0;i<18;i++){
			if(VoltageRounded[i]>minimum){
				if(i == 0 ) GPIO_SetBits(GPIOB,GPIO_Pin_2);
				if(i == 1 ) GPIO_SetBits(GPIOF,GPIO_Pin_12);
				if(i == 2 ) GPIO_SetBits(GPIOF,GPIO_Pin_11);
				if(i == 3 ) GPIO_SetBits(GPIOF,GPIO_Pin_14);
				if(i == 4 ) GPIO_SetBits(GPIOF,GPIO_Pin_13);
				if(i == 5 ) GPIO_SetBits(GPIOG,GPIO_Pin_0);
				if(i == 6 ) GPIO_SetBits(GPIOF,GPIO_Pin_15);
				if(i == 7 ) GPIO_SetBits(GPIOE,GPIO_Pin_7);
				if(i == 8 ) GPIO_SetBits(GPIOG,GPIO_Pin_1);
		    if(i == 9 ) GPIO_SetBits(GPIOE,GPIO_Pin_9);
				if(i == 10 ) GPIO_SetBits(GPIOE,GPIO_Pin_8);
				if(i == 11 ) GPIO_SetBits(GPIOE,GPIO_Pin_11);
				if(i == 12 ) GPIO_SetBits(GPIOE,GPIO_Pin_10);
				if(i == 13 ) GPIO_SetBits(GPIOE,GPIO_Pin_13);
				if(i == 14 ) GPIO_SetBits(GPIOE,GPIO_Pin_12);
				if(i == 15 ) GPIO_SetBits(GPIOE,GPIO_Pin_15);
				if(i == 16 ) GPIO_SetBits(GPIOE,GPIO_Pin_14);
				if(i == 17 ) GPIO_SetBits(GPIOB,GPIO_Pin_11);
				
			}
			
		}
		//0.75sn rölelerin açik tutulmasi
		Delayms(750);
		//rölelerin kapatilmasi
		GPIO_ResetBits(GPIOB,GPIO_Pin_2);
		GPIO_ResetBits(GPIOF,GPIO_Pin_12);
		GPIO_ResetBits(GPIOF,GPIO_Pin_11);
		GPIO_ResetBits(GPIOF,GPIO_Pin_14);
		GPIO_ResetBits(GPIOF,GPIO_Pin_13);
		GPIO_ResetBits(GPIOG,GPIO_Pin_0);
		GPIO_ResetBits(GPIOF,GPIO_Pin_15);
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		GPIO_ResetBits(GPIOG,GPIO_Pin_1);
		GPIO_ResetBits(GPIOE,GPIO_Pin_9);
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
		GPIO_ResetBits(GPIOE,GPIO_Pin_11);
		GPIO_ResetBits(GPIOE,GPIO_Pin_10);
		GPIO_ResetBits(GPIOE,GPIO_Pin_13);
		GPIO_ResetBits(GPIOE,GPIO_Pin_12);
		GPIO_ResetBits(GPIOE,GPIO_Pin_15);
		GPIO_ResetBits(GPIOE,GPIO_Pin_14);
		GPIO_ResetBits(GPIOB,GPIO_Pin_11);	
		//0.3Sn sistem kararliligini saglamak için sistemin bekletirmesi.
		Delayms(300);
		*/
			
			
		}
		
		
		
		
		
		
		
		
		
		
		
		
	
	
	
	}}
		