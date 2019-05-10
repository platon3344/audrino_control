//#include <LiquidCrystal\src\LiquidCrystal.h>
//#include <SPI\SPI.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#define Channel_A 0x10
#define Channel_B 0x24

volatile int state = HIGH;
//以下端口设定
const int tempSetValue = 11000; 
const float PWMSetValue = 90; // mA; I = 40*V;
const float PWMSetJudgeValue = 98;//mA 
const int DACFullValue = 65536;
const float DACRefVolt = 5;

//AD 参考电压
const int ADFullValue = 1024;
const float ADRefVolt = 5.0;

//设置GPIO口
int Pin_EN1 = 11;//EN-1
int Pin_LD = 10;//LD
int Pin_TEC = 9;//TEC
//int Pin_PWM = 5;// PWM output,由8831替代了。
int Pin_Pid = 6;//PID
// pin2(0),pin3(1),pin18(5),pin19(4),pin20(3),pin21(2).
int pbIn = 5;//中断端口,接管脚18

//设置AD采样
int Pin_AD0 = A0;//analog
int Pin_AD1 = A1;//
int Pin_AD2 = A2;//analog LD_MON

//设置LCD口
uint8_t lcd_rs = 12;
uint8_t lcd_en = 11;
uint8_t lcd_d4 = 5;
uint8_t lcd_d3 = 4;
uint8_t lcd_d2 = 3;
uint8_t lcd_d1 = 2;
LiquidCrystal lcd(lcd_rs, lcd_en, lcd_d4, lcd_d3, lcd_d2,lcd_d1);

const int maxLen = 1024;
int ErrSig[1024] = {0};
int PeakVolt[1024] = {0};

//设置DAC8552 SPI
//需要连接2560开发板的 MOSI = 51，SCK = 52,SS = 53管脚
const int slaveSelectPin = 53;//该管脚设置为SS对于的管脚值

void SPISetup(){
	pinMode(slaveSelectPin, OUTPUT);
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE2);
	SPI.begin();
}

void SPIDigitalPortWrite(int channel ,int value){
	digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(channel);
	SPI.transfer((value & 0xff00) >> 8);
  SPI.transfer(value & 0xff);
	digitalWrite(slaveSelectPin, HIGH);
	delay(1);//写完以后等待1ms
}

void lcdSetup(){
	lcd.begin(16,2);
	lcd.print("hello,world");
}

//显示已经到达温度
void showSuccess(){
	delay(100);
	lcd.clear();
	lcd.write("Temp Arrived Goal!");
}

//显示误差值
void showValue(int value){
	delay(100);
	lcd.clear();
	lcd.write("Error Value :");
  delay(10);
	lcd.write(value);
}

void showError(){
	delay(100);
	lcd.clear();
	lcd.write("Error!");  
}

void showOverflowError(int len){
	delay(100);
	lcd.clear();
	lcd.write("ErrSig Data Len : ");
  delay(10);
	lcd.write(len);
}

void blink(){
  state != state;
}

void ShowShutPower(){
  delay(100);
  lcd.clear();
  lcd.write("ShutDown Power!");
}

void shutdownProgram(){
	digitalWrite(Pin_Pid,LOW);
	delay(2000);
	//analogWrite(Pin_PWM,0);
	SPIDigitalPortWrite(Channel_A,0);
	delay(2000);
	digitalWrite(Pin_LD,LOW);
	delay(2000);
	digitalWrite(Pin_TEC,LOW);
	delay(2000);
	digitalWrite(Pin_EN1,LOW);
	delay(2000);
  //显示关机，切断电源 
   ShowShutPower();
}

//中断 监听中断关机中断
void interruptSetup(){
  attachInterrupt(pbIn,blink,LOW);
  delay(1);
}

int findPeakValue(){
  int tempMax = 0 ;
  int peakValue = -1;
	for (int i = 0; i < maxLen - 1; i++){
    int slope = abs(ErrSig[i] - ErrSig[i+1]);
    if(slope > tempMax){
      tempMax = slope;
      peakValue = PeakVolt[i];
    }
	}
 return peakValue;
}

//初始化端口工作
void initPort(){
	Serial.begin(9600);
	pinMode(Pin_EN1,OUTPUT);
	pinMode(Pin_LD,OUTPUT);
	pinMode(Pin_TEC,OUTPUT);
	pinMode(Pin_Pid,OUTPUT);
//	pinMode(Pin_PWM,OUTPUT);
	pinMode(Pin_AD0,INPUT);
	pinMode(Pin_AD1,INPUT);
	pinMode(Pin_AD2,INPUT);  
}
void work(){
	digitalWrite(Pin_EN1,HIGH); 
	digitalWrite(Pin_TEC,HIGH);
	float ad0_value = 0 ;
	float bias_value = 10000; //init value not satify
	while(1){
		float ad0_value = analogRead(Pin_AD0)/ADFullValue * ADRefVolt * 0.05*1000 ;//输出单位为千欧
		bias_value = abs(ad0_value - tempSetValue);    
		Serial.print(bias_value);
		if(bias_value < 100){
			break;
		}
	}
	showSuccess();
	digitalWrite(Pin_LD,HIGH);
	int PWMVoltOut = 0;//设定从0开始++,
	float volt = PWMSetValue / 40.0 * DACFullValue / DACRefVolt; //
	while (PWMVoltOut < volt){
		PWMVoltOut++; 
		SPIDigitalPortWrite(Channel_A,PWMVoltOut);//SPI输出电压值
    delay(1);
	}
	float ad2_value = analogRead(Pin_AD2) * ADRefVolt / ADFullValue;//电压范围为0~1023。
	//这个地方很有可能不相等，是否考虑在误差在一定范围内
	if(ad2_value != volt){ //如果和设定值90mA对应电压值不同，报错
		showError();
    delay(1);
		shutdownProgram();
	}
	else{
		float ad1_readValue = 0;
		int index = 0;
		while(1){
			ad1_readValue = analogRead(Pin_AD1);
      if(ad1_readValue > 0){
        ErrSig[index] = ad1_readValue; 
        PeakVolt[index] = PWMVoltOut;
        index++;
      }
			PWMVoltOut++;
			SPIDigitalPortWrite(Channel_A,PWMVoltOut);//该地方是都考虑要使用spi输出该值
			float judgeValue = ad1_readValue * ADRefVolt / ADFullValue * 40;
			if (judgeValue > 98.0){//跳出循环，吸收峰检测
				break;
			}
			if (index >= (maxLen - 1)){
				showOverflowError(maxLen);
				break;
			}
		}
		int PeakValueFind = findPeakValue();
		if( PeakValueFind == -1){ //-1 表示没找到
			showError();
			shutdownProgram();
		}
		else{
			int peakVolt = PeakValueFind;
			SPIDigitalPortWrite(Channel_A,peakVolt);
      delay(1);
			digitalWrite(Pin_Pid,HIGH);
		}    
	}
}


void setup() {
	initPort();
	lcdSetup();//lcd init
	SPISetup();
	interruptSetup();//interrupt init
  if(state == HIGH){
    work();//program work
  }
}

void loop() {
  if(state == LOW){
    state = HIGH;
    shutdownProgram();
  }
	int errorValue = analogRead(Pin_AD1);
	float errorReal = errorValue * ADRefVolt / ADFullValue;
	showValue(errorReal);
	Serial.print(errorReal);
}
