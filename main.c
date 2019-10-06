

/* Includes */
#include "stm32f0xx.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/


void init_GPIO(void);
void init_EXTI (void);
void init_PWM(void);
void init_TIM6(void);
void init_TIM14(void);

void leftWheelForward(void);
void rightWheelForward(void);
void leftWheelReverse(void);
void rightWheelReverse(void);
void leftWheelIdle(void);
void rightWheelIdle(void);
void turnLeft(void);
void turnRight(void);
void goStraight(void);
void uTurn(void);
void idle(void);
void delay(int time);
int checkSensor(int bitMask);
void stop(void);
void checkEnd(void);
void poll(void);
void turn(void);
void checkUturn(void);
void TIM6_IRQHandler (void);
void uTurn(void);
void turnAround(void);
void solve(void);


/////////   Variables   ///////////
#define frontSensor 	    0b1000
#define leftSensor  		   0b1
#define rightSensor           0b10
#define leftStabiliser       0b100
#define rightStabiliser  0b1000000
#define wheelSensor       0b100000
#define unit		   1//*3.1415/4 // (cm) D*pi/ number of points on wheel
#define button          0b10000000

#define leftMotorInput1_HIGH         0b1000000
#define leftMotorInput1_LOW    0b1111110111111
#define leftMotorInput2_HIGH        0b10000000
#define leftMotorInput2_LOW   0b11111101111111
#define rightMotorInput1_HIGH      0b100000000
#define rightMotorInput1_LOW   0b1111011111111
#define rightMotorInput2_HIGH     0b1000000000
#define rightMotorInput2_LOW  0b11110111111111

int left, right, forward;
int count = 0;
int compass = 0;

int path[10];
int explore[50];

int totalPoints = 0;
int rewriteCount = 0;
int previewCount = 0;


int pointNum = 0;
int stopCount = 0;

int main(void)
{
	init_GPIO();
	init_EXTI();
	init_PWM();
	init_TIM6();
	init_TIM14();
	for(int i=0; i<50; i++)
	{
		explore[i] = 0;
	}
while(1)
{
	while(checkSensor(button))
	{

	}
	goStraight();
  while (1)
  {
	  checkUturn();
	  if((!checkSensor(frontSensor)) && checkSensor(leftStabiliser) && checkSensor(rightStabiliser))
	  		{
                count = 0;
		  	  	checkEnd();
	  			goStraight();
	  			TIM2->CCR3 = 100 * 80; // Duty Cycle for PB10 motor 1 left
	  			TIM2->CCR4 = 100 * 80; // Duty Cycle for PB11 motor 2 right
	  		}
	  		else if(!checkSensor(leftStabiliser))
	  		{
                count = 0;
	  			checkEnd();
	  			GPIOB -> ODR &= 0b111111101111;
	  			GPIOB -> ODR |= 0b100000;
	  			goStraight();
	  			//leftWheelIdle();
	  			TIM2->CCR3 = 85 * 80; // Duty Cycle for PB10 motor 1 left
	  			TIM2->CCR4 = 100 * 80; // Duty Cycle for PB11 motor 2 right
	  		}
	  		else if(!checkSensor(rightStabiliser))
	  		{
                count = 0;
	  			checkEnd();
	  			GPIOB -> ODR &= 0b111111011111;
	  			GPIOB -> ODR |= 0b10000;
	  			goStraight();
	  			TIM2->CCR3 = 100 * 80; // Duty Cycle for PB10 motor 1 left
	  			//rightWheelIdle();
	  			TIM2->CCR4 = 85 * 80; // Duty Cycle for PB11 motor 2 right
	  		}
	  if(!checkSensor(leftSensor))
	  {
          count = 0;
		  checkEnd();
		  poll();
	  }
	  else if(!checkSensor(rightSensor))
	  {
          count = 0;
		  checkEnd();
		  poll();
	  }
	  else
		{
		  checkEnd();
		  goStraight();
		}

  }
}
}

/////////////////////////////
//////////functions
/////////////////////////////
void poll(void)
{
	left = 0;
	right = 0;
	forward = 0;
	for(int i =0;i<5000;i++)
	{
		if((!checkSensor(frontSensor))|(!checkSensor(leftStabiliser))|(!checkSensor(rightStabiliser)))
		{
			forward = 1;
		}
		if(!checkSensor(leftSensor))
		{
			left = 1;
		}
		if(!checkSensor(rightSensor))
		{
			right = 1;
		}
	}
		turn();
		return;

}
void turn(void)
{


	if(left)
	{
		turnLeft();
		return;
	}
	else if (forward)
	{
		goStraight();
		return;
	}
	else if(right)
	{
		turnRight();
		return;
	}

	/*
	checkEnd();
	if(path[pointNum] == 0)
	{
		turnLeft();
		pointNum ++;
		return;
	}
	else if(path[pointNum] == 1)
	{
		goStraight();
		pointNum ++;
		return;
	}
	else if(path[pointNum]==2)
	{
		turnRight();
		pointNum ++;
		return;
	}
	*/
}

void solve(void)
{

	for(int i=0;i<totalPoints;i++)
	{
		if(explore[previewCount] ==0)
		{
			if(explore[previewCount + 1] == 3)
			{
				if(explore[previewCount +2] == 2)
				{
					explore[rewriteCount] = 3;
					rewriteCount ++;
				}
				else if(explore[previewCount +2] == 1)
				{
					explore[rewriteCount] = 2;
					rewriteCount ++;
				}
				else if(explore[previewCount +2] == 0)
				{
					explore[rewriteCount] = 1;
					rewriteCount ++;
				}
			}
		}
		else if(explore[previewCount] == 1)
		{
			if(explore[previewCount + 1] == 3)
			{
				if(explore[previewCount +2] == 0)
				{
					explore[rewriteCount] = 2;
					rewriteCount ++;
				}
				else if(explore[previewCount +2] == 1)
				{
					explore[rewriteCount] = 3;
					rewriteCount ++;
				}

			}
		}
		else if(explore[previewCount] == 2)
		{
			if(explore[previewCount + 1] == 3)
			{
				if(explore[previewCount + 2] == 0)
				{
					explore[rewriteCount] = 3;
				}
			}
		}
	}
}
void goStraight(void)
{
	leftWheelForward();
	rightWheelForward();
}

void turnRight()
{
	compass --;
	delay(7);
	checkEnd();
	leftWheelForward();
	rightWheelReverse();
	checkEnd();
	delay(30);
	checkEnd();
	while(checkSensor(rightStabiliser))
	{

	}
	TIM2->CCR3 = 70 * 80; // Duty Cycle for PB10 motor 1 left
	TIM2->CCR4 = 70 * 80; // Duty Cycle for PB11 motor 2 right
	while(checkSensor(frontSensor))
	{

	}
	checkEnd();
	goStraight();
	TIM2->CCR3 = 100 * 80; // Duty Cycle for PB10 motor 1 left
	TIM2->CCR4 = 100 * 80; // Duty Cycle for PB11 motor 2 right
	delay(30);

}

void turnLeft()
{
	compass ++;
	delay(7);
	checkEnd();
	rightWheelForward();
	leftWheelReverse();
	checkEnd();
	delay(30);
	checkEnd();
	while(checkSensor(leftStabiliser))
	{

	}
	TIM2->CCR3 = 70 * 80; // Duty Cycle for PB10 motor 1 left
	TIM2->CCR4 = 70 * 80; // Duty Cycle for PB11 motor 2 right
	while(checkSensor(frontSensor))
	{

	}
	checkEnd();
	goStraight();
	TIM2->CCR3 = 100 * 80; // Duty Cycle for PB10 motor 1 left
	TIM2->CCR4 = 100 * 80; // Duty Cycle for PB11 motor 2 right
	delay(30);
}

void uTurn(void)
{	compass -= 2;
	rightWheelForward();
	leftWheelReverse();
	while(checkSensor(leftStabiliser))
	{

	}
	TIM2->CCR3 = 70 * 80; // Duty Cycle for PB10 motor 1 left
	TIM2->CCR4 = 70 * 80; // Duty Cycle for PB11 motor 2 right
	while(checkSensor(frontSensor))
	{

	}
	goStraight();
	TIM2->CCR3 = 100 * 80; // Duty Cycle for PB10 motor 1 left
	TIM2->CCR4 = 100 * 80; // Duty Cycle for PB11 motor 2 right
	delay(30);
}
void turnAround(void)
{

	rightWheelForward();
	leftWheelReverse();
	delay(60);
	while(checkSensor(leftStabiliser))
	{

	}
	TIM2->CCR3 = 70 * 80; // Duty Cycle for PB10 motor 1 left
	TIM2->CCR4 = 70 * 80; // Duty Cycle for PB11 motor 2 right
	while(checkSensor(frontSensor))
	{

	}
	goStraight();
	TIM2->CCR3 = 100 * 80; // Duty Cycle for PB10 motor 1 left
	TIM2->CCR4 = 100 * 80; // Duty Cycle for PB11 motor 2 right
	delay(30);
}

void checkEnd(void)
{
	  if( !checkSensor(frontSensor)&&!checkSensor(leftSensor)&&!checkSensor(rightSensor)&&!checkSensor(leftStabiliser)&&!checkSensor(rightStabiliser))
	  {
		  leftWheelIdle();
		  rightWheelIdle();
		  while(checkSensor(button))
		  {
			  GPIOB -> ODR |= 0b100000;
			  delay(50);
			  GPIOB -> ODR &= 0b000000;
			  delay(50);
		  }
		  count = 0;
		  compass = 0;
		  GPIOB -> ODR = 0;
		  main();
	  }
}

void checkUturn(void)
{
	if(checkSensor(frontSensor)&&checkSensor(leftSensor)&&checkSensor(rightSensor)&&checkSensor(leftStabiliser)&&checkSensor(rightStabiliser))
	{
		if(count>40)
		{
			uTurn();
		}
		else
		{
			return;
		}
	}
	else
	{
		return;
	}
}

void TIM6_IRQHandler (void)
{
	count ++;
	TIM6 -> SR &= ~TIM_SR_UIF;// EXIT TIM14 INTERRUPT EVENT
}

void leftWheelForward(void)
{
	GPIOB -> ODR |= leftMotorInput1_HIGH;
	GPIOB -> ODR &= leftMotorInput2_LOW;
}

void rightWheelForward(void)
{
	GPIOB -> ODR |= rightMotorInput1_HIGH;
	GPIOB -> ODR &= rightMotorInput2_LOW;
}

void leftWheelReverse(void)
{
	GPIOB -> ODR &= leftMotorInput1_LOW;
	GPIOB -> ODR |= leftMotorInput2_HIGH;
}

void rightWheelReverse(void)
{
	GPIOB -> ODR &= rightMotorInput1_LOW;
	GPIOB -> ODR |= rightMotorInput2_HIGH;
}

void leftWheelIdle(void)
{
	GPIOB -> ODR &= leftMotorInput1_LOW;
	GPIOB -> ODR &= leftMotorInput2_LOW;
}

void rightWheelIdle(void)
{
	GPIOB -> ODR &= rightMotorInput1_LOW;
	GPIOB -> ODR &= rightMotorInput2_LOW;
}

void delay(int time) // pass in time in miliseconds
{
	int clock = 0;
	TIM14 -> CNT = 0;
	while(1)
	{
		clock += TIM14->CNT;
		if(clock > 1000000*time)
		{
			TIM14 -> CNT = 0;
			return;
		}
	}
}

int checkSensor(int bitMask)
{
	if((GPIOA->IDR&bitMask) == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}



////////////////////////////////////////////////////////////////////////
//////////////////////////////initialisations
///////////////////////////////////////////////////////////////////////
void init_GPIO(void){
	RCC-> AHBENR  |= RCC_AHBENR_GPIOBEN;
	GPIOB-> MODER |= 0b01010101000001010101010101010101;//setting pB 0-9 to output
	GPIOB-> ODR   |= 0;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // timer 2 for pwm
	GPIOB->MODER |= GPIO_MODER_MODER10_1; // PB10 = AF
	GPIOB->MODER |= GPIO_MODER_MODER11_1; // PB11 = AF
	GPIOB->AFR[1] |= (2 << (4*(10 - 8))); // PB10_AF = AF2 (ie: map to TIM2_CH3) // pwm channels
	GPIOB->AFR[1] |= (2 << (4*(11 - 8))); // PB11_AF = AF2 (ie: map to TIM2_CH4)

	RCC-> AHBENR  |= RCC_AHBENR_GPIOAEN;
	GPIOA-> MODER &= 0b11111111111111010000000000000000; // sets pA 0-8 as outputs
	GPIOA-> PUPDR |= 0b000101010101010101; //01 pull up ; 10 pull down
}

void init_EXTI (void)
{
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // ENABLE EXTI BUS CLK
	SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // MAP INTERRUPT TO PA0
	SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; // MAP INTERRUPT TO PA1
	SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA; // MAP INTERRUPT TO PA2
	EXTI -> IMR |= EXTI_IMR_MR0; // UNBLOCK INTERRUPT LINE 1 BUS
	EXTI -> IMR |= EXTI_IMR_MR1; // UNBLOCK INTERRUPT LINE 1 BUS
	EXTI -> IMR |= EXTI_IMR_MR2; // UNBLOCK INTERRUPT LINE 1 BUS
	EXTI -> FTSR |= EXTI_FTSR_TR0; // CONDITION CHECK: FALLING-EDGE
	EXTI -> FTSR |= EXTI_FTSR_TR1; // CONDITION CHECK: FALLING-EDGE
	EXTI -> FTSR |= EXTI_FTSR_TR2; // CONDITION CHECK: FALLING-EDGE


	//NVIC_EnableIRQ(EXTI0_1_IRQn); // ENABLE LINE 0 &amp; LINE 1 INTERRUPT
	//NVIC_EnableIRQ(EXTI2_3_IRQn); // ENABLE LINE 0 &amp; LINE 1 INTERRUPT
}

void init_PWM(void)
{
	TIM2->ARR = 7999; // PWM freqeuncy = 500 Hz
	// Set PWM mode: OCxM bits in CCMRx
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1
	TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1
	TIM2->CCR3 = 20 * 80; // Duty Cycle for PB10 = 20%
	TIM2->CCR4 = 90 * 80; // Duty Cycle for PB11 = 90%
	// Enable the OC channels
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCER |= TIM_CCER_CC4E;
	// Enable the Counter
	TIM2->CR1 |= TIM_CR1_CEN;
}

void init_TIM6 (void)
{
	RCC -> APB1ENR |= RCC_APB1ENR_TIM6EN; // ENABLE TIM14 BUS CLK
	TIM6 -> PSC = 599/60;
	TIM6 -> ARR = 7999; // OVERFLOW OCCURS EVERY APPROX 1 SECOND
	TIM6 -> DIER |= TIM_DIER_UIE; // ENABLE UPDATE INTERRUPT
	TIM6 -> CR1 |= TIM_CR1_ARPE; // AUTO RELOAD PRELOAD BUFFER ENABLE
	TIM6 -> CR1 |= TIM_CR1_CEN; // START THE TIM14 COUNTER
	NVIC_EnableIRQ(TIM6_DAC_IRQn); // ENABLE TIM14 BUS
}
void init_TIM14 (void)
{
	RCC -> APB1ENR |= RCC_APB1ENR_TIM14EN; // ENABLE TIM14 BUS CLK
	TIM14 -> PSC = 5999;
	TIM14 -> ARR = 7999; // OVERFLOW OCCURS EVERY APPROX 1 SECOND
	TIM14 -> DIER |= TIM_DIER_UIE; // ENABLE UPDATE INTERRUPT
	TIM14 -> CR1 |= TIM_CR1_ARPE; // AUTO RELOAD PRELOAD BUFFER ENABLE
	TIM14 -> CR1 |= TIM_CR1_CEN; // START THE TIM14 COUNTER
}
