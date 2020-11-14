#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>


static uint8_t msg[] = "Renode Alive !!\n";
static uint8_t pressedMsg[] = "Button is pressed !!\n";
static uint8_t releasedMsg[] = "Button is released !!\n";

static uint8_t msgA[] = "Task A has been initialized.\n";
static uint8_t msgB[] = "Task B has been initialized.\n";
static uint8_t msgC[] = "Task C has been initialized.\n";

static uint8_t msg_done[] = "This ask has finished running: ";

static uint8_t msgA_done[] = "Task A has finished running\n";
static uint8_t msgB_done[] = "Task B has finished running\n";
static uint8_t msgC_done[] = "Task C has finished running\n";


struct Queue {
	int currSize; 
	int maxSize;
};

struct Task {
	char TaskName;
	int prior;
	int delay;
};

struct Queue* readyQueue;
struct Queue* delayQueue;

static char buttonPressed = 1;
static char timerFlag = 0;
static volatile uint8_t stopFlag = 0;


void SysTick_Handler(void);
void USART2_IRQHandler(void);
void EXTI0_IRQHandler(void);
static void sendUART(uint8_t * data, uint32_t length);
static uint8_t receiveUART(void);

//Tasks
void TaskA(void);
void TaskB(void);
void TaskC(void);

//TaskScheduler functions
void Init(void);
void QueTask(void *task);
void Dispatch(void);
void ReRunMe(int delay);

//queue functions
void insert(struct Queue*);



void SysTick_Handler(void)  {
	timerFlag = 1;
}

void USART2_IRQHandler(void) {
	/* pause/resume UART messages */
	stopFlag = !stopFlag;
	
	/* dummy read */
	(void)receiveUART();
}

void EXTI0_IRQHandler(void) {
		/* Clear interrupt request */
		EXTI->PR |= 0x01;
		/* send msg indicating button state */
		if(buttonPressed)
		{
				sendUART(pressedMsg, sizeof(pressedMsg));
				buttonPressed = 0;
		}
		else
		{
				sendUART(releasedMsg, sizeof(releasedMsg));
				buttonPressed = 1;
		}
}

static void sendUART(uint8_t * data, uint32_t length)
{
	 for (uint32_t i=0; i<length; ++i){
      // add new data without messing up DR register
      uint32_t value = (USART2->DR & 0x00) | data[i];
		  // send data
			USART2->DR = value;
      // busy wait for transmit complete
      while(!(USART2->SR & (1 << 6)));
		  // delay
      for(uint32_t j=0; j<1000; ++j);
      }
}

static uint8_t receiveUART()
{
	  // extract data
	  uint8_t data = USART2->DR & 0xFF;
	
	  return data;
}

static void gpioInit()
{	
    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);

    // set pin modes as alternate mode 7 (pins 2 and 3)
    // USART2 TX and RX pins are PA2 and PA3 respectively
    GPIOA->MODER &= ~(0xFU << 4); // Reset bits 4:5 for PA2 and 6:7 for PA3
    GPIOA->MODER |=  (0xAU << 4); // Set   bits 4:5 for PA2 and 6:7 for PA3 to alternate mode (10)

    // set pin modes as high speed
    GPIOA->OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)

    // choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[0] |= (0x7 << 8); // for pin A2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin A3
}

static void uartInit()
{
	
    // enable USART2 clock, bit 17 on APB1ENR
    RCC->APB1ENR |= (1 << 17);
	
	  // USART2 TX enable, TE bit 3
    USART2->CR1 |= (1 << 3);

    // USART2 rx enable, RE bit 2
    USART2->CR1 |= (1 << 2);
	
	  // USART2 rx interrupt, RXNEIE bit 5
    USART2->CR1 |= (1 << 5);

    // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
    //   for fCK = 16 Mhz, baud = 115200, OVER8 = 0
    //   USARTDIV = 16Mhz / 115200 / 16 = 8.6805
    // Fraction : 16*0.6805 = 11 (multiply fraction with 16)
    // Mantissa : 8
    // 12-bit mantissa and 4-bit fraction
    USART2->BRR |= (8 << 4);
    USART2->BRR |= 11;

    // enable usart2 - UE, bit 13
    USART2->CR1 |= (1 << 13);
}

void swap(int *a, int *b) {
	int temp = *b;
	*b = *a;
	*a = temp;
}

void TaskA() {
	srand(time(0));
	int priority = rand() % 7;
	
	sendUART(msgA, sizeof(msgA));
	sendUART(msgA_done, sizeof(msgA_done));
	ReRunMe(0);
}

void TaskB() {
	srand(time(0));
	int priority = rand() % 7;
	
	sendUART(msgB, sizeof(msgB));
	sendUART(msgB_done, sizeof(msgB_done));
}

void TaskC() {
	srand(time(0));
	int priority = rand() % 7;
	
	sendUART(msgC, sizeof(msgC));
	sendUART(msgC_done, sizeof(msgC_done));
}

void Init(){
	//init the 2 queues
	struct Queue* readyQueue = (struct Queue*)malloc(sizeof(struct Queue*));
	struct Queue* delayQueue = (struct Queue*)malloc(sizeof(struct Queue*));
}

void insert(struct Queue* Q) {
	
	//if the queue is full
	if (Q->currSize == Q->maxSize)
		return;
	
  for (int i = 0; i <= Q->currSize; i++) {

        if (data >= pri_que[i])

        {

            for (int j = rear + 1; j > i; j--)

            {

                pri_que[j] = pri_que[j - 1];

            }

            pri_que[i] = data;

            return;

        }

    }

    pri_que[i] = data;

}
}

int main()
{	
	  /* startup code initialization */
	  SystemInit();
	  SystemCoreClockUpdate();
	  /* intialize UART */
	  gpioInit();
		/* intialize UART */
	  uartInit();
	  /* enable SysTick timer to interrupt system every second */
	  SysTick_Config(SystemCoreClock);
	  /* enable interrupt controller for USART2 external interrupt */
		NVIC_EnableIRQ(USART2_IRQn);
		/* Unmask External interrupt 0 */
		EXTI->IMR |= 0x0001;
	  /* Enable rising and falling edge triggering for External interrupt 0 */
		EXTI->RTSR |= 0x0001;
		EXTI->FTSR |= 0x0001;
	  /* enable interrupt controller for External interrupt 0 */
		NVIC_EnableIRQ(EXTI0_IRQn);
	
	  while(1)
		{
				if(timerFlag && !stopFlag)
				{
						sendUART(msg, sizeof(msg));
					  timerFlag = 0;
				}
		}
}
