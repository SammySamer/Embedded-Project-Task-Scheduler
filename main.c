#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

#define MAX_SIZE 49


static uint8_t pressedMsg[] = "Button is pressed !!\n";
static uint8_t releasedMsg[] = "Button is released !!\n";

static uint8_t msgA[] = "Task A has been initialized.\n";
static uint8_t msgB[] = "Task B has been initialized.\n";
static uint8_t msgC[] = "Task C has been initialized.\n";

static uint8_t msgA_done[] = "Task A has finished running\n";
static uint8_t msgB_done[] = "Task B has finished running\n";
static uint8_t msgC_done[] = "Task C has finished running\n";


struct Task {
	int prior;
	int delay;
	void (*fncName)(void);
};

struct Queue {
	int currSize; 
	int maxSize;
	struct Task* task[50];
};

static 


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
void QueTask(void (*task)(void));
void Dispatch(void);
void ReRunMe(int delay);

//queue functions
void insert(struct Queue*, struct Task*);

static struct Queue* readyQueue;
static struct Queue* delayQueue;

void Init(){
	
	//init the 2 queues
	readyQueue = (struct Queue*)malloc(sizeof(struct Queue*));
	readyQueue->currSize = 0;
	readyQueue->maxSize = MAX_SIZE;
	
	delayQueue = (struct Queue*)malloc(sizeof(struct Queue*));
	delayQueue->currSize = 0;
	delayQueue->maxSize = MAX_SIZE;
	
	
}

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


void TaskA() {
	
	sendUART(msgA, sizeof(msgA));
	sendUART(msgA_done, sizeof(msgA_done));
	//ReRunMe(0);
}

void TaskB() {
	
	sendUART(msgB, sizeof(msgB));
	sendUART(msgB_done, sizeof(msgB_done));
}

void TaskC() {
	
	sendUART(msgC, sizeof(msgC));
	sendUART(msgC_done, sizeof(msgC_done));
}


void insert(struct Queue* Q, struct Task* T) {
	
	//if the queue is full, do nothing
	if (Q->currSize == Q->maxSize)
		return;
	
  for (int i = 0; i <= Q->currSize; i++) {

		//if the current tasks's priority is more than what we're currently
		//pointing to, we want to place it in that position
		if (T->prior >= Q->task[i]->prior) {
			
			//start shifting everything to the right to make space
				for (int j = Q->currSize + 1; j > i; j--)
						Q->task[j] = Q->task[j - 1];

				Q->task[i] = T;
				Q->currSize = Q->currSize + 1;
			
				return;
		}
	}

	//if the priority is smaller than everything else, place it at the end.
	Q->currSize = Q->currSize + 1;
	Q->task[Q->currSize] = T;

}

void QueTask(void (*task)(void)) {
	
	srand(time(0));
	int priority = rand() % 7;
	
	struct Task* newTask = (struct Task*)malloc(sizeof(struct Task*));
	
	newTask->delay = 0;
	newTask->prior = priority;
	newTask->fncName = task;
	
	insert(readyQueue, newTask);
	
}

void Dispatch() {
	
	//if it's not empty
	if (readyQueue->currSize != 0) {
		void (*runTask)(void) = readyQueue->task[0]->fncName;
		(*runTask)();
		readyQueue->currSize = readyQueue->currSize - 1;
	}
}

int main()
{	
	Init();
	
	
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


	QueTask(TaskA);
	QueTask(TaskB);
	QueTask(TaskC);
	QueTask(TaskB);
	
	while(1)
	{
			if(timerFlag && !stopFlag)
			{
				Dispatch();
				timerFlag = 0;
			}
	}
}
