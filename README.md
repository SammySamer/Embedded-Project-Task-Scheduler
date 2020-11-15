# Done by
 Samer Basta
 Abdelrahman Rashdan
 Nayera Farghaly

# Embedded-Project-Task-Scheduler
 The first project for CSCE course Embedded Systems, it involves utilizing the Renode system to simulate the F407VE STM32 microcontroller to program a Task Scheduling program.

# Code Run Through
 0) There are 2 Structs: 
  a) Task: that involves int delay, int priority and void (*fncName)(void) to store the required task details.
  b) Queue: that involves int currSize, int maxSize and an array of struct Task. 
  The queue is currently sized to 50 to put a limit to the queue, this is done in the Init() function.
 
 1) The Renode setup as highlighted in the tutorial must be fully done on the STM32407VETx. 
 Don't forget to change the .axf file's bin directory in the STM32F4_discovery.resc

 2) Simply, you queue in functions as tasks, making sure that the functions are in the form of...
 void FncName ()... being of type void, and accepting no parameters.

 3) You will have to write QueTask(FncName), the program will now queue in this function as a task to be run when 
 Dispatch() is called in the while loop.
 All you have to do is write in the task functions you want to queue, and to QueTask(FncName), everything else is handled.

 4) QueTask() will randomly generate a priority from 0-7, supporting 8 different priority levels. Based on this, it'll
 insert the task based on the highest priority first (descendingly) through the InsertRQueue() function. 
 If anything has the same priority as anything else,
 it would follow FIFO rule; the task inserted first will be higher in the priority queue. 
 The tasks are right-shifted if a new task needs to be put in the middle.

 5) Dispatch() will then simply look at the highest priority task, save the details of the task (to be used in the ReRunMe) into a
 variable called currTask, run the function (aka task), and remove it from the queue. The queue is left shifted to accomplish that.

 6) ReRunMe() has 2 options
  a) ReRunMe(0), if it's 0, that means the delay is 0, so it instantly puts this task that just ran back into the priority
  queue with another randomly generated priority. (At first it was the same priority, but it made sense to be a new one).
  b) ReRunMe(X), if it's any positive number, then it's inserted into the delayQueue with the same logic in step (4), but rather instead 
  of looking at the priority, it's looking at the delay as the sorting variable. Negative numbers will be ignored and terminate
  the ReRunMe function.

 7) To handle the delayQueue, we implemented a SysTick at the rate of 100ms/tick. It is supposed to be the SystemCoreClock
 divided by 10, but through numerous tests, we have found that dividing it by 3 yieled the closest results to the desired rate. 
 Every 100ms, it'll decrease the delay variable in each task, and once this delay variable hits 0, InsertRQueue() is called,
 so that this new task can sorted by priority to be scheduled for running.