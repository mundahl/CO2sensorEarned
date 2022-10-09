# CO2sensorFromScratch
What if a handheld sensor and computer could read CO2 levels?



CAPTAIN'S LOG -------------------------------------------------

NEW DATE :: 2022.10.08 ----------
  Summary: I did 0 to 1 for replacing the HAL toggle. Now 1 to 2 for replacing HAL Delay. It's using the timers and interrupts and plenty of initialization, so lots of learning to do. // I'm understanding vector tables, NVIC operations, internal oscillators, external oscillators, phase locked loop (!?), prescalers, some Reset and Clock Control (RCC) register purposes, basic timers, and their implementation in C code / HAL files and STM32CubeIDE GUI. Truthfully, there was an explosion of operations within PLLs, so I stopped tracing every operation there. We don't even need to use PLLs since internal or external crystal (?) oscillators meet our needs. 

NEW DATE :: 2022.10.06 ----------
  Summary: Two of the past 6 days, I continued the charge to drop HAL for the toggle. I debugged my way to understanding exactly what registers where modified, why and how. Then I wrote my own implementation with bit banging. HAL for the toggle is gone and toggle still works! 

NEW DATE :: 2022.09.29 ----------
  Summary: We're in github. Now to download, open in another folder and see if it functions. // So it didn't function. We blew up that repo and started from scratch - without the embedded git problem. I started a repo on github > cloned it to a blank folder > opened that folder as a "workspace" in STM32cubeIDE > generated a new default project for the STM32F7508-DK > added the HAL lines in the while loop > built it > found it didn't function > replaced the main.c with a known-good one that toggles up and down > built it > worked flawlessly > git add and commit.
  Summary: How about that debug not working? Recall the toggling wouldn't work in "Run as Debug". I ran in debug and stepped through the code. Each time I stepped from main.c, function TIM6_DAC_IRQHandler was triggered. Ahh an interrupt! Google results: my stepping is ~too slow~ and the timer (TIM) possibly for the digital-analog-converter (DAC) ? triggers an interrupt reQuest (IRQ) and that function simply HANDLES the result. How do I turn off that interrupt temporarily? Ahh yes, if I surround the text with __disable_irq() and then __enable_irq() then the Interrupt ReQuests will be off. Interestingly I need the IRQ to execute HAL delay, which makes sense because an interrupt ticks the timer up once every N clock ticks (usually 1 millisecond or so). Next time, I can explore the registers of this toggle more and see if I can simplify. Err maybe there's something higher priority.

NEW DATE :: 2022.09.28 ----------
  Summary: Stymied by 1Password SSH and (as expected) spent more time on other projects. Now back with bit bash support via https. // Can confirm this eventually worked. Turns out Github, unlike bitbucket, does not allow https + passwords. I setup a Personal Auth Token instead. 

NEW DATE :: 2022.09.11 ----------
  Written Notes: https://imgur.com/a/jfEczwi
  Summary: On our STM32F7508-DK, we can use STM32CubeIDE to toggle the GPIO Pin I2 on and off and verify the voltage change with my Siglent SDS 1201X-E oscilloscope. However, we have to perform the run in DEBUG (not sure why?), we're using HAL (instead of registers or something with more control), we aren't using the USART pins or protocol we'll need, and we haven't been using VERSION CONTROL. Next up: VERSION CONTROL, then maybe DEBUG, then HAL, then USART, then more Oscilloscope control.
