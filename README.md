# CO2sensorFromScratch
What if a handheld sensor and computer could read CO2 levels?



CAPTAIN'S LOG -------------------------------------------------

NEW DATE :: 2022.12.01 PM ----------
  Summary: There was another, secret boss: removing the HAL from the header calls. It's done. The only headers that we load are stdint.h, cmsis_gcc.h, system_stm32f7xx.h. They probably call some other header files, but they likely don't call any HALs. Wow, now I feel like I suddenly have enough grasp to write drivers. Next up, USART control. Then after proving I've got something going, translating up to USART controls that would control the CO2 sensor. 

NEW DATE :: 2022.11.13 PM ----------
  Summary: It's messy, but the HAL has been rid from the function calls! Now, I'm super interested if I can just delete the HAL files and if everything will still work. Maybe I'll rename them temporarily so that I can return to use them when I eventually get UART up and running (soon soon). 

NEW DATE :: 2022.11.13 ----------
  Summary: Long time no README.md commits. BUT I've been putting in huge hours to strip HAL out. I've been battling a monster named HAL_InitTick(). To fight it, I had to solidify my register understanding, my debug with console abilities, my function creation and mimicking abilities to find that the real bosses were XXX and YYY and for some reason ZZZ. I finally replaced those with my own register calls and definitions. Next up is to truly strip all of the HAL code - some I added back into fight these final bosses (fewer variables = more control = more learning). Then clean clean clean. Then question why each remaining line is actually necessary. Soo so excited to learn, no *EARN*, that minimum viable program and microcontroller manipulations. 
  Aside: In case I need to reference my git flow it's: "git status" (for curiosity), then "git add ." or individual "git add xxx" calls, then "git status" (to check additions), then "git commit -m "[Message]"", then "git status" (for curiosity), then "git push -u origin", then "git status" (for curiosity / confirmation).

NEW DATE :: 2022.10.14 ----------
  Summary: Finished deconstructing HAL_GPIO_Init(). Wrote code to replace HAL with distilled register manipulation. Ran code, didn't work. I debugged until I found I forgot to type in a line I had written down.. (Gotta love __disable_irq() and __enable_irq() for helping me debug most things!) Now it works flawlessly! Oh I did decide to omit SystemClock_Config() calls for now even though it seems to be creating some dilation (2x my delay) and instability (eventually the toggling stops). Next up, replace HAL_Init() and then come back to SystemClock_Config() before moving onto U(s)ART with my CO2 sensor !

NEW DATE :: 2022.10.13 ----------
  Summary: In my quest to control registers more granularly by boiling down HAL code into bit manipulation, I worked deconstructed MX_GPIO_Init() into __HAL_RCC_GPIOI_CLK_ENABLE() and HAL_GPIO_Init(). I distilled down __HAL_RCC...() into register manipulation. I started deconstructing HAL_GPIO_Init() too. 

EW DATE :: 2022.10.11 ----------
  Summary: In my quest to control registers more granularly by boiling down HAL code into bit manipulation, I worked determine the microcontroller busses and HAL calls that were still needed. Results: MX_GPIO_Init() and HAL_Init().

NEW DATE :: 2022.10.09 ----------
  Summary: OH BOY! I took all of my HAL_Delay learnings and summarized all of the places where HAL_Delay() calls a HAL (sub)function. There are a lot. I devised a plan to copy this structure, but simplify and package it into my own code that doesn't use a single HAL. I had to debug around my implementation of the global variable that tracks time (and frequency). But IT's ALIVE! The code updated in this commit will toggle, then delay 300ms for the HAL_Delay() and then immediately delay 700ms for my JHM_Delay(). Sweet sweet feeling of bringing a vision to life. I can't wait to keep expanding that vision (into real life). // This HAL_Delay() replacement, which is no longer called now, is less like going from 1 to 2 and more like 1 to 3 or 1 to 10. Now the delay is just the 700 ms from my own implementation. 

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
