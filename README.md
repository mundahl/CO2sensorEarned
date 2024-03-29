# CO2sensorFromScratch
What if a handheld sensor and computer could read CO2 levels?



CAPTAIN'S LOG -------------------------------------------------

NEW DATE :: 2023.01.28 PM2 ----------
  Summary: Completed all LTDC bare metal setup steps via https://sudonull.com/post/15693-We-start-the-display-on-STM32-through-LTDC-on-registers. Code compiles and I can debug my way through the LTDC setup. Then somewhere, there's a hardfault that's triggered. 

NEW DATE :: 2023.01.28 PM1 ----------
  Summary: Problem = UART to (and from) the CO2 sensor is communicating at the wrong baud rate. Oscope shows it's much faster than the 9600 needed. I expected this because setting up the LTDC required changing the system clock to be much faster. How much faster? I scaled by UART speed setting until the speed roughly lined up on my oscope. I needed to slow down 6-7x. My UART code was setting its speed dynamically based on SystemCoreClock. But SystemCoreClock wasn't changing from before LTDC clock changes to after. I searched online and found https://electronics.stackexchange.com/questions/205875/setting-stm32f4-systemcoreclock-to-100-mhz-but-cant-get-it-only-get-57-6-mhz. Seemingly the operative similarity was calling SystemCoreClockUpdate();. So I did that before my UART code called SystemCoreClock. Bam it worked! So now the LTDC is partially setup and the CO2 sensor calls still work.

NEW DATE :: 2023.01.26 PM1 ----------
  Summary: LTDC code uncommented. Added setup of all LTDC pins to AF, high speed, and the correct AF mode. Untested additions.

NEW DATE :: 2023.01.22 PM1 ----------
  Summary: My Github authentication - a personal access token - expired on 2022.12.27, so when I returned to committing today, I had to renew that. In short, I tried to "git push -u origin" a commit, but got "Invalid username or password." and a suggestion to "Please see https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-https-urls ". So I did, generated a new personal access key, copied the key, checked all of the access boxes. Then I called "git push -u origin" and logged in with "mundahl" as username and the key as my password. Everything was good. To get a few more reps at this, I set the expiration to just 60 days, so I'll have to do it again soon. 
  The code that's been added and modified is to setup the LCD display from bare-metal. It's a doozy. This specific commit doesn't have the CO2 reading work anymore. Not exactly sure why. I might debug that first, then hop back into the LCD support. 

NEW DATE :: 2022.12.23 PM2 ----------
  Summary: I didn't have any ~guides to follow for the Rx capture. So I thought through my prior steps and didn't love that the receive enable bit for UART6 was turned 'on' right before looking for a received message and right after sending the transmit message. I moved the Rx Enable bit setting before the Tx messaging. BOM! It instantly started identifying Rx messages and reading them. New problem: the Rx while loop never ended. Fixed that with a byte count error (9 is now 8). We're working now! Next up, improve my dev tools to see what's received easier. And in parallel send new commands to actually read CO2 !

NEW DATE :: 2022.12.23 PM ----------
  Summary: Three weeks ago I tried to implement the faster baud rate as discussed in the previous log. I hit a few roadblocks that were surely surmountable. Normally, I'd forge ahead. This time, I more benefit would come from learning *up* one lever of abstraction: UART peripherals! Two week ago I started that. I pieced together various intel from the datasheet and guides online. This week, I ran code to transmit the signal. It didn't work. I debugged software in spots that seemed especially shaky. No dice. Then, I decide to dive deep deep, line by line through the code. I found errors during debug where a register was set to "1", when it could have been 6 << 7. Damn, I had written "6 < 7" instead, so TRUE = 1. Fixed that and oscope detected Tx signal! I setup a logic analyzer for the first time. It confirmed the signal matched my intended UART packer. I then coded in more packets. Logic analyzer confirmed them too. We were rolling!
  Onto receiving a signal back! I hooked up the SenseAir S8 to the STM32 dev board. But yeah... I scoped and logic-analyzed the sensor's Tx line. Nothing. I changed the UART message. No message back. I swapped Tx and Rx pins. Nothing. I wallowed in torment. Obviously nothing. I realized at one point the Vcc and ground pins were swapped. Yikes. Swapped them back, but still nothing. Maybe the sensor is damaged? I connected up a spare sensor. BOOM. Instantly received a message via oscope and logic analyzer! It only sends the message back once and it's an echo - same exact bytes received as sent. Also, the receive portion of the UART setup doesn't seem to recognize this message. Next up is solving that issue. 

NEW DATE :: 2022.12.04 PM ----------
  Summary: With the help of an oscope to record the output signal, USB drive to transfer that signal to my computer, and this python script for (bespoke..) analysis, I can now confirm the committed code actually sends the intended packets. Next up is correcting the baud to the correct value. Then architect for sending the message once then listening for the response. https://replit.com/@johnmundahl/AnotherWealthyForce#main.py

NEW DATE :: 2022.12.03 PM ----------
  Summary: Moving onto U(S)ART! Defined some SenseAir-S8-specific constants. Defined some STM32F7509-DK-JHM-setup-specific constants. Defined some SenseAir-S8-specific demo payloads. Architected a schema to communicate all of the packets in the demo payload to the same pin used for checking that the blinky works (we didn't actually blink an LED..). It seems to work on first blush with a placeholder delay of 1/2 second rather than using the baud rate. Next is to verify that the right comms are coming out of the packets. Then baud rate. 

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
