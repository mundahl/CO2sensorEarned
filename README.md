# CO2sensorFromScratch
What if a handheld sensor and computer could read CO2 levels?



CAPTAIN'S LOG -------------------------------------------------

NEW DATE :: 2022.09.29 ----------
  Summary: We're in github. Now to download, open in another folder and see if it functions.

NEW DATE :: 2022.09.28 ----------
  Summary: Stymied by 1Password SSH and (as expected) spent more time on other projects. Now back with bit bash support via https. // Can confirm this eventually worked. Turns out Github, unlike bitbucket, does not allow https + passwords. I setup a Personal Auth Token instead. 

NEW DATE :: 2022.09.11 ----------
  Written Notes: https://imgur.com/a/jfEczwi
  Summary: On our STM32F7508-DK, we can use STM32CubeIDE to toggle the GPIO Pin I2 on and off and verify the voltage change with my Siglent SDS 1201X-E oscilloscope. However, we have to perform the run in DEBUG (not sure why?), we're using HAL (instead of registers or something with more control), we aren't using the USART pins or protocol we'll need, and we haven't been using VERSION CONTROL. Next up: VERSION CONTROL, then maybe DEBUG, then HAL, then USART, then more Oscilloscope control.
