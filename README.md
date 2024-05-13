# LoR_Core_BluePad32
 BluePad32 is for controlling MiniBot LoR_Core modules with a variety of bluetooth devices like game pad controllers.

 Our User Guide / documentation for getting your MiniBot running with BluePad32
  - https://tr.ee/t8cqMRw3Hu

 
*Dependencies*

- Based on the BluePad32 Library _ https://bluepad32.readthedocs.io/en/stable/

- LoR Library - please install the LoR library - https://github.com/LordofRobots/LoR

- Adafuit Neopixel library - install from the arduino library manager



*Known Issues*

- TDSB network admin (on both wifi and lan) seem to block access to some third party libraries that Arduino IDE uses. This will cause issues when installing and updating your libraries. If you are having trouble installing or updating, it is best to connect to the internet on a private network to install and update these items.

- Disconnecting gamepads:

  **MiniBot** = connect to PC and use Serial Monitor to send "obliviate" to the MiniBot. It should respond with "mischief managed".

  **xbox** = hold "pair" button until the controller vibrates twice and turns off (about 10 sec)

  **PS4** = On the back of your controller, you will find a tiny hole – the reset button is inside this hole. Use a small, unfolded paper clip (or something similar) to press and hold the reset button for at least five seconds. Your controller should now have been reset.

  **PS5** = Find the reset button inside the small hole next to the Sony® logo on the back of your controller. Use a pin or a similar tool (not included) to press and hold the reset button for at least 5 seconds. Your controller resets.

