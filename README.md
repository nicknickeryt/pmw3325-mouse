# pmw3325-mouse

I've had my Mionix AVIOR Shark Fin mouse for quite a long time and apparently it's based on STM32F042G6U6 and PMW3325DB-TWV1.\
As I love STM32 MCUs, I decided to write a firmware for it from scratch - just for fun and learning purposes.\
It wasn't that hard, but I had to reverse engineer the "Power‐up initialization register setting" thing that is required to talk with most of the PixArt's optical sensors.\
This is proprietary and without it you can't read sensor data via SPI - it just won't respond. Some of these "chip passwords" (as I call it) can be found on the Internet, but not the PMW3325 one, so that's why I share this project here.\
You can find the power up sequence in sensor.c - void powerUpSensor(). Hopefully that helps somebody!\
\
The code is currently really bare-bones and hacky, but all the basic mouse things are working. \
I'll probably add more features to it shortly, such as CPI control, RGB LEDs (the PCB has a solder pads for it!), and a firmware-based macro/double click.
