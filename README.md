# SY_Bird
 Ornithopter flight controller(continuous updates)

！！！The current version still have problems and is for learning and testing only; it cannot be used directly in a production environment.

email:brawnchome@gmail.com(1842964100@qq.com)

Applicable model: one motor controls the flapping wings, and the left and right servos control the V tail.

Use stm32f103c8t6

pin configs:

LED:PB0

PPM:PA6

Left servo:PA1

Right servo:PA3

ESC:PA2

MPU6050:
    SCL:PB10
    SDA:PB11

USART:PA9

Current issue: The MPU6050's response to attitude angle detection is not fast enough, the selection of kp, ki, kd values in the control code is not detailed enough, and it cannot automatically detect orientation after power on, requiring reprogramming.

if you have any advice, please send me email,thank you very much!
