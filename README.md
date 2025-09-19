# SY_Bird
 Ornithopter flight controller(continuous updates)

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
