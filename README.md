# Space Vector Pulse Width Modulation w/ STM32

block diagram: https://www.cnx-software.com/wp-content/uploads/2014/09/STM32F7_Block_Diagram.png
docs: https://www.st.com/en/evaluation-tools/nucleo-f767zi.html#documentation

## Pinouts

|Name|Signal|
|-|-|

## Timers
|Timer| funciton|
|-|-|
|TIM1| 10 kHz PWM output|
|TIM2|precise ```f_out```|
| TIM4| 1.5 kHz Sampling interrupt generation|
|TIM14|Timebase source|


## Resources
https://hasanyavuz.ozderya.net/?p=437


## Checklist

- [ ] pwm output: 10kHz
- [ ] deadtime: 500ns
- [ ] update frequency: 1.5-2 kHz
- [ ] ```Vm``` changes output avg voltage
- [ ] ```f_out``` changes output freq. (not pwm freq.!)
- [ ] ```f_out```: 0-100Hz

## ToDo list

## Mérések
**M001**: _pwm 10kHz ellenőrzése_
**M002**: kitöltési tényező (U: 10%, V: 50%)
**M003**: deadtime 500ns