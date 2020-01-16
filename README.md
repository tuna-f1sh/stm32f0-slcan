STM32 SLCAN
===========

Port of [stm32-slcan](https://github.com/GBert/misc/tree/master/stm32-slcan) to STM32F0 (Cortex M0). An alternative to `candleLight` firmware on Cantact compatiable schematic to use USART rather than USB.

Build
-----

```
git submodule init
git submodule update
cd libopencm3/ && make && cd ../
make
```

Flashing SWD
------------

Using SWD interface with `st-flash` or `gdb`

```
# SWDIO PA13, SWDCLK PA14 and NRST to Black Magic Probe
make flash
```

CAN-Interface
-------------
```
/* CAN1 / CAN GPIO */
#define GPIO_CAN_PB_RX                  GPIO8           /* PB8 */
#define GPIO_CAN_PB_TX                  GPIO9           /* PB9 */
```

Usage
-----

```
/* USART2 GPIO */
#define GPIO_USART2_CTS                 GPIO0           /* PA0 */
#define GPIO_USART2_RTS                 GPIO1           /* PA1 */
#define GPIO_USART2_TX                  GPIO2           /* PA2 */
#define GPIO_USART2_RX                  GPIO3           /* PA3 */
```

and do:
```
sudo modprobe can
sudo modprobe can-raw
sudo modprobe slcan
sudo slcand -s5 -S500000 /dev/ttyUSB0 can0 # CAN Speed 5 ->250 kBaud - 500,000 Baud serial (fix)
ifconfig can0 up
```

Now you can use it as SocketCAN interface - enjoy :-)

Links
-----

See 'ref/' folder for useful material.

* [Original STM32F1 Schematic](https://github.com/GBert/misc/raw/master/stm32-slcan/pictures/STM32F103C8T6-DEV-BOARD-SCH.pdf).
* [opencm3 lib CAN example with STM32F0](https://github.com/rschlaikjer/hello-stm32-3-canbus) and [blog post](https://rhye.org/post/stm32-with-opencm3-3-canbus/).
* [CAN Bit Time Calculator](http://www.bittiming.can-wiki.info/).
