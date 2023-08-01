.. _lp_mspm0g3507:

MSPM0G3507 LaunchPad
########################

Overview
********

MSPM0G350x microcontrollers (MCUs) are part of the MSP highly integrated, ultra-low-power 32-bit MCU 
family based on the enhanced Arm® Cortex®-M0+ 32-bit core platform operating at up to 80-MHz frequency. 
These cost-optimized MCUs offer high-performance analog peripheral integration, support extended temperature 
ranges from -40°C to 125°C, and operate with supply voltages ranging from 1.62 V to 3.6 V.
The MSPM0G350x devices provide up to 128KB embedded flash program memory with built-in error correction 
code (ECC) and up to 32KB SRAM with ECC and hardware parity option. These MCUs also incorporate a 
memory protection unit, 7-channel DMA, math accelerator, and a variety of high-performance analog peripherals 
such as two 12-bit 4-Msps ADCs, configurable internal shared voltage reference, one 12-bit 1-Msps DAC, three 
high speed comparators with built-in reference DACs, two zero-drift zero-crossover op-amps with programmable 
gain, and one general-purpose amplifier. These devices also offer intelligent digital peripherals such as two 
16-bit advanced control timers, five general-purpose timers (with one 16-bit general-purpose timer for QEI 
interface, two 16-bit general-purpose timers for STANDBY mode, and one 32-bit general-purpose timer), two 
windowed-watchdog timers, and one RTC with alarm and calendar modes. These devices provide data integrity 
and encryption peripherals (AES, CRC, TRNG) and enhanced communication interfaces (four UART, two I2C, 
two SPI, CAN 2.0/FD).

.. figure:: img/lp_mspm0g3507.png
     :align: center
     :alt: MSPM0G3507 LaunchPad development board

Features:
=========

- Onboard XDS110 debug probe
- EnergyTrace technology available for ultra-low-power debugging
- 2 buttons, 1 LED and 1 RGB LED for user interaction
- Temperature sensor circuit
- Light sensor circuit
- External OPA2365 (default buffer mode) for ADC (up to 4 Msps) evaluation
- Onboard 32.768-kHz and 48-MHz crystals
- RC filter for ADC input (unpopulated by default)

Details on the MSPM0G3507 LaunchPad can be found on the MSPM0G3507 LaunchPad `User Guide`_.

Supported Features
==================

The MSPM0G3507 LaunchPad development board configuration supports the following hardware features:

+-----------+------------+-----------------------+
| Interface | Controller | Driver/Component      |
+===========+============+=======================+
| NVIC      | on-chip    | nested vectored       |
|           |            | interrupt controller  |
+-----------+------------+-----------------------+
| SYSTICK   | on-chip    | system clock          |
+-----------+------------+-----------------------+
| UART      | on-chip    | serial                |
+-----------+------------+-----------------------+
| GPIO      | on-chip    | gpio                  |
+-----------+------------+-----------------------+

More details about the supported peripherals are available in MSPM0G350X TRM.
Other hardware features are not currently supported by the Zephyr kernel.

Building and Flashing
*********************

Building
========

Follow the :ref:`getting_started` instructions for Zephyr application development.

For example, to build the :ref:`hello_world` application for the MSPM0G3507 LaunchPad:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: lp_mspm0g3507
   :goals: build

The resulting ``zephyr.bin`` binary in the build directory can be flashed onto
MSPM0G3507 LaunchPad using the program mentioned below.

Flashing
========

`UniFlash`_ is used to program the flash memory. Only bin loading is currently supported.
Make sure to select the checkbox for binary loading.

Debugging
=========

MSPM0G3507 LaunchPad board supports debugging primarily using `CCS IDE`_. More information
on debugging using CCS can be found in CCS Debug Handbook.

In general, the steps for debugging in CCS is to:

   1. Open CCS
   2. Launch target configuration using the provided .ccxml file in the lp_mspm0g3507/ directory
   3. Plug in the device and connect to it
   4. Go to Run > Load > Load Symbols and load in the zephyr.elf file loaded
   5. Use CCS to debug 

References
**********

TI MSPM0 MCU Page:
   https://www.ti.com/microcontrollers-mcus-processors/arm-based-microcontrollers/arm-cortex-m0-mcus/overview.html

TI MSPM0G3507 Product Page:
   https://www.ti.com/product/MSPM0G3507

TI MSPM0 SDK:
   https://www.ti.com/tool/MSPM0-SDK

.. _User Guide:
   https://www.ti.com/lit/ug/slau873a/slau873a.pdf?ts=1686687008417

.. _UniFlash:
   http://processors.wiki.ti.com/index.php/UniFlash_v4_Quick_Guide#Command_Line_Interface

.. _CCS IDE:
   http://www.ti.com/tool/ccstudio
