REF: CAL000                                          HERCROM400G2                  Ver 1.3




                                      HERCROM400G2

                                              Specification

                                                 CAL000
                                                    Ver 1.3




                                              CALYPSO




Department: European Wireless Terminal Chipset Business Unit

                                                           Originator
Name                                                       Michel Gac
Date                                                  25 - February- 2000




                                               TI- Proprietary Information –   PAGE: 1/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.          Strictly Private
Features characteristic data and other
information are subject to change.
                                               UNDER NON DISCLOSURE            DO NOT COPY
                                                    AGREEMENT
REF: CAL000                                         HERCROM400G2                        Ver 1.3

                                                 HISTORY


 Version                    Date                Author        Approval         Approval       Notes
                                                              manager            date
     0.1             25 – Feb - 2000           Michel Gac                                      1,2
     0.2             23 - May - 2000           Michel Gac                                       3
     0.3             26 - June - 2000          Michel Gac                                       4
     0.4             12 – July – 2000          Michel Gac     A.Boyadjian     16Aug-2000       5-6
                                                              L.Tannyeres
     0.5            10 – Augt- 2000           Michel Gac      L.Tannyeres      11Aug-2000       7
     0.6            15 – Sept – 2000          Michel Gac      L.Tannyeres       19Oct-2000      8
     0.7            26 – Jan – 2001           Rodolphe S.      Michel Gac       26jan-2001      9
     0.8            22 – Mar – 2001           Rodolphe S.      Michel Gac      22mar-2001      10
     0.9             3 – April – 2001         Rodolphe S.      Michel Gac       10Apr-2001     11
    0.91            26 – April – 2001         Rodolphe S.      Eric Balard      26Apr-2001     12
    0.92            14 – May – 2001           Rodolphe S.      Eric Balard     14May-2001      13
    0.93            20 – Augt – 2001          Rodolphe S.      Michel Gac     21-Augt-2001     14
     1.0            12 – Oct – 2001            R. Servato      Michel Gac      15-Oct-2001     15
     1.1            15 – Oct – 2001            R. Servato      Michel Gac      14-Nov-2001     16
     1.2              28- Nov -2001            R. Servato      Michel Gac     28- Nov -2001    17
     1.3            18 – June -2002            R. Servato      Michel Gac     20–June-2002     18




Notes:
1. Creation of document.
2. Version derived from specification SAMS000 v1.7
3. Final ball-out assignment.
4. Change MPU adress.
5. Updated §4.5.14, §4.5.22, §4.5.23, §4.5.24, §4.9
6. Updated connection figures
7. Fix bug in MPU paragraph.
8. Fix bug for KBC, DATA, TSPEN reset values.
9. Updated section 11.3 : removed warning concerning GNA2 pin connexion.
10. Updated Table 6 INT4n, Update §11.3 (Ground name), remove §reference document,
    added restrication with NAUSICA usage
11. Update table 3 : Compatibility connection with Nausica, update Table 6 (remove pull up on
    nIBOOT)
12. Remove all wrong references document.
13. Add §10-Table8 : PAD I/O Buffer type, update §3.5.21 MPU (minimum granularity of 8
    bytes), update §3.5.22 DU, update §3.5.24 WRB.
14. Remove Pin EN_LMM_PWR in Ch 8 table Power management (wrong reference), update
    FIG 4 (1.8V instead of 1.4V)
15. update IOTA to power split DBB supply connections (ch 11.2.1.1)
16. add features for C035 : pin 140 nCS4 : update Table 7&8, Update Ch 7.1(Memory interface
    using ADD(22), using CS4)
17. Improve lisibility for modifications done in previous version (Ch 8, 9, 10)



                                              TI- Proprietary Information –       PAGE: 2/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.          Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE                DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                  Ver 1.3

18. remove blank chapter 11.1 (Example of mobile terminal appication), update IOTA to power
    split DBB supply connections figure (pin VLRTC=1.5v)




                                              TI- Proprietary Information –   PAGE: 3/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                             HERCROM400G2                         Ver 1.3

                                              IMPORTANT NOTICE

Texas Instruments (TI) reserves the right to make changes to its products or to
discontinue any semiconductor product or service without notice, and advises its
customers to obtain the latest version of relevant information to verify, before placing
orders, that the information being relied on is current.

TI warrants performance of its semiconductor products and related software to the
specifications applicable at the time of sale in accordance with TI’s standard warranty. Testing
and other quality control techniques are utilized to the extent TI seems necessary to support this
warranty. Specific testing of all parameters of each device is not necessarily performed, except
those mandated by government requirements.

Certain applications using semiconductor products may involve potential risks of death,
personal injury, or severe property or environmental damage (“Critical Applications”).

TI SEMICONDUCTOR PRODUCTS ARE NOT DESIGNED, INTENDED, AUTHORIZED, OR
WARRANTED TO BE SUITABLE FOR USE IN LIFE-SUPPORT APPLICATIONS, DEVICES
OR SYSTEMS OR OTHER CRITICAL APPLICATIONS.

Inclusion of TI-products in such applications is understood to be fully at the risk of the customer.
Use of TI products in such applications requires the written approval of an appropriate TI officer.
Questions concerning potential risk applications should be directed to TI through a local SC
sales office.

In order to minimize risks associated with the customer’s applications, adequate design and
operating safeguards should be provided by customer to minimize inherent or procedural
hazards.

TI assumes no liability for applications assistance, customer product design, software
performance, or infringement of patents or services described herein. Not does TI warrant or
represent that any license, either express or implied, is granted under any patent right,
copyright, mask work right, or other intellectual property right of TI covering or relating to any
combination, machine, or process in which such semiconductor products or services might be
or are used.




                                     Copyright © 1998, Texas Instruments Incorporated.




                                              TI- Proprietary Information –              PAGE: 4/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.              Strictly Private
Features characteristic data and other
information are subject to change.
                                               UNDER NON DISCLOSURE                      DO NOT COPY
                                                    AGREEMENT
REF: CAL000                                                            HERCROM400G2                                                     Ver 1.3

                                                                SUMMARY

1.      GLOSSARY....................................................................................................................................... 8

2.      GENERAL DESCRIPTION............................................................................................................. 9

3.      BLOCK SPECIFICATION.............................................................................................................. 9
     3.1     INTRODUCTION.................................................................................................................... 9
     3.2     ARM MEGACELL (ARM7TDMIE) ......................................................................................... 10
     3.3     DSP SUBCHIP (S28C128)........................................................................................................ 11
     3.4     CLOCK SQUARER CELL ............................................................................................................ 11
     3.5     ARM PERIPHERALS ................................................................................................................. 11
        3.5.1    Memory Interface. ............................................................................................................. 11
        3.5.2    Internal Static RAM........................................................................................................... 11
        3.5.3    Internal Boot Memory ....................................................................................................... 12
        3.5.4    Die ID cell ......................................................................................................................... 12
        3.5.5    Interrupt Handler (INTH). ................................................................................................ 12
        3.5.6    General purposes I/O (ARMI/O). ...................................................................................... 12
        3.5.7    Micro Wire interfaces (UWIRE)........................................................................................ 12
        3.5.8    Timers (TIMER). ............................................................................................................... 13
        3.5.9    IrDA Universal Async Receiver/transmitter 16C750 (UART-IRDA). ............................... 13
        3.5.10 Universal Async Receiver/transmitter 16C750 (UART-MODEM).................................... 14
        3.5.11 Subscriber Identity Module Interface (SIM)...................................................................... 14
        3.5.12 Serial Port Interface (SPI) ................................................................................................ 14
        3.5.13 Time Processing Unit (TPU)............................................................................................. 14
        3.5.14 Time Serial Port (TSP)...................................................................................................... 15
        3.5.15 Direct Memory Access controller (DMA).......................................................................... 15
        3.5.16 Clock Management (CLKM) ............................................................................................. 15
        3.5.17 Pulse Width Tones (PWT) ................................................................................................. 15
        3.5.18 Pulse Width Light (PWL)................................................................................................... 16
        3.5.19 Light Pulse Generator (LPG)............................................................................................ 16
        3.5.20 I2C master serial interface (I2C) ...................................................................................... 16
        3.5.21 Memory Protection Unit (MPU) ....................................................................................... 16
        3.5.22 Debug Unit (DU)............................................................................................................... 17
        3.5.23 GPRS Encryption Algorithm (GEA1-2) ............................................................................ 17
        3.5.24 Internal RAM write buffer (WRB) ..................................................................................... 17
        3.5.25 Real Time Clock (RTC) ..................................................................................................... 18
        3.5.26 Ultra Low-Power Down controller (ULPD) ..................................................................... 18
     3.6     SPECIFIC POWER-SPLIT FOR RTC............................................................................................. 18
     3.7     DSP PERIPHERALS .................................................................................................................. 20
        3.7.1    Radio interface (RIF) ........................................................................................................ 20
        3.7.2    Multi-Channel Serial Interface (MCSI)............................................................................. 20
        3.7.3    Ciphering processor (CRYPT) .......................................................................................... 20
        3.7.4    Universal Asynchronous Receiver/transmitter (16C750).................................................. 20
        3.7.5    Direct Memory Access controller (DMA)......................................................................... 20
        3.7.6    Interrupt Handler (INTH). ................................................................................................ 21
     3.8     GENERAL PURPOSE PERIPHERALS ............................................................................................ 21
        3.8.1    JTAG ................................................................................................................................. 21
        3.8.2    IDDQ................................................................................................................................. 21
     3.9     BLOCK DIAGRAM ................................................................................................................... 22
4.      INITIALIZATION PROTOCOL .................................................................................................. 22
     4.1       HARDWARE LOGIC RESET ........................................................................................................ 22
     4.2       ARM CODE DOWNLOADING .................................................................................................... 23

                                                          TI- Proprietary Information –                                       PAGE: 5/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.                             Strictly Private
Features characteristic data and other
information are subject to change.
                                                           UNDER NON DISCLOSURE                                              DO NOT COPY
                                                                AGREEMENT
REF: CAL000                                                             HERCROM400G2                                                      Ver 1.3

     4.3       DSP CODE EXECUTION ............................................................................................................ 23
5.      INTERRUPTS MANAGEMENT .................................................................................................. 24
     5.1       DSP INTERRUPTS ............................................................................................................... 24
     5.2       MCU INTERRUPTS ............................................................................................................. 25
6.      DMA MAPPING ............................................................................................................................. 26

7.      ADDRESS MAPPING .................................................................................................................... 27
     7.1     ARM MEMORY SPACE ...................................................................................................... 27
        7.1.1    Memory interface: using CS4 (default) ............................................................................. 27
        7.1.2    Memory interface: using ADD(22).................................................................................... 27
        7.1.3    External Flash/ROM image .............................................................................................. 28
     7.2     DSP MEMORY SPACE ........................................................................................................ 31
        7.2.1    API .................................................................................................................................... 31
        7.2.2    XIO-RHEA ........................................................................................................................ 32
8.      CALYPSO PINS DESCRIPTION ................................................................................................. 33

9.      CALYPSO BALL-OUT.................................................................................................................. 37
     9.1       179GHH PACKAGE ............................................................................................................. 37
10.         PAD I/O BUFFER TYPE ........................................................................................................... 42

11.         APPLICATION EXAMPLE ...................................................................................................... 46
     11.1 POWER-SUPPLIES CONNECTION ............................................................................................... 46
       11.1.1 Compatibility schematic Nausica CALYPSO .................................................................... 46
       11.1.2 IOTA to power split DBB supply connections................................................................... 47
       11.1.3 Power Supplies connection with IOTA ABB ..................................................................... 48
     11.2 32KHZ QUARTZ CONNECTION ................................................................................................. 49
     11.3 SIM CARD CONNECTION ......................................................................................................... 50
     11.4 KEYBOARD CONNECTION ........................................................................................................ 51




                                                          TI- Proprietary Information –                                        PAGE: 6/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.                              Strictly Private
Features characteristic data and other
information are subject to change.
                                                            UNDER NON DISCLOSURE                                               DO NOT COPY
                                                                 AGREEMENT
REF: CAL000                                                     HERCROM400G2                                                 Ver 1.3

                                                  TABLE SUMMARY

TABLE 1: RESET MANAGEMENT ................................................................................................................. 23
TABLE 2: DMA CHANNELS ALLOCATION ................................................................................................... 26
TABLE 3: ARM MEMORY SPACE ................................................................................................................ 29
TABLE 4: ARM DATA FORMAT .................................................................................................................. 30
TABLE 5: DSP XIO MEMORY SPACE ......................................................................................................... 32
TABLE 6: CALYPSO PINS DESCRIPTION .................................................................................................... 36
TABLE 7: CALYPSO BALL MAPPING ......................................................................................................... 41
TABLE 8: I/O BUFFER TYPE IN 179 BALLS VERSION.................................................................................... 45
TABLE 9: KEYBOARD SCANNING SEQUENCE .............................................................................................. 51


                                                 FIGURE SUMMARY

FIGURE 1: CALYPSO INTERNAL ARCHITECTURE ...................................................................................... 22
FIGURE 3: COMPATIBILITY CONNECTION WITH NAUSICA ........................................................................... 46
FIGURE 4: POWER SUPPLIES CONNECTION WITH IOTA ABB ..................................................................... 48
FIGURE 5: SIM CARD CONNECTION............................................................................................................ 50
FIGURE 6: KEYBOARD CONNECTION .......................................................................................................... 51




                                                    TI- Proprietary Information –                                   PAGE: 7/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.                      Strictly Private
Features characteristic data and other
information are subject to change.
                                                     UNDER NON DISCLOSURE                                          DO NOT COPY
                                                          AGREEMENT
REF: CAL000                                         HERCROM400G2                    Ver 1.3


1.        GLOSSARY
APIF:                 Arm Port InterFace.
BGA:                  Ball Grid Array.
CLOCK M:              CLOCK Managment.
CPU:                  Central Processor Unit
DAI:                  Digital Audio Interface
DSP:                  Digital Signal Processor.
FLASH:                Flash E2prom
GSM:                  Global System for Mobile communications.
HPI:                  Host Port Interface.
H/W:                  HardWare.
INTH:                 INTerrupt Handler.
ISO:                  International Standards Organization
I/O:                  Input / Output.
JTAG:                 Joined Test Action Group
MxxLyy                Mega-module DSP with xx RAM and yy ROM.
LCD:                  Liquid Cristal Display
LMM:                  DSP Mega-Module
MCU:                  Micro-Controller Unit.
MCSI:                 Multi Channels Serial Interface
MEM INT:              MEMory INTerface.
P.C.:                 Personal Computer
PMT:                  Parallel Multiplexing Test.
PLL:                  Phase Loop Lock
RIF:                  Radio Interface Function.
RISC:                 Reduced Instruction Set Computer
RAM:                  Random Access Memory
ROM:                  Read Only Memory
SIM:                  Subscriber Identity Module.
S/W:                  SoftWare.
TDMA:                 Time Division Multiple Access.
TPU:                  Time Processing Unit.
TQFP:                 Thin Quad Flat Pack.
TSP:                  Time Serial Port.
USART:                Universal Synchronous/Asynchronous Receiver Transmitter
USB:                  Universal Serial Bus
RTC:                  Real Time Clock
ULPD:                 Ultra Low Power Device
BIST:                 Built In Self Test
LPG:                  Led Pulse Generator
PWL:                  Pseudo-noise pulse Width Light modulator
PWT:                  Pulse Width Tone generator
I2C:                  Inter IC Control




                                              TI- Proprietary Information –     PAGE: 8/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE              DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                         Ver 1.3


2.       GENERAL DESCRIPTION
CALYPSO is a chip implementing the digital base-band processes of a GSM/GPRS mobile
phone. This chip combines a DSP sub-chip (LEAD2 CPU) with its program and data memories,
a Micro-Controller core with emulation facilities (ARM7TDMIE), internal 8Kb of Boot ROM
memory, 4M bit SRAM memory, a clock squarer cell, several compiled single-port or 2-ports
RAM and CMOS gates.
The application of this circuit is the management of the GSM/GPRS base-band processes
through the GSM layer 1, 2 and 3 protocols as described in the ETSI standard with a specific
attention to the power consumption in both GSM dedicated and idle modes, and GPRS (class
12) capability.
The chip will fully support the GSM full-level test approval (FTA) for both Full-Rate, Enhanced
Full-Rate and Half-Rate speech coding.
CALYPSO implements all features for the structural test of the logic (full-SCAN, BIST, PMT,
JTAG boundary-SCAN).

3.       BLOCK SPECIFICATION

3.1       INTRODUCTION
CALYPSO architecture is based on two processor cores ARM7 and LEAD2 using the generic TI
RHEA bus standard as interface with their associated application peripherals.
CALYPSO is composed from the following blocks:

           •    ARM7TDMIE
                    ARM7TDMI CPU core (32/16 bits RISC processor)
                    + ARM ice crusher for emulation purpose
           •    DSP subchip S28C128
                    LEAD2 DSP core with 28K words of RAM and 128K words of ROM
                    + API (8 Kw part of the 28K of RAM)
                    + SPI
                    + TIMER
           •    Clock Squarer analog cell.
           •    ARM peripherals:
                    General purpose peripherals
                    • ARM Memory Interface for external RAM, Flash or ROM
                    • RHEA bridge
                    • 4 Mbit Static RAM with write-buffer
                    • Memory Protection Unit (MPU)
                    • Debug Unit (DU)
                    • 64 Kbit of via2-ROM for internal boot.
                    • Die-ID cell (48 bits + 5 spare).
                    Application peripherals
                    • ARM General purposes I/O with keyboard interface and two PWM
                        modulation signals for light and buzzer with possible tones generation.
                    • Micro Wire interfaces for LCD and EEPROM.
                    • 3 Timers (generic, watchdog)
                    • UART 16C750 interface (UART_IRDA) with
                            - IRDA control capabilities (SIR)
                            - Software flow control (UART mode).
                            - hardware flow protocol (DCD, CTS/RTS)
                    • UART 16C750 interface (UART_MODEM) with
                            - hardware flow protocol (DCD, CTS/RTS)
                            - autobaud function
                    • SIM Interface.


                                              TI- Proprietary Information –       PAGE: 9/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE                DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                         Ver 1.3

                      •     ARM interrupts Handler (INTH).
                      •     GSM real-time sequencer (TPU).
                      •     GSM real-time Serial Port (TSP).
                      •     DMA controller (4 channels – 2 ports)
                      •     Real Time Clock (RTC)
                      •     GSM Ultra Low-Power Device (ULPD)
                      •     Clock generator & control with Digital Phase Locked Loop (CLKM)
                      •     Programmable controller for Led pulse generation (LPG)
                      •     Enhanced tone generator (PWT)
                      •     Pseudo-noise modulator for light level control (PWL)
                      •     Master I2C serial interface
                      •     GPRS Encryption Algorithm module 1 & 2.

           •    ASIC DSP peripheral:
                   General purpose peripherals
                   • RHEA bridge
                   Application peripherals
                   • Radio interface (RIF).
                   • Multi Channels Serial Interface (MCSI).
                   • A51/A52 ciphering (CRYPT)
                   • UART 16C750 interface (UART_MODEM) with
                           - hardware flow protocol (DCD, CTS/RTS)
                           - autobaud function, local echo.
                   • DMA controller (4 channels)
                   • DSP interrupts Handler (INTH).
           •    OTHER ASIC peripherals.
                   • JTAG TAP controller.

3.2       ARM megacell (ARM7TDMIE)
The ARM7TDMI is a 32 bits RISC micro-controller core. This microprocessor work in 32 bits or
16 bit instructions and on 32, 16 or 8 bit data. The ARM7 architecture is based on reduced
instruction set computer (RISC). Pipelining is employed so that all parts of the processing and
memory systems can operate continuously. Typically, while one instruction is being executed,
its successor is being decoded, and a third instruction is being fetched from memory. In
CALYPSO chips family, ARM7 is intended to work in ‘little endian’ mode only.
The CPU is associated with an emulation module called IceCrusher, which provides many
debug functions:

           •    Single processor and multiprocessor debug.
           •    High level language and assembly debug (Run, Halt, Step...).
           •    Real Time (CPU continuously running) or Non real-time (CPU stopped) debugs
                options.
           •    Combined 32 and 16 bit mode for ARM processor.
           •    Endianess transparency.
           •    Unlimited breakpoints via op-code replacement (S/W breakpoint).
           •    2 Hardware breakpoints (one configurable as SW breakpoint) with maskable cycle
                type, address and data compare.
           •    2 external breakpoint events.
           •    Internal events generate external triggers.
           •    Benchmarking / profiling capability.




                                              TI- Proprietary Information –       PAGE: 10/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE                DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                    Ver 1.3


3.3       DSP subchip (S28C128)
The DSP subchip is a Digital Signal Processor core compliant with the TMS320C54x family.
The CPU core LEAD2 is associated with a ARM Port Interface (API), an interrupt handler, a
parallel interface XIO, a Timer, 28K words of RAM including 8K words of API shared memory,
128K words of ROM, a serial port, and a JTAG interface.
The input clock frequency is delivered by an external DPLL and the functional cycle frequency is
planned to be in the range [0 – 91 MHz].



3.4       Clock Squarer cell
The Clock Squarer cell is an analog cell which function is to reshape a clock signal provided by
an external oscillator. The input signal is assumed to be a pseudo-sinusoidal signal with a
limited dynamic transformed by the Clock Squarer cell in a square waveform with an amplitude
of VCC (see electrical characteristics of signal CLKIN in Appendix B).

3.5       ARM peripherals

3.5.1         Memory Interface.
 The Arm memory interface handles:
•  External ARM memory access management: It performs
      ⇒ ARM read and write access size adaptation to the memory width (from 8-bit up to
          32-bit).
      ⇒ ARM access duration management (wait state insertion or using ‘nREADY’ input) to
          enable the connection of slow memory devices.
      ⇒ Memory control signals generation (chip-selects, write strobe generation,...) with up
          to 4 chip-select signals corresponding each to an address range of 8 Mbytes.
• ARM to API memory access management:
      ⇒ ARM access size adaptation for API read and writes access. A 32-bit API READ
          transaction is transformed into 2 16-bit read access. A 32-bit API WRITE access is
          transformed into 2 16-bit write transaction.
      ⇒ Address signal-timing adaptation to be compliant with the API interface
          requirement.
• ARM to Rhea bridge access management:
      ⇒ ARM accesses size adaptation for Rhea accesses. The access size adaptation is
          done regarding the Rhea peripheral minimal transaction size. If the Rhea peripheral
          can not handle byte transaction, then no byte write transaction can be done.
• ARM nWAIT and access control flags generation (byte-latch, etc...).



3.5.2         Internal Static RAM
Four megabit of static RAM (SRAM) are embedded on the die and mapped on nCS6 chip-select
of the memory interface. These memories can be read or written either in 8, 16 or 32 bits
format. The access cycle is guaranteed with 0 wait-state for any cycle frequency up to fmax
MHz.




                                              TI- Proprietary Information –   PAGE: 11/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                    Ver 1.3


3.5.3         Internal Boot Memory
A 64Kbit ROM is embedded and mapped on nCS7 chip-select of the memory interface. This
memory can be read either in 8, 16 or 32 bits format. The access cycle is guaranteed with 0
wait-state for any cycle frequency up to fmax MHz.
Depending of the state of the nIBOOT signal, this memory is mapped on nCS0 or nCS7, to
allow the CPU to boot (at reset) on external or internal memory.

3.5.4         Die ID cell
The Die ID cell is a 64-bit register composed of fuse cells programmed by Laser during
fabrication process. Each Die ID is unique.

See chapter 16.4.

3.5.5         Interrupt Handler (INTH).
The interrupt handler provides 21 prioritized and maskable interrupts to the ARM core.
It receives interrupts from both internal modules and external chip environment. Each incoming
interrupt is configured as a low level sensitive or falling edge sensitive interrupt and can be
individually masked using dedicated configuration registers.
An Interrupt Level Register is associated to each incoming interrupt to define a priority to the
corresponding interrupt. If several interrupts have the same priority level, they are sent in a
predefined order.
Each interrupt can be routed to one of the two input interrupts of the ARM core FIQ (Fast
Interrupt request) and IRQ (Low priority Interrupt request).



3.5.6         General purposes I/O (ARMI/O).
We provide fifteen I/O pins configurable in read or write mode by internal registers.
10 special I/O pins are dedicated for keyboard connection: 5 output (Columns) and 5 input
(Rows). The five input pins are connected on 1 interruption for wake-up mode.
A Pulse Width Modulated pin is dedicated to the control of the buzzer. A dedicated timer is used
to generate the waveform shape, which modulates the buzzer PWM signal thus producing
tones in the human audible spectrum.



3.5.7         Micro Wire interfaces (UWIRE).
This serial interface can drive external devices as serial EEPROM or LCD with 2 chip-selects
with respect to the micro wire standard.
The serial clock period is derived from the reference 13MHz clock and can be configured as:
TSCLK = CK_FREQ * Csi_FRQ * T13M = [2/4/7/10] * [2/4/8] * T13M

Note: the use of the uWIRE interface is exclusive with the use of the I2C interface (same
     I/O pins).




                                              TI- Proprietary Information –   PAGE: 12/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                      Ver 1.3


3.5.8         Timers (TIMER).
The chip implements three 16 bits timers configurable either in ‘auto reload’ or in ‘one shot’
modes. The timers generate interrupts to the ARM when equal to zero.
The first timer is configured by default as a Watchdog for the MCU. If a programmer doesn’t
want to have this functionality, he must write a specific sequence into a dedicated register in
order to configure it as a general-purpose timer.
The two other timers are general-purpose timers.



3.5.9         IrDA Universal Async Receiver/transmitter 16C750 (UART-IRDA).
This UART interface is compatible with 16C750 compliant devices. It includes the Slow Infra
Red protocol in order to be connected with an infrared transmitter to any external data
peripherals with an IrDA compliant data interface.
The IR function can be disabled and the UART connected through a standard wired interface.
This UART is primarily intended to be linked with the modem of an external PC for concurrent
debugging purpose.

The module integrates two 64 words (9 and 11 bits) receive and transmit FIFO and one 8 words
(8 bits) status FIFO (IrDA only) which trigger levels are programmable. The baud-rate is
internally generated from a programmable divisor.
Transmission parity can be even, odd, none and the number of stop bits is 1, 1.5 or 2.
The receiver can detect break, idle or framing errors, and FIFO overflow and parity errors.
The transmitter can detect FIFO underflow.
All modem operations are controllable via a software interface.
In IrDA mode, this upgraded UART 16C750 includes the following additional features:

•    IrDA 1.0 SIR support allows serial communication at baud rates up to 115.2 kbaud.
•    Pulse shaping, and pulse recovering. Sending a single infrared pulse signals a zero. A one
                                                                                              th
     is signaled by not sending any pulse. The width of the pulse can be either 1.6 us or 3/16 of
     a single bit time.

•    The device operation, in IrDA 1.0 SIR, is similar to the operation in UART mode. The main
     difference being that the data transfer operations are normally performed in half-duplex, the
     modem control and status signals are not used.

•    Frame formatting: addition of variable xBOF characters and EOF characters

•    Uplink/downlink CRC generation/detection

•    Asynchronous transparency (automatic insertion of break character)

•    8 characters status FIFO available to monitor frames length and frame errors.

•    Variable frame length for RX and TX IrDA frame.

Note: MIR and FIR are not implemented.




                                              TI- Proprietary Information –    PAGE: 13/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE             DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                        Ver 1.3


3.5.10          Universal Async Receiver/transmitter 16C750 (UART-MODEM)
This UART interface is compatible with the NS 16C750 device.
This UART 16C750 is devoted to the connection to a Modem through a standard wired
interface.

The module integrates two 64 words (9 and 11 bits) receive and transmit FIFOs which trigger
levels are programmable. The baud-rate is internally generated from a programmable divisor.
Transmission parity can be even, odd, or none and the number of stop bits is either1, 1.5 or 2.
The receiver can detect break, idle or framing errors, FIFO overflow and parity errors.
The transmitter can detect FIFO underflow.

All modem operations are controllable either via a software interface or using hardware flow
control signals.
This upgraded UART 16C750 includes the following additional features:
         - Hardware flow control (DCD, RTS/CTS)
         - Auto-bauding with the possibility to match to baud-rate from 1200 to 115.2Kbits/s.

This UART is shared between the ARM7 and the DSP processors. One processor only at a
time can controlled the UART. The allocation of the UART is defined by the ARM7
(configuration register). By default, the UART is connected to the ARM7 RHEA bus.



3.5.11          Subscriber Identity Module Interface (SIM).
The Subscriber Identity Module interface will be fully compliant with the GSM 11.11 and ISO/IEC
7816-3 standards.
Its external interface is 3 Volts only. 5 Volts adaptation will be based on external level shifters.
The SIM interface supports the cold and warm reset procedures and the disabling of the clock
(static SIM cards).



3.5.12          Serial Port Interface (SPI)
The SPI is a full-duplex serial port configurable from 1 to 32 bits and provides 3 enable signals
programmable either as positive or negative edge or level sensitive.
The serial clock period is derived from the reference 13MHz clock and can be configured as:
TMCUCLK = PTV * T13M = [1/2/4/8/16] * T13M



3.5.13          Time Processing Unit (TPU)
The TPU is a real-time sequencer dedicated to the monitoring of GSM base-band processing’s.
Working from an event table referring to a GSM TDMA time base, the TPU activates tasks to
control DSP peripherals with respect of the time constraints related to the GSM sequencing.
To store the real-time microinstructions of the sequencer, the TPU includes one 2 ports RAM of
1024 words of 16 bits with a dual page addressing capability.
MCU could access at the full RAM in write mode only when TPU is running.




                                              TI- Proprietary Information –     PAGE: 14/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE               DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                              HERCROM400G2                        Ver 1.3


3.5.14          Time Serial Port (TSP)
The TSP is a peripheral of the TPU, which includes both a serial port (32 bits) and a parallel
interface. The serial port can be programmed by the TPU with a time accuracy of the quarter of
GSM bit. The serial port is uni-directional (transmit only) when used with NAUSICA but can be
configured in bi-directional to keep compliant with VEGA3. The serial ports provide 4 enable
signals programmable either as positive or negative edge or level sensitive.
The parallel interface allows controlling 13 external individual output and 1 internal signal with a
time accuracy of the quarter of GSM bit.
These parallel output signals are mainly used to control the RF activity.
The single internal signal is looped on one interrupt line of the DSP in order to offer a possibility
of sequencing the DSP tasks scheduling on a quarter of GSM bit time accurate event.
The serial clock frequency is fixed to 6.5MHz.



3.5.15          Direct Memory Access controller (DMA)
The DMA controller manages data transfers between the RHEA peripherals and the DSP API
memory. The main features are:

           - Four independent DMA channels
           - Configuration of channels from the MCU only (master, direction)
           - Either the DSP or the ARM host can control each DMA channel
           - All DMA transactions are using the ARM Rhea bus
           - The DMA transaction width can be either 8 or 16-bit data
           - Unknown transfer length support and double-page destination buffers



3.5.16          Clock Management (CLKM)
This module is in charge of the control of the clock activity for the DSP, MCU and RHEA
peripherals. It includes one DPLL and configuration register for DSP and MCU clocks
frequencies programming.
CLKM manages also the reset of all modules connected either to the MCU Rhea bus or the
DSP Rhea bus and the scheduling of the deep power of the external FLASH memory.
The DPLL is programmable in "multiplication-mode" with the following values:
                             m
           Fout = Fin ×                       m = 1 up-to 32 (step 1) d = 1, 2, 3 or 4
                             d
It could be also programmable in "division-Mode" with the following:
                                 Fin
                       Fout =                         k = 1, 2 or 4
                                 k



3.5.17          Pulse Width Tones (PWT)
This module generates a modulated frequency signal for the external buzzer. Frequency is
programmable between 349Hz and 5276Hz with 12 half tone frequencies per octave. The
volume is also programmable.




                                                 TI- Proprietary Information –           PAGE: 15/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.               Strictly Private
Features characteristic data and other
information are subject to change.
                                                 UNDER NON DISCLOSURE                    DO NOT COPY
                                                      AGREEMENT
REF: CAL000                                         HERCROM400G2                   Ver 1.3


3.5.18          Pulse Width Light (PWL)
This module allows the control of the backlight of LCD and keypad by employing a 4096bit
random sequence. This voltage level control technique decreases the spectral power at the
modulator harmonic frequencies. The block uses a switchable clock of 32kHz, independent of
UPS.



3.5.19          Light Pulse Generator (LPG)
This peripheral produces the signal for the blinking LED. Blink period and duration are
programmable.



3.5.20          I2C master serial interface (I2C)
The I2C is a half-duplex serial port using 2 lines (data and clock) for data transmission with
software addressable external devices. The interface is compliant with the Philips standard.
The main features of the interface are:
        - single master only
        - standard (100KHz) and the fast (400KHz) transmission modes
        - support both burst write, single read and combined read modes
        - transmit burst buffer of 16 words
        - 3 bits programmable spike filtering logic
        - error handling capability during I2C bus access

Note: the use of the I2C interface is exclusive with the use of the uWIRE interface (same
     I/O pins).



3.5.21          Memory Protection Unit (MPU)
Within a memory space, the MPU allows defining memory sub-regions, each having a separate
Read/Write protection attribute; this permits for partitioning the memory space into program
instruction, system data, user data, stack …
The application program configures the MPU, which interfaces to the processor via the RHEA
bus. The address bus directly issued from the processor is monitored providing a real-time
position of the memory region accessed. When a protection breach attempt occurs, the
memory control signals are affected, not selecting the memory, and the fault condition is
indicated to the processor.
The MPU allows for controlling:
      • Up to four programmable protected regions within a memory space of 512 Kbyte.
      • A maximum protected size of 128 Kbyte for each region (512 Kbyte for 4 regions).
      • A minimum granularity of 8 bytes.
      • A privileged-code memory region.
For each region, memory mapped control registers define:
       •     Protected memory region base-address.
       • Starting/ending addresses within the protected memory region.
       •     Protection mode (Non-User R/W, User Read-only, ROM, Privileged-region write…
            -see Protection Mode definition- ) applied to the memory sub-region bounded by the
            starting/ending addresses.
       •     Out-of-protection (upper-bound) indication enabled/disabled.



                                              TI- Proprietary Information –   PAGE: 16/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                  Ver 1.3

The MPU generates:
      •    An illegal-access signal if the application program attempts a non-authorized
          access to a memory region (i.e. User write access to a region programmed for User
          Read-only accesses).
      •    An Out-of-protection signal when the application program attempts to read or write
          to a location within a memory region (defined by the base-address) and above the
          upper limit (end-address) of the protected sub-region. (helpful for stack
          overflow/under-flow monitoring).
      •    A MPU-fault signal which is the OR’ed combination of both Illegal-access and Out-
          of-protection signals.
      •    A fault indication (Illegal-access and/or Out-of-protection) flagged into the MPU
          status register which is available to the processor for fault analyze.

3.5.22          Debug Unit (DU)
The Debug Unit is a hardware resource intended to provide additional support to a software
abort-handler. The DU provides 64 stages deep history table of the last memory accesses prior
entering the abort mode, then permitting analysis of previous bus transaction’ s.

           The DU is an autonomous function that does not need to be configured, hence, does
           not interfaces to a control bus such as RHEA. The DU is connected directly to the
           processor busses (Address & Control) from where it collects the data, and to the
           memory interface system where the saved history table can be read.

The Debug Unit offers the following:

           •    Sixty-four 32-bit words deep FIFO register file
           •    26-bit Processor Address and 8 Processor control signals recorded
                (nM[1:0], MAS[1:0], nEXEC, nOPC, nMREQ, nRW)
           •    Continuous storage for every processor fetch (either instruction or data)
           •    Data record automatically frozen upon switch to abort mode
           •    Memory-like read access during abort operating mode
           •    Enable/disable control from input module pin (typically connected to a chip-
                configuration register-bit).
           •    General purpose RAM when debug function is disabled.



3.5.23          GPRS Encryption Algorithm (GEA1-2)
In GPRS mode, the data confidentiality is performed by a ciphering function (GPRS Encryption
Algorithm). The ciphering is executed within the LLC upper layer. According to the option
negotiated with the network the GEA mode 1 or mode 2 may be selected.
The purpose of the LLC is to convey information between the mobile station and the Serving
GPRS Support Node. The procedures used are modeled upon the HDLC concepts. The LLC
shall support both acknowledge and unacknowledged mode and implement a FCS according to
the mode used.
The HW block provided support the computation of the FCS according to the LLC frame to
send/received as well as the ciphering/deciphering GEA mode 1 and 2. These procedures are
described in 01.61, 04.64 GSM recommendations and detailed in GSM MoU documents.

3.5.24          Internal RAM write buffer (WRB)
The write buffer increases the speed of memory accesses, thanks to write pipe mechanism.
(The idea is to allow a complete CPU cycle time for read access).



                                              TI- Proprietary Information –   PAGE: 17/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                       Ver 1.3

3.5.25          Real Time Clock (RTC)
The RTC block is an embedded Real Time Clock module, directly accessible from RHEA bus
interface.
Its basic functions are:
   - Time information (seconds/minutes/hours) coded in BCD
   - Calendar Information (Day/Month/Year/ Day of the week) directly in BCD code up to year
       2099
   - Alarm       function   with    interrupts    generation     based     on    a    periodical
       (second/minute/hour/day) or a precise time event in the century (second accuracy).
   - 30s time range correction
   - 32KHz oscillator frequency gauging.

3.5.26          Ultra Low-Power Down controller (ULPD)
The ULPD block is used for the DEEP SLEEP MODE management. It allows stopping the
accurate 13MHz VTCXO during the discontinuous reception phases in GSM idle mode in order
to save power consumption. Moreover, the ULPD sequencer is used to monitor the chip activity
in terminal OFF mode, thus deriving the original GSM Deep Sleep mode concept to the whole
terminal activity.
The main functions of the ULPD block are:
  - gauging of the 32 kHz quartz based oscillator
  - maintenance of GSM time during deep sleep mode with the minimum time accuracy to
      allow a burst demodulation at wake-up
  - programmable timer to exit deep sleep mode
  - delivery of the 13MHz master to the CLKM module
  - switching between 13MHz and 32KHz
  - generation of chip functional reset

3.6       Specific power-split for RTC
To have a minimum of consumption in mobile equipment, only the active logic elements have to
be supplied. This approach is possible by using Split Power.
It consists by the use of Tactical Cells to split the core power domain in two sub domains
powered with different voltage supplies.
The separation between the core and the active domain is realized by Tactical cells and more
precisely by Internal Level Shifters. The Internal Level Shifters (ILS) are library standard macro-
cells to interface two core domains with two different supplies.
When the core domain is unknown, these cells have a Power-Down signal, which allows to set
the output of the ILS to 0 and to have any through current. So ILS are going to be placed on
each wire between the active elements and the core.
The aim of the Split Power is to have a minimum of consumption in OFF state. For a mobile the
only necessary element to supply in OFF is the RTC. So the tactical cells will be placed to
isolate the RTC.
To use the Split Power, Calypso had to have pins VDDRTC, VDDSRTC, VSSRTC as specific
power-supplies for both core and IOs.
The pin which are going to be suplied with specific power supplies are :
        - ON/OFF                (in)
        - NRESPWRON             (in)
        - IT_WAKEUP             (out)
        - CLK32K_OUT            (out)
        - OSC32K_IN             (Xtal in)
        - OSC32K_OUT            (Xtal out)




                                              TI- Proprietary Information –     PAGE: 18/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE              DO NOT COPY
                                                   AGREEMENT
          REF: CAL000                                         HERCROM400G2                  Ver 1.3



                            VDD                                                            VDD-RTC



Clk 32k                         CTS
                          UC469                               UC470


D                                                                                            L
A                         UC469                               UC470   pwrdn
                                                                                             O        IT-WakeUp

T                                                                                            G
                                                                                             I
A                         UC469                               UC470   pwrdn
                                                                                             C        Clk_32_kHz

                                                                                             B
R                                                                                            L        on_noff
H
                          UC469                               UC470 pwrdn        RTC         O
                                                                                             C        nrespwron
E                                                                                            K
                          UC469                               UC470   pwrdn
A
B                         UC469                               UC470 pwrdn                                rhea
U                                                                                                       bus
S                                                                     pwrdn


                  rtc_bu_interface




                                                        TI- Proprietary Information –   PAGE: 19/51
          PRELIMINARY          documents      contain
          information on a product under development
          and is issued for evaluation purposes only.         Strictly Private
          Features characteristic data and other
          information are subject to change.
                                                        UNDER NON DISCLOSURE            DO NOT COPY
                                                             AGREEMENT
REF: CAL000                                         HERCROM400G2                      Ver 1.3


3.7       DSP peripherals

3.7.1         Radio interface (RIF)
The RIF (Radio Interface) Module is a buffered serial port derived from the BSP peripheral
module of the defined for TMS320C5X
The external serial data transmission is supported by a full-duplex double buffered serial port
interface.
The DSP exchanges data with the RIF through either its
        -XIO interface for configuration data and radio data in word by word protocol
or its
        - API interface for radio data in DMA mode (buffered mode with data block transfer)
For each data transfer between DSP and RIF, an interrupt is sent to DSP (XIO mode) or a DMA
request and an ‘end-DMA’ request is sent to ARM.
Transmit serial clock is either generated internally or externally. Receive serial clock is always
generated externally.
The RIF uses an internal 13MHz clock to manage the auto-buffering function.



3.7.2         Multi-Channel Serial Interface (MCSI)
The MCSI is a serial interface supporting both single and multi-channels (x16) communications.
Words length, frame format, baud-rates are fully programmable.
The interface can be additionally configured to support GSM DAI test mode with the possibility
to synchronize DAI with the voice frame signal.



3.7.3         Ciphering processor (CRYPT)
The ciphering processor implements both A51 and A52 algorithms as defined in GSM Rec03.20
and detailed in GSM MoU documents.



3.7.4         Universal Asynchronous Receiver/transmitter (16C750)
UART interface compatible with the NS 16C750 device.
This UART is shared between the ARM7 and the DSP processors. One processor only at a
time can control the UART. The ARM7 configuration register defines the allocation of the
UART). By default, the UART is connected to the ARM7 RHEA bus.



3.7.5         Direct Memory Access controller (DMA)
The DMA module is shared between DSP and MCU processors. The DMA have only two port
DSP-API memory and MCU-RHEA, so it allows RHEA peripheral to use DMA channel between
us and the API memory.




                                              TI- Proprietary Information –    PAGE: 20/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE             DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                      Ver 1.3


3.7.6          Interrupt Handler (INTH).
The interrupt handler provides 21 interrupts to the DSP core.
It receives interrupts from both internal modules and external chip environment. Each incoming
interrupt is configured as a low level sensitive or falling edge sensitive interrupt. The mask and
the interruption level of the interrupts are configured in the DSP core itself.
           .


3.8       General purpose peripherals

3.8.1          JTAG
The JTAG interface (TAP) of the chip can be selected either

1. to access the 2 processors on-chip emulators with a pseudo IEEE JTAG protocol for
   emulation purposes. A PC or workstation can be connected to the interface to set the bi-
   emulation mode with the ARM core linked to DSP core. The IceCrusher module supports
   the synchronization between the 2 cores.

Or

2. To dialog with an embedded TAP controller which instructions set support all the IEEE 1149
   BSCAN modes, the programming of the chip I/Os configuration (PMT modes, full-scan
   modes, BIST modes, functional 1 or 2 modes) and the selection of the boundary-scan chain
   for FLASH EPROM programming.



3.8.2          IDDQ
Pulling high the IDDQ pin sets the IDDQ configuration.
In IDDQ mode, all analog devices are disabled thus allowing reducing the chip current
consumption to the single leakage currents. Moreover, a direct control of the ARM and DSP
resets and clocks is allowed through, respectively, the pin BCLKR and CLKTCXO of the chip.




                                              TI- Proprietary Information –    PAGE: 21/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE             DO NOT COPY
                                                   AGREEMENT
 REF: CAL000                                                       HERCROM400G2                                       Ver 1.3


 3.9       Block Diagram
                                                                                                                     32KHz CRYSTAL
                  13MHz or 26 MHz TCXO

                                                                                                                      IT Alarm
               SLICER     DIV - 2              13MHz                                                         RTC
                                                                                 ENABLE_CK13Mhz

                                                                                 Asynchronous WAKE_UP          CK32Khz
                          DPLL & CLKM              WTDOG       INTH
                                                                                                             ULPD

                                                                                      SPI
                   Boot ROM                         MCU top-cell
                                                                                                               GSM time
External                                                                              GEA
 ARM7                                    MEMIF                                                               TPU
                                                           B
Memories            Memory                                 R                         Die ID
                                                                                                  RHEA bus
                    Protect         Debug Unit             I
                     Unit                                  D                                                 TSP
                                                                                     TIMER1
                                                           G
                1Mbit        W
                                         ARM7              E
                                                                                     TIMER2
                                                                                                             SIM

                 SRAM        r                                                                               PWL
                             i                                                       ARMIO
                1Mbit        t                            DMA (4+)                                           UART
                 SRAM                                                                uWIRE                    IRDA
                             e
                1Mbit                                                                                        PWT
                 SRAM         b                            B
                                               8K API
                              u                            R              RHEA bus                           LPG
                 1Mbit        f          cDSP              I                          CRYPT
                 SRAM         .          S28C128           D
                                                                                        RIF                   I2C
                                                           G
                                                           E
                                     DSP subchip                   INTH                MCSI
                                                                                                             UART
                               JTAG                                                                          modem


                               Figure 1: CALYPSO internal architecture

 4.       INITIALIZATION PROTOCOL

 4.1       Hardware logic reset
 The chip receives one external reset signal (active low):

 * The power-on reset (nRESPWRON) initializes the whole digital logic devices. It is activated
 each time the power supply is applied to the chip.

 The functional reset is generated internally from the ULPD controller:

 * The internal functional reset (nRESET) initializes the logic devices working on any clocks
 derived from the 13MHz-clock reference or the processor strobes. This reset is activated each
 time the terminal is switched in OFF mode and maintained active as long as the external control
 pin ON_nOFF is at low level.




                                                        TI- Proprietary Information –                        PAGE: 22/51
 PRELIMINARY          documents      contain
 information on a product under development
 and is issued for evaluation purposes only.                       Strictly Private
 Features characteristic data and other
 information are subject to change.
                                                        UNDER NON DISCLOSURE                                 DO NOT COPY
                                                             AGREEMENT
REF: CAL000                                           HERCROM400G2                          Ver 1.3

Internally to the chip, three reset signals are provided in order to dissociate the reset of the logic
working on the 32KHz or TCK clocks and the reset of the logic working on clocks derived from
the reference clock 13MHz or the processors strobes. Namely:

- nRESET_32K                ⇒ JTAG, RTC and ULPD modules.

- nRESET_DSP                ⇒ all logic mapped on the DSP Rhea bus except 32KHz based logic

- nRESET_MCU                ⇒ all logic mapped on the MCU Rhea bus except 32KHz based logic

Externally to the chip, two reset signals can be controlled by the MCU to reset selectively
external components and to control the deep power mode of external FLASH memories

- nRESET_OUT                ⇒ external components.

- FDP                       ⇒ FLASH deep power (reset delay controlled with
                              CNTL_ARM_CLK (deep_power).



                                                                Reset signal
Control                             nRESET_32K     NRESET_DSP   nRESET_MCU     nRESET_OUT     FDP
NRESPWRON                                 Yes          Yes          Yes           Yes         Yes
NRESET                                    No           Yes          Yes           Yes         Yes
RST_CMD watchdog                          No           Yes          Yes           No          Yes
CNTL_RST(DSP_reset)                       No           Yes          No            No          No
CNTL_RST(ext_reset)                       No           No           No            Yes         No

                                         Table 1: Reset management
* CNTL_RST is a control register mapped in the MCU memory space



After ARM reset the ARM program counter point to 0000:0000 address.
After DSP reset the DSP program counter point to 0000:FF80 address.

Each module implements in its control register a software-reset bit, which can be activated by
its master processor. By default, these bits are set as soon as the processor reset
(nRESET_DSP or nRESET_DSP) is activated.

4.2       ARM code downloading
In debug mode, when external Ram is used to replace FLASH or ROM, the program code is
downloaded thanks to JTAG facilities. Flash memory program is downloaded through the JTAG
TAP using the dedicated boundary-scan chain.

ROM or EPROM devices must be programmed before assembly.



4.3       DSP code execution
When releasing the DSP reset signal nRESET_DSP, the DSP fetches the reset interrupt vector
at address FF80h and begins the execution of the program code.




                                                TI- Proprietary Information –           PAGE: 23/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                                UNDER NON DISCLOSURE                    DO NOT COPY
                                                     AGREEMENT
REF: CAL000                                             HERCROM400G2                           Ver 1.3


5.       INTERRUPTS MANAGEMENT

5.1       DSP INTERRUPTS
The DSP subchip owns 17 interrupt lines with 11 of which INT0n to INT10n are dedicated for
external peripherals. These interrupts are mapped as follows:

- RSN                         ->              reset (HW or SW)
- INT0n -             (level) ->              RIF receive interrupt
- INT1n               (level) ->              RIF transmit interrupt
- INT2n               (level) ->              UART interrupt
                                                  1. Error on receiver line.
                                                  2. Receive timeout.
                                                  3. Received character.
                                                  4. Character to transmit.
                                                  5. Modem status change.
                                                  6. Received XOFF / special character detected.
                                                  7. CTS/RTS deactivation.

- TINT                           ->           Timer interrupts
- RINT                           ->           SPI receive interrupt
- RINT                           ->           SPI transmit interrupt
- INT3n               (level)    ->           MCSI receive interrupt
- INT4n               (level)    ->           MCSI transmit interrupt
- INT5n               (level)    ->           MCSI frame duration error interrupt
- INT6n               (level)    ->           MCSI DAI interrupt
- INT7n               (edge)     ->           CYPHER interrupts
                                                  1. end of ciphering process
                                                  2. error of processing
- INT8n               (edge) ->               TPU frame interrupt
- AINT                        ->              API interrupts
- INT9n               (edge) ->               TPU programmable interrupt
- INT10n              (level) ->              DMA interrupt
- nMIN                        ->              Abort on Rhea bus OR INT4n redirection

Note: The TPU programmable interrupt (INT9n) is a facility offered to the DSP programmer in
order to allow the generation of a DSP interrupt at a dedicated time with a quarter of GSM bit
accuracy. The interrupt is set in a scenario by using a time-stamped instruction.




                                                TI- Proprietary Information –           PAGE: 24/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.              Strictly Private
Features characteristic data and other
information are subject to change.
                                                 UNDER NON DISCLOSURE                   DO NOT COPY
                                                      AGREEMENT
REF: CAL000                                              HERCROM400G2                           Ver 1.3


5.2       MCU INTERRUPTS
The ARM7 owns 2 interrupt lines nIRQ and nFIQ.
The NAUSICA fast interrupt is mapped on nFIQ
All other peripheral interrupts are mapped on nIRQ as follows:
 Name             Sense        IRQ       FIQ       Function
 IRQ0             edge          4                  Watchdog TIMER interrupts
 IRQ1             edge          4                  TIMER1 interrupt
 IRQ2             edge          4                  TIMER2 interrupt
 IRQ3                                         4    TSP receives interrupt
 IRQ4             edge           4                 TPU frame interrupt
 IRQ5             edge           4                 TPU page interrupt
 IRQ6             edge           4                 SIM interrupt
                                                   1. no answer to reset
                                                   2. character underflow
                                                   3. character overflow
                                                   4. character to transmit
                                                   5. received character
                                                   6. SIM card insertion/extraction
 IRQ7             level          4                 UART_MODEM interrupts
                                                   1. error on receiver line
                                                   2. receive timeout
                                                   3. received character
                                                   4. character to transmit
                                                   5. modem status change
                                                   6. Received XOFF / special character detected
                                                   7. CTS/RTS deactivation
                                                   8. DSR/RxD activity detection (OFF mode only)
 IRQ8             level          4                 Keyboard or JogDial interrupt
 IRQ9             edge           4                 RTC periodical timer interrupt
 IRQ10            level          4                 RTC ALARM or I2C data transfer error / completion
 IRQ11            edge           4                 ULPD end of gauging interrupt
 IRQ12            level          4                 External interrupt
 IRQ13            edge           4                 SPI interrupt
                                                   1. received data
                                                   2. data to transmit
 IRQ14            level          4                 DMA interrupt
 IRQ15            edge           4                 API interrupts (nHINT)
 IRQ16                                        4    SIM card-detect fast interrupt
 IRQ17                                        4    Fast external interrupt
 IRQ18            level          4                 UART_IRDA interrupts
                                                   1. error on receiver line
                                                   2. receive timeout
                                                   3. received character
                                                   4. character to transmit
                                                   5. modem status change
                                                   6. Received XOFF / special character detected
                                                   7. CTS/RTS deactivation
 IRQ19            level          4                 ULPD GSM timer
 IRQ20            level          4                 GEA interrupt




                                                  TI- Proprietary Information –          PAGE: 25/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.               Strictly Private
Features characteristic data and other
information are subject to change.
                                                  UNDER NON DISCLOSURE                   DO NOT COPY
                                                       AGREEMENT
REF: CAL000                                         HERCROM400G2                      Ver 1.3


6.        DMA MAPPING
The DMA controller is managing the access to the DSP API 6K-word shared memory.
     1.   The MCU ARM7.
     2.   The Radio InterFace (RIF).
     3.   The MODEM UART.
     4.   The IRDA UART.
The RIF-RX and RIF-TX have a dedicated channel each. For UART the following combination
are possible:
  ð UART-MODEM have 2 channels:
                 • ch#2: TX
                 • ch#3: RX
  ð UART-IRDA have 2 channels:
                 • ch#2: RX
                 • ch#3: TX
  ð UART-MODEM and UART-IRDA have one channel each
                 • ch#2: UART-MODEM
                 • ch#3: UART-IRDA
      RX and/or TX are independently selectable for each module.
  ð UART-MODEM have one channel
                 • ch#2: RX or TX
  ð UART-IRDA have one channel
                 • ch#3: RX or TX
  ð UARTs have no DMA.
After reset, all modules have DMA functions disabled.
                                                                 channel
DMA request
                                               0             1                2           3
RIF_DMA_REQ_X                                 4
RIF_DMA_REQ_R                                               4
nDMA_REQ_ARM(0) MODEM                                                      4
nDMA_REQ_ARM(1) MODEM                                                                    4
nDMA_REQ_ARM(0) IRDA                                                                     4
nDMA_REQ_ARM(1) IRDA                                                       4
                                    Table 2: DMA channels allocation

Note: Only one UART at a time should be allocated to one DMA channel (2 or 3). The
      potential conflicts between concurrent DMA requests MUST be solved at system
      level with only one peripheral configured in DMA mode.




                                              TI- Proprietary Information –       PAGE: 26/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE                DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                          HERCROM400G2                          Ver 1.3


7.       ADDRESS MAPPING

7.1       ARM MEMORY SPACE
ARM memory space is shared between external Memory Interface and RHEA bus.
The Memory Interface is providing 6 chip-select signals.
All internal peripherals are mapped on ARM memory space with a range of 32Kbytes.


7.1.1         Memory interface: using CS4 (default)
                  Important : nCS4 is only available from Calypso C035 – F751619
(*): External memory
  Device name                   nIBOOT        Start address   Stop address   Size (byte)    Data
                                   1           0000:0000       003F:FFFF     4M
 nCS0 (*)                                                                                  8/16/32
                                   0           0000:2000       003F:FFFF     4M – 8K
 nCS6                              -           0080:0000       00BF:FFFF     512K          8/16/32
 not allocated                     -           00C0:0000       00FF:FFFF     -                -
 nCS1 (*)                          -           0100:0000       013F:FFFF     4M            8/16/32
 nCS2 (*)                          -           0180:0000       01BF:FFFF     4M            8/16/32
 nCS3 (*)                          -           0200:0000       023F:FFFF     4M            8/16/32
 CS4 (*)                           -           0280:0000       02BF:FFFF     4M            8/16/32
 nCS4 (*)                          -           0280:0000       02BF:FFFF     4M            8/16/32
 nCS0 image                        -           0300:0000       033F:FFFF     4M            8/16/32
                                   1           0380:0000       03BF:FFFF     4M
 nCS7                                                                                      8/16/32
                                   0           0000:0000        0000:1FFF    8K
 Debug Unit (DU)                   -           03C0:0000       03FF:FFFF     32             32
 not allocated                     -           0400:0000       FFCF:FFFF     -               -
 API RAM                           -           FFD0:0000       FFD0:3FFF     16K           16/32
 API control register              -           FFE0:0000       FFE0:0001     2              16

7.1.2         Memory interface: using ADD(22)
                  Important : nCS4 is only available from Calypso C035 – F751619
(*): External memory
  Device name                   nIBOOT        Start address   Stop address   Size (byte)    Data
                                   1           0000:0000       007F:FFFF     8M
 nCS0 (*)                                                                                  8/16/32
                                   0           0000:2000       007F:FFFF     8M – 8K
 nCS6                              -           0080:0000       00BF:FFFF     512K          8/16/32
 not allocated                     -           00C0:0000       00FF:FFFF     -                -
 nCS1 (*)                          -           0100:0000       017F:FFFF     8M            8/16/32
 nCS2 (*)                          -           0180:0000       01FF:FFFF     8M            8/16/32
 nCS3 (*)                          -           0200:0000       027F:FFFF     8M            8/16/32
 nCS4 (*)                          -           0280:0000       02BF:FFFF     8M            8/16/32
 nCS0 image                        -           0300:0000       037F:FFFF     8M            8/16/32
                                   1           0380:0000       03FF:FFFF     8M
 nCS7                                                                                      8/16/32
                                   0           0000:0000       0000:1FFF     8K
 Debug Unit (DU)                   -           03C0:0000       03FF:FFFF     32             32
 not allocated                     -           0400:0000       FFCF:FFFF     -               -
 API RAM                           -           FFD0:0000       FFD0:3FFF     16K           16/32
 API control register              -           FFE0:0000       FFE0:0001     2              16
 Debug Unit                        -


                                              TI- Proprietary Information –         PAGE: 27/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                               UNDER NON DISCLOSURE                 DO NOT COPY
                                                    AGREEMENT
REF: CAL000                                         HERCROM400G2                  Ver 1.3




7.1.3         External Flash/ROM image
In all case and whatever the value of the nIBOOT signal, the external memory mapped on nCS0
is accessible at address defined in nCS0-image range.




                                              TI- Proprietary Information –   PAGE: 28/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                          HERCROM400G2                        Ver 1.3


RHEA peripherals
Strobe 0 - Device name                    Start address   Stop address   Size in bytes   Data
reserved             ---                    FFFF:0000      FFFF:0FFF            -            -
TPU registers       CS2                     FFFF:1000       FFFF:13FF          1K           16
           reserved  ---                    FFFF:1400      FFFF:4FFF            -            -
UART_IRDA           CS10                    FFFF:5000      FFFF:57FF           2K            8
UART_MODEM          CS11                    FFFF:5800      FFFF:5FFF           2K            8
UART_MODEM          CS12                    FFFF:6000      FFFF:67FF           2K            8
           reserved  ---                    FFFF:6800      FFFF:6FFF            -            -
RIF                 CS14                    FFFF:7000      FFFF:77FF           2K           16
           reserved  ---                    FFFF:7800      FFFF:97FF            -            -
TPU RAM             CS18                    FFFF: 9000      FFFF:97FF          2K           16
DPLL configuration CS19                     FFFF:9800       FFFF:9801           2           16
      not allocated  ---                    FFFF:9802      FFFF:F7FF            -            -
GEA                 CS24                    FFFF:C000      FFFF:C7FF           2K          8/16
      not allocated  ---                    FFFF:C800      FFFF:F7FF            -            -
Watchdog timer      CS31                    FFFF:F800      FFFF:F8FF           256          16
RHEA bridge                                 FFFF:F900      FFFF:F9FF           256          16
INTH                                        FFFF:FA00      FFFF:FAFF           256          16
Memory Interface                            FFFF:FB00      FFFF:FBFF           256          16
DMA controller                              FFFF:FC00      FFFF:FCFF           256          16
CLKM                                        FFFF:FD00      FFFF:FDFF           256          16
JTAG ID code                                FFFF:FE00      FFFF:FE03            4           16
MPU                                         FFFF:FF00      FFFF:FFFF           256          16
Strobe 1 – Device Name                    Start address   Stop address   Size in bytes   Data
SIM                 CS0                     FFFE:0000      FFFE:07FF           2K           16
TSP                 CS1                     FFFE:0800      FFFE:0FFF           2K           16
           reserved  ---                    FFFE:1000      FFFE:17FF            -            -
RTC                 CS3                     FFFE:1800      FFFE:1FFF           2K            8
ULPD                CS4                     FFFE:2000      FFFE:27FF           2K           16
I2C                 CS5                     FFFE:2800      FFFE:2FFF           2K            8
SPI                 CS6                     FFFE:3000      FFFE:37FF           2K           16
TIMER1              CS7                     FFFE:3800      FFFE:3FFF           2K           16
UWIRE               CS8                     FFFE:4000      FFFE:47FF           2K           16
ARMIO               CS9                     FFFE:4800      FFFE:4FFF           2K           16
           reserved  ---                    FFFE:5000      FFFE:67FF            -            -
TIMER2              CS13                    FFFE:6800      FFFE:6FFF           2K           16
           reserved  ---                    FFFE:7000      FFFE:77FF            -            -
LPG                 CS15                    FFFE:7800      FFFE:7FFF           2K            8
PWL                 CS16                    FFFE:8000      FFFE:87FF           2K            8
PWT                 CS17                    FFFE:8800      FFFE:8FFF           2K            8
           reserved  ---                    FFFE:9000      FFFE:EFFF            -            -
JTAG ID code        CS30                    FFFE:F000       FFFE:F003           4
DSP configuration                           FFFE:F004       FFFE:F005           2          16
ARM configuration                           FFFE:F006       FFFE:F007           2          16
Asic configuration                          FFFE:F008       FFFE:F009           2          16
IO selection config                         FFFE:F00A      FFFE:F00B            2          16
Osc32K config                               FFFE:F00C      FFFE:F00D            2          16
MCU emu config                              FFFE:F00E      FFFE:F00F            2          16
DIE id code                                 FFFE:F010       FFFE:F017           8          16
           reserved CS31                    FFFE:F800      FFFE:FFFF            -           -
                                        Table 3: ARM memory space

                                              TI- Proprietary Information –        PAGE: 29/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                               UNDER NON DISCLOSURE                DO NOT COPY
                                                    AGREEMENT
REF: CAL000                                           HERCROM400G2                       Ver 1.3


D32 --------------> D24    D23 --------------> D16 D15 ---------------->D8 D7 -----------------> D0
                                                 nCS0
                                                 nCS1
                                                 nCS2
                                                 nCS3
                                                  CS4
                                                 nCS6
                                                 nCS7
                                                API RAM
                                                                          MPU
                                                                           GEA
                                                                          APIC
                                                                           SIM
                                                                           TSP
                                                                        TPU_REG
                                                                        TPU_RAM
                                                       NOT MAPPED                    RTC
                                                                          ULPD
                                                       NOT MAPPED                    I2C
                                                                           SPI
                                                                         TIMER1
                                                                                     LPG
                                                       NOT MAPPED
                                                                                    PWL
                     NOT MAPPED
                                                                        Reserved
                                                       NOT MAPPED                   PWT
                                                                         UWIRE
                                                                         ARMIO
                                                                              UART_IRDA
                                                       NOT MAPPED
                                                                              UART_MODEM
                                                                         TIMER2
                                                                      RHEA bridge
                                                                          INTH
                                                                   Memory Interface
                                                                     DMA controller
                                                                          CLKM
                                                                      JTAG ID code
                                                                       Die ID code

                                              Table 4: ARM data format




                                                TI- Proprietary Information –     PAGE: 30/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.            Strictly Private
Features characteristic data and other
information are subject to change.
                                                 UNDER NON DISCLOSURE             DO NOT COPY
                                                      AGREEMENT
REF: CAL000                                           HERCROM400G2                  Ver 1.3


7.2         DSP MEMORY SPACE
1.    Provides 8K of API RAM starting at 0800h
2.    RAM in a minimum system would be 8K.

                  Data        Prog0       Prog1        Prog2        Prog3
     0000                   DARAM overlay over the program area - 2K
     0800
     1000                         API overlay over the program area
     1800                                        8K
     2000
     2800
     3000
     3800
     4000
                               DARAM overlay over the program area
     4800
                                              18K
     5000
     5800
     6000
     6800
     7000
     7800
     8000
     8800                                                                PROM
     9000                                                                 8K
     9800
                                 PROM
     A000                         28K
     A800
                 DROM                          PROM       PROM
     B000                                       32K        32K
                  20K
     B800
     C000
     C800
     D000
     D800
     E000
     E800               PDROM
     F000                 8K
     F800

7.2.1         API
The API interface offers a dual access capability to 8 kWords of 16 bits of mixed data program
memory, it can be configured to manage data access of 8,16 or 32 bits through the API control
registers and the memory interface configuration registers (See document [7]).
The API dual access capability is either enabled (SAM mode) or disabled (HOM mode) by the
DSP. SAM mode is the default configuration when the DSP exits from a reset phase.
In SAM mode (Shared Access Mode), ARM (or DMA controller) and DSP can both access
simultaneously to this shared memory space with ARM access resynchronized on DSP cycle-
clock (3 times ratio required between ARM and DSP cycle clocks).
In HOM mode (Host Only Mode), the API RAM is dedicated to external access under the control
of either the ARM or the DMA controller and therefore the access time is limited by the
maximum access time of the used DARAM..


                                              TI- Proprietary Information –     PAGE: 31/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE              DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                          HERCROM400G2                        Ver 1.3


7.2.2         XIO-RHEA
Internal and external peripherals are mapped on XIO or data memory spaces. Theses spaces
are accessible through nXSTROBE[3:0] with a range of 2Kbytes for external peripherals
allowing to connect up to :
• 6 external devices on program space
• 26 external devices on data space
• 31 external devices on I/O space
• internals peripherals are connected on I/O space or data memory space

DSP XIO-RHEA mapping.
Device name                               Start address   Stop address   Size in bytes   Data

External peripherals mapping - Program space
Strobe 0
not allocated       CS0          0000                         07FF            2K           16
         …            …           …                            …              …            …
not allocated       CS5          3000                         37FF            2K           16
External peripherals mapping - Data Space 1
Strobe 1
not allocated       CS6          3800                         3FFF            2K           16
         …            …           …                            …              …            …
not allocated       CS15         7800                         7FFF            2K           16
External peripherals mapping - Data Space 2
Strobe 2
UART_MODEM          CS16         8000                         87FF            2K            8
not allocated       CS17         8800                         8FFF            2K           16
         …            …           …                            …              …            …
not allocated       CS31         F800                         FFFF            2K           16
External peripherals mapping -I/O Space
Strobe 3
RIF                 CS0          0000                         07FF            2K           16
MCSI                CS1          0800                         0FFF            2K           16
not allocated       CS2          1000                         17FF            2K           16
not allocated       CS3          1800                         1FFF            2K           16
not allocated       CS4          2000                         27FF            2K           16
CYPHER              CS5          2800                         2FFF            2K           16
         …            …           …                            …               …           …
not allocated       CS30         F000                         F7FF            2K           16
XI0-2-RHEA bridge CS31           F800                         F8FF            256          16
API Control                      F900                         F9FF            256          16
INTH                             FA00                         FAFF            256          16
not allocated                    FB00                         FBFF            256
DMA controller                   FC00                         FCFF            256          16
not allocated                    FD00                         FDFF            256
not allocated                    FE00                         FEFF            256
not allocated                    FF00                         FFFF            256

                                     Table 5: DSP XIO memory space




                                              TI- Proprietary Information –         PAGE: 32/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                               UNDER NON DISCLOSURE                 DO NOT COPY
                                                    AGREEMENT
REF: CAL000                                         HERCROM400G2                      Ver 1.3


8.       CALYPSO PINS DESCRIPTION
TI code name: F741979

MICRO_WIRE INTERFACE : 5 pins.                                                 Pull      Reset
SDI           IN      Data in.                                                 LPU      input
SDO           OUT     Data out.                                                         0
SCLK          OUT     Serial clock.                                                     0
nSCS0         OUT     chip select 0.                                                    1
nSCS1         OUT     chip select 1.                                                    1

UART16C750 INTERFACE (UART_IRDA): 5 pins.                                     Pull      Reset
TXIR_IRDA     OUT     Infra-Red transmit pulse.                                         0
TX_IRDA       OUT     Transmit Data                                                     1
RXIR_IRDA     IN      Infra-Red receive pulse.                                          Input
RX_IRDA       IN      Receive Data.                                                     Input
SD_IRDA       OUT     IRDA transceiver ShutDown mode.                                   1

UART16C750 INTERFACE (UART_MODEM): 5 pins.                                    Pull      Reset
TX_MODEM      OUT     Transmit Data.                                                    1
RX_MODEM      IN      Receive Data.                                                     Input
CTS_MODEM     IN      Clear To Send.                                                    Input
RTS_MODEM     OUT     Request To Send.                                                  1
DSR_MODEM     IN      Data Set Ready.                                                   Input

GENERIC I/O PORT : 16 pins.                                                    Pull      Reset
BU            OUT        Buzzer output with PWM.                                        0
LT            OUT        Light output with PWM.                                         0
IO(3:0)       IN/OUT     generic Input/Output.                                          input
KBC(4:0)      OUT        Keyboard matrix 5 by 5 column access.                          11111
KBR(4:0)      IN         Keyboard matrix 5 by 5 row access.                     PU      input

ARM MEMORY INTERFACE: 49 pins.                                                Pull      Reset
ADD(21:0)    OUT    ARM address bus out                                                 00000
DATA(15:0)   IN/OUT ARM data bus                                                        Output
nCS(3:0)     OUT    4 chip select active low.                                           1111
CS4          OUT    1 chip-select active high                                           0
RnW          OUT    RAM/FLASH memory read no write signal.                              1
nBHE         OUT    Ext RAM1/RAM2 chip select .                                         0
nBLE         OUT    Ext RAM1/RAM2 chip select.                                          0
nFOE         OUT    Flash output enable for standby mode..                              1
nFWE         OUT    VPP command for flash write.                                        1
FDP          OUT    Flash deep low-power                                                0

Important: From Calypso C035 - F751619:
Replace line above by
 nCS(4:0)          OUT      5 chip select active low.                                   11111




                                              TI- Proprietary Information –   PAGE: 33/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                           HERCROM400G2                    Ver 1.3

TPU PARALLEL PORT : 12 pins.                                                  Pull      Reset
TSPACT(11:0)  OUT      Synchronous activation signal (GSM qbit )                        H0000


TPU SERIAL PORT : 7 pins.                                                     Pull      Reset
TSPDO         OUT         Output serial data.                                           0
TSPDI         IN          Input serial data (VEGA3 compatibility only)                  input
TSPEN(3)      OUT         Configurable triggers (edge/level)                            1
TSPEN(2)      OUT         Configurable triggers (edge/level)                            0
TSPEN(1:0)    OUT         Configurable triggers (edge/level)                            11
TSPCLKX       OUT         Transfer serial clock (VEGA3 compatible)                      0

ARM SERIAL PORT : 5 pins.                                                     Pull      Reset
MCUDI        IN          Input serial data.                                             input
MCUDO        OUT         Output serial data.                                            0
MCUEN(2:0)   OUT         Configurable enable triggers (edge/level)                      111

JTAG PORT : 7 pins.                                                            Pull      Reset
TDI             IN                       Test Data Input.                      LPU      Input
TDO             OUT                      Test Data Output.                              Z
TMS             IN                       Test Mode Select.                     LPU      Input
TCK             IN                       Test Clock.                           LPD      Input
nEMU0           IN/OUT                   Test Emulation pin 0.                  PU      Input
nEMU1           IN/OUT                   Test Emulation pin 1.                  PU      Input
NBSCAN          IN                       Boundary-scan selection                PU      1

MISCELLANEOUS : 11 pins.                                                       Pull      Reset
CLKTCXO     IN          VCTXO input clock (13MHz or 26MHz).                             Input
CLK13M_OUT  OUT         CLKM output clock (13MHz).                                      0
OSC32K_IN   IN          Input component signal of 32KHz quartz.                         Input
OSC32K_OUT  IN          Output component signal of 32KHz quartz.                        Input
CLK32K_OUT  OUT         32KHz oscillator square waveform output                         0
NRESPWRON   IN          Chip Power-On reset                                             0
nRESET_OUT  OUT         Reset of external peripherals.                                  0
EXT_IRQ     IN          External interrupt for ARM                              PU      Input
EXT_FIQ     IN          Fast external interrupt for ARM                         PU      Input
IDDQ        IN          Static config. in IDDQ mode (active high)                       Input
nIBOOT      IN          Internal/External Boot memory                                   Input

POWER MANAGEMENT : 5 pins.                                                     Pull      Reset
IT_WAKEUP   OUT      Wake-up interrupt of Real Time Clock.                              0
TCXOEN      OUT      External TCXO enable                                               0
RFEN        OUT      External RF IC enable                                              0
ON_OFF      IN       Regulators activity                                                Input

VOICE BAND INTERFACE : 4 pins.                                                 Pull      Reset
VCLKRX        IN       Transmit/Receive clock.                                          Input
VDX           OUT      Transmit Data.                                                   0
VDR           IN       Receive data.                                                    Input
VFSRX         IN       Transmit/Receive Synchro.                                        Input




                                              TI- Proprietary Information –   PAGE: 34/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                               UNDER NON DISCLOSURE           DO NOT COPY
                                                    AGREEMENT
REF: CAL000                                           HERCROM400G2                        Ver 1.3


MCSI INTERFACE : 4 pins.                                                           Pull      Reset
MCSI_TXD      OUT                        Transmit serial data                               0
MCSI_RXD      IN                         Receive serial data                                Input
MCSI_CLK      IN/OUT                     Bit synchronization clock                          Input
MCSI_FSYNCH IN/OUT                       Frame synchronization clock or SS reset            Input

BASE BAND INTERFACE : 6 pins.                                                      Pull      Reset
BFSR          IN       Receive synchro.                                                     Input
BDR           IN       Receive data.                                                        Input
BFSX          OUT      Transmit synchro.                                                    0
BDX           OUT      Transmit data.                                                       0
BCLKR         IN       Receive serial clock (VEGA3 compatibility)                           Input
BCLKX         IN/OUT   Transmit serial clock (VEGA3 compatibility)                          Input

SIM INTERFACE : 6 pins.                                                            Pull      Reset
SIM_RST        OUT                       Sim reset.                                         0
SIM_PWCTRL     OUT                       Power Control.                                     0
SIM_IO         IN/OUT                    Input output signal.                               0
SIM_CLK        OUT                       Output clock.                                      0
SIM_CD         IN                        Card detect (VEGA3 compatibility)                  Input
SIM_RnW        OUT                       Direction of IO data line                          0

POWER & GROUND : 33 pins.                                                                    Nb
VDD             -      VCC voltage for ASIC core & CPUs                                       6
VDDPLL          -      VCC voltage for DPLL                                                   1
VDDRTC          -      VCC voltage for RTC split powered                                      1
VDDS-MIF        -      VCC voltage dedicated to memory interface                              4
VDDS-RTC        -      VCC voltage dedicated to RTC split powered I/Os                        1
VDDS-1                                                                                        2
                -      VCC voltage for ASIC I/Os
VDDS-2                                                                                        1
VSS             -      common ground for ASIC core, I/Os and CPUs                            12
VSSPLL          -      ground dedicated to DPLL                                               1
VSSRTC          -      ground dedicated to RTC split powered                                  1
VDDANG          -      Analog VCC for clock squarer.                                          1
VSSANG          -      Analog ground for clock squarer                                        1
VSSO            -      Reference ground for 32KHz oscillator.                                 1




                                              TI- Proprietary Information –        PAGE: 35/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.            Strictly Private
Features characteristic data and other
information are subject to change.
                                               UNDER NON DISCLOSURE                DO NOT COPY
                                                    AGREEMENT
REF: CAL000                                         HERCROM400G2                     Ver 1.3


FUNCTIONAL MUXED : 23 pins                                                    Pull     Reset
nSCS2        OUT      uWIRE chip select 2.                                                1
CK16X_IRDA   OUT      IRDA mode: 16X serial transmission clock                            0
IO(15:4)     IN/OUT   Additional generic Input/Output.                                  Input
ADD(22)      OUT      ARM address bus out                                                 0
CLKX_SPI     OUT      SPI Serial clock output                                             0
SCL          OUT      I2C INTERFACE Master serial clock                                   Z
SDA          OUT      I2C INTERFACE Serial bidirectional data                             Z
LPG          OUT      Light pulse Generator LED control signal                            0
PWT          OUT      Pulse Width modulated signal for buzzer                             0
PWL          OUT      Pulse Width modulated signal for light                              0
ARMCLK       IN       ARM external clock input                                          Input
nRDYMEM      IN       Ready signal for slow peripheral

SOFTWARE DEBUG : 35 pins                 (test and validation only)           Pull     Reset
XDI_O(7:0)  OUT       DSP XI/O Data bus.                                                N/A
X_A(4:0)    OUT       DSP XI/O Address bus                                              N/A
X_IOSTRB    OUT       DSP XI/O strobe                                                     1
TOUT        OUT       DSP timer output.                                                   0
XF          OUT       DSP External flag.                                                  0
INT10n      OUT       DSP external interrupt 10 (DMA int.).                               1
INT1n       OUT       DSP external interrupt 1 (RIF transmit int.).                       1
INT4n       OUT       DSP external interrupt 4 (MCSI Tx int.).                            1
IRQ4        OUT       ARM external interrupt 4 (TPU frame int.).                          1
IACKn       OUT       DSP interrupt acknowledge.                                          1
CLKOUT_DSP  OUT       DSP clock out.                                                      0
DPLLCLK     OUT       DPLL clock out.                                                     0
MCLK        OUT       ARM clock in                                                        0
nOPC        OUT       ARM not op-code fetch                                               0
nMREQ       OUT       ARM not memory request                                              0
nWAIT       OUT       ARM not wait                                                        0
MAS(1:0)    OUT       ARM memory access size                                             10
nFIQ        OUT       ARM fast interrupts.                                                1
nIRQ        OUT       ARM normal interrupts.                                              1
nCS6        OUT       ARM chip-select RAM interne                                         1
TPU_WAIT    OUT       TPU in WAIT mode (WAIT / AT instruction)                            0
TPU_IDLE    OUT       TPU in IDLE mode (SLEEP instruction)                                1
START_BIT   OUT       Start bit detection                                                 0

                                  Table 6: CALYPSO pins description

Note: reset values are defined as follows,
        0 : driven low (output)
        1 : driven high (output)
        Z : high impedance (output disabled)
        N/A : non-applicable




                                              TI- Proprietary Information –   PAGE: 36/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
     REF: CAL000                                           HERCROM400G2                          Ver 1.3


     9.          CALYPSO BALL-OUT
     9.1         179GHH PACKAGE

Important: From Calypso C035 – F751619, in table below, replace
        C11    140                  VDD         VSS
By
          C11       140                            VDD     VSS                     nCS4




Important: From Calypso C035 – F751619, in table below, replace
       K7      63                   VDD         VSS       BU                     PWT
        L7     64                   VDD         VSS       LT                     PWL
By
          K7        63                             VDD     VSS    BU             PWT/DmaDbg
          L7        64                             VDD     VSS    LT             PWL/nEndDma

The DmaDbg mode is entered by setting bit 3&4 of ASIC_CONF_REG



          Ball     Pad         IO supply           Core    Gnd      Functional       Dual mode
          B2         1       VDDS-MIF              VDD     VSS   RnW
          C2         2                             VDD     VSS   nCS0
          C3         3                             VDD     VSS   nCS1
          B1*        4                                           VSS
          C1         5                             VDD     VSS   nCS2
          D3         6                             VDD     VSS   nCS3            INT4N
          D2         7                             VDD     VSS   CS4             ADD(22)
          D1         8                                           VDDS-MIF
          F5         9                             VDD     VSS   nBHE            IO(14)
          E4        10                             VDD     VSS   nBLE            IO(15)
          E2        11                             VDD     VSS   nFOE            X_A(3)
          E3        12                             VDD     VSS   nFWE            X_A(0)
          E1        13                                           VDD
          F4        14                             VDD     VSS   FDP             NIACK
          F3        15                             VDD     VSS   ADD(0)
          F2        16                             VDD     VSS   ADD(1)
          F1        17                                           VSS
          G5        18                             VDD     VSS   ADD(2)
          G4        19                             VDD     VSS   ADD(3)
          G2        20                             VDD     VSS   ADD(4)
          G3        21                             VDD     VSS   ADD(5)
          G1        22                                           VDDS-MIF
          H1        23                             VDD     VSS   ADD(6)
          H3        24                             VDD     VSS   ADD(7)
          H2        25                             VDD     VSS   ADD(8)



                                                     TI- Proprietary Information –         PAGE: 37/51
     PRELIMINARY          documents      contain
     information on a product under development
     and is issued for evaluation purposes only.           Strictly Private
     Features characteristic data and other
     information are subject to change.
                                                      UNDER NON DISCLOSURE                 DO NOT COPY
                                                           AGREEMENT
REF: CAL000                                          HERCROM400G2                       Ver 1.3

    H4         26                             VDD    VSS     ADD(9)
    H5         27                             VDD    VSS     ADD(10)
    J1         28                             VDD    VSS     ADD(11)
    J2         29                             VDD    VSS     ADD(12)
    J3         30                             VDD    VSS     ADD(13)
    J4         31                             VDD    VSS     ADD(14)
    K1         32                                            VSS
    K3         33                             VDD    VSS     ADD(15)
    K2         34                             VDD    VSS     ADD(16)
    K4         35                             VDD    VSS     ADD(17)
    J5         36                             VDD    VSS     ADD(18)
    L1         37                             VDD    VSS     ADD(19)
    L2         38                             VDD    VSS     ADD(20)
    L3         39                             VDD    VSS     ADD(21)      CK16X_IRDA
   M1          40         VDDS-1                    VDD
   N1*         41                             VDD     VSS    nIBOOT
   M3          42                             VDD     VSS    EXT_IRQ
   M2          43                             VDD     VSS    IDDQ
   P1*         44                             VDD     VSS    EXT_FIQ
   N2          45                             VDD     VSS    nRESET_OUT   IO(7)
   P2*         46                                   VSS
   N3          47                             VDD     VSS    IO(0)        TPU-WAIT
   P3          48                             VDD     VSS    IO(1)        TPU-IDLE
   L4          49                             VDD     VSS    IO(2)        IRQ4
   M4          50                             VDD     VSS    IO(3)        SIM_RNW
   N4          51                             VDD     VSS    KBC(0)       NFIQ
   P4          52                                   VSS
   K5          53                             VDD     VSS    KBC(1)       NIRQ
   L5          54                             VDD     VSS    KBC(2)       XDIO(0)
   N5          55                                   VDDS-1
   P5          56                             VDD     VSS    KBC(3)       XDIO(1)
   M5          57                             VDD     VSS    KBC(4)       XDIO(2)
   K6          58                             VDD     VSS    KBR(0)       XDIO(3)
   M6          59                             VDD     VSS    KBR(1)       XDIO(4)
   P6          60                             VDD     VSS    KBR(2)       XDIO(5)
   N6          61                             VDD     VSS    KBR(3)       XDIO(6)
   L6          62                             VDD     VSS    KBR(4)       XDIO(7)
   K7          63                             VDD     VSS    BU           PWT
   L7          64                             VDD     VSS    LT           PWL
   P7          65                                   VDD
   N7          66                             VDD     VSS    MCUDI
   M7          67                             VDD     VSS    MCUDO
   M8          68                             VDD     VSS    MCUEN(0)
   N8          69                                   VSS
   P8          70                             VDD     VSS    MCUEN(1)     IO(8)
   L8          71                             VDD     VSS    MCUEN(2)     IO(13)
   K8          72                             VDD     VSS    SDO          INT10n
   L9          73                             VDD     VSS    nSCS0        SCL
   N9          74                             VDD     VSS    nSCS1        X_A(2)
   P9          75                             VDD     VSS    SCLK         INT1n


                                               TI- Proprietary Information –        PAGE: 38/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                                UNDER NON DISCLOSURE                DO NOT COPY
                                                     AGREEMENT
REF: CAL000                                          HERCROM400G2                    Ver 1.3

   M9          76                             VDD     VSS   SDI         SDA
   K9          77                             VDD     VSS   MCSI_FSYNCH IO(12)
  M10          78                             VDD     VSS   MCSI_RXD    IO(10)
  P10          79                                   VSS
  N10          80                             VDD     VSS   MCSI_CLK    IO(11)
  L10          81                             VDD     VSS   MCSI_TXD    IO(9)
  K10          82                             VDD     VSS   BDR
  P11          83                             VDD     VSS   BCLKR       ARMCLK
  N11          84                             VDD     VSS   BCLKX       IO(6)
  M11          85                             VDD     VSS   BDX
  L11          86                             VDD     VSS   BFSR
  P12          87                             VDD     VSS   BFSX
  N12          88                             VDD     VSS   VCLKRX
  P13*         89                                           VSS
  N13          90                             VDD    VSS    VDR
  P14*         91                             VDD    VSS    VDX
  M13          92                             VDD    VSS    VFSRX
  M12          93                             VDD    VSS    TSPACT(0)
  N14*         94                                           VDD
  M14          95                             VDD    VSS    TSPACT(1)
  L12          96                             VDD    VSS    TSPACT(2)
  L13          97                             VDD    VSS    TSPACT(3)
  L14          98                                           VDDS-1
  J10          99                             VDD    VSS    TSPACT(4)   NREADYMEM
  K11         100                             VDD    VSS    TSPACT(5)   DPLLCLK
  K13         101                             VDD    VSS    TSPACT(6)   NCS6
  K12         102                             VDD    VSS    TSPACT(7)   CLKX_SPI
  K14         103         VDDS-1              VDD    VSS    TSPACT(8)   NMREQ
  J11         104                             VDD    VSS    TSPACT(9)   MAS(1)
  J12         105                             VDD    VSS    TSPACT(10)  NWAIT
  J13         106                             VDD    VSS    TSPACT(11)  MCLK
  J14         107                             VDD    VSS    TSPCLKX
  H10         108                             VDD    VSS    TSPDI       IO(4)
  H11         109                             VDD    VSS    TSPDO
  H13         110                             VDD    VSS    TSPEN(0)
  H12         111                             VDD    VSS    TSPEN(1)
  H14         112                             VDD    VSS    TSPEN(2)
  G14         113                                           VSS
  G12         114                             VDD    VSS    TSPEN(3)    NSCS2
  G13         115                             VDD    VSS    SIM_IO
  G11         116                             VDD    VSS    SIM_CD      MAS(0)
  G10         117                             VDD    VSS    SIM_RST
  F14         118                             VDD    VSS    SIM_PWRCTRL IO(5)
  F13         119                             VDD    VSS    SIM_CLK
  F12         120                             VDD    VSS    CLK13M_OUT START_BIT
  F11         121                                           VDDPLL
  E14         122                                           VSSPLL




                                               TI- Proprietary Information –     PAGE: 39/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                                UNDER NON DISCLOSURE             DO NOT COPY
                                                     AGREEMENT
REF: CAL000                                               HERCROM400G2                     Ver 1.3


   E12        123                                       VSSANG
   E13        124        VDDANG           VDDANG VSSANG CLKTCXO
   E11        125                                       VDDANG
   F10        126                         VDDRTC VSSRTC ON_OFF
   D14        127                                                VDDRTC
   D13        128                                       VDDS-RTC
   D12        129                         VDDRTC VSSRTC RESPWRONZ
   C14        130                                       VSSRTC
                       VDDS-RTC
  B14*        131                         VDDRTC VSSRTC IT_WAKEUP
   C12        132                         VDDRTC VSSRTC CLK32K_OUT
   C13        133                         VDDRTC          VSSO   OSC32K_IN
  A14*        134                                                VSSO
   B13        135                         VDDRTC          VSSO   OSC32K_OUT
  A13*        136                             VDD         VSS    RFEN         NOPC
   B12        137                                                VDD
   A12        138                             VDD         VSS    TCXOEN
   D11        139                             VDD         VSS    nBSCAN
   C11        140                             VDD         VSS
   B11        141                             VDD         VSS    nEMU0
   A11        142                                                VDDS-2
   E10        143                             VDD         VSS    nEMU1
   D10        144                             VDD         VSS    TDI
   B10        145                             VDD         VSS    TCK
   A10        146                                   VSS          VSS
   C10        147         VDDS-2              VDD         VSS    TDO
    E9        148                             VDD         VSS    TMS
    C9        149                             VDD         VSS    CTS_MODEM    XF
    A9        150                             VDD         VSS    RX_MODEM
    B9        151                             VDD         VSS    TX_MODEM     XIOSTRBN
    D9        152                             VDD         VSS    DSR_MODEM    LPG
    E8        153                             VDD         VSS    RTS_MODEM    TOUT
    D8        154                             VDD         VSS    RX_IRDA
    A8        155                             VDD         VSS    RXIR_IRDA    X_A(1)
    B8        156                             VDD         VSS    SD_IRDA      CLKOUT_DSP
    C8        157                             VDD         VSS    TX_IRDA
    C7        158                             VDD         VSS    TXIR_IRDA    X_A(4)




                                               TI- Proprietary Information –           PAGE: 40/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.                Strictly Private
Features characteristic data and other
information are subject to change.
                                                UNDER NON DISCLOSURE                   DO NOT COPY
                                                     AGREEMENT
REF: CAL000                                          HERCROM400G2                  Ver 1.3



   B7         159                             VDD    VSS   DATA(0)
   A7         160                                          VSS
   D7         161                             VDD    VSS   DATA(1)
   E7         162                             VDD    VSS   DATA(2)
   D6         163                             VDD    VSS   DATA(3)
   B6         164                                          VDDS-MIF
   A6         165                             VDD    VSS   DATA(4)
   C6         166                             VDD    VSS   DATA(5)
   E6         167                             VDD    VSS   DATA(6)
   C5         168                             VDD    VSS   DATA(7)
   A5         169       VDDS-MIF                           VDD
   B5         170                             VDD    VSS   DATA(8)
   D5         171                             VDD    VSS   DATA(9)
   E5         172                             VDD    VSS   DATA(10)
   A4         173                                          VDDS-MIF
   B4         174                             VDD    VSS   DATA(11)
   C4         175                             VDD    VSS   DATA(12)
   D4         176                             VDD    VSS   DATA(13)
   A3         177                             VDD    VSS   DATA(14)
   B3         178                             VDD    VSS   DATA(15)
   A2*        179                                          VSS




                                     Table 7: CALYPSO ball mapping




                                               TI- Proprietary Information –   PAGE: 41/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.          Strictly Private
Features characteristic data and other
information are subject to change.
                                                UNDER NON DISCLOSURE           DO NOT COPY
                                                     AGREEMENT
      REF: CAL000                                         HERCROM400G2                  Ver 1.3



      10.        PAD I/O BUFFER TYPE

Important: From Calypso C035 – F751619, in table below, replace
   C11      140 PWR xxxAVAILABLExxx IDI091
By
     C11        140       PWR        nCS4                          OUI831



      PAD Definition                                     BUFFER TYPE
      Ball Finger PAD PAD_name                           Input_Pad Output_Pad
       B2     1   IO  RnW                                IDI041    OUI831
       C2     2   O   nCS0                                         OUI831
       C3     3   O   nCS1                                         OUI831
       B1*    4   PWR VSS                                *power*   *power*
       C1     5   O   nCS2                                         OUI831
       D3     6   IO  nCS3                               IDI041    OUI831
       D2     7   IO  CS4                                IDI041    OUI831
       D1     8   PWR VDDS-MIF                           *power*   *power*
       F5     9   IO  nBHE                               IDI041    OUI831
       E4    10 IO    nBLE                               IDI041    OUI831
       E2    11 O     nFOE                                         OUI831
       E3    12 IO    nFWE                               IDI041    OUK831
       E1    13 PWR VDD                                  *power*   *power*
       F4    14 IO    FDP                                IDI041    OUO431
       F3    15 O     ADD_15_0(0)                                  OUI831
       F2    16 O     ADD_15_0(1)                                  OUI831
       F1    17 PWR VSS                                  *power*   *power*
       G5    18 O     ADD_15_0(2)                                  OUI831
       G4    19 O     ADD_15_0(3)                                  OUI831
       G2    20 O     ADD_15_0(4)                                  OUI831
       G3    21 O     ADD_15_0(5)                                  OUI831
       G1    22 PWR VDDS-MIF                             *power*   *power*
       H1    23 O     ADD_15_0(6)                                  OUI831
       H3    24 O     ADD_15_0(7)                                  OUI831
       H2    25 O     ADD_15_0(8)                                  OUI831
       H4    26 O     ADD_15_0(9)                                  OUI831
       H5    27 O     ADD_15_0(10)                                 OUI831
       J1    28 O     ADD_15_0(11)                                 OUI831
       J2    29 O     ADD_15_0(12)                                 OUI831
       J3    30 O     ADD_15_0(13)                                 OUI831
       J4    31 O     ADD_15_0(14)                                 OUI831
       K1    32 PWR VSS                                  *power*   *power*
       K3    33 O     ADD_15_0(15)                                 OUI831
       K2    34 IO    ADD_20_16(16)                      IDI041    OUI831
       K4    35 IO    ADD_20_16(17)                      IDI041    OUI831
       J5    36 IO    ADD_20_16(18)                      IDI041    OUI831

                                                    TI- Proprietary Information –   PAGE: 42/51
      PRELIMINARY          documents      contain
      information on a product under development
      and is issued for evaluation purposes only.         Strictly Private
      Features characteristic data and other
      information are subject to change.
                                                    UNDER NON DISCLOSURE            DO NOT COPY
                                                         AGREEMENT
REF: CAL000                                         HERCROM400G2                  Ver 1.3

  L1        37     IO  ADD_20_16(19)               IDI041    OUI831
  L2        38     IO  ADD_20_16(20)               IDI041    OUI831
  L3        39     O   ADD_21                                OUI831
 M1         40     PWR VDD                         *power*   *power*
 N1*        41     I   nIBOOT                      IDI091
 M3         42     I   EXT_IRQ                     IDI091
 M2         43     I   IDDQ                        IDI091
 P1*        44     I   EXT_FIQ                     IDI091
 N2         45     IO  nRESET_OUT                  IDI041    OUO231
 P2*        46     PWR VSS                         *power*   *power*
 N3         47     IO  IO(0)                       IDI041    OUO431
 P3         48     IO  IO(1)                       IDI041    OUO431
  L4        49     IO  IO(2)                       IDI041    OUO431
 M4         50     IO  IO(3)                       IDI041    OUO431
 N4         51     IO  KBC(0)                      IDI041    OUO431
 P4         52     PWR VSS                         *power*   *power*
 K5         53     IO  KBC(1)                      IDI041    OUO431
  L5        54     IO  KBC(2)                      IDI041    OUO431
 N5         55     PWR VDDS-1                      *power*   *power*
 P5         56     IO  KBC(3)                      IDI041    OUO431
 M5         57     IO  KBC(4)                      IDI041    OUO431
 K6         58     IO  KBR(0)                      IDI041    OUO231
 M6         59     IO  KBR(1)                      IDI041    OUO231
 P6         60     IO  KBR(2)                      IDI041    OUO231
 N6         61     IO  KBR(3)                      IDI041    OUO231
  L6        62     IO  KBR(4)                      IDI041    OUO231
 K7         63     O   BU                                    OUK831
  L7        64     O   LT                                    OUK431
 P7         65     PWR VDD                         *power*   *power*
 N7         66     I   MCUDI                       IDI041
 M7         67     O   MCUDO                                 OUK431
 M8         68     IO  MCUEN(0)                    IDI041    OUK431
 N8         69     PWR VSS                         *power*   *power*
 P8         70     IO  MCUEN(1)                    IDI041    OUK431
  L8        71     IO  MCUEN(2)                    IDI041    OUK431
 K8         72     O   SDO                                   OUK431
  L9        73     O   nSCS0                                 OUK431
 N9         74     O   nSCS1                                 OUK431
 P9         75     O   SCLK                                  OUI431
 M9         76     IO  SDI                         IDI041    OUO431
 K9         77     IO  MCSI_FSYNCH                 IDI041    OUK431
 M10        78     IO  MCSI_RXD                    IDI041    OUK431
 P10        79     PWR VSS                         *power*   *power*
 N10        80     IO  MCSI_CLK                    IDI041    OUI431
 L10        81     IO  MCSI_TXD                    IDI041    OUK431
 K10        82     I   BDR                         IDI041
 P11        83     I   BCLKR                       IDI091
 N11        84     IO  BCLKX                       IDI091    OUI431
 M11        85     O   BDX                                   OUK431


                                              TI- Proprietary Information –   PAGE: 43/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.          Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                  Ver 1.3

 L11        86     I   BFSR                        IDI041
 P12        87     IO  BFSX                        IDI041    OUK431
 N12        88     I   VCLKRX                      IDI091
 P13*       89     PWR VSS                         *power*   *power*
 N13        90     I   VDR                         IDI041
 P14*       91     O   VDX                                   OUK431
 M13        92     I   VFSRX                       IDI041
 M12        93     IO  TSPACT_4_0(0)               IDI041    OUO231
 N14*       94     PWR VDD                         *power*   *power*
 M14        95     IO  TSPACT_4_0(1)               IDI041    OUO231
 L12        96     IO  TSPACT_4_0(2)               IDI041    OUO231
 L13        97     IO  TSPACT_4_0(3)               IDI041    OUO231
 L14        98     PWR VDDS-1                      *power*   *power*
 J10        99     IO  TSPACT_4_0(4)               IDI041    OUO231
 K11       100     O   TSPACT_12_5(5)                        OUO231
 K13       101     O   TSPACT_12_5(6)                        OUO231
 K12       102     O   TSPACT_12_5(7)                        OUO231
 K14       103     O   TSPACT_12_5(8)                        OUO231
 J11       104     O   TSPACT_12_5(9)                        OUO231
 J12       105     O   TSPACT_12_5(10)                       OUO231
 J13       106     O   TSPACT_12_5(11)                       OUO431
 J14       107     O   TSPCLKX                               OUI431
 H10       108     IO  TSPDI                       IDI041    OUK431
 H11       109     IO  TSPDO                       IDI041    OUK431
 H13       110     O   TSPEN_0                               OUK431
 H12       111     IO  TSPEN_3_1(1)                IDI041    OUK431
 H14       112     IO  TSPEN_3_1(2)                IDI041    OUK431
 G14       113     PWR VSS                         *power*   *power*
 G12       114     IO  TSPEN_3_1(3)                IDI041    OUK431
 G13       115     IO  SIM_IO                      IDI041    OUO431
 G11       116     IO  SIM_CD                      IDI091    OUO431
 G10       117     IO  SIM_RST                     IDI041    OUO431
 F14       118     IO  SIM_PWRCTRL                 IDI041    OUO431
 F13       119     IO  SIM_CLK                     IDI041    OUO431
 F12       120     IO  CLK13M_OUT                  IDI041    OUI431
 F11       121     PWR VDDPLL                      *power*   *power*
 E14       122     PWR VSSPLL                      *power*   *power*
 E12       123     PWR VSSANG                      *power*   *power*
 E13       124     I   CLKTCXO                     IOH121
 E11       125     PWR VDDANG                      *power*   *power*
 F10       126     I   ON_OFF                      IDI091
 D14       127     PWR VDDRTC                      *power*   *power*
 D13       128     PWR VDDS-RTC                    *power*   *power*
 D12       129     I   nRESPWRON                   IDI091
 C14       130     PWR VSSRTC                      *power*   *power*
 B14*      131     O   IT_WAKEUP                             OUK431
 C12       132     O   CLK32K_OUT                            OUK431
 C13       133     I   OSC32K_IN                   OS0171
 A14*      134     PWR VSSO                        *power*   *power*


                                              TI- Proprietary Information –   PAGE: 44/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.          Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                  Ver 1.3

 B13       135     I   OSC32K_OUT                  UOS266
 A13*      136     O   RFEN                                  OUK431
 B12       137     PWR VDD                         *power*   *power*
 A12       138     O   TCXOEN                                OUO431
 D11       139     I   NBSCAN                      IDI041
 C11       140     PWR xxxAVAILABLExxx             IDI091
 B11       141     IO  nEMU0                       IDI091    OUO231
 A11       142     PWR VDDS-2                      *power*   *power*
 E10       143     IO  nEMU1                       IDI091    OUO231
 D10       144     I   TDI                         IDI091
 B10       145     I   TCK                         IDI091
 A10       146     PWR VSS                         *power*   *power*
 C10       147     O   TDO                                   OUK431
  E9       148     I   TMS                         IDI091
  C9       149     IO  CTS_MODEM                   IDI091    OUO231
  A9       150     IO  RX_MODEM                    IDI091    OUO231
  B9       151     O   TX_MODEM                              OUK831
  D9       152     IO  DSR_MODEM                   IDI091    OUK831
  E8       153     O   RTS_MODEM                             OUK831
  D8       154     I   RX_IRDA                     IDI091
  A8       155     IO  RXIR_IRDA                   IDI091    OUK431
  B8       156     O   SD_IRDA                               OUK431
  C8       157     IO  TX_IRDA                     IDI041    OUK831
  C7       158     IO  TXIR_IRDA                   IDI041    OUK831
  B7       159     IO  DATA(0)                     IDI041    OUI831
  A7       160     PWR VSS                         *power*   *power*
  D7       161     IO  DATA(1)                     IDI041    OUI831
  E7       162     IO  DATA(2)                     IDI041    OUI831
  D6       163     IO  DATA(3)                     IDI041    OUI831
  B6       164     PWR VDDS-1                      *power*   *power*
  A6       165     IO  DATA(4)                     IDI041    OUI831
  C6       166     IO  DATA(5)                     IDI041    OUI831
  E6       167     IO  DATA(6)                     IDI041    OUI831
  C5       168     IO  DATA(7)                     IDI041    OUI831
  A5       169     PWR VDD                         *power*   *power*
  B5       170     IO  DATA(8)                     IDI041    OUI831
  D5       171     IO  DATA(9)                     IDI041    OUI831
  E5       172     IO  DATA(10)                    IDI041    OUI831
  A4       173     PWR VDDS-1                      *power*   *power*
  B4       174     IO  DATA(11)                    IDI041    OUI831
  C4       175     IO  DATA(12)                    IDI041    OUI831
  D4       176     IO  DATA(13)                    IDI041    OUI831
  A3       177     IO  DATA(14)                    IDI041    OUI831
  B3       178     IO  DATA(15)                    IDI041    OUI831
  A2       179     PWR VSS                         *power*   *power*

                           Table 8: I/O buffer type in 179 balls version




                                              TI- Proprietary Information –   PAGE: 45/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.          Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
             REF: CAL000                                              HERCROM400G2                            Ver 1.3



             11.        APPLICATION EXAMPLE

             11.1         Power-supplies connection

             11.1.1          Compatibility schematic Nausica CALYPSO

                                                           CLK13M      2.8V          CLK13M_OUT
             NAUSICA
                                                                                     VDDS_MIF
                                                           VR2
                         VR2                                                         VDD , VDDPLL
                                                               10uF
                                        VR2_BU                                                           CALYPSO
                                                           VR2B         2.8V       VDDS1,VDDS2,VDDANG
     VCC2
                                                               10uF
                                        VR2B_BU                        VDDS-RTC                Split Power ring
     0.2 Ω                                                 VR1
                                                              1.8V     VDD-RTC
      VBAT              VR1                                                               RTC & SPLIT DOMAIN
                                                               10uF
                                        VR1_BU
MB
                                                                                                     DATA               &
                  BBS                                      UPR                                       CONTROL
   VBACKU
        P                                                      1uF                                                Powerdown of internal
                                                                                                                  level shifter
BUB
                                                                                          Split
                                                           2.8V I/O ON_nOFF               Power           G2_ON_nOFF
                                                                                          Logic
                                                                                                          G2_RESPWONZ

                       ABB                                 2.8V I/O RESPWONZ
                     VRPC core                                                                    RTC and XO32K
                                                                                                  reset


                                                           2.8V I/O IT_WAKEUP            IT_WAKE_FROM_SLEEP

                                                                                          RTC_ALARM


                                                                                   XO32K          Conf
                                                                      XO32K_OUT    registers
                                                                                     XO32K




                                    Figure 2: Compatibility connection with Nausica


             Restrictions :

                      1/ CALYPSO could be used with NAUSICA without using the CALYPSO split power
             capability.
                      2/ It is not possible to use 1.8V external memory on CALYPSO when using a NAUSICA.

                                                             TI- Proprietary Information –            PAGE: 46/51
             PRELIMINARY          documents      contain
             information on a product under development
             and is issued for evaluation purposes only.              Strictly Private
             Features characteristic data and other
             information are subject to change.
                                                              UNDER NON DISCLOSURE                     DO NOT COPY
                                                                   AGREEMENT
    REF: CAL000                                            HERCROM400G2                                       Ver 1.3


    11.1.2          IOTA to power split DBB supply connections

                              ABB                     RSIM
 VBAT                                               1.8/2.9 V         VRSIM
                                                      10 mA                                     SIM CARD
                                                  SLP                    1uF


                   VCRAM                             RRAM
                                                    1.8/2.8 V         VRRAM
                                                     50 mA                                        Ext SRAM
                                                  SLP                    4.7uF


                                                      RMEM
                   VCMEM                             1.8/2.8 V        VRMEM
                                                      60 mA                                        FLASH
                                                  SL                     4.7uF

        Sel 2.8V   VLMEM                                                            VDDS-MIF                   MEMORY
        Sel 1.8V                                                                                             INTERFACE,
                                                                     VSDBB
                                                      RDBB
                                                  1.3/1.5/1.8V                           VDD
                   VCDBB                                             VRDBB               VDDPLL
                                                       120 mA                                           All ASIC modules,
                                                  SL                     10uF                                 ARM7,
                                                                                                              LMM,
                                                          VLRTC
                                                                                                             Internal SRAM
                    VCIO1                                             VRIO1                                        PLL
                                                        RIO
                                                                                          VDDS1
                    VCIO2                              2.8 V
                                                                      VRIO2               VDDS2
                                                  SL 100 mA                                                   IOTA I/O,
                    VCABB                                                 10uF                                 RF I/O,
                                                     ABB                                                     13 MHz_IN
                                                  Digital core                                               13MHz_Out
                    VBAT
                                                       io

                                                     ABB
                                                  Analog core
                                                                                                      DBB split power

                                                      RABB           VRABB •
                                                      2.8 V
                                                                          2.2uF
                                                   SL 80 mA
                                                                      UPR •
                VBACKUP              BBS                                                             Power split domain
                                                                          1uF

BACKUP                                                                              Sel 1.8V
                                                                     VLRTC

                                                                         Sel 1.5V                                RTC
                                                                                                                XO32K
                                ABB                    RRTC           VRRTC                                     RTC I/O
                                                                                      VDD-RTC
                                                   1.3/1.5/1.8V                                              (IT_WAKEUP,
                              VRPC core
                                                                                                             nRESPWONZ
                                                  BK 10 uA                1uF
                                                                                                             , ON_nOFF)
                                                                                    VDDS-RTC
BK          : regulators ON in BACKUP/ SLEEP / NORMAL mode
SL          : regulators ON in SLEEP / NORMAL mode
SLP         : idem SL + reverse current protection
NM          : regulators ON in NORMAL mode
•           :




                                                  TI- Proprietary Information –                      PAGE: 47/51
    PRELIMINARY          documents      contain
    information on a product under development
    and is issued for evaluation purposes only.                Strictly Private
    Features characteristic data and other
    information are subject to change.
                                                  UNDER NON DISCLOSURE                               DO NOT COPY
                                                       AGREEMENT
          REF: CAL000                                               HERCROM400G2                            Ver 1.3


          11.1.3          Power Supplies connection with IOTA ABB




                       IOTA                                                        G2
                                                                      Vdds_bu                  Split Power ring
                               RRTC                VRRTC              Vdd_bu
                           1.18/1.5/1.8 V                                                RTC & SPLIT DOMAIN
   VBAT                        10 uA                      1uF


                                                                                                    DATA & CONTROL
                                                        UPR
VBACKUP    BBS
                                                                                                                  Powerdown of internal
                                                                                                                      level shifter

                                                                                            Split
                                                        ON_nOFF       ON_nOFF              Power          G2_ON_nOFF
                                                                                           Logic
                                                                                                          G2_RESPWONZ


                                                               RESPWONZ                         RTC and XO32K reset
                ABB
              VRPC core

                                                               IT_WAKEUP                 IT_WAKE_FROM_SLEEP

                                                                                         RTC_ALARM


                                                                                 XO32K Conf registers

                                                    CK32K          XO32K_OUT
                                                                                        XO32K




                             Figure 3: Power Supplies connection with IOTA ABB




                                                              TI- Proprietary Information –             PAGE: 48/51
          PRELIMINARY          documents      contain
          information on a product under development
          and is issued for evaluation purposes only.               Strictly Private
          Features characteristic data and other
          information are subject to change.
                                                              UNDER NON DISCLOSURE                      DO NOT COPY
                                                                   AGREEMENT
REF: CAL000                                         HERCROM400G2                  Ver 1.3


11.2         32KHz quartz connection

See Technical memo 32pcbguide.doc




                                              TI- Proprietary Information –   PAGE: 49/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.         Strictly Private
Features characteristic data and other
information are subject to change.
                                              UNDER NON DISCLOSURE            DO NOT COPY
                                                   AGREEMENT
REF: CAL000                                              HERCROM400G2                         Ver 1.3


11.3         SIM card connection
The SIM card or the SIM level-shifters are connected to the chip using:
- SIM_CLK SIM card reference clock
- SIM_RST SIM card async/sync reset
- SIM_IO       SIM card bidirectional data line
- SIM_PWCTRL SIM card power activation
- SIM_RnW SIM card data line direction
- SIM_CD       SIM card presence detection

The SIM_RnW signal is only used with some external devices to control level-shifters direction.
The SIM_CD signal should be connected to the SIM card reader in order to get SIM_CD at low
level (GND) when no card is present and at high level (VDDS2) when a card is present.
The SIM_PWCTRL signal can be used for a direct control of the SIM card power-supply.
However, this control is S/W programmable through the MCU serial port when using the
NAUSICA chip for the SIM card interface.




               SIM_PWCTRL                                     SVDD

                                          20K
                                                    ABB                     20K
                                                                                        VDD
                       SIM_IO                    SIO3          SIO5               IO
                                                                                  CLK
                                                                                        CARD
                    SIM_CLK                      SCLK3       SCLK5
                    SIM_RST                      SRST3        SRST5               RST




                     SIM_CD




                                       Figure 4: SIM card connection




                                                TI- Proprietary Information –           PAGE: 50/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.              Strictly Private
Features characteristic data and other
information are subject to change.
                                                UNDER NON DISCLOSURE                    DO NOT COPY
                                                     AGREEMENT
REF: CAL000                                           HERCROM400G2                 Ver 1.3


11.4         Keyboard connection
The keyboard is connected to the chip using:
- KBR (4:0) input pins for row lines
- KBC (4:0) output pins for column lines

If a key button of the keyboard matrix is pressed, the corresponding row and column lines are
shorted together.
To allow key press detection, all input pins (KBR) are pulled up to VCC and all output pins
(KBC) are driving a low level. Any action on a button will generate an interrupt to the micro-
controller which will, as answer, scan the column lines with the sequence describe below.
This sequence is written to allow detection of simultaneous press actions on several key
buttons.

               RESET           IDLE                        KEYBOARD SCANNING                     IDLE
 KBC(0)          1               0            1        0       1       1       1       1           0
 KBC(1)          1               0            1        1       0       1       1       1           0
 KBC(2)          1               0            1        1       1       0       1       1           0
 KBC(3)          1               0            1        1       1       1       0       1           0
 KBC(4)          1               0            1        1       1       1       1       0           0


                               Table 9: Keyboard scanning sequence




                                      Figure 5: Keyboard connection


                                              TI- Proprietary Information –    PAGE: 51/51
PRELIMINARY          documents      contain
information on a product under development
and is issued for evaluation purposes only.           Strictly Private
Features characteristic data and other
information are subject to change.
                                                  UNDER NON DISCLOSURE         DO NOT COPY
                                                       AGREEMENT
