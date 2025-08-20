# GPIB Controller
## A Low-Cost Solution for GPIB Communication

Turn a cheap clone Arduino Nano V3 running at 5 volts into an HP GPIB controller. No Visa, no Prologix, no National Instruments card, and probably most importantly, almost no money.

There are limitations; I have personally only tested it on a single instrument bus. However, since the controller is inexpensive, if it can't handle all your instruments, you can simply add another. The major limiting factor is the lack of pull-up resistors, but you could add 16 5.1k pull-up resistors to communicate with more instruments.

You'll need a male 57 series 24 pin Centronics Connector such as [this one from AliExpress](https://www.aliexpress.us/item/3256806916172520.html) or [this one from Digikey](https://www.digikey.com/en/products/detail/t/294004), or you can use an old parallel printer cable and a Dremel to cut it down to 24 pins.

## Hardware
![image of pinouts see header for pin numbers](https://github.com/Chisight/GPIBnano/blob/main/Docs/Nano_Pins.png)
Ensure that all grounds are connected; do not leave them floating. My connector does not have a chassis ground, so that can probably float when connecting only one instrument. My ribbon cable is under a foot long, and I have not tested longer cables. When connecting cable shielding to a shell, connect only at one end.

## Installation
```bash
cd ~/Arduino/libraries/
git clone https://github.com/Chisight/GPIBnano
```
open the example in the IDE and burn it to your nano v3.

Open the example in the IDE and burn it to your Nano V3. I used Arduino IDE 2.3.6, but any 2.x version should work. 

If you have an older Nano, remember to select Tools/Processor: ATmega328P (Old Bootloader) or it will fail to connect. 
It should also work on an Arduino Mega or other boards with an AVR in the ATmega group that have enough available GPIO pins. If you use another board, remember to set your pin numbers in the header; they are optimized for easy soldering on the Nano V3.


## Serial Command Reference

All commands must begin with a '*' and be terminated with a newline (Enter) or comma (,).
Commands may be chained comma delimited on one line to the limit of MAX_COMMAND_LENGTH.

  - *INIT <addr>: Sends an Interface Clear (IFC), enables remote control (REN),
    and sends UNLISTEN and UNTALK commands.
  - *WRITE <string>: Sends a string to the bus, asserting EOI on the last character.
    including setting the talker and listener.
  - *LISTEN: Configures the bus by commanding the previously initialized device
    to TALK, then puts the controller into a listening state to receive a data
    string. The operation completes when EOI is detected or after a timeout.

The shell script in examples/gpib_nano_v3 shows controlling an HP 3456A for a single read of 4-wire resistance.  Beyond the first read, you really only need `*WRITE T3,*LISTEN`.

If there is enough interest, I may add the ability to use Prologix commands so as to support other existing software but for now this simple interface meets my needs.

