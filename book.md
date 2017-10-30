<h1>Embedded development with Ivory Tower</h1>
$$\forall dev \in embedded . dev \Join ivory = \heartsuit$$

# Overview

## Target board
## Ivory

http://ivorylang.org/


## Tower

http://ivorylang.org/tower-overview.html

# Environment setup

## Stack

Install stack first according to
[theses instructions](http://docs.haskellstack.org/en/stable/install_and_upgrade/).

## Repositories

* [ivory](https://github.com/GaloisInc/ivory/)
* [tower](https://github.com/GaloisInc/tower/)
* [ivory-tower-stm32](https://github.com/GaloisInc/ivory-tower-stm32/)
* [smaccmpilot-stm32f4](https://github.com/GaloisInc/smaccmpilot-stm32f4/)
* [smaccmpilot-build](https://github.com/GaloisInc/smaccmpilot-build)
* [gidl](https://github.com/GaloisInc/gidl)

```bash
mkdir embedded
cd embedded
git clone  https://github.com/GaloisInc/ivory/
git clone  https://github.com/GaloisInc/tower/
git clone https://github.com/GaloisInc/ivory-tower-stm32/
```

## BlackMagic probe

To see device states during the process it is useful to open a second terminal and run
```
dmesg -Hw
```

Next build firmware for stlink target and upload it to target
```bash
# make PROBE_HOST=stlink
# flashing blackmagic_dfu via black magic probe
git clone https://github.com/blacksphere/blackmagic/
cd blackmagic
make PROBE_HOST=stlink
arm-none-eabi-gdb --ex 'target extended-remote /dev/ttyACM0' src/blackmagic_dfu
# in GDB
monitor swdp_scan
attach 1
monitor option erase
# restart target so it unlocks, repeat up to 'monitor option erase' then issue
load
# restart target, it should boot to black magic dfu
# [  +0.000002] usb 1-1.2: Product: Black Magic (Upgrade) for STLink/Discovery, (Firmware v1.6-rc0-257-gd6e2977)
# upload black magic via dfu
#
# disconnect your programmer so only target board is connected
#
sudo dfu-util -s 0x08002000:leave -D src/blackmagic.bin
# if dfu-util can't find your device you can specify -S <serial> from dmesg output
sudo dfu-util -S 7EBA7BA4 -s 0x08002000:leave -D src/blackmagic.bin
```

Sample session:
```bash
$ arm-none-eabi-gdb --ex 'target extended-remote /dev/ttyACM0' src/blackmagic_dfu
Reading symbols from src/blackmagic_dfu...done.
Remote debugging using /dev/ttyACM0
(gdb) monitor swdp_scan
Target voltage: unknown
Available Targets:
No. Att Driver
 1      STM32F1 medium density
(gdb) attach 1
Attaching to program: /home/rmarko/embedded/blackmagic/src/blackmagic_dfu, Remote target
0x08010064 in ?? ()
(gdb) monitor option erase
0x1FFFF800: 0x0000
0x1FFFF802: 0x0000
0x1FFFF804: 0x0000
0x1FFFF806: 0x0000
0x1FFFF808: 0x0000
0x1FFFF80A: 0x0000
0x1FFFF80C: 0x0000
0x1FFFF80E: 0x0000
(gdb) load
Error erasing flash with vFlashErase packet
# ^^ target needs rebooting to unlock flash
(gdb) quit

# <TARGET REBOOTED>

$ arm-none-eabi-gdb --ex 'target extended-remote /dev/ttyACM0' src/blackmagic_dfu
Reading symbols from src/blackmagic_dfu...done.
Remote debugging using /dev/ttyACM0
(gdb) monitor swdp_scan
Target voltage: unknown
Available Targets:
No. Att Driver
 1      STM32F1 medium density
(gdb) attach 1
Attaching to program: /home/rmarko/embedded/blackmagic/src/blackmagic_dfu, Remote target
0xfffffffe in ?? ()
(gdb) load
Loading section .text, size 0x1c48 lma 0x8000000
Loading section .data, size 0x90 lma 0x8001c48
Start address 0x8001498, load size 7384
Transfer rate: 11 KB/sec, 820 bytes/write.
(gdb) quit

# <TARGET REBOOTED>

$ dmesg
[691195.851258] usb 1-1.6: new full-speed USB device number 112 using ehci-pci
[691195.943994] usb 1-1.6: New USB device found, idVendor=1d50, idProduct=6017
[691195.943997] usb 1-1.6: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[691195.943999] usb 1-1.6: Product: Black Magic (Upgrade) for STLink/Discovery, (Firmware v1.6.1-98-g9a5b31c)
[691195.944001] usb 1-1.6: Manufacturer: Black Sphere Technologies
[691195.944002] usb 1-1.6: SerialNumber: 7EBA7BA4

$ sudo dfu-util -S 7EBA7BA4 -s 0x08002000:leave -D src/blackmagic.bin
dfu-util 0.9

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2016 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

dfu-util: Invalid DFU suffix signature
dfu-util: A valid DFU suffix will be required in a future dfu-util release!!!
Opening DFU capable USB device...
ID 1d50:6017
Run-time device DFU version 011a
Claiming USB DFU Interface...
Setting Alternate Setting #0 ...
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 011a
Device returned transfer size 1024
DfuSe interface name: "Internal Flash   "
Downloading to address = 0x08002000, size = 60660
Download	[=========================] 100%        60660 bytes
Download done.
File downloaded successfully
Transitioning to dfuMANIFEST state

# <TARGET REBOOTED>

$ dmesg
[691227.339977] usb 1-1.6: new full-speed USB device number 113 using ehci-pci
[691227.432954] usb 1-1.6: New USB device found, idVendor=1d50, idProduct=6018
[691227.432956] usb 1-1.6: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[691227.432957] usb 1-1.6: Product: Black Magic Probe (STLINK), (Firmware v1.6.1-98-g9a5b31c)
[691227.432958] usb 1-1.6: Manufacturer: Black Sphere Technologies
[691227.432959] usb 1-1.6: SerialNumber: 7EBA7BA4
[691227.433654] cdc_acm 1-1.6:1.0: ttyACM0: USB ACM device
[691227.434290] cdc_acm 1-1.6:1.2: ttyACM1: USB ACM device
```

### Usage

```bash
arm-none-eabi-gdb --ex 'target extended-remote /dev/ttyACM0'
# Use following commands when in gdb
monitor swdp_scan
attach 1
```

Sample session:

```bash
(gdb) monitor swdp_scan
Target voltage: unknown
Available Targets:
No. Att Driver
1      STM32F4xx
(gdb) attach 1
Attaching to Remote target
Error while running hook_stop:
Invalid type combination in equality test.
0x080035c2 in ?? ()
(gdb) bt
#0  0x080035c2 in ?? ()
#1  0x08001032 in ?? ()
#2  0x08001032 in ?? ()
Backtrace stopped: previous frame identical to this frame (corrupt stack?)
```

To access serial bridge:
```bash
screen /dev/ttyACM1 115200
```

### UART pass-through

To enable `UART` bridge from F4 discovery board to BMP enabled programmer
you need to solder wires between `UART2` (`PA2/PA3`) on `STM32F407`
to pins `PA2/PA3` on `STM32F103`. These are located in the corners of the chips.
Take care when soldering `PA3` on F407 as it's
positioned near `VSS`, if you manage to short these pins try lifting `PA3`
from the pad completely and then soldering wire directly to it.

## UDev
For persistent device names you should create udev rules files in `/etc/udev/rules.d/`. For
example to create rules for your F4 Discovery board flashed with Black Magic probe firmware you can use
the following template:

```
# F4 Discovery
SUBSYSTEMS=="usb", ATTRS{manufacturer}=="Black Sphere Technologies", ATTRS{serial}!="E3C09CF4", GOTO="f4_bmp_end"
ACTION=="add", SUBSYSTEM=="tty", ATTRS{interface}=="Black Magic GDB Server", MODE="0660", GROUP="dialout", SYMLINK+="f4gdb"
ACTION=="add", SUBSYSTEM=="tty", ATTRS{interface}=="Black Magic UART Port", MODE="0660", GROUP="dialout", SYMLINK+="f4uart"
LABEL="f4_bmp_end"
```

Make sure to set correct `serial` number, this can be found by running `dmesg` after the board is plugged in.

Create similar file as `/etc/udev/rules.d/48-bmp.rules` and run
```
udevadm control -R
```
to reload UDev daemon. Next time you plug in your device two nodes should be created

* `/dev/f4gdb` - for attaching GDB
* `/dev/f4uart` - UART2 pass-through

# UART

```bash
miniterm.py -e /dev/f4uart 115200
screen /dev/f4uart 115200
tail -f /dev/f4uart | hexdump -C
```

# Hello Ivory Tower world

# Platforms

## Clock config

# GDB basics

# Ivory

## Casts

## Bit twiddling

Lots of embedded code revolves around bit manipulation.

Ivory offers `bitdata` quasi-quoter for defining binary layouts. This is used heavily
in device drivers and for controlling registers of the STM32 MCU.

```haskell
[ivory|
 bitdata MyReg :: Bits 16 = my_reg
   { myreg_bit_rw   :: Bit
   , myreg_bit_crc  :: Bit
   , myreg_bit_mode :: Bits 2
   , myreg_data     :: Bits 12
   }
|]
```

BitRep

Bit representation

```
-- | Type function: "BitRep (n :: Nat)" returns an Ivory type given a
-- bit size as a type-level natural.  Instances of this type family
-- for bits [1..64] are generated using Template Haskell.
```

## Initializers

## Structs

## Arrays

## Statements

### Loops

### Conditionals

## Functions

## Comments

## Modules

## Standard library

### Operators

```
~>*
%=
%=!
+=
```

### Arrays

```
arrayCopy
```

### Conditionals

```
when
unless
cond_

cond
ifte
```

### Maybe

### Working with strings

String is just an alias for struct with two fields

* `stringDataL`
* `stringLengthL`

Define string with the help of ivory quasi quoter:

```haskell
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE TypeFamilies #-}
{-# OPTIONS_GHC -fno-warn-orphans #-}

module FW.Types where

[ivory| string struct TestBuffer 1024 |]

fwTypes :: Module
fwTypes = package "c4dTypes" $ do
  defStringType (Proxy :: Proxy TestBuffer)

fwTowerDeps :: Tower e ()
fwTowerDeps = do
  towerDepends fwTypes
  towerModule fwTypes
```

This creates a `TestBuffer` type with `IvoryString` typeclass with maximum capacity of 1024 characters.
We also define two helper functions `fwTypes` and `fwTowerDeps` to provide shortcuts for dependency management.

stringInit "foo"

# Serialization

Ivory.Serialize
Ivory.Serialize.Little

# Tower


## Structure of a tower

```haskell

sampleTower :: ChanOutput ('Stored Uint8)
            -> Tower e (ChanOutput ('Stored Uint8))
sampleTower inChan = do
  {-
  here we can define additional channels, towers and periods (which are also towers)
  -}
  chan <- channel

  {-
  afterwards we define a monitor - which basically defines a controller block
  -}
  monitor "sampleMonitor" $ do
    {-
    in monitor we can define its states
    -}
    lastValue <- state "lastValue"

    {-
    and a handler for our input channel
    -}
    handler inChan "sampleMonitorIn" $ do
      {-
      in handler, we can define its emitters, to be used for writing into channels (ChanInput(s))
      -}
      out <- emitter chan 1
      callback $ \ref -> do
        value <- deref ref
        store lastValue value

        when (value >= 9000) $ emit out ref

  return $ snd chan

```

## Channels

ChanInput ChanOuput
```haskell
chan <- channel
(chanIn, chanOut) <- channel
```

## Periods

```haskell
periodic <- period (Milliseconds 500)
-- per <- period (Microseconds 1000)

monitor "sampleMonitor" $ do
  handler periodic "samplePeriod" $ do
    callback $ const $ return ()
```

## Interrupts

## HAL

Ivory.Tower.HAL.Bus.Interface
tower/tower-hal/src/Ivory/Tower/HAL/Bus/Interface.hs

### BackpressureTransmit

BackpressureTransmit request response

```haskell
data BackpressureTransmit value status = BackpressureTransmit
  { backpressureTransmit :: ChanInput value
  , backpressureComplete :: ChanOutput status
  }
```


### AbortableTransmit

```haskell
data AbortableTransmit value status = AbortableTransmit
  { abortableTransmit :: ChanInput value
  , abortableAbort :: ChanInput ('Stored IBool)
  , abortableComplete :: ChanOutput status
  }
```

## Coroutines


## Schedule

Ivory.Tower.HAL.Bus.Sched

Citing from `schedule`s docstring
```
-- | Multiplex a request/response bus across any number of tasks that
-- need to share it. Tasks may submit requests at any time, but only one
-- task's request will be submitted to the bus at a time. When that
-- request's response arrives, it is forwarded to the appropriate task
-- and the next waiting task's request is sent.
--
-- If multiple tasks have outstanding requests simultaneously, then this
-- component will choose the highest-priority task first. Earlier tasks
-- in the list given to 'schedule' are given higher priority.
```

## Drivers

uartDriver
i2cDriver
canDriver

# Driver tutorial

## `forever`

forever :: Ivory (E.AllowBreak eff) () -> Ivory eff ()

^^ noBreak needed sometimes due to this, forever $ noBreak $ do

# App tutorial

# Porting

## Unoffical ports

# Tower backends

## Posix

# Gidl

# Projects built with Ivory/Tower

## SMACCMPilot

## DistRap

## hexamon

# Further reading

# opts
