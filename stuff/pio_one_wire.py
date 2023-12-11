# Globals we use
#import gbls

import asyncio
from machine import Pin
import rp2
import onewire

dbg = 0

# Onewire commands
OW_READ_ROM = bytearray([0x33])
OW_MATCH_ROM = bytearray([0x55])
OW_SKIP_ROM = bytearray([0xCC])
OW_SRCH_ROM = bytearray([0xF0])

# DS18x20 commands
OW_CONVERT_T = bytearray([0x44])
OW_READ_SP = bytearray([0xBE])
OW_WRITE_SP = bytearray([0x4E])

###########
# PIO program for reset & write/read slot timing
# Each period is 2uS
###########
@rp2.asm_pio(out_init = rp2.PIO.IN_LOW, set_init = rp2.PIO.IN_LOW,
             in_shiftdir = rp2.PIO.SHIFT_RIGHT, out_shiftdir = rp2.PIO.SHIFT_RIGHT)
def ow_pgm():
    # Get reset timing & bit count word
    pull(block)

    # Reset low time, bits [5:0] - this should be set to 50 (500us) if cmd is reset,
    # and 0 if doing a regular OW transaction
    out(x, 6)

    # Skip if no reset
    jmp(not_x, "no_reset")

    # 0 = not present for reset command
    set(y, 0)

    # Do low time
    set(pindirs, 1)
    label("reset_low")
    jmp(x_dec, "reset_low") [4]

    # Do reset high time before present, bits [9:6]  - this should be set to 7 (70uS)
    out(x, 4)
    set(pindirs, 0)
    label("reset_high1")
    jmp(x_dec, "reset_high1") [4]

    # Sample present
    jmp(pin, "not_present")
    set(y, 1) # 1 = present
    label("not_present")

    # Do rest high time after present, bits [15:10] - this should be set to 43 (430uS)
    out(x, 6)
    label("reset_high2")
    jmp(x_dec, "reset_high2") [4]

    # Move present bit to the FIFO, and exit
    mov(isr, y)
    push(block)
    jmp("exit")

    # No reset - dump the remaining reset timing bits, and then get the OW bit count
    label("no_reset")
    out(x, 10)

    # How many OW bits to do
    out(x, 16)

    ###
    # The bit loop
    label("bits")

    # Pull/push as needed
    pull(ifempty, block)
    push(iffull, block)

    # Wait until we sample high
    wait(1, pin, 0)
    # Then after 2uS, set low to start the slot
    set(pindirs, 1)
    # After 2uS, drive the data, and wait another 8uS
    out(pindirs, 1) [4]
    # Then (about 12uS into slot), sample the data to
    # detect a conflict on write, or capture read data on read
    in_(pins, 1) [25]
    # About 62uS in, end the slot
    set(pindirs, 0)

    # Repeat!
    jmp(x_dec, "bits")
    ###

    # Push remaining input data
    push(block)

    # Signal done
    label("exit")
    irq(rel(0))

###########
# PIO program for rom search
# Each period is 2uS
###########
@rp2.asm_pio(out_init = rp2.PIO.IN_LOW, set_init = rp2.PIO.IN_LOW,
             in_shiftdir = rp2.PIO.SHIFT_RIGHT, out_shiftdir = rp2.PIO.SHIFT_RIGHT)
def ow_search_pgm():
    # Get bit count (should be 63)
    pull(block)
    out(x, 32)

    ###
    # The bit loop
    label("bits")

    # Default is '1'
    set(y, 1)

    # Pull if needed
    pull(ifempty, block)

    # Would have liked to wait until we sample high...
    # wait(1, pin, 0)

    ####
    # Slot 0
    ####
    # Set low to start the slot
    set(pindirs, 1)

    # After 2uS, release
    set(pindirs, 0) [4]

    # Then, about 12uS into slot, sample the data
    jmp(pin, "skip0")
    set(y, 0)
    label("skip0")
    in_(y, 1) [24]

    # About 64uS in, end the slot
    ####

    # Would liked to wait until we sample high...
    # wait(1, pin, 0)

    ####
    # Slot 1
    ####
    # Set low to start the slot
    set(pindirs, 1)

    # After 2uS, release
    set(pindirs, 0) [4]

    # Then, about 12uS into slot, sample the data
    in_(pins, 1)
    jmp(pin, "cmpl1") [20] # Total 8

    # The compliment pin is 0 - check with first bit
    jmp(not_y, "conflict")

    # Will drive a "1" as we got "1" and "0"
    out(y, 1) # dump this value
    set(y, 1)
    jmp("drive")

    # The compliment pin is 1 - check with first bit
    label("cmpl1")
    mov(y, invert(y))
    jmp(not_y, "conflict")

    # Will drive a "0" as we got "0" and "1"
    out(y, 1) # dump this value
    set(y, 0)
    jmp("drive")

    # Will drive the passed in value since there was a conflict
    label("conflict")
    out(y, 1) [2]

    # About 64uS in, end the slot
    label("drive")

    # Would have liked to wait until we sample high...
    # wait(1, pin, 0)

    ####
    # Slot 2
    ####
    # Set low to start the slot
    set(pindirs, 1)

    # After 2uS, drive the value and wait until the end of the slot
    jmp(not_y, "drive0")
    set(pindirs, 0) [29]
    jmp("endslot")

    label("drive0")
    set(pindirs, 1) [29]

    # About 64uS in, end the slot
    label("endslot")
    set(pindirs, 0)

    # Push, if needed
    push(iffull, block)

    # Repeat!
    jmp(x_dec, "bits")

    # Signal done
    irq(rel(0))

# Alias to the CRC function
crc8 = onewire._ow.crc8

# CRC routine that works on first "bytelen" bytes of an int
def calcCrc(data, bytelen):
  return(crc8(data.to_bytes(bytelen, "little")))

# Class to wrap up a one-wire PIO instance
class PioOneWire:

    # Reset length in 10uS periods: RST_LOW[5:0], RST_HIGH_PRESAMP[9:6], RST_HIGH_POSTSAMP[15:10]
    resetCnt = (50 << 0) | (7 << 6) | (43 << 10)

    def __init__(self, sm, pin, pini = None):
        self.pin = pin
        self.pini = pini if pini != None else pin

        # Access lock
        self.lock = asyncio.Lock()

        # Counts of error types
        self.errCmd = 0
        self.errCrc = 0

        # Constant "TX" for issuing read commands
        self.rdCmd = bytearray([0xff] * 12)
        self.rdCmdMv = memoryview(self.rdCmd)

        # Setup state machine
        self.sm = rp2.StateMachine(sm, ow_pgm, freq=500000,
                                   in_base=self.pini, jmp_pin=self.pini, out_base=self.pin, set_base=self.pin)

        # No harm in a weak pull-up - useful for testing
        self.pin.init(pull = Pin.PULL_UP)
        self.pini.init(pull = Pin.PULL_UP)

        # Event for done trigger
        self.evnt = asyncio.ThreadSafeFlag()

        # Done event handler - just sets the event
        self.sm.irq(lambda p: self.evnt.set(), hard = True)

        # Start up the state machine
        self.sm.active(1)

        # Present indicator
        self.present = False

        # RX buffer - only support up to 12 bytes
        self.rx = bytearray([0] * 12)
        self.rxb = None

    # Low level interface to OW transaction
    async def _doOW(self, reset, tx):
        l = len(tx)
        if dbg > 1: print("_doOW(%d, %d)" % (reset, l))
        if reset:
            # Push in reset command
            self.sm.put(self.resetCnt)

            # Wait for it to complete
            await self.evnt.wait()

            # Get present result
            wrd = self.sm.get()
            if dbg: print("present = %d" % (wrd))
            self.present = True if wrd != 0 else False


        if l and (not reset or self.present):
            if l > 12:
                raise Exception("Onewire operation must be from 1 to 12 bytes")

            # Prep for data
            self.rxb = memoryview(self.rx)[:l]
            rem = l & 0x3
            rng = range(0, l & 0xfc, 4)

            # Start TX command
            bitLen = (l << 3) - 1
            if dbg > 5: print("bitLen = %d" % (bitLen))
            self.sm.put(bitLen << 16)

            # TX data
            i = None
            for i in rng:
                wrd = int.from_bytes(tx[i:i+4], "little")
                if dbg > 5: print("wrd = %x" % (wrd))
                self.sm.put(~wrd)
            i = 0 if i == None else i + 4
            if rem:
                wrd = int.from_bytes(tx[i:], "little")
                if dbg > 5: print("wrd = %x" % (wrd))
                self.sm.put(~wrd)

            # Wait for it to complete
            await self.evnt.wait()

            # Grab RX data
            i = None
            for i in rng:
                wrd = self.sm.get()
                if dbg > 5: print("wrd = %x" % (wrd))
                self.rxb[i:i+4] = wrd.to_bytes(4, "little")
            i = 0 if i == None else i + 4
            if rem:
                wrd = self.sm.get()
                if dbg > 5: print("rdw = %x" % (wrd))
                self.rxb[i:] = wrd.to_bytes(4, "little")[4-rem:]

    ######################
    # Issues a single byte command
    # Returns True if the data observed != written (collision), and non-zero otherwise.
    ######################
    async def doCmd(self, reset, cmd):
        await self._doOW(reset, cmd)
        err = (reset and not self.present) or (cmd != self.rxb)
        if err: self.errCmd += 1
        return(err)

    ######################
    # Performs a write of the given length.
    # Returns True if the data observed != written (collision), and non-zero otherwise.
    ######################
    async def doWrite(self, reset, wrData):
        await self._doOW(reset, wrData)
        err = (reset and not self.present) or (wrData != self.rxb)
        if err: self.errCmd += 1
        return(err)

    ######################
    # Performs a read of the given length.  Data must be used/copied
    # before issuing another OW command!
    ######################
    async def doRead(self, l):
        rdTx = self.rdCmdMv[:l]
        await self._doOW(False, rdTx)
        return self.rxb

    ######################
    # Performs a command followed by a read of the given length.  Data
    # must be used/copied before issuing another OW command!
    # Returns True if the command observed != written (collision), and non-zero otherwise.
    ######################
    async def doCmdRead(self, reset, cmd, l):
        rdTx = self.rdCmdMv[:l]
        cl = len(cmd)
        cmdRd = cmd + rdTx
        await self._doOW(reset, cmdRd)
        err = (reset and not self.present) or (cmd != self.rxb[:cl])
        if err: self.errCmd += 1
        return (err, self.rxb[cl:])

    ######################
    # Starts temp converison on all devices
    ######################
    async def allCvtT(self):
        cmd = OW_SKIP_ROM + OW_CONVERT_T
        err = await self.doCmd(True, cmd)
        if err: self.errCmd += 1
        return err

    ######################
    # Reads the temperature (in SP) on all devices
    # -> Generally only useful if there is 1 device!
    ######################
    async def allReadTemp(self):
        return(await self.readTemp())

    ######################
    # Reads the temperature for the given ROM, or skips
    # ROM match if none given (generally only useful
    # if there is 1 device).
    ######################
    async def readTemp(self, rom=None):
        if rom != None:
            cmd = OW_MATCH_ROM + rom.to_bytes(8, "little") + OW_READ_SP
        else:
            cmd = OW_SKIP_ROM + OW_READ_SP

        err = await self.doWrite(True, cmd)
        if not err:
            data = await self.doRead(9)

        if err:
            self.errCmd += 1
            return(None)

        if crc8(data):
            self.errCrc += 1
            return(None)

        temp = int.from_bytes(data[0:2], "little", True)
        if (temp & 0x8000):
            temp = -1 * ((~temp & 0xffff) + 1)
        temp = temp / 16
        return(temp)

    ##########################################
    # Find all connected ROM values
    ##########################################
    async def allRomSearch(self):
        conVal = 0 # Write values to use for conflicts
        nxtConVal = 0 # Next value to use
        conBits = 0 # Conflict bits of last scan
        wrBits = 0 # Write values of last scan
        roms = list() # Roms we found
        err = 0
        if dbg: print("Started")
        while True:
            # Advance to the next set of conflict values to use
            conVal = nxtConVal

            # Test the new set of conVal on the attached devices
            if dbg: print("B: c/w = %x/%x" % (conBits, wrBits))
            (conBits, wrBits, err) = await self.testConValPio(conVal)
            if dbg: print("A: c/w = %x/%x" % (conBits, wrBits))

            # If the last bit found is ever in conflict, something broke!
            if err or ((conBits >> 63) & 1):
                break;

            # Save the ROM value found if not in error
            if not calcCrc(wrBits, 8):
                roms.append(wrBits)

            # Check if done: See if next conflict values are different
            nxtConVal = PioOneWire.nextConVal(conVal, conBits)
            if dbg: print("Next = %x" % (nxtConVal))
            if conVal == conBits:
                break;

            # No, go on to use next one
            conVal = nxtConVal

        return(roms, err)

    ##########################################
    # Calculate the next conflict values to use given the current, and the last set of conflicts seen
    ##########################################
    def nextConVal(conVal, conBits):
        for i in reversed(range(64)):
            bit = 1 << i
            mask = ~(~0 << (i + 1))
            if (conBits & bit) and not (conVal & bit):
                conVal = (conVal | bit) & mask
                break
        return(conVal)

    ##########################################
    # Tests one conflict value against attached devices, using PIO
    ##########################################
    async def testConValPio(self, conVal):
        # Issue reset and rom search command
        cmd = OW_SRCH_ROM
        err = await self.doCmd(True, cmd);

        conBits = 0
        wrBits = 0

        if not err:
            # Swap in search program
            self.sm.active(0)
            rp2.PIO(0).remove_program(ow_pgm)
            self.sm.init(ow_search_pgm, freq=500000, in_base=self.pini, jmp_pin=self.pini, out_base=self.pin, set_base=self.pin)
            self.sm.active(1)

            # Perform search
            self.sm.put(63) # Number of bits - 1
            self.sm.put(conVal & 0xffffffff)
            self.sm.put((conVal >> 32) & 0xffffffff)

            await self.evnt.wait()

            # Process the conflicts
            res = self.sm.get()
            res |= self.sm.get() << 32
            res |= self.sm.get() << 64
            res |= self.sm.get() << 96

            for i in range(64):
                bits = res & 0x3
                # Both bits 0 - conflict
                if bits == 0:
                    wrBits |= (conVal & 1) << i
                    conBits |= 1 << i
                # Bit = 0 (0 then 1 driven)
                elif bits == 2:
                    pass
                # Bit = 1 (1 then 0 driven)
                elif bits == 1:
                    wrBits |= 1 << i
                # Both bits 1 - error
                else:
                    err = 1
                    conBits = 0
                    wrBits = 0
                    break

                conVal = conVal >> 1
                res = res >> 2

            # Swap back to regular program
            self.sm.active(0)
            rp2.PIO(0).remove_program(ow_search_pgm)
            self.sm.init(ow_pgm, freq=500000, in_base=self.pini, jmp_pin=self.pini, out_base=self.pin, set_base=self.pin)
            self.sm.active(1)

        return(conBits, wrBits, err)

# Routines to convert from integer ROM to common string representation
def rom2str(rom):
    # Drop CRC
    rom &= 0x00ffffffffffffff
    # Grab and shift off LSB (family)
    fam = rom & 0xff
    rom >>= 8
    str = b"%02x-%012x" % (fam, rom)
    return(str)

def str2rom(str):
    fam = str[0:2]
    rom = str[3:]
    rom = int(rom,16) << 8
    rom |= int(fam,16)
    rom |= calcCrc(rom,7) << 56
    return(rom)
