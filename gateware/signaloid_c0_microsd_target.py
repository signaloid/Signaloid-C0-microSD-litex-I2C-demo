#!/usr/bin/env python3

# 	Copyright (c) 2024, Signaloid.
#
# 	Permission is hereby granted, free of charge, to any person obtaining a copy
# 	of this software and associated documentation files (the "Software"), to
# 	deal in the Software without restriction, including without limitation the
# 	rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# 	sell copies of the Software, and to permit persons to whom the Software is
# 	furnished to do so, subject to the following conditions:
#
# 	The above copyright notice and this permission notice shall be included in
# 	all copies or substantial portions of the Software.
#
# 	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# 	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# 	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# 	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# 	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# 	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# 	DEALINGS IN THE SOFTWARE.


from litex.gen import KILOBYTE, MEGABYTE, LiteXModule
from litex.soc import doc as docs_builder
from litex.soc.cores.ram import Up5kSPRAM
from litex.soc.integration.builder import Builder
from litex.soc.integration.doc import AutoDoc, ModuleDoc
from litex.soc.integration.soc import (
    ClockDomain,
    Instance,
    Module,
    Signal,
    SoCRegion,
)
from litex.soc.integration.soc_core import SoCCore
from litex.soc.interconnect.csr import (
    AutoCSR,
    CSRField,
    CSRStorage,
)
from litex_boards.platforms import signaloid_c0_microsd
from migen import If
from migen.genlib.resetsync import AsyncResetSynchronizer
from iCE40_I2C_LiteX_integration import ICE40UP_I2C


#   USER_DATA_OFFSET is the SPI Flash address of the first byte of the firmware
#   binary. It is used as a boot address for the Signaloid C0-microSD card SoC.
#
#   The SPI Flash is connected to the Core through the SoC's memory mapped bus.
#   Hence, the CPU Reset Address is set to:
#   SPI_FLASH_BUS_ADDRESS + USER_DATA_OFFSET
#
#   As described on the official Signaloid C0-microSD documentation, the
#   USER_DATA_OFFSET defines the start of a 14MiB address space where the
#   firmware binary and data can be stored.
#
#   More info:
#   https://c0-microsd-docs.signaloid.io/hardware-overview/bootloader-addresssing.html
USER_DATA_OFFSET = "0x200000"


class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        self.cd_por = ClockDomain()
        self.cd_clk10khz = ClockDomain()

        assert sys_clk_freq in [6e6, 12e6, 24e6, 48e6]

        #   #     #

        #   Power On Reset
        por_count = Signal(16, reset=2**16 - 1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        #   High frequency oscillator (HFSOC) up to 48MHz
        clk_hf_div = {48e6: "0b00", 24e6: "0b01", 12e6: "0b10", 6e6: "0b11"}[
            sys_clk_freq
        ]

        self.specials += Instance(
            "SB_HFOSC",
            p_CLKHF_DIV=clk_hf_div,
            i_CLKHFEN=0b1,
            i_CLKHFPU=0b1,
            o_CLKHF=self.cd_sys.clk,
        )
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~por_done)
        platform.add_period_constraint(self.cd_sys.clk, 1e9 / sys_clk_freq)

        #   Low frequency oscillator (LFOSC) at 10kHz
        self.specials += Instance(
            "SB_LFOSC",
            i_CLKLFEN=0b1,
            i_CLKLFPU=0b1,
            o_CLKLF=self.cd_clk10khz.clk,
        )


class Leds(Module, AutoCSR, AutoDoc):
    """Signaloid C0-microSD LED control"""

    def __init__(self, platform: signaloid_c0_microsd.Platform) -> None:
        self.intro = ModuleDoc(
            """Signaloid C0-microSD LED control.
            Set the LED bit to 1 to turn it on and 0 to turn it off.
            """
        )

        self._out = CSRStorage(
            size=2,
            fields=[
                CSRField(
                    name="red",
                    description="""The Red LED on the Signaloid C0-microSD.
                    On when 1, off when 0.""",
                ),
                CSRField(
                    name="green",
                    description="""The Green LED on the Signaloid C0-microSD.
                    On when 1, off when 0.""",
                ),
            ],
        )

        #   Drive the LEDs directly.
        #
        #   Driving the LEDs directly from the FPGA is SAFE on
        #   the Signaloid C0-microSD, since the LEDs are hard-wired to external
        #   current limiting resistors.
        #   Hence, for simplicity, the SB_RGBA_DRV hard IP can be omitted.
        #
        #   In general, direct drive can lead to overvoltage, and damage to the
        #   LEDs and/or the board. Lattice iCE40 mitigates this issue by
        #   incorporating the SB_RGBA_DRV hard IP on LEDs without external
        #   current limiting resistors.
        #
        #   LED driver pins are in an open-drain configuration so they should
        #   be inverted to be intuitively controlled through software.
        #
        self.comb += platform.request("user_led", number=0).eq(
            ~self._out.storage[0]
        )
        self.comb += platform.request("user_led", number=1).eq(
            ~self._out.storage[1]
        )


class BaseSoC(SoCCore):
    def __init__(
        self,
        flash_offset,
        sys_clk_freq=24e6,
        with_i2c=False,
        **kwargs,
    ):
        platform = signaloid_c0_microsd.Platform()

        #   CRG
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        #   SoCCore
        #   Disable Integrated ROM/SRAM since too large for iCE40 and UP5K has
        #   dedicated SPRAM.
        kwargs["integrated_sram_size"] = 0
        kwargs["integrated_rom_size"] = 0
        SoCCore.__init__(
            self,
            platform,
            sys_clk_freq,
            **kwargs,
        )

        #   128KB SPRAM
        spram_size = 128 * KILOBYTE
        self.spram = Up5kSPRAM(size=spram_size)
        self.bus.add_slave(
            name="sram",
            slave=self.spram.bus,
            region=SoCRegion(
                size=spram_size,
                linker=True,
            ),
        )

        #   SPI Flash
        #   Signaloid C0-microSD uses the AT25QL128A SPI flash with the QPI mode
        #   disabled. Hence, the AT25SL128A module is used instead, which is
        #   compatible with Signaloid C0-microSD's AT25QL128A with the QPI mode
        #   disabled.
        from litespi.modules import AT25SL128A
        from litespi.opcodes import SpiNorFlashOpCodes as Codes

        self.add_spi_flash(
            mode="1x", module=AT25SL128A(Codes.READ_1_1_1), with_master=False
        )

        #   Add ROM linker region
        self.bus.add_region(
            name="rom",
            region=SoCRegion(
                origin=self.bus.regions["spiflash"].origin + flash_offset,
                size=14 * MEGABYTE,
                linker=True,
            ),
        )
        self.cpu.set_reset_address(self.bus.regions["rom"].origin)

        #   Leds
        self.leds = Leds(self.platform)

        #   Add an I2C interface if enabled
        if with_i2c:
            self.sb_i2c = ICE40UP_I2C(
                scl_pin=platform.request("SD_DAT0"),
                sda_pin=platform.request("SD_DAT1"),
                sys_clk=self.crg.cd_sys.clk
            )


def main():
    from litex.build.parser import LiteXArgumentParser

    parser = LiteXArgumentParser(
        platform=signaloid_c0_microsd.Platform,
        description="LiteX SoC on Signaloid C0-microSD.",
    )
    parser.add_target_argument(
        "--sys-clk-freq",
        default=24e6,
        type=float,
        help="""System clock frequency. Possible values are:
            [6e6, 12e6, 24e6, 48e6]""",
    )
    parser.add_target_argument(
        "--flash-offset",
        default=USER_DATA_OFFSET,
        help="Boot offset in SPI Flash.",
    )
    parser.add_target_argument(
        "--build_docs",
        action="store_true",
        help="Builds documentation.",
    )
    parser.add_target_argument(
        "--add_uart",
        action="store_true",
        help="Enable UART interface.",
    )
    parser.add_target_argument(
        "--add_i2c",
        action="store_true",
        help="Enable I2C interface.",
    )
    args = parser.parse_args()

    if not args.add_uart:
        args.no_uart = True

    soc = BaseSoC(
        flash_offset=int(args.flash_offset, 0),
        sys_clk_freq=args.sys_clk_freq,
        with_i2c=args.add_i2c,
        **parser.soc_argdict,
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.build_docs:
        docs_builder.generate_docs(
            soc,
            "build/documentation/",
            project_name="Signaloid C0-microSD LiteX RISC-V Example SoC",
            author="Signaloid",
        )


if __name__ == "__main__":
    main()
