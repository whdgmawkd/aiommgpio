import asyncio
from concurrent.futures import ThreadPoolExecutor
import mmap
from typing import Optional, Union
import time
import struct

from constants import *


def _model_detect() -> MODEL:
    """Detect Raspberry Pi model

    Returns:
        MODEL: detected model
    """
    _model_map: dict[int, MODEL] = {
        0xb76: MODEL.BCM2708,
        0xc07: MODEL.BCM2709,
        0x0d3: MODEL.BCM2709,
        0xd08: MODEL.BCM2711
    }
    _revision: str = None
    _model = None
    _part = None
    with open('/proc/cpuinfo', 'r') as f:
        cpuinfos = f.readlines()
        for cpu in cpuinfos:
            if cpu.startswith('Revision'):
                _revision = cpu.rstrip().split()[-1].strip()
            elif cpu.startswith('Model'):
                _model = cpu.rstrip().split(':')[-1].strip()
            elif cpu.startswith('CPU part'):
                _part = int(cpu.rstrip().split()[-1].strip(), 16)
    print('Detect Hardware')
    print('='*30)
    print(f'Model: {_model}')
    print(f'Revision: {_revision}')
    print(f'Part: {_part:0x}')

    if _part in _model_map:
        print(f'-> Detected Hardware: {_model_map[_part]}')
        return _model_map[_part]
    else:
        print('Failed to detect hardware. assume the hardware is BCM2708')
        print(f'-> Detected Hardware: BCM2708')
        return MODEL.UNKNOWN


class BitArray(list):

    def __init__(self, data: bytes, is_int: bool = False) -> None:
        data_to_u32 = struct.unpack("@I", data)[0] if not is_int else (int(data.hex(), 16) & 0xffffffff)
        u32_to_bin = f'{bin(data_to_u32)[2:]:0>32s}'
        super().__init__([int(d) == 1 for d in u32_to_bin[::-1]])
        if self.__len__() % 8 != 0:
            raise ValueError('BitArray size should be multiple of 8')

    def to_bytes(self) -> bytes:
        bin_to_str = "".join([str(int(x)) for x in self[::-1]])
        ret = []
        for byte in range(4):
            ret.append(int(bin_to_str[8*byte:8*byte+8], 2))
        ret = ret[::-1]
        return bytes(ret)

    def to_u32(self) -> int:
        return struct.unpack("@I", self.to_bytes())[0]

    def set_bit(self, bit: int, value: bool) -> 'BitArray':
        self[bit] = value
        return self


class MMIO:

    def __init__(self, mem_file: str = '/dev/mem', address: int = 0x0, length: int = -1):
        try:
            self._mem = open(mem_file, 'r+b')
        except IOError as e:
            print(e)
            raise e
        except Exception as e:
            print(e)
            raise e
        self._address = address
        self._length = length
        self._mmio = mmap.mmap(
            self._mem.fileno(),
            offset=address,
            length=length,
            access=mmap.ACCESS_WRITE,
            prot=mmap.PROT_READ | mmap.PROT_WRITE,
            flags=mmap.MAP_SHARED
        )
        self._mem.close()

    def __getitem__(self, offset: int) -> BitArray:
        """get 4bytes data from memory

        Args:
            offset (int): data offset

        Raises:
            IOError: when offset is not multiple of 4

        Returns:
            bytes: 4byte data of given offset
        """
        if offset % 4 != 0:
            raise IOError('Memory offset shoud be multiple of 4')
        return BitArray(self._mmio[offset:offset+4])

    def __setitem__(self, offset: int, data: BitArray):
        """set 4bytes data to memory

        Args:
            offset (int): offset to write data
            data (BitArray): 4bytes data

        Raises:
            IOError: when offset is not multiple of 4
            ValueError: when data is not 4bytes (32bits)
        """
        if offset % 4 != 0:
            raise IOError('Memory offset shoud be multiple of 4')
        if len(data) != 32:
            raise ValueError('Data is not 4bytes long')
        self._mmio[offset:offset+4] = data.to_bytes()


class RPiMMIO:
    """MMIO for Raspberry Pi

    * VideoCore Peripheral Address is 0x7E000000
            * BCM2708(BCM2835) : ARM Physical Address is 0x20000000
            * BCM2709(BCM2836) : ARM Physical Address is 0x3F000000
            * BCM2711 : ARM Physical Address is 0xFE000000

    * GPIO Address Offset is 0x00200000

    * PWM0 Address Offset is 0x0020C000
            * BCM2711 Only : 0x0020C800 for PWM1

    * each GPIO & PWM Memory Address Size are 0x00010000 and 0x00001000
    """

    _CLK_OFFSET = 0x101000
    _CLK_LENGTH = 0x1000
    _CLK_PASSWORD = 0x5a000000
    _PERI_OFFSET = 0x200000
    _PERI_LENGTH = 0x10000

    # start address of peripheral ARM mapped memory
    _ADDRESS_MAP = {
        MODEL.BCM2708: 0x20000000,
        MODEL.BCM2709: 0x3F000000,
        MODEL.BCM2711: 0xFE000000,
    }

    def __init__(self, model: MODEL = MODEL.AUTO):
        """MMIO for Paspberry Pi

        Args:
            model (MODEL, optional): Raspberry Pi Model. Defaults to MODEL.AUTO.
        """

        if model == MODEL.AUTO:
            model = _model_detect()
        self._model = model
        self._address = self._ADDRESS_MAP[self._model]
        self.peri_mmio = MMIO('/dev/mem', self._address + self._PERI_OFFSET, self._PERI_LENGTH)
        self.clk_mmio = MMIO('/dev/mem', self._address + self._CLK_OFFSET, self._CLK_LENGTH)
        self._cleanuped = False

        # pin dictionary
        self._pins: dict[int, Union[GPIO, PWM]] = dict()

    async def get_gpio(self, bcm_pin: int, mode: GPIO_MODE) -> 'GPIO':
        if bcm_pin in self._pins:
            if isinstance(self._pins[bcm_pin], GPIO):
                return self._pins[bcm_pin]
            else:
                # if gpio setting is required for PWM
                if mode in [GPIO_MODE.ALT_FUNC0, GPIO_MODE.ALT_FUNC5]:
                    return GPIO(self, bcm_pin, mode)
                else:
                    raise IOError(f'{bcm_pin} is in PWM mode.')
        self._pins[bcm_pin] = GPIO(self, bcm_pin, mode)
        return self._pins[bcm_pin]

    async def get_pwm(self, bcm_pin: int, mode: PWM_MODE) -> 'PWM':
        if bcm_pin in self._pins:
            if isinstance(self._pins[bcm_pin], PWM):
                return self._pins[bcm_pin]
            else:
                raise IOError(f'{bcm_pin} is in GPIO mode.')
        self._pins[bcm_pin] = PWM(self, bcm_pin, mode)
        return self._pins[bcm_pin]

    async def cleanup(self):
        for pin in self._pins:
            await self._pins[pin].cleanup()
        self._pins.clear()
        self._cleanuped = True

    def __del__(self):
        if not self._cleanuped:
            print("WARN: peripherals are not cleaned up.")


class GPIO:

    _GFPSEL_BASE = 0x0  # Function Select
    _GPSET_BASE = 0x1c  # Output Set
    _GPCLR_BASE = 0x28  # Output Clear
    _GPLEV_BASE = 0x34  # Pin Level
    _GPEDS_BASE = 0x40  # Event Detect Status
    _GPREN_BASE = 0x4c  # Rising Edge Detect Enable
    _GPFEN_BASE = 0x58  # Falling Edge Detect Enable
    # GPHEN, GPLEN, GPAREN, GPAFEN, GPIO_PUP_PDN_CNTRL is not implemented.
    _GPHEN_BASE = 0x64  # High Level Detect Enable
    _GPLEN_BASE = 0x70  # Low Level Detect Enable
    _GPAREN_BASE = 0x7c  # Async REN
    _GPAFEN_BASE = 0x88  # Async FEN
    _GPIO_PUP_PDN_CNTRL = 0xe4  # Pull Up&Down Select

    def __init__(self, mmio: RPiMMIO, bcm_pin: int, mode: GPIO_MODE = GPIO_MODE.OUTPUT):
        """use bcm_pin as gpio pin

        Args:
            mmio (RPiMMIO): RPiMMIO object
            bcm_pin (int): Broadcom GPIO Pin number
            mode (GPIO_MODE): input/output mode select. defaults to output
        """
        self._mmio = mmio
        self._peri_mmio = mmio.peri_mmio
        self._bcm_pin = bcm_pin
        self._mode = mode
        self._MODEBITS = [False, False, False]  # default INPUT 000
        if self._mode == GPIO_MODE.OUTPUT:
            self._MODEBITS[0] = True  # 27bit
        if self._mode == GPIO_MODE.ALT_FUNC0:
            self._MODEBITS[2] = True  # 29bit
        elif self._mode == GPIO_MODE.ALT_FUNC5:
            self._MODEBITS[1] = True  # 28bit
        self._GFPSEL = (bcm_pin // 10) * 4 + self._GFPSEL_BASE
        self._GFPSEL_BIT = (bcm_pin % 10) * 3
        pin_offset = (bcm_pin // 32) * 4
        self._REG_PIN_BIT = bcm_pin % 32
        self._GPSET = self._GPSET_BASE + pin_offset
        self._GPCLR = self._GPCLR_BASE + pin_offset
        self._GPLEV = self._GPLEV_BASE + pin_offset
        self._GPEDS = self._GPEDS_BASE + pin_offset
        self._GPREN = self._GPREN_BASE + pin_offset
        self._GPFEN = self._GPFEN_BASE + pin_offset

        # GFPSEL setup
        gfpsel = self._peri_mmio[self._GFPSEL]
        gfpsel[self._GFPSEL_BIT: self._GFPSEL_BIT+3] = self._MODEBITS
        self._peri_mmio[self._GFPSEL] = gfpsel

    async def cleanup(self):
        """Reset GPIO
        """
        gfpsel = self._peri_mmio[self._GFPSEL]
        gfpsel[self._GFPSEL_BIT: self._GFPSEL_BIT+3] = [False, False, False]
        self._peri_mmio[self._GFPSEL] = gfpsel

    async def write(self, value: bool):
        if self._mode != GPIO_MODE.OUTPUT:
            raise IOError('GPIO is not in OUTPUT mode')
        self.write_nowait(value)
        await asyncio.sleep(0)

    def write_nowait(self, value: bool):
        if self._mode != GPIO_MODE.OUTPUT:
            raise IOError('GPIO is not in OUTPUT mode')
        if value:
            gpset = self._peri_mmio[self._GPSET]
            gpset[self._REG_PIN_BIT] = True
            self._peri_mmio[self._GPSET] = gpset
        else:
            gpclr = self._peri_mmio[self._GPCLR]
            gpclr[self._REG_PIN_BIT] = True
            self._peri_mmio[self._GPCLR] = gpclr

    async def read(self) -> bool:
        if self._mode != GPIO_MODE.INPUT:
            raise IOError('GPIO is not in INPUT mode')
        return self.read_nowait()

    def read_nowait(self) -> bool:
        if self._mode != GPIO_MODE.INPUT:
            raise IOError('GPIO is not in INPUT mode')
        return self._peri_mmio[self._GPLEV][self._GPLEV_BIT]

    async def wait_for_edge(self, edge: EDGE = EDGE.BOTH) -> bool:
        if self._mode != GPIO_MODE.INPUT:
            raise IOError('GPIO is not in INPUT mode')
        # setup edge detect enable register
        if edge == EDGE.FALLING or edge == EDGE.BOTH:
            gpfen = self._peri_mmio[self._GPFEN]
            gpfen[self._REG_PIN_BIT] = True
            self._peri_mmio[self._GPFEN] = gpfen
        if edge == EDGE.RISING or edge == EDGE.BOTH:
            gpren = self._peri_mmio[self._GPREN]
            gpren[self._REG_PIN_BIT] = True
            self._peri_mmio[self._GPREN] = gpren
        while True:
            if self._peri_mmio[self._GPEDS][self._REG_PIN_BIT]:
                # clear current pin status
                gpeds = [False for _ in range(32)]
                gpeds[self._REG_PIN_BIT] = True
                self._peri_mmio[self._GPEDS] = gpeds
                break
            await asyncio.sleep(0)
        if edge == EDGE.FALLING or edge == EDGE.BOTH:
            gpfen = self._peri_mmio[self._GPFEN]
            gpfen[self._REG_PIN_BIT] = False
            self._peri_mmio[self._GPFEN] = gpfen
        if edge == EDGE.RISING or edge == EDGE.BOTH:
            gpren = self._peri_mmio[self._GPREN]
            gpren[self._REG_PIN_BIT] = False
            self._peri_mmio[self._GPREN] = gpren
        return True


class PWM:

    _HARDWARE_PWM_PINS = [12, 13, 18, 19]

    _CM_PWMCTL_BASE = 0xa0
    _CM_PWMDIV_BASE = 0xa4
    _CM_PASSWD = BitArray(struct.pack("@I", 0x5a000000))
    _CTL_BASE = 0xc000
    _STA_BASE = 0xc004
    _RNG_BASE = 0xc010
    _DAT_BASE = 0xc014

    _PWM_CLOCK = 100000000  # 100MHz
    _TIME_UNIT = 1000000000  # nanoseconds
    _PWM_DIV = _TIME_UNIT // _PWM_CLOCK

    def __init__(self, mmio: RPiMMIO, bcm_pin: int, pwm_mode: PWM_MODE = PWM_MODE.AUTO):
        """use bcm_pin as pwm pin

        if you want use more percise pwm in software mode, use `pwm_mode=PWM_MODE.THREADED`

        * Maximum PWM Frequency
                * Hardware PWM : 50MHz (duty is only available at 0%, 50%, 100%)
                * Software PWM : Unknown

        Args:
            mmio (RPiMMIO): RPiMMIO object
            bcm_pin (int): Broadcom GPIO Pin number
            pwm_mode (bool): PWM control mode. Hardware mode is available on pin 12, 13, 18, 19
        """
        self._mmio = mmio
        self._bcm_pin = bcm_pin
        if self._bcm_pin in self._HARDWARE_PWM_PINS and pwm_mode != PWM_MODE.SOFTWARE:
            self._pwm_mode = PWM_MODE.HARDWARE
        else:
            self._pwm_mode = PWM_MODE.SOFTWARE
        self._period: int = self._PWM_CLOCK//1000  # 1KHz
        self._duty: int = self._period//2  # 50% duty
        self._software_pwm_task = None
        self._running = False
        self._cleanuped = False
        self._executor = ThreadPoolExecutor(thread_name_prefix='PWM_THREAD_')
        # Hardware PWM related address
        self._PWM_CHANNEL = 1 if self._bcm_pin in [12, 18] else 2
        self._PWM_OFFSET = (0x0 if self._PWM_CHANNEL == 1 else 0x10)
        self._MSEN_BIT = 7 if self._PWM_CHANNEL == 1 else 15
        self._PWEN_BIT = 0 if self._PWM_CHANNEL == 1 else 8
        # GPIO setup
        self._gpio_mode = (GPIO_MODE.ALT_FUNC0 if self._bcm_pin in [12, 13] else GPIO_MODE.ALT_FUNC5) if self._pwm_mode == PWM_MODE.HARDWARE else GPIO_MODE.OUTPUT

    def __del__(self):
        if self._running:
            print("WARN: PWM Still running")

    @ property
    def is_running(self):
        return self._running

    async def _parameter_changed(self):
        if not self._running:
            return
        if self._pwm_mode == PWM_MODE.HARDWARE:
            await self._hardware_pwm()

    async def cleanup(self):
        if self._cleanuped:
            return
        self._running = False
        if self._pwm_mode != PWM_MODE.HARDWARE:
            await self._software_pwm_task
        await self._gpio.cleanup()
        self._cleanuped = True

    async def start(self):
        """Start PWM
        """
        self._running = True
        self._gpio = await self._mmio.get_gpio(self._bcm_pin, self._gpio_mode)
        if self._pwm_mode == PWM_MODE.HARDWARE:
            await self._hardware_pwm_setup()
            await self._hardware_pwm()
        elif self._pwm_mode == PWM_MODE.THREADED:
            self._software_pwm_task = asyncio.get_event_loop().run_in_executor(self._executor, self._threaded_pwm)
        else:
            self._software_pwm_task = asyncio.create_task(self._software_pwm())

    def _threaded_pwm(self):
        print(f'INFO: start PWM thread on pin {self._bcm_pin}')
        while self._running:
            if self._duty > 0:
                self._gpio.write_nowait(True)
                time.sleep(self._duty / self._TIME_UNIT)
            if self._duty < self._period:
                self._gpio.write_nowait(False)
                time.sleep((self._period - self._duty) / self._TIME_UNIT)

    async def _software_pwm(self):
        try:
            print(f'INFO: start software PWM on pin {self._bcm_pin}')
            while self._running:
                if self._duty > 0:
                    await self._gpio.write(True)
                    await asyncio.sleep(self._duty / self._TIME_UNIT)
                if self._duty < self._period:
                    await self._gpio.write(False)
                    await asyncio.sleep((self._period - self._duty) / self._TIME_UNIT)
        except asyncio.CancelledError as e:
            pass

    async def _hardware_pwm_setup(self):
        # CLKPWM
        cm_pwmctl = self._mmio.clk_mmio[self._CM_PWMCTL_BASE]
        cm_pwmctl[4] = False
        # PASSWD
        cm_pwmctl[24:32] = self._CM_PASSWD[24:32]  # 5a
        self._mmio.clk_mmio[self._CM_PWMCTL_BASE] = cm_pwmctl
        # wait not busy
        while True:
            if not self._mmio.clk_mmio[self._CM_PWMCTL_BASE][7]:
                break
            await asyncio.sleep(0)
        # set 100MHz clock
        # Raspberry Pi 4      : PLLD 750MHz
        # Raspberry Pi Others : PLLD 500MHz
        cm_pwmdiv = self._CM_PASSWD.to_u32()
        divi = 7 if self._mmio._model == MODEL.BCM2711 else 5
        divf = 2048 if self._mmio._model == MODEL.BCM2711 else 0
        cm_pwmdiv |= (divi << 12)
        cm_pwmdiv |= divf
        self._mmio.clk_mmio[self._CM_PWMDIV_BASE] = BitArray(struct.pack("@I", cm_pwmdiv), True)
        # set PWMCTL use PLLD
        cm_pwmctl = self._mmio.clk_mmio[self._CM_PWMCTL_BASE]
        # PASSWD
        cm_pwmctl[24:32] = self._CM_PASSWD[24:32]  # 5a
        cm_pwmctl[4] = True  # enable
        cm_pwmctl[0:4] = [False, True, True, False]  # 6 = PLLD
        cm_pwmctl[9:11] = [True, False] if self._mmio._model == MODEL.BCM2711 else [False, False]  # MASH2 if BCM2711
        self._mmio.clk_mmio[self._CM_PWMCTL_BASE] = cm_pwmctl
        # wait busy
        while True:
            if self._mmio.clk_mmio[self._CM_PWMCTL_BASE][7]:
                break
            await asyncio.sleep(0)
        # enable pwm
        pwmctl = self._mmio.peri_mmio[self._CTL_BASE]
        pwmctl[self._MSEN_BIT] = True
        pwmctl[self._PWEN_BIT] = True
        self._mmio.peri_mmio[self._CTL_BASE] = pwmctl

    async def _hardware_pwm(self):
        self._mmio.peri_mmio[self._RNG_BASE + self._PWM_OFFSET] = BitArray(struct.pack("@I", self._period // self._PWM_DIV))
        self._mmio.peri_mmio[self._DAT_BASE + self._PWM_OFFSET] = BitArray(struct.pack("@I", self._duty // self._PWM_DIV))
        await asyncio.sleep(0)

    @ property
    def period(self):
        return self._period

    async def set_period(self, period: int):
        """set pwm period in nanoseconds

        * NOTE: minimum period is 10nanoseconds.

        Args:
            period (int): period nanoseconds
        """
        if period < 10:
            raise ValueError("Minimum period is 10 nanoseconds")
        self._period = period
        await self._parameter_changed()

    @ property
    def frequency(self):
        return self._TIME_UNIT//self._period

    async def set_frequency(self, frequency: int):
        """set pwm frequency in hz

        * NOTE: maximum frequency is 50MHz.

        Args:
            frequency (int): pwm frequency in hz
        """
        if frequency > 50000000:
            raise ValueError("Maximum frequency is 50MHz")
        self._period = self._TIME_UNIT//frequency
        await self._parameter_changed()

    @ property
    def duty(self):
        return self._duty

    async def set_duty(self, duty: int):
        """set pwm duty in nanoseconds

        * NOTE: minimum duty is 10nanoseconds.

        Args:
            duty (int): duty in nanoseconds
        """
        if duty < 10:
            raise ValueError("Minimum duty is 10 nanoseconds")
        self._duty = duty
        await self._parameter_changed()
