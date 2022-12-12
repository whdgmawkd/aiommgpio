class MODEL:
    """Enumerate Raspberry Pi models
    """
    AUTO = 'AUTO'
    UNKNOWN = 'UNKNOWN'
    BCM2711 = 'BCM2711'
    BCM2709 = 'BCM2709'
    BCM2708 = 'BCM2708'
    BCM2837 = BCM2709
    BCM2836 = BCM2709
    BCM2835 = BCM2708


class GPIO_MODE:
    INPUT = 0
    OUTPUT = 1
    ALT_FUNC0 = 2
    ALT_FUNC5 = 3


class GPIO_PUD:
    DISABLE = 0
    PULL_DOWN = 1
    PULL_UP = 2


class PWM_MODE:
    AUTO = 'AUTO'
    SOFTWARE = 'SOFTWARE'
    HARDWARE = 'HARDWARE'
    THREADED = 'THREADED'


class EDGE:
    RISING = 'RISING'
    FALLING = 'FALLING'
    BOTH = 'BOTH'
