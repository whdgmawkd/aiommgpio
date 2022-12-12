import asyncio
from aiommgpio import RPiMMIO, PWM_MODE, GPIO_MODE


async def main():
    mmio = RPiMMIO()
    pwm = await mmio.get_pwm(18, PWM_MODE.HARDWARE)
    gpio = await mmio.get_gpio(12, GPIO_MODE.OUTPUT)
    await pwm.set_frequency(25000)  # 25KHz frequency
    await pwm.set_duty(20000)  # 20000 nanoseconds duty
    await pwm.start()
    print(f'PWM Period: {pwm.period}')
    print(f'PWM Frequency: {pwm.frequency}')
    print(f'PWM Duty: {pwm.duty}')
    await gpio.write(True)
    await asyncio.sleep(5)
    await gpio.write(False)
    await pwm.cleanup()
    await gpio.cleanup()


if __name__ == '__main__':
    asyncio.run(main())
