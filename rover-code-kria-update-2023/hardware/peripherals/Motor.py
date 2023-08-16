from pynq.lib.rpi import GPIO

class Motor:
    def __init__(self):
        self.gpio_pin = None

    def select_gpio_pin(self, pin_number):
        self.gpio_pin = Arduino_IO(pin_number, 'out')

    def update_pwm(self):
        current_time = time.monotonic()
        if current_time >= self.next_toggle_time:
            if GPIO.input(self.pwm_pin) == GPIO.HIGH:
                off_time = 1 / self.frequency * (1 - (self.duty_cycle / 100))
                self.next_toggle_time = current_time + off_time
                GPIO.output(self.pwm_pin, GPIO.LOW)
            else:
                on_time = 1 / self.frequency * (self.duty_cycle / 100)
                self.next_toggle_time = current_time + on_time
                GPIO.output(self.pwm_pin, GPIO.HIGH)

        
        