from pynq.lib.arduino import Arduino_IO

class Motor:
    def __init__(self):
        self.gpio_pin = None

    def select_gpio_pin(self, pin_number):
        self.gpio_pin = Arduino_IO(pin_number, 'out')

    def update_gpio(self, value):
        if self.gpio_pin is None:
            raise ValueError("No GPIO pin selected. Call select_gpio_pin() first.")
        self.gpio_pin.write(value)