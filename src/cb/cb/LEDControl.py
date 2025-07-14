import time
import threading
from pi5neo import Pi5Neo

class LEDControl:
    def __init__(self, device_path, led_count, spi_speed=800):
        self.neo = Pi5Neo(device_path, led_count, spi_speed)
        self.led_count = led_count
        self.off_color = (0, 0, 0)
        self.blink_thread = None
        self.stop_blink_event = threading.Event()

    def fill(self, r, g, b):
        self.neo.fill_strip(r, g, b)
        self.neo.update_strip()

    def set_color(self, index, r, g, b):
        self.neo.set_led_color(index, r, g, b)
        self.neo.update_strip()
        
    def fill_selected(self, indices, r, g, b):
        """Set a group of specific LEDs to the same color."""
        for i in indices:
            self.neo.set_led_color(i, r, g, b)
        self.neo.update_strip()

    def _blink_loop(self, indices, frequency, color, duration=None):
        delay = 1 / (2 * frequency)
       
        r, g, b = color
        if duration:
            end_time = time.time() + duration
            while time.time() < end_time and not self.stop_blink_event.is_set():
                for i in indices:
                    self.neo.set_led_color(i, r, g, b)
                self.neo.update_strip()
                time.sleep(delay)

                for i in indices:
                    self.neo.set_led_color(i, *self.off_color)
                self.neo.update_strip()
                time.sleep(delay)#klicken
        else:    
            while not self.stop_blink_event.is_set():
                for i in indices:
                    self.neo.set_led_color(i, r, g, b)
                self.neo.update_strip()
                time.sleep(delay)

                for i in indices:
                    self.neo.set_led_color(i, *self.off_color)
                self.neo.update_strip()
                time.sleep(delay) 

    def blink(self, indices, frequency, color, duration=None):
        """Start blinking in a non-blocking way. Duration=None for infinite."""
        if self.blink_thread and self.blink_thread.is_alive():
            self.stop_blink_event.set()
            self.blink_thread.join()

        self.stop_blink_event.clear()
        self.blink_thread = threading.Thread(
            target=self._blink_loop,
            args=(indices, frequency, color, duration),
            daemon=True
        )
        self.blink_thread.start()

    def stop_blink(self):
        """Stop ongoing blinking."""
        self.stop_blink_event.set()
        if self.blink_thread:
            self.blink_thread.join()



if __name__ == '__main__':
    wrapper = LEDControl('/dev/spidev0.0', 8)

    print("Filling strip with white...")
    wrapper.fill(50, 50, 50)
    time.sleep(2)

    print("Turning off all LEDs.")
    wrapper.fill(0, 0, 0)

    print("Blinking LEDs 1 and 3 in green at 2Hz for 5 seconds...")
    while True:
        wrapper.blink(list(range(0, 9)), frequency=2, color=[0, 50, 0], duration=None)

    #time.sleep(10)  # Allow blinking to complete

#    print("Turning off all LEDs.")
#    wrapper.fill(0, 0, 0)
