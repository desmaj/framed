import signal
import subprocess
import threading
import time

import gpiozero


# The pin numbers of the output relays.
RELAY_ONE = 18
RELAY_TWO = 17

# The pin numbers of the buttons.
BUTTON_ONE_START = 21
BUTTON_TWO_PAUSE = 22
BUTTON_THREE_SHUTDOWN = 23

# Define relay state values
RELAY_STATE_OPEN = 1  # HIGH
RELAY_STATE_CLOSED = 0  # LOW

# The command issued when the shotdown button is pressed.
SHUTDOWN_COMMAND = ["python", "/home/pi/Desktop/relay2.py"]


class FrameControls(object):
    """ The control interface for a digital frame.

        The frame is assumed to have three buttons.
        Button 1:
         - If there is a slideshow running and paused, unpause it. If there
           is no running slideshow, start one.
        Button 2:
         - If there is a slideshow running, pause it.
        Button 3:
         - Shutdown the device.
    """

    def __init__(self, slideshow_image_count, slideshow_delay):
        self._slideshow_image_count = slideshow_image_count
        self._slideshow_delay = slideshow_delay

        self._setup_buttons()
        self._setup_relay_pins()

        self._is_running = False
        self._is_running_lock = threading.RLock()
        self._should_run = False
        self._should_run_lock = threading.RLock()

    def _setup_buttons(self):
        """ Setup the button pins and assign them actions. """

        # Button one is the start button
        self._start_button = gpiozero.Button(BUTTON_ONE_START)
        self._start_button.when_pressed(self.start)

        # Button one is the stop button
        self._start_button = gpiozero.Button(BUTTON_TWO_PAUSE)
        self._start_button.when_pressed(self.pause)

        # Button one is the shutsown button
        self._start_button = gpiozero.Button(BUTTON_THREE_SHUTDOWN)
        self._start_button.when_pressed(self.shutdown)

    def _setup_relay_pins(self):
        """ Setup the relay pins and assign their initial state. """

        self._relay_one_pin = gpiozero.Device.pin_factory.pin(RELAY_ONE)
        self._relay_one_pin.output_with_state(RELAY_STATE_OPEN)
        self._relay_two_pin = gpiozero.Device.pin_factory.pin(RELAY_TWO)
        self._relay_two_pin.output_with_state(RELAY_STATE_OPEN)

    @property
    def should_run(self):
        with self._should_run_lock:
            return self._should_run

    @property
    def is_running(self):
        with self._is_running_lock:
            return self._is_running

    def start(self):
        """ Start a slideshow, or unpause one that is paused. """
        if self.is_running:
            with self._should_run_lock:
                self._should_run = True
        else:
            with self._is_running_lock:
                self._is_running = True
                self.run()

    def pause(self):
        """ Request that the slideshow pause. """
        with self._should_run_lock:
            self._should_run = False

    def shutdown(self):
        subprocess.call(SHUTDOWN_COMMAND)

    def run(self):
        slideshow_thread = threading.Thread(
            target=self._slideshow,
            args=(
                self._slideshow_image_count,
                self._slideshow_delay,
            ),
        )
        slideshow_thread.start()

    def _slideshow(self, image_count, step_delay):
        """ Start a slideshow that advances periodically. """

        for image_index in range(image_count):
            if self.should_run:
                self._advance()
            else:
                time.sleep(self._slideshow_delay)
        with self._is_running_lock:
            self._is_running = False
        with self._should_run_lock:
            self._should_run = False

    def _advance(self):
        """ Perform the pin manipulation required to advance the slideshow. """
        relay_open_duration = 0.25
        relay_closed_duration = (
            (self._slideshow_delay / 2.0) -
            relay_open_duration
        )

        self._relay_one_pin.state = RELAY_STATE_CLOSED
        time.sleep(relay_open_duration)
        self._relay_one_pin.state = RELAY_STATE_OPEN
        time.sleep(relay_closed_duration)
        self._relay_two_pin.state = RELAY_STATE_CLOSED
        time.sleep(relay_open_duration)
        self._relay_two_pin.state = RELAY_STATE_OPEN
        time.sleep(relay_closed_duration)


if __name__ == '__main__':
    controls = FrameControls(90, 1.5)
    signal.pause()
