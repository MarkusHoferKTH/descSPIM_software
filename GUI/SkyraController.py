from CoboltLaserLogic import CoboltLaser
#import time

import serial
from serial.tools import list_ports
from serial.serialutil import SerialException
import time
import sys
import re
import logging

logger = logging.getLogger(__name__)

class SkyraController:

    """
    Creates a laser controller object called SkyraController that handles active laser wavelengths, power settings, and 
    emission functionality
    """

    def __init__(self, port="COM7", serial=None):

        """
        Initialization of the SkyraController-object and definition of possible illumination options
        """

        print("Connecting to Cobolt Skyra...")

        self.laser = CoboltLaser(port=port)

        self.force_safe_state(self.laser)

        self.turn_on_system()

        self.wait_until_key_switch(self.laser)

        self.laser.send_cmd("1eswm")
        self.laser.send_cmd("2eswm")
        self.laser.send_cmd("3eswm")
        self.laser.send_cmd("4eswm")

        # Wavelengths
        self.channels = {
            561: {"line": 1, "max_power": 100},
            638: {"line": 2, "max_power": 120},
            488: {"line": 3, "max_power": 80},
            405: {"line": 4, "max_power": 120},
        }

        self.active_wavelengths = set()

        self.is_laser_button_on = False

    def force_safe_state(self, laser):

        """
        Makes sure that all wavelengths are disabled and set to 0 mW in power
        """
    
        # Turn off power
        laser.send_cmd("l0")

        # Disable all wavelengths
        for line in [1, 2, 3, 4]:
            laser.send_cmd(f"{line}l0")
            laser.send_cmd(f"{line}sla 0")

        # Set all powers to 0
        for line in [1, 2, 3, 4]:
            laser.send_cmd(f"{line}p 0")

        time.sleep(5)

    def wait_until_key_switch(self, laser, timeout=60):
    
        """
        Waits for the safety key to Cobolt Skyra to be turned in order to turn on the laser system
        """

        print("\nWaiting for laser to reach COMPLETED state...")

        start = time.time()

        # Keeps printing until the key has been turned or time runs out
        while True:
            state = laser.send_cmd("gom?")
            print("STATE:", state)

            try:
                state=int(state)
            except ValueError:
                state=-1

            if state == 4: # 4 means ready
                print("Laser ready.")
                return True
            
            print()
            print("=== SWITCH KEY TO OFF AND ON AGAIN ===")
            print()

            if time.time() - start > timeout:
                raise RuntimeError("Laser never became ready.")

            time.sleep(3)
    
    def turn_on_system(self):

        """
        Turn on the laser system
        """

        self.laser.turn_on()
        self.laser.send_cmd("l1")
        

    def turn_off_system(self):

        """
        Turn off the laser system
        """

        self.laser.send_cmd("l0")
        self.laser.turn_off()

    def select_wavelength(self, wavelength):

        """
        Select avtive wavelengths from user entry
        """

        if wavelength not in self.channels:
            raise ValueError("Invalid wavelength")
        
        if wavelength in self.active_wavelengths:
            return
        
        line = self.channels[wavelength]["line"]
        print(f"Selected {wavelength} nm (line {line})")

        self.laser.send_cmd(f"{line}cp")
        if self.is_laser_button_on:
            self.laser.send_cmd(f"{line}l1")
            self.laser.send_cmd(f"{line}sla 1")

        self.active_wavelengths.add(wavelength)
    
    def deselect_wavelength(self, wavelength):

        """
        Deselect active wavelength from user entry
        """

        if wavelength not in self.channels:
            raise ValueError("Invalid wavelength")
        
        if wavelength not in self.active_wavelengths:
            return

        line = self.channels[wavelength]["line"]
        print(f"Deselected {wavelength} nm (line {line})")

        self.laser.send_cmd(f"{line}l0")
        self.laser.send_cmd(f"{line}sla 0")

        self.active_wavelengths.remove(wavelength)
    
    def set_power(self, power_mw):

        """
        Set laser power of selected wavelengths from user entry
        """

        if not self.active_wavelengths:
            raise RuntimeError("No wavelength selected")

        for wavelength in self.active_wavelengths:

            channel = self.channels[wavelength]
            line = channel["line"]

            if power_mw > channel["max_power"]:
                raise ValueError(f"{wavelength} exceeds max ({channel['max_power']} mW)")
        
            power_w = float(power_mw / 1000)

            self.laser.send_cmd(f"{line}p {power_w}")

            print(f"{wavelength} nm → {power_mw} mW set")
    
    def emission_on(self):
        
        """
        Turn on the laser emission
        """

        self.is_laser_button_on = True

        if not self.active_wavelengths: 
            raise RuntimeError("No wavelength selected")

        for wavelength in self.active_wavelengths:
            line = self.channels[wavelength]["line"]

            self.laser.send_cmd(f"{line}l1")
            self.laser.send_cmd(f"{line}sla 1")
    
    def emission_off(self):

        """
        Turn off the laser emission
        """

        self.is_laser_button_on = False

        for wavelength in list(self.active_wavelengths):
            line = self.channels[wavelength]["line"]

            self.laser.send_cmd(f"{line}l0")
            self.laser.send_cmd(f"{line}sla 0")

    def shutdown(self):

        """
        Safe shutdown of the laser system
        """

        self.emission_off()
        self.turn_off_system()
        self.laser.disconnect()
