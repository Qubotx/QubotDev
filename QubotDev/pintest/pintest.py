#!/usr/bin/env python3
"""
Jetson Orin NX GPIO Pin 33 Test Script
Interactive terminal control for PWM, GPIO HIGH/LOW, and duty cycle testing
"""

import Jetson.GPIO as GPIO
import time
import sys


class Pin33Controller:
    def __init__(self):
        self.pin = 33
        self.pwm_frequency = 300
        self.pwm_object = None
        self.pwm_enabled = False
        self.current_duty_cycle = 0

        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        print(f"GPIO Pin {self.pin} initialized as OUTPUT")

    def enable_pwm(self):
        """Enable PWM on the pin"""
        if not self.pwm_enabled:
            self.pwm_object = GPIO.PWM(self.pin, self.pwm_frequency)
            self.pwm_object.start(self.current_duty_cycle)
            self.pwm_enabled = True
            print(
                f"PWM enabled at {self.pwm_frequency}Hz, duty cycle: {self.current_duty_cycle}%"
            )
        else:
            print("PWM is already enabled")

    def disable_pwm(self):
        """Disable PWM on the pin"""
        if self.pwm_enabled and self.pwm_object:
            self.pwm_object.stop()
            self.pwm_object = None
            self.pwm_enabled = False
            print("PWM disabled")
        else:
            print("PWM is already disabled")

    def set_duty_cycle(self, duty_cycle):
        """Set PWM duty cycle (0-100)"""
        if not self.pwm_enabled:
            print("PWM is not enabled. Enable PWM first.")
            return

        duty_cycle = max(0, min(100, float(duty_cycle)))
        self.current_duty_cycle = duty_cycle
        self.pwm_object.ChangeDutyCycle(duty_cycle)
        print(f"Duty cycle set to: {duty_cycle}%")

    def set_frequency(self, frequency):
        """Change PWM frequency (requires PWM restart)"""
        frequency = max(1, int(frequency))
        was_enabled = self.pwm_enabled

        if was_enabled:
            self.disable_pwm()

        self.pwm_frequency = frequency
        print(f"PWM frequency changed to: {frequency}Hz")

        if was_enabled:
            self.enable_pwm()

    def set_high(self):
        """Set pin to HIGH (3.3V)"""
        if self.pwm_enabled:
            print("Warning: PWM is enabled. Disable PWM first for clean GPIO control.")
            return
        GPIO.output(self.pin, GPIO.HIGH)
        print("Pin set to HIGH (3.3V)")

    def set_low(self):
        """Set pin to LOW (0V)"""
        if self.pwm_enabled:
            print("Warning: PWM is enabled. Disable PWM first for clean GPIO control.")
            return
        GPIO.output(self.pin, GPIO.LOW)
        print("Pin set to LOW (0V)")

    def get_status(self):
        """Display current pin status"""
        print(f"\n--- Pin {self.pin} Status ---")
        print(f"PWM Enabled: {self.pwm_enabled}")
        print(f"PWM Frequency: {self.pwm_frequency}Hz")
        print(f"Current Duty Cycle: {self.current_duty_cycle}%")
        if not self.pwm_enabled:
            print("GPIO Mode: Available for HIGH/LOW control")
        print("------------------------\n")

    def cleanup(self):
        """Clean up GPIO resources"""
        if self.pwm_enabled:
            self.disable_pwm()
        GPIO.cleanup()
        print("GPIO cleanup completed")


def print_help():
    """Print available commands"""
    help_text = """
Available Commands:
    pwm on          - Enable PWM
    pwm off         - Disable PWM
    duty <0-100>    - Set duty cycle (0-100%)
    freq <Hz>       - Set PWM frequency in Hz
    high            - Set pin HIGH (3.3V)
    low             - Set pin LOW (0V)
    status          - Show current pin status
    help            - Show this help
    exit/quit       - Exit program
    
Examples:
    duty 50         - Set 50% duty cycle
    freq 1000       - Set frequency to 1000Hz
    pwm on          - Enable PWM
    high            - Set pin to 3.3V
"""
    print(help_text)


def main():
    print("=== Jetson Orin NX Pin 33 Test Controller ===")
    print("Type 'help' for available commands")

    controller = Pin33Controller()

    try:
        while True:
            try:
                command = input("\nPin33> ").strip().lower()

                if not command:
                    continue

                parts = command.split()
                cmd = parts[0]

                if cmd in ["exit", "quit"]:
                    break
                elif cmd == "help":
                    print_help()
                elif cmd == "status":
                    controller.get_status()
                elif cmd == "pwm":
                    if len(parts) > 1:
                        if parts[1] == "on":
                            controller.enable_pwm()
                        elif parts[1] == "off":
                            controller.disable_pwm()
                        else:
                            print("Use 'pwm on' or 'pwm off'")
                    else:
                        print("Use 'pwm on' or 'pwm off'")
                elif cmd == "duty":
                    if len(parts) > 1:
                        try:
                            duty = float(parts[1])
                            controller.set_duty_cycle(duty)
                        except ValueError:
                            print("Invalid duty cycle value. Use 0-100")
                    else:
                        print("Usage: duty <0-100>")
                elif cmd == "freq":
                    if len(parts) > 1:
                        try:
                            freq = int(parts[1])
                            controller.set_frequency(freq)
                        except ValueError:
                            print("Invalid frequency value. Use integer Hz")
                    else:
                        print("Usage: freq <Hz>")
                elif cmd == "high":
                    controller.set_high()
                elif cmd == "low":
                    controller.set_low()
                else:
                    print(
                        f"Unknown command: {cmd}. Type 'help' for available commands."
                    )

            except KeyboardInterrupt:
                print("\nUse 'exit' to quit properly")
            except Exception as e:
                print(f"Error: {e}")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        controller.cleanup()
        print("Program terminated")


if __name__ == "__main__":
    main()
