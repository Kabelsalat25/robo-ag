from machine import I2C, Pin
import time
import math
from motoron import MotoronI2C

# I2C: ESP32 DevKit – SDA GPIO 21, SCL GPIO 22
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)

# Motoron M3S256 und IR Seeker
motor_controller = MotoronI2C(bus=i2c)
ir_address = 0x08

# Motorkanäle am Motoron: M1, M2, M3
motor_channels = (1, 2, 3)

# Fahrtrichtungen der Räder, von vorne aus
wheel_angles = (60, 180, 300)

# Bei Bedarf für einzelne Motoren auf -1 setzen, falls die Richtung verkehrt ist
motor_polarity = (1, 1, 1)

max_speed = 800
loop_delay_ms = 100

# Bewegungswerte zwischen -1.0 und +1.0
orbit_speed = 0.16
alignment_speed = 0.12
forward_speed = 0.14
rear_escape_speed = 0.14

# Maximale Zeit für den Vorwärtsantrieb, wenn der Ball direkt vorne ist
forward_limit_ms = 1000
forward_started_at = None


def stop_motors():
    for channel in motor_channels:
        motor_controller.set_speed(channel, 0)


def read_ball():
    """Liest Richtung und Signalstärke des IR Seekers."""
    try:
        data = i2c.readfrom(ir_address, 2)
        sector = data[0]
        strength = data[1]

        # 0 bedeutet: kein Ball erkannt
        if sector == 0:
            return None, strength

        # Sektoren außerhalb des erwarteten Bereichs ignorieren
        if not 1 <= sector <= 12:
            return None, strength

        return sector, strength

    except OSError:
        # Bei einem Lesefehler sicherheitshalber stoppen
        return None, 0


def mix_wheels(right, forward):
    """
    Berechnet die drei Radgeschwindigkeiten.
    right: Bewegung nach rechts
    forward: Bewegung nach vorne
    """
    speeds = []

    for angle in wheel_angles:
        radians = math.radians(angle)

        # Projektion der gewünschten Bewegung auf die jeweilige Radachse
        wheel_speed = right * math.sin(radians) + forward * math.cos(radians)
        speeds.append(wheel_speed)

    largest = max(1.0, max(abs(speed) for speed in speeds))
    return [speed / largest for speed in speeds]


def drive(right, forward):
    wheel_speeds = mix_wheels(right, forward)

    for index, channel in enumerate(motor_channels):
        speed = int(
            wheel_speeds[index]
            * motor_polarity[index]
            * max_speed
        )
        motor_controller.set_speed(channel, speed)


def choose_movement(sector):
    """
    Einfache Ausweich- und Annäherungslogik.
    Positive right-Werte bedeuten Bewegung nach rechts.
    """
    if sector is None:
        return 0.0, 0.0

    # Vorne: kurz geradeaus fahren
    if sector == 12:
        return 0.0, forward_speed

    # Ball vorne rechts: nach rechts ausrichten
    if sector in (1, 2):
        return alignment_speed, 0.0

    # Ball rechts: rechts herum zum Ball fahren
    if sector in (3, 4):
        return orbit_speed, -orbit_speed

    # Ball hinten rechts: nach hinten und rechts ausweichen
    if sector in (5,):
        return rear_escape_speed, -rear_escape_speed

    # Ball hinten: nach hinten ausweichen
    if sector == 6:
        return 0.0, -rear_escape_speed

    # Ball hinten links: nach hinten und links ausweichen
    if sector in (7,):
        return -rear_escape_speed, -rear_escape_speed

    # Ball links: links herum zum Ball fahren
    if sector in (8, 9):
        return -orbit_speed, -orbit_speed

    # Ball vorne links: nach links ausrichten
    if sector in (10, 11):
        return -alignment_speed, 0.0

    return 0.0, 0.0


# Motoron vorbereiten
motor_controller.reinitialize()
motor_controller.disable_crc()
motor_controller.clear_reset_flag()
motor_controller.set_command_timeout_milliseconds(500)

stop_motors()
print("Robotersteuerung gestartet")

while True:
    sector, strength = read_ball()

    if sector == 12:
        if forward_started_at is None:
            forward_started_at = time.ticks_ms()

        elapsed_ms = time.ticks_diff(time.ticks_ms(), forward_started_at)

        if elapsed_ms > forward_limit_ms:
            right, forward = 0.0, 0.0
        else:
            right, forward = choose_movement(sector)
    else:
        forward_started_at = None
        right, forward = choose_movement(sector)

    drive(right, forward)

    print(
        "Sektor:", sector,
        "Signal:", strength,
        "Rechts:", round(right, 2),
        "Vorwaerts:", round(forward, 2)
    )

    time.sleep_ms(loop_delay_ms)