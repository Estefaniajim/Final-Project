"""
Cat Runner - Distance-Based Endless Runner
ESP32S3 XIAO - CircuitPython
Control a sleepy cat using a distance sensor!
"""

import time
import board
import busio
import displayio
import terminalio
import digitalio
import random
import math
from pwmio import PWMOut
from adafruit_display_text import label
import i2cdisplaybus
import adafruit_displayio_ssd1306
import adafruit_adxl34x
import neopixel
from rainbowio import colorwheel
from adafruit_debouncer import Debouncer
from rotary_encoder import RotaryEncoder
import adafruit_hcsr04

# ============================================
# HARDWARE INITIALIZATION
# ============================================

print("[INIT] Starting Cat Runner...")

# Release any existing displays
displayio.release_displays()

# I2C setup (Display + Accelerometer)
i2c = busio.I2C(board.SCL, board.SDA)

# Display setup (SSD1306)
display_bus = i2cdisplaybus.I2CDisplayBus(i2c, device_address=0x3C)
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=64)

# Accelerometer setup (ADXL345)
accelerometer = adafruit_adxl34x.ADXL345(i2c)

# Distance Sensor setup (HC-SR04 using Adafruit library)
# Increase timeout to 0.1s to avoid timing out during busy game loop
sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.D0, echo_pin=board.D1, timeout=0.1)

# Rotary Encoder setup (D8, D9) - For MENU navigation only
# Note: Adjust pulses_per_detent (1, 2, or 4) to match your encoder hardware
encoder = RotaryEncoder(board.D8, board.D9, debounce_ms=50, pulses_per_detent=4)

# Rotary button (D3) - Pause/Select
rotary_btn_pin = digitalio.DigitalInOut(board.D3)
rotary_btn_pin.direction = digitalio.Direction.INPUT
rotary_btn_pin.pull = digitalio.Pull.UP
rotary_btn = Debouncer(rotary_btn_pin)

# Push button (D10) - Boost
push_btn_pin = digitalio.DigitalInOut(board.D10)
push_btn_pin.direction = digitalio.Direction.INPUT
push_btn_pin.pull = digitalio.Pull.DOWN
push_btn = Debouncer(push_btn_pin)

# Buzzer (D6)
buzzer = PWMOut(board.D6, variable_frequency=True)
buzzer.frequency = 440
buzzer.duty_cycle = 0

# Neopixel (D7)
pixel = neopixel.NeoPixel(board.D7, 1, brightness=0.5, auto_write=False)

print("[INIT] ✓ Hardware initialized!")
print("[INIT] - Distance Sensor: D0 (TRIG), D1 (ECHO)")
print("-" * 50)

# ============================================
# CONSTANTS & CONFIGURATION
# ============================================

# Display dimensions
SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64

# Game states
STATE_DIFFICULTY_SELECT = 0
STATE_LEVEL_START = 1
STATE_PLAYING = 2
STATE_PAUSED = 3
STATE_ASLEEP = 4
STATE_LEVEL_COMPLETE = 5
STATE_LEVEL_FAILED = 6
STATE_GAME_WON = 7

# Difficulty settings
DIFFICULTY_EASY = 0
DIFFICULTY_MEDIUM = 1
DIFFICULTY_HARD = 2

DIFFICULTY_NAMES = ["EASY", "MEDIUM", "HARD", "EXIT"]

# Lane system
NUM_LANES = 3
LANE_HEIGHT = 16
LANE_Y_POSITIONS = [14, 30, 46]  # Center Y of each lane
HUD_HEIGHT = 10

# Distance sensor ranges (cm)
DISTANCE_RANGES = [
    (0, 10),    # Lane 0 (TOP)
    (10, 20),   # Lane 1 (MIDDLE)
    (20, 30)    # Lane 2 (BOTTOM)
]
DISTANCE_LOCK_THRESHOLD = 30  # Lock lane if > 30cm

# Cat constants
CAT_WIDTH = 12
CAT_HEIGHT = 12
CAT_X = 10  # Fixed X position (left side)
LANE_TRANSITION_SPEED = 4  # Pixels per frame

# Game constants per difficulty [Easy, Medium, Hard]
BASE_SPEEDS = [30, 40, 50]  # Pixels per second
OBSTACLE_SPAWN_INTERVALS = [2000, 1400, 1000]  # ms
SLEEP_CHANCE_MULTIPLIERS = [1.0, 1.5, 2.0]
TIME_PER_500M = [240, 180, 120]  # seconds (doubled for more generous time limits)
HEALTH_DRAIN_ASLEEP = [1, 2, 3]  # % per second

# Level progression
MAX_LEVELS = 10
LEVEL_DISTANCES = [100, 120, 140, 160, 180, 200, 220, 240, 260, 280]  # meters (very short!)
COIN_REQUIREMENTS = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11]  # minimum fish per level 

# Object constants
OBJ_WIDTH = 8
OBJ_HEIGHT = 8
COIN_SPAWN_INTERVAL = 1500  # ms

# Boost system (discrete charges)
MAX_BOOST_CHARGES = 4  # Maximum boost charges
BOOST_DURATION = 1000  # ms per boost use
BOOST_RECHARGE_TIME = 3000  # ms to gain 1 charge back
BOOST_SPEED_MULTIPLIER = 2.0

# Sleep mechanic
SLEEP_CHECK_INTERVAL = 10000  # ms
SLEEP_CHANCE_BASE = [5, 12, 20]  # % for levels 1, 5, 10
SHAKE_THRESHOLD = 1.5  # g (lowered from 2.0 for easier wake-up)

# Health system
MAX_HEALTH = 100
FOOD_HEAL_AMOUNT = 10  # HP restored per fish
OBSTACLE_DAMAGE = 20
INVINCIBILITY_TIME = 500  # ms after hit

# Frame rate
TARGET_FPS = 30
FRAME_TIME = 1000 // TARGET_FPS

# Object types
OBJ_FENCE = 0

# Colors
COLOR_GREEN = (0, 255, 0)
COLOR_YELLOW = (255, 255, 0)
COLOR_RED = (255, 0, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_OFF = (0, 0, 0)

# ============================================
# SPRITE DEFINITIONS
# ============================================

# Cat running (12x12)
# They are the same, no animation sorry
SPRITE_CAT_RUN_1 = [
    0b111100001111,
    0b100110011001,
    0b100011110001,
    0b110100001011,
    0b111000000110,
    0b010100001010,
    0b011110011110,
    0b010100001010,
    0b010001100010,
    0b001000000100,
    0b000100001000,
    0b000011110000,
]

SPRITE_CAT_RUN_2 = [
    0b111100001111,
    0b100110011001,
    0b100011110001,
    0b110100001011,
    0b111000000110,
    0b010100001010,
    0b011110011110,
    0b010100001010,
    0b010001100010,
    0b001000000100,
    0b000100001000,
    0b000011110000,
]

# Cat sleeping (12x12)
SPRITE_CAT_SLEEP = [
    0b111100001111,
    0b100110011001,
    0b100011110001,
    0b110100001011,
    0b111000000110,
    0b010100001010,
    0b011110011110,
    0b010100001010,
    0b010001100010,
    0b001000000100,
    0b000100001000,
    0b000011110000,
]

# Cat boosting (12x12)
SPRITE_CAT_BOOST = [
    0b111100001111,
    0b100110011001,
    0b100011110001,
    0b110100001011,
    0b111000000110,
    0b010000000010,
    0b011110011110,
    0b010000000010,
    0b010001100010,
    0b001000000100,
    0b000100001000,
    0b000011110000,
]

SPRITE_FENCE = [
    0b10010010,
    0b10010010,
    0b11111111,
    0b10010010,
    0b10010010,
    0b11111111,
    0b10010010,
    0b10010010,
]

# Fish coin (6x6)
SPRITE_FISH = [
    0b000000,
    0b000110,
    0b101001,
    0b110001,
    0b101001,
    0b000110,
]

# ============================================
# HELPER FUNCTIONS
# ============================================

# This performs a linear interpolation between two numbers.
# It returns a value that is t percent of the way between start and end.
# Calculating sleep probability based on level progress
def lerp(start, end, t):
    """Linear interpolation"""
    return start + (end - start) * t

# Draws a bitmap sprite (pixel art) on the OLED screen.
# Drawing the cat
# Drawing obstacles (fences)
# Drawing fish
# Drawing lane dotted lines
def draw_sprite(group, sprite_data, x, y, width, height, color=0xFFFFFF):
    """Draw a sprite bitmap"""
    bitmap = displayio.Bitmap(width, height, 2)
    for row in range(height):
        if row < len(sprite_data):
            for col in range(width):
                if sprite_data[row] & (1 << (width - 1 - col)):
                    bitmap[col, row] = 1
    palette = displayio.Palette(2)
    palette[0] = 0x000000
    palette[1] = color
    tile_grid = displayio.TileGrid(bitmap, pixel_shader=palette, x=int(x), y=int(y))
    group.append(tile_grid)
    return tile_grid

# Creates a text label and adds it to the display group.
def draw_text(group, text, x, y, color=0xFFFFFF):
    """Draw text label"""
    lbl = label.Label(terminalio.FONT, text=text, x=x, y=y, color=color)
    group.append(lbl)
    return lbl

# ============================================
# CLASSES
# ============================================

# Wraps the HC-SR04 ultrasonic sensor so the game can convert distance → lane control.
# Throttling readings so the sensor doesn’t get spammed
class DistanceSensor:
    """HC-SR04 Ultrasonic Distance Sensor (using Adafruit library)"""
    def __init__(self, sonar):
        self.sonar = sonar
        self.last_distance = 15  # Default to middle lane
        self.last_valid_distance = 15  # Cache last valid reading
        self.locked = False
        self.last_lane = 1  # Start in middle lane
        self.last_read_time = 0
        self.read_interval = 150  # Read every 300ms - game loop is very busy
        self.timeout_count = 0  # Track timeouts for logging

    def read_distance(self):
        """Returns distance in cm, or None if invalid (throttled reading)"""
        current_time = time.monotonic_ns() // 1000000

        # Throttle readings - HC-SR04 needs time between measurements
        if current_time - self.last_read_time < self.read_interval:
            # Return cached value (not a real reading)
            return self.last_valid_distance

        self.last_read_time = current_time

        try:
            distance = self.sonar.distance

            # Sanity check (HC-SR04 range: 2cm - 400cm)
            if distance < 1 or distance > 400:
                print(f"[DISTANCE] Out of range: {distance:.1f}cm, using cached")
                return self.last_valid_distance  # Return cached on error

            # Update cache with new reading
            if distance != self.last_valid_distance:
                print(f"[DISTANCE] NEW reading: {distance:.1f}cm (was {self.last_valid_distance:.1f}cm)")
            self.last_valid_distance = distance
            return distance

        except RuntimeError as e:
            # Timeout or no echo - return cached value
            # Only log every 10th timeout to reduce spam
            self.timeout_count += 1
            if self.timeout_count % 10 == 0:
                print(f"[DISTANCE] Timeouts: {self.timeout_count}, using cached value")
            return self.last_valid_distance
        except Exception as e:
            print(f"[DISTANCE] Error: {e}, using cached {self.last_valid_distance}cm")
            return self.last_valid_distance

    def get_lane(self):
        """Returns lane number (0=top, 1=mid, 2=bottom) or None if locked"""
        distance = self.read_distance()

        print(f"[DISTANCE] Measured: {distance}cm")

        # If no valid reading or > threshold, lock current lane
        if distance is None or distance > DISTANCE_LOCK_THRESHOLD:
            if not self.locked:
                print(f"[DISTANCE] Locked to lane {self.last_lane}")
            self.locked = True
            return None  # Keep current lane

        self.locked = False
        self.last_distance = distance

        # Map distance to lane
        for lane_idx, (min_dist, max_dist) in enumerate(DISTANCE_RANGES):
            if min_dist <= distance < max_dist:
                if lane_idx != self.last_lane:
                    print(f"[DISTANCE] {distance:.1f}cm → Lane {lane_idx}")
                    self.last_lane = lane_idx
                return lane_idx

        # If distance is 20-30, return bottom lane
        if distance >= 20 and distance <= 30:
            if 2 != self.last_lane:
                print(f"[DISTANCE] {distance:.1f}cm → Lane 2")
                self.last_lane = 2
            return 2

        return None

# Centralizes all buzzer sound effects, including:
class AudioManager:
    """Buzzer sound effects"""
    def __init__(self, buzzer):
        self.buzzer = buzzer
        self.tone_start = 0
        self.tone_duration = 0
        self.is_playing = False

    def play_tone(self, frequency, duration):
        """Play a tone for duration (ms)"""
        self.buzzer.frequency = frequency
        self.buzzer.duty_cycle = 32768
        self.tone_start = time.monotonic_ns() // 1000000
        self.tone_duration = duration
        self.is_playing = True

    def update(self):
        """Update sound playback"""
        if self.is_playing:
            elapsed = (time.monotonic_ns() // 1000000) - self.tone_start
            if elapsed >= self.tone_duration:
                self.buzzer.duty_cycle = 0
                self.is_playing = False

    def stop(self):
        self.buzzer.duty_cycle = 0
        self.is_playing = False

    # Sound effects
    def menu_nav(self):
        self.play_tone(200, 50)

    def menu_confirm(self):
        self.play_tone(400, 100)

    def coin_collect(self):
        self.play_tone(600, 80)

    def obstacle_hit(self):
        self.play_tone(100, 200)

    def boost_start(self):
        self.play_tone(800, 150)

    def fall_asleep(self):
        self.play_tone(300, 300)

    def wake_up(self):
        self.play_tone(400, 200)

    def level_complete(self):
        self.play_tone(600, 300)

    def level_failed(self):
        self.play_tone(150, 500)

    def game_won(self):
        self.play_tone(1000, 500)

# Controls the single RGB neopixel for feedback.
class NeopixelManager:
    """LED feedback"""
    def __init__(self, pixel):
        self.pixel = pixel
        self.flash_start = 0
        self.flash_duration = 0
        self.flash_color = COLOR_OFF
        self.base_color = COLOR_GREEN
        self.is_flashing = False
        self.pulse_time = 0

    def set_health_color(self, health):
        """Set color based on health"""
        if health > 66:
            self.base_color = COLOR_GREEN
        elif health > 33:
            self.base_color = COLOR_YELLOW
        else:
            self.base_color = COLOR_RED
        if not self.is_flashing:
            self.pixel.fill(self.base_color)
            self.pixel.show()

    def flash(self, color, duration):
        """Flash a color briefly"""
        self.flash_color = color
        self.flash_duration = duration
        self.flash_start = time.monotonic_ns() // 1000000
        self.is_flashing = True
        self.pixel.fill(color)
        self.pixel.show()

    def pulse_blue(self):
        """Pulsing blue for sleep"""
        self.pulse_time += 0.1
        brightness = int((math.sin(self.pulse_time) + 1) * 64)
        self.pixel.fill((0, 0, brightness))
        self.pixel.show()

    def update(self):
        """Update LED state"""
        if self.is_flashing:
            elapsed = (time.monotonic_ns() // 1000000) - self.flash_start
            if elapsed >= self.flash_duration:
                self.is_flashing = False
                self.pixel.fill(self.base_color)
                self.pixel.show()

    def rainbow_cycle(self, step):
        """Rainbow effect"""
        self.pixel.fill(colorwheel(step & 255))
        self.pixel.show()

    def set_color(self, color):
        """Set solid color"""
        self.pixel.fill(color)
        self.pixel.show()

    def off(self):
        """Turn off"""
        self.pixel.fill(COLOR_OFF)
        self.pixel.show()

# Reads all input devices consistently:
class InputHandler:
    """All input devices"""
    LONG_PRESS_MS = 1000  # 1 second for long press (acts as rotary button)

    def __init__(self, encoder, rotary_btn, push_btn, accelerometer, distance_sensor):
        self.encoder = encoder
        self.rotary_btn = rotary_btn
        self.push_btn = push_btn
        self.accelerometer = accelerometer
        self.distance_sensor = distance_sensor
        self.last_encoder_pos = 0
        # Long press tracking for push button
        self.push_press_start = 0
        self.push_long_triggered = False

    def read_encoder_delta(self):
        """Encoder rotation (for menus)"""
        self.encoder.update()
        current_pos = self.encoder.position
        delta = current_pos - self.last_encoder_pos

        # Detailed logging for debugging
        if delta != 0:
            print(f"[ENCODER] Position: {self.last_encoder_pos} → {current_pos} | Delta: {delta:+d}")

        self.last_encoder_pos = current_pos
        return delta

    def read_rotary_button(self):
        """Rotary button press"""
        self.rotary_btn.update()
        return self.rotary_btn.fell

    def update_push_button(self):
        """Update push button state and track timing for long press"""
        self.push_btn.update()
        current_time = time.monotonic_ns() // 1000000

        # With pull-down circuit (VCC->btn->pin->resistor->GND):
        # - rose = button pressed (signal goes HIGH)
        # - fell = button released (signal goes LOW)

        # Button just pressed down - start timing
        if self.push_btn.rose:
            self.push_press_start = current_time
            self.push_long_triggered = False
            print(f"[PUSH] Button PRESSED (rose) | start_time={current_time}")

        # Button released - reset
        if self.push_btn.fell:
            hold_duration = current_time - self.push_press_start if self.push_press_start > 0 else 0
            print(f"[PUSH] Button RELEASED (fell) | held for {hold_duration}ms | long_triggered={self.push_long_triggered}")
            self.push_press_start = 0

    def read_push_button(self):
        """Push button short press (released before 3 seconds)"""
        # Only count as short press if we didn't trigger long press
        # fell = button released with pull-down circuit
        if self.push_btn.fell and not self.push_long_triggered:
            return True
        return False

    def read_push_long_press(self):
        """Push button long press (held for 3 seconds) - acts as rotary button"""
        current_time = time.monotonic_ns() // 1000000

        # Check if button is held and we haven't triggered yet
        # Note: value is False when pressed (pull-down: VCC->btn->pin->resistor->GND)
        is_pressed = not self.push_btn.value

        if self.push_press_start > 0 and not self.push_long_triggered:
            hold_duration = current_time - self.push_press_start

            # Debug: log every ~500ms
            if hold_duration % 500 < 50:
                print(f"[PUSH] Holding: {hold_duration}ms | is_pressed={is_pressed} | start={self.push_press_start}")

            # Trigger on 3 seconds regardless of current button state
            # (in case value reading is unreliable)
            if hold_duration >= self.LONG_PRESS_MS:
                self.push_long_triggered = True
                print(f"[INPUT] *** LONG PRESS TRIGGERED *** (held {hold_duration}ms)")
                return True

        return False

    def read_push_held(self):
        """Push button held (for boost) - only if not in long press territory"""
        # Don't report held if we've triggered long press
        if self.push_long_triggered:
            return False
        # With pull-down circuit: value is True when pressed
        return self.push_btn.value

    def read_lane(self):
        """Read distance sensor for lane"""
        return self.distance_sensor.get_lane()

    def read_shake(self):
        """Check for shake"""
        x, y, z = self.accelerometer.acceleration
        magnitude = math.sqrt(x*x + y*y + z*z) / 9.8

        if magnitude > SHAKE_THRESHOLD:
            print(f"[SHAKE] ✓ DETECTED! Magnitude: {magnitude:.2f}g")
            return True
        return False

# The main character
class Cat:
    """Player character - the cat!"""
    def __init__(self):
        self.x = CAT_X
        self.current_lane = 1  # Start in middle
        self.target_lane = 1
        self.y = LANE_Y_POSITIONS[1]
        self.health = MAX_HEALTH
        self.is_sleeping = False
        self.is_boosting = False
        self.invincible_until = 0
        self.anim_frame = 0
        self.anim_timer = 0

    def set_target_lane(self, lane):
        """Set which lane to move to"""
        if lane is not None and 0 <= lane < NUM_LANES:
            self.target_lane = lane

    def update(self, dt):
        """Update cat state"""
        # Smooth lane transition
        target_y = LANE_Y_POSITIONS[self.target_lane]
        if self.y < target_y:
            self.y = min(self.y + LANE_TRANSITION_SPEED, target_y)
        elif self.y > target_y:
            self.y = max(self.y - LANE_TRANSITION_SPEED, target_y)

        if self.y == target_y:
            self.current_lane = self.target_lane

        # Animation
        if not self.is_sleeping:
            self.anim_timer += dt
            if self.anim_timer > 200:  # Change frame every 200ms
                self.anim_frame = 1 - self.anim_frame
                self.anim_timer = 0

    def get_sprite(self):
        """Get current sprite"""
        if self.is_sleeping:
            return SPRITE_CAT_SLEEP
        elif self.is_boosting:
            return SPRITE_CAT_BOOST
        else:
            return SPRITE_CAT_RUN_1 if self.anim_frame == 0 else SPRITE_CAT_RUN_2

    def damage(self, amount):
        """Take damage"""
        current_time = time.monotonic_ns() // 1000000
        if current_time < self.invincible_until:
            return False  # Still invincible

        old_health = self.health
        self.health = max(0, self.health - amount)
        self.invincible_until = current_time + INVINCIBILITY_TIME
        print(f"[CAT] Hit! Health: {old_health:.0f} → {self.health:.0f}")
        return True

    def heal(self, amount):
        """Restore health"""
        old_health = self.health
        self.health = min(MAX_HEALTH, self.health + amount)
        print(f"[CAT] Healed: +{amount} HP, {old_health:.0f}% → {self.health:.0f}%")

    def drain_health(self, rate, dt):
        """Passive health drain while sleeping"""
        drain = rate * (dt / 1000.0)
        self.health = max(0, self.health - drain)

    def fall_asleep(self):
        """Cat falls asleep"""
        self.is_sleeping = True
        print("[CAT] Fell asleep! ZZZ...")

    def wake_up(self):
        """Wake up!"""
        self.is_sleeping = False
        print("[CAT] Woke up!")

    def is_invincible(self):
        """Check if currently invincible"""
        return time.monotonic_ns() // 1000000 < self.invincible_until

# Base class for all objects that move left across the screen (obstacles & coins).
class GameObject:
    """Base class for obstacles and coins"""
    def __init__(self):
        self.x = SCREEN_WIDTH
        self.y = 0
        self.lane = 0
        self.width = OBJ_WIDTH
        self.height = OBJ_HEIGHT
        self.active = False
        self.obj_type = OBJ_FENCE

    def spawn(self, lane, obj_type, x_offset=0):
        """Spawn object in lane"""
        self.lane = lane
        self.obj_type = obj_type
        self.x = SCREEN_WIDTH + x_offset
        self.y = LANE_Y_POSITIONS[lane] - self.height // 2
        self.active = True

    def update(self, scroll_speed, dt):
        """Move object left"""
        if self.active:
            self.x -= scroll_speed * (dt / 1000.0)
            if self.x < -self.width:
                self.active = False

    def check_collision(self, cat):
        """Check collision with cat"""
        if not self.active or cat.current_lane != self.lane:
            return False

        return (cat.x < self.x + self.width and
                cat.x + CAT_WIDTH > self.x and
                cat.y < self.y + self.height and
                cat.y + CAT_HEIGHT > self.y)

# is a fish not a coin and it is food
class Coin(GameObject):
    """Fish coin collectible"""
    def __init__(self):
        super().__init__()
        self.width = 6
        self.height = 6

# Manages the entire game world for a single level.
class World:
    """Game world manager"""
    def __init__(self, level, difficulty):
        self.level = level
        self.difficulty = difficulty
        self.base_speed = BASE_SPEEDS[difficulty]
        self.current_speed = self.base_speed
        self.distance_traveled = 0  # pixels
        self.target_distance = LEVEL_DISTANCES[level - 1] * 10  # Convert m to pixels
        self.obstacles = [GameObject() for _ in range(10)]
        self.coins = [Coin() for _ in range(5)]
        self.last_obstacle_spawn = 0
        self.last_coin_spawn = 0
        self.coins_collected = 0
        self.coin_requirement = COIN_REQUIREMENTS[level - 1]

    def update(self, dt, boost_active):
        """Update world state"""
        # Calculate current speed
        speed = self.current_speed
        if boost_active:
            speed *= BOOST_SPEED_MULTIPLIER

        # Update distance
        self.distance_traveled += speed * (dt / 1000.0)

        # Update all objects
        for obj in self.obstacles:
            obj.update(speed, dt)

        for coin in self.coins:
            coin.update(speed, dt)

        # Speed scaling (10% increase per 25% progress)
        progress = self.distance_traveled / self.target_distance
        speed_boost = 1.0 + (progress * 0.4)  # Up to 40% faster
        self.current_speed = self.base_speed * speed_boost

    def spawn_obstacle(self):
        """Spawn a random obstacle (not on Easy difficulty)"""
        # Easy mode has no obstacles - only collect fish and avoid sleeping!
        if self.difficulty == DIFFICULTY_EASY:
            return

        for obj in self.obstacles:
            if not obj.active:
                lane = random.randint(0, NUM_LANES - 1)
                obj_type = OBJ_FENCE
                obj.spawn(lane, obj_type)
                print(f"[WORLD] Spawned obstacle in lane {lane}")
                break

    def spawn_coin(self):
        """Spawn a fish coin"""
        for coin in self.coins:
            if not coin.active:
                lane = random.randint(0, NUM_LANES - 1)
                coin.spawn(lane, 0)  # Type doesn't matter for coins
                print(f"[WORLD] Spawned fish in lane {lane}")
                break

    def get_distance_meters(self):
        """Get distance in meters"""
        return int(self.distance_traveled / 10)

    def get_target_meters(self):
        """Get target distance in meters"""
        return int(self.target_distance / 10)

    def is_level_complete(self):
        """Check if level goals met"""
        return self.distance_traveled >= self.target_distance

# Implements the discrete boost charge system.
class BoostMeter:
    """Boost charge system (discrete charges)"""
    def __init__(self):
        self.charges = MAX_BOOST_CHARGES
        self.is_boosting = False
        self.boost_start_time = 0
        self.last_recharge_time = 0
        self.is_recharging = False

    def activate(self):
        """Try to activate boost (consumes 1 charge)"""
        current_time = time.monotonic_ns() // 1000000

        # Can only use if we have charges AND not currently boosting
        if self.charges > 0 and not self.is_boosting:
            self.charges -= 1
            self.is_boosting = True
            self.boost_start_time = current_time

            # Start recharging timer if not already recharging
            if not self.is_recharging:
                self.is_recharging = True
                self.last_recharge_time = current_time

            print(f"[BOOST] Activated! Charges: {self.charges}/{MAX_BOOST_CHARGES}")
            return True
        elif self.charges == 0:
            print(f"[BOOST] No charges available! Wait for recharge...")
        return False

    def deactivate(self):
        """Deactivate boost"""
        if self.is_boosting:
            print(f"[BOOST] Deactivated. Charges: {self.charges}/{MAX_BOOST_CHARGES}")
        self.is_boosting = False

    def update(self, dt):
        """Update boost system"""
        current_time = time.monotonic_ns() // 1000000

        # Check if boost duration expired
        if self.is_boosting:
            if current_time - self.boost_start_time >= BOOST_DURATION:
                self.deactivate()

        # Recharge charges over time
        if self.is_recharging and self.charges < MAX_BOOST_CHARGES:
            if current_time - self.last_recharge_time >= BOOST_RECHARGE_TIME:
                self.charges += 1
                self.last_recharge_time = current_time
                print(f"[BOOST] Recharged! Charges: {self.charges}/{MAX_BOOST_CHARGES}")

                # Stop recharging when full
                if self.charges >= MAX_BOOST_CHARGES:
                    self.is_recharging = False

    def get_charges(self):
        """Get current boost charges"""
        return self.charges

# The game state machine.
class Game:
    """Main game state"""
    def __init__(self):
        self.state = STATE_DIFFICULTY_SELECT
        self.difficulty = DIFFICULTY_EASY
        self.current_level = 1
        self.difficulty_selection = 0
        self.pause_selection = 0
        self.failed_selection = 0

        self.cat = Cat()
        self.world = None
        self.boost_meter = BoostMeter()

        self.level_start_time = 0
        self.time_limit = 0
        self.last_sleep_check = 0

    def start_level(self):
        """Initialize level"""
        print(f"[GAME] Starting Level {self.current_level}, Difficulty: {DIFFICULTY_NAMES[self.difficulty]}")
        self.cat = Cat()
        self.world = World(self.current_level, self.difficulty)
        self.boost_meter = BoostMeter()
        self.level_start_time = time.monotonic_ns() // 1000000
        self.last_sleep_check = self.level_start_time

        # Calculate time limit
        target_distance = LEVEL_DISTANCES[self.current_level - 1]
        time_per_500m = TIME_PER_500M[self.difficulty]
        self.time_limit = int((target_distance / 500.0) * time_per_500m * 1000)  # ms

    def get_time_remaining(self):
        """Get remaining time in seconds"""
        current_time = time.monotonic_ns() // 1000000
        elapsed = current_time - self.level_start_time
        remaining = self.time_limit - elapsed
        return max(0, remaining // 1000)

    def check_sleep_trigger(self):
        """Random sleep check"""
        current_time = time.monotonic_ns() // 1000000
        if current_time - self.last_sleep_check < SLEEP_CHECK_INTERVAL:
            return False

        self.last_sleep_check = current_time

        # Calculate sleep chance based on level
        level_progress = (self.current_level - 1) / (MAX_LEVELS - 1)
        base_chance = lerp(SLEEP_CHANCE_BASE[0], SLEEP_CHANCE_BASE[2], level_progress)
        sleep_chance = base_chance * SLEEP_CHANCE_MULTIPLIERS[self.difficulty]

        roll = random.random() * 100
        print(f"[GAME] Sleep check: {roll:.1f} vs {sleep_chance:.1f}%")
        return roll < sleep_chance

# Handles all drawing to the screen.
class DisplayManager:
    """Screen rendering"""
    def __init__(self, display):
        self.display = display
        self.main_group = displayio.Group()

    def clear(self):
        self.main_group = displayio.Group()

    def show(self):
        self.display.root_group = self.main_group

    def draw_difficulty_select(self, selection):
        """Difficulty menu"""
        self.clear()
        draw_text(self.main_group, "CAT RUNNER", 25, 5)
        draw_text(self.main_group, "Select Difficulty:", 5, 18)

        y = 30
        for i, name in enumerate(DIFFICULTY_NAMES):
            prefix = ">" if i == selection else " "
            draw_text(self.main_group, f"{prefix} {name}", 20, y)
            y += 10
        self.show()

    def draw_level_start(self, level, distance, coins_needed):
        """Level start screen"""
        self.clear()
        draw_text(self.main_group, f"LEVEL {level}", 40, 10)
        draw_text(self.main_group, f"Run: {distance}m", 35, 25)
        draw_text(self.main_group, f"Fish: {coins_needed}", 35, 35)
        draw_text(self.main_group, "Press to Start", 20, 60)
        self.show()

    def draw_gameplay(self, game):
        """Main gameplay"""
        self.clear()

        # HUD - Top line (carefully positioned to avoid overlap)
        # HP (left side)
        draw_text(self.main_group, f"HP:{int(game.cat.health)}%", 0, 5)

        # Fish count (middle-left)
        draw_text(self.main_group, f"F:{game.world.coins_collected}", 45, 5)

        # Distance (middle-right)
        dist_text = f"{game.world.get_distance_meters()}m"
        draw_text(self.main_group, dist_text, 70, 5)

        # Boost charges (far right)
        boost_charges = game.boost_meter.get_charges()
        draw_text(self.main_group, f"B:{boost_charges}", 100, 5)

        # Lane dividers
        for y in [12, 28, 44, 60]:
            # Draw dotted line
            for x in range(0, SCREEN_WIDTH, 4):
                draw_sprite(self.main_group, [0b11], x, y, 2, 1, 0x444444)

        # Draw obstacles
        for obj in game.world.obstacles:
            if obj.active:
                sprite = SPRITE_FENCE  # Only using fence obstacles now
                draw_sprite(self.main_group, sprite, obj.x, obj.y, 8, 8, 0xFFFFFF)

        # Draw coins
        for coin in game.world.coins:
            if coin.active:
                draw_sprite(self.main_group, SPRITE_FISH, coin.x, coin.y, 6, 6, 0xFFFF00)

        # Draw cat
        cat_sprite = game.cat.get_sprite()
        cat_color = 0xFFFFFF
        if game.cat.is_invincible():
            cat_color = 0xFF00FF  # Purple when invincible
        draw_sprite(self.main_group, cat_sprite, game.cat.x, game.cat.y - CAT_HEIGHT // 2, CAT_WIDTH, CAT_HEIGHT, cat_color)

        # Sleep overlay
        if game.cat.is_sleeping:
            draw_text(self.main_group, "ZZZ...", game.cat.x + 15, game.cat.y - 5)
            draw_text(self.main_group, "SHAKE TO WAKE!", 20, 32)

        self.show()

    def draw_pause_menu(self, selection):
        """Pause menu"""
        self.clear()
        draw_text(self.main_group, "PAUSED", 45, 15)
        options = ["Resume", "Restart", "Exit Menu"]
        y = 30
        for i, opt in enumerate(options):
            prefix = ">" if i == selection else " "
            draw_text(self.main_group, f"{prefix} {opt}", 25, y)
            y += 12
        self.show()

    def draw_level_complete(self, level, coins, coins_needed):
        """Level complete"""
        self.clear()
        draw_text(self.main_group, f"LEVEL {level} DONE!", 20, 15)
        draw_text(self.main_group, f"Fish: {coins}/{coins_needed}", 30, 30)
        if coins >= coins_needed * 2:
            draw_text(self.main_group, "PERFECT!", 35, 42)
        draw_text(self.main_group, "Press to Continue", 15, 56)
        self.show()

    def draw_level_failed(self, selection, reason):
        """Level failed"""
        self.clear()
        draw_text(self.main_group, "LEVEL FAILED", 25, 10)
        draw_text(self.main_group, reason, 10, 25)
        options = ["Retry", "Main Menu"]
        y = 40
        for i, opt in enumerate(options):
            prefix = ">" if i == selection else " "
            draw_text(self.main_group, f"{prefix} {opt}", 35, y)
            y += 12
        self.show()

    def draw_game_won(self):
        """Victory!"""
        self.clear()
        draw_text(self.main_group, "YOU WON!", 35, 15)
        draw_text(self.main_group, "All 10 Levels!", 25, 30)
        draw_text(self.main_group, "Press for Menu", 20, 50)
        self.show()


# ============================================
# MAIN GAME LOOP
# ============================================

def main():
    """Main game loop"""

    # Initialize managers
    distance_sensor = DistanceSensor(sonar)
    audio = AudioManager(buzzer)
    neopixel_mgr = NeopixelManager(pixel)
    input_handler = InputHandler(encoder, rotary_btn, push_btn, accelerometer, distance_sensor)
    display_mgr = DisplayManager(display)
    game = Game()

    # Initial display
    display_mgr.draw_difficulty_select(game.difficulty_selection)

    # Startup beep to confirm game loaded
    audio.play_tone(600, 100)
    time.sleep(0.15)
    audio.play_tone(800, 100)

    # Loop variables
    last_frame_time = time.monotonic_ns() // 1000000
    rainbow_step = 0

    print("=" * 50)
    print("CAT RUNNER - Game Started!")
    print("=" * 50)

    while True:
        frame_start = time.monotonic_ns() // 1000000
        dt = frame_start - last_frame_time
        last_frame_time = frame_start

        # Read inputs
        encoder_delta = input_handler.read_encoder_delta()
        rotary_pressed = input_handler.read_rotary_button()
        # Update push button state first (tracks timing for long press)
        input_handler.update_push_button()
        push_pressed = input_handler.read_push_button()
        push_long_pressed = input_handler.read_push_long_press()
        push_held = input_handler.read_push_held()

        # Long press on push button acts as rotary button (fallback if rotary doesn't work)
        if push_long_pressed:
            rotary_pressed = True

        # ========================================
        # STATE MACHINE
        # ========================================

        if game.state == STATE_DIFFICULTY_SELECT:
            # Navigation: encoder rotation OR push button to cycle
            if encoder_delta != 0:
                game.difficulty_selection = (game.difficulty_selection + encoder_delta) % 4
                audio.menu_nav()
                display_mgr.draw_difficulty_select(game.difficulty_selection)
            elif push_pressed:
                game.difficulty_selection = (game.difficulty_selection + 1) % 4
                audio.menu_nav()
                display_mgr.draw_difficulty_select(game.difficulty_selection)
                print(f"[MENU] Push button navigation → option {game.difficulty_selection}")

            if rotary_pressed:
                if game.difficulty_selection == 3:  # EXIT
                    print("[GAME] Exiting...")
                    audio.menu_confirm()
                    break
                else:
                    game.difficulty = game.difficulty_selection
                    game.current_level = 1
                    game.state = STATE_LEVEL_START
                    audio.menu_confirm()
                    display_mgr.draw_level_start(
                        game.current_level,
                        LEVEL_DISTANCES[0],
                        COIN_REQUIREMENTS[0]
                    )

        elif game.state == STATE_LEVEL_START:
            # Start level: rotary button OR push button
            if rotary_pressed or push_pressed:
                game.start_level()
                game.state = STATE_PLAYING
                audio.menu_confirm()
                neopixel_mgr.set_health_color(game.cat.health)

        elif game.state == STATE_PLAYING:
            # Pause check
            if rotary_pressed:
                game.state = STATE_PAUSED
                game.pause_selection = 0
                audio.menu_nav()
                display_mgr.draw_pause_menu(game.pause_selection)
                continue

            # Lane control
            lane = input_handler.read_lane()
            game.cat.set_target_lane(lane)

            # Boost control - Press button to activate (consumes 1 charge, lasts 1 second)
            if push_pressed and not game.cat.is_sleeping:
                if game.boost_meter.activate():
                    game.cat.is_boosting = True
                    audio.boost_start()
                    neopixel_mgr.set_color(COLOR_YELLOW)

            # Check if boost duration expired
            if game.cat.is_boosting and not game.boost_meter.is_boosting:
                game.cat.is_boosting = False
                neopixel_mgr.set_health_color(game.cat.health)

            # Update game systems
            game.cat.update(dt)
            game.boost_meter.update(dt)
            game.world.update(dt, game.cat.is_boosting)

            # Spawning
            current_time = time.monotonic_ns() // 1000000
            obstacle_interval = OBSTACLE_SPAWN_INTERVALS[game.difficulty]

            if current_time - game.world.last_obstacle_spawn > obstacle_interval:
                game.world.spawn_obstacle()
                game.world.last_obstacle_spawn = current_time

            if current_time - game.world.last_coin_spawn > COIN_SPAWN_INTERVAL:
                game.world.spawn_coin()
                game.world.last_coin_spawn = current_time

            # Collision detection
            for obj in game.world.obstacles:
                if obj.check_collision(game.cat):
                    if game.cat.damage(OBSTACLE_DAMAGE):
                        audio.obstacle_hit()
                        neopixel_mgr.flash(COLOR_RED, 200)
                    obj.active = False

            for coin in game.world.coins:
                if coin.check_collision(game.cat):
                    game.world.coins_collected += 1
                    game.cat.heal(FOOD_HEAL_AMOUNT)  # Fish restore health!
                    audio.coin_collect()
                    neopixel_mgr.flash(COLOR_GREEN, 100)
                    print(f"[GAME] Fish collected! Total: {game.world.coins_collected}, HP: {game.cat.health:.0f}%")
                    coin.active = False

            # Health drain while sleeping
            if game.cat.is_sleeping:
                drain_rate = HEALTH_DRAIN_ASLEEP[game.difficulty]
                game.cat.drain_health(drain_rate, dt)

            # Update health LED
            if not game.cat.is_boosting:
                neopixel_mgr.set_health_color(game.cat.health)

            # Check for death
            if game.cat.health <= 0:
                game.state = STATE_LEVEL_FAILED
                game.failed_selection = 0
                audio.level_failed()
                neopixel_mgr.set_color(COLOR_RED)
                display_mgr.draw_level_failed(0, "Cat fainted!")
                continue

            # Sleep trigger
            if not game.cat.is_sleeping and game.check_sleep_trigger():
                game.cat.fall_asleep()
                game.state = STATE_ASLEEP
                print(f"[GAME] === STATE CHANGED TO ASLEEP ===")
                audio.fall_asleep()
                continue

            # Check level complete
            if game.world.is_level_complete():
                if game.world.coins_collected >= game.world.coin_requirement:
                    game.state = STATE_LEVEL_COMPLETE
                    audio.level_complete()
                    display_mgr.draw_level_complete(
                        game.current_level,
                        game.world.coins_collected,
                        game.world.coin_requirement
                    )
                else:
                    game.state = STATE_LEVEL_FAILED
                    audio.level_failed()
                    display_mgr.draw_level_failed(0, f"Need {game.world.coin_requirement} fish!")
                continue

            # Check time up
            if game.get_time_remaining() == 0:
                game.state = STATE_LEVEL_FAILED
                game.failed_selection = 0
                audio.level_failed()
                display_mgr.draw_level_failed(0, "Time's up!")
                continue

            # Render
            display_mgr.draw_gameplay(game)

        elif game.state == STATE_ASLEEP:
            print("[STATE] === IN ASLEEP STATE ===")

            # Check for pause button (should work even when sleeping)
            if rotary_pressed:
                game.state = STATE_PAUSED
                game.pause_selection = 0
                audio.menu_nav()
                display_mgr.draw_pause_menu(game.pause_selection)
                print("[GAME] Paused while sleeping")
                continue

            # Check for shake to wake up
            print("[GAME] Cat is sleeping, checking for shake...")
            if input_handler.read_shake():
                game.cat.wake_up()
                game.state = STATE_PLAYING
                audio.wake_up()
                neopixel_mgr.set_health_color(game.cat.health)
                print("[GAME] Cat woke up! Returning to PLAYING state")
            else:
                # Continue game logic
                game.cat.update(dt)
                game.world.update(dt, False)

                # Spawn
                current_time = time.monotonic_ns() // 1000000
                obstacle_interval = OBSTACLE_SPAWN_INTERVALS[game.difficulty]

                if current_time - game.world.last_obstacle_spawn > obstacle_interval:
                    game.world.spawn_obstacle()
                    game.world.last_obstacle_spawn = current_time

                if current_time - game.world.last_coin_spawn > COIN_SPAWN_INTERVAL:
                    game.world.spawn_coin()
                    game.world.last_coin_spawn = current_time

                # Collisions
                for obj in game.world.obstacles:
                    if obj.check_collision(game.cat):
                        if game.cat.damage(OBSTACLE_DAMAGE):
                            audio.obstacle_hit()
                        obj.active = False

                # Health drain
                drain_rate = HEALTH_DRAIN_ASLEEP[game.difficulty]
                game.cat.drain_health(drain_rate, dt)

                # Death check
                if game.cat.health <= 0:
                    game.state = STATE_LEVEL_FAILED
                    game.failed_selection = 0
                    audio.level_failed()
                    display_mgr.draw_level_failed(0, "Cat fainted!")
                    continue

                # Check level complete (even while sleeping!)
                if game.world.is_level_complete():
                    if game.world.coins_collected >= game.world.coin_requirement:
                        game.state = STATE_LEVEL_COMPLETE
                        audio.level_complete()
                        display_mgr.draw_level_complete(
                            game.current_level,
                            game.world.coins_collected,
                            game.world.coin_requirement
                        )
                        print("[GAME] Level complete while sleeping!")
                    else:
                        game.state = STATE_LEVEL_FAILED
                        audio.level_failed()
                        display_mgr.draw_level_failed(0, f"Need {game.world.coin_requirement} fish!")
                        print("[GAME] Level failed while sleeping (not enough fish)")
                    continue

                # Check time up
                if game.get_time_remaining() == 0:
                    game.state = STATE_LEVEL_FAILED
                    game.failed_selection = 0
                    audio.level_failed()
                    display_mgr.draw_level_failed(0, "Time's up!")
                    print("[GAME] Time's up while sleeping!")
                    continue

                # Pulse LED
                neopixel_mgr.pulse_blue()

                # Render
                display_mgr.draw_gameplay(game)

        elif game.state == STATE_PAUSED:
            # Navigation: encoder rotation OR push button to cycle
            if encoder_delta != 0:
                game.pause_selection = (game.pause_selection + encoder_delta) % 3
                audio.menu_nav()
                display_mgr.draw_pause_menu(game.pause_selection)
            elif push_pressed:
                game.pause_selection = (game.pause_selection + 1) % 3
                audio.menu_nav()
                display_mgr.draw_pause_menu(game.pause_selection)
                print(f"[MENU] Push button navigation → option {game.pause_selection}")

            if rotary_pressed:
                if game.pause_selection == 0:  # Resume
                    game.state = STATE_PLAYING
                    audio.menu_confirm()
                elif game.pause_selection == 1:  # Restart
                    game.state = STATE_LEVEL_START
                    audio.menu_confirm()
                    display_mgr.draw_level_start(
                        game.current_level,
                        LEVEL_DISTANCES[game.current_level - 1],
                        COIN_REQUIREMENTS[game.current_level - 1]
                    )
                else:  # Exit
                    game.state = STATE_DIFFICULTY_SELECT
                    game.difficulty_selection = 0
                    audio.menu_confirm()
                    display_mgr.draw_difficulty_select(0)

        elif game.state == STATE_LEVEL_COMPLETE:
            rainbow_step = (rainbow_step + 5) % 255
            neopixel_mgr.rainbow_cycle(rainbow_step)

            # Continue: rotary button OR push button
            if rotary_pressed or push_pressed:
                if game.current_level >= MAX_LEVELS:
                    game.state = STATE_GAME_WON
                    audio.game_won()
                    display_mgr.draw_game_won()
                else:
                    game.current_level += 1
                    game.state = STATE_LEVEL_START
                    audio.menu_confirm()
                    display_mgr.draw_level_start(
                        game.current_level,
                        LEVEL_DISTANCES[game.current_level - 1],
                        COIN_REQUIREMENTS[game.current_level - 1]
                    )

        elif game.state == STATE_LEVEL_FAILED:
            # Navigation: encoder rotation OR push button to cycle
            if encoder_delta != 0:
                game.failed_selection = (game.failed_selection + encoder_delta) % 2
                audio.menu_nav()
                # Re-render with new selection (need to track reason)
                display_mgr.draw_level_failed(game.failed_selection, "Try again!")
            elif push_pressed:
                game.failed_selection = (game.failed_selection + 1) % 2
                audio.menu_nav()
                display_mgr.draw_level_failed(game.failed_selection, "Try again!")
                print(f"[MENU] Push button navigation → option {game.failed_selection}")

            if rotary_pressed:
                if game.failed_selection == 0:  # Retry
                    game.state = STATE_LEVEL_START
                    audio.menu_confirm()
                    display_mgr.draw_level_start(
                        game.current_level,
                        LEVEL_DISTANCES[game.current_level - 1],
                        COIN_REQUIREMENTS[game.current_level - 1]
                    )
                else:  # Menu
                    game.state = STATE_DIFFICULTY_SELECT
                    game.difficulty_selection = 0
                    audio.menu_confirm()
                    display_mgr.draw_difficulty_select(0)

        elif game.state == STATE_GAME_WON:
            rainbow_step = (rainbow_step + 5) % 255
            neopixel_mgr.rainbow_cycle(rainbow_step)

            # Continue: rotary button OR push button
            if rotary_pressed or push_pressed:
                game.state = STATE_DIFFICULTY_SELECT
                game.difficulty_selection = 0
                game.current_level = 1
                audio.menu_confirm()
                display_mgr.draw_difficulty_select(0)

        # Update peripherals
        audio.update()
        neopixel_mgr.update()

        # Frame timing
        frame_elapsed = (time.monotonic_ns() // 1000000) - frame_start
        if frame_elapsed < FRAME_TIME:
            time.sleep((FRAME_TIME - frame_elapsed) / 1000.0)

    # Cleanup
    print("[GAME] Shutting down...")
    neopixel_mgr.off()
    audio.stop()
    print("[GAME] Goodbye!")


# ============================================
# START GAME
# ============================================
if __name__ == "__main__":
    main()