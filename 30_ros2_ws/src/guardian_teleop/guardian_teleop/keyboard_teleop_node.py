# MIT License
# GUARDIAN — StarkHacks 2026
# Node: keyboard_teleop_node
# Purpose: WASD keyboard control -> /cmd_vel, with genuine press/release
#          detection. A plain terminal (termios raw mode) has NO key-up
#          event — it only ever tells you "a byte arrived" — and standard
#          X11/terminal key auto-repeat sustains only ONE actively-
#          repeating key at a time, so approximating "held" from repeat
#          timing silently breaks the moment a second key is added. This
#          reads the keyboard directly via evdev (Linux input subsystem),
#          which reports real KEY_DOWN/KEY_UP events independent of the
#          terminal and of how many keys are down at once. A background
#          thread updates a small set of currently-pressed direction keys;
#          a 30Hz ROS timer reads that set and publishes the summed
#          velocity — zero for any direction not currently pressed, the
#          configured speed for any that is.
#
# Requires read access to /dev/input/eventN for the keyboard, which on a
# stock Ubuntu install means being in the 'input' group:
#   sudo usermod -aG input $USER   (then log out and back in)
# See guardian_teleop/README.md for the one-time setup and why this
# approach was chosen over reading the terminal.

import math
import threading

import evdev
from evdev import ecodes
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Real motor spec: 170 RPM rated max continuous wheel speed. wheel_radius
# matches the CAD-measured value in guardian_description/urdf/dimensions.xacro
# and guardian_bringup/config/robot_params.yaml — keep all three in sync.
# 100% speed (speed_scale=1.0) is defined to be exactly this: any straight
# drive/strafe command at full speed_scale asks for precisely 170 RPM at
# each wheel, matching the hardware's real limit instead of an arbitrary
# small default that left most of the available speed unused.
WHEEL_RADIUS_M = 0.0775
MAX_WHEEL_RPM = 170.0
MAX_LINEAR_MPS = MAX_WHEEL_RPM * (2.0 * math.pi / 60.0) * WHEEL_RADIUS_M

# evdev keycode -> (vx, vy, oz) contribution while held
KEYS = {
    ecodes.KEY_W: ( 1,  0,  0),
    ecodes.KEY_S: (-1,  0,  0),
    ecodes.KEY_A: ( 0,  1,  0),
    ecodes.KEY_D: ( 0, -1,  0),
    ecodes.KEY_Q: ( 0,  0,  1),
    ecodes.KEY_E: ( 0,  0, -1),
}
SPEED_UP_KEYS   = (ecodes.KEY_EQUAL, ecodes.KEY_KPPLUS)
SPEED_DOWN_KEYS = (ecodes.KEY_MINUS, ecodes.KEY_KPMINUS)
STOP_KEYS       = (ecodes.KEY_SPACE, ecodes.KEY_K)
QUIT_KEYS       = (ecodes.KEY_X, ecodes.KEY_ESC)

PUBLISH_RATE_HZ = 30.0

BANNER = """
GUARDIAN Keyboard Teleop (evdev — real press/release)
-------------------------------------------------------
  w     forward           (only while held)
  s     backward          (only while held)
  a     strafe left       (only while held)
  d     strafe right      (only while held)
  q     rotate left       (only while held)
  e     rotate right      (only while held)
  space stop everything
  +/-   speed up/down
  x/Esc quit
-------------------------------------------------------
Any combination of movement keys may be held together
for diagonal motion (e.g. w+a). This window does not
need to be focused — evdev reads the keyboard directly.
"""


def find_keyboard_device():
    """Pick the first /dev/input/eventN that reports KEY_W (i.e. is a
    real keyboard, not a mouse/other HID device)."""
    for path in evdev.list_devices():
        dev = evdev.InputDevice(path)
        caps = dev.capabilities().get(ecodes.EV_KEY, [])
        if ecodes.KEY_W in caps:
            return dev
    return None


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.declare_parameter('linear_speed', MAX_LINEAR_MPS)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('device_path', '')  # '' = auto-detect
        # Exclusively grabs the keyboard so keystrokes don't also leak to
        # whatever window has focus — off by default since it locks your
        # keyboard away from everything else while this node runs.
        self.declare_parameter('grab_device', False)
        # Temporary diagnostic: logs every raw key event (code, value) —
        # turn on to see exactly what evdev is reporting when release
        # doesn't behave as expected.
        self.declare_parameter('debug_events', False)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        device_path = self.get_parameter('device_path').value
        self.device = (evdev.InputDevice(device_path) if device_path
                        else find_keyboard_device())
        if self.device is None:
            raise RuntimeError(
                'keyboard_teleop_node: no keyboard found among '
                f'{evdev.list_devices()} — set the device_path parameter '
                'explicitly (see `python3 -m evdev.evtest` to list devices), '
                "and make sure you're in the 'input' group "
                '(sudo usermod -aG input $USER, then log out/in).')
        self.get_logger().info(
            f'keyboard_teleop_node: reading {self.device.path} '
            f'({self.device.name})')

        if self.get_parameter('grab_device').value:
            self.device.grab()

        self._lock = threading.Lock()
        self._pressed = set()
        self._speed_scale = 0.3
        self._quit = threading.Event()

        self._reader_thread = threading.Thread(
            target=self._read_events, daemon=True)
        self._reader_thread.start()

        self._last_published = None
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_tick)

        print(BANNER)
        print(f'Speed: {self._speed_scale*100:.0f}%  (use +/- to adjust)')

    def _read_events(self):
        """Runs on a background thread — evdev's read_loop() blocks."""
        debug = self.get_parameter('debug_events').value
        try:
            for event in self.device.read_loop():
                if self._quit.is_set():
                    return
                if event.type != ecodes.EV_KEY:
                    continue
                code = event.code
                if debug:
                    self.get_logger().info(
                        f'raw event: code={code} '
                        f'({ecodes.KEY.get(code, "?")}) value={event.value}')
                if event.value == 1:  # key down
                    if code in KEYS:
                        with self._lock:
                            self._pressed.add(code)
                            snapshot = set(self._pressed)
                        if debug:
                            self.get_logger().info(f'pressed now: {snapshot}')
                    elif code in SPEED_UP_KEYS:
                        with self._lock:
                            self._speed_scale = min(1.0, self._speed_scale + 0.1)
                        print(f'\rSpeed: {self._speed_scale*100:.0f}%  ',
                              end='', flush=True)
                    elif code in SPEED_DOWN_KEYS:
                        with self._lock:
                            self._speed_scale = max(0.1, self._speed_scale - 0.1)
                        print(f'\rSpeed: {self._speed_scale*100:.0f}%  ',
                              end='', flush=True)
                    elif code in STOP_KEYS:
                        with self._lock:
                            self._pressed.clear()
                    elif code in QUIT_KEYS:
                        self._quit.set()
                        rclpy.get_default_context().try_shutdown()
                        return
                elif event.value == 0:  # key up
                    if code in KEYS:
                        with self._lock:
                            self._pressed.discard(code)
                            snapshot = set(self._pressed)
                        if debug:
                            self.get_logger().info(f'pressed now: {snapshot}')
        except OSError:
            # Device disappeared (unplugged) or node shutting down.
            pass

    def _publish_tick(self):
        with self._lock:
            pressed = set(self._pressed)
            speed_scale = self._speed_scale

        lin = self.get_parameter('linear_speed').value
        ang = self.get_parameter('angular_speed').value

        vx = sum(KEYS[c][0] for c in pressed)
        vy = sum(KEYS[c][1] for c in pressed)
        oz = sum(KEYS[c][2] for c in pressed)

        twist = Twist()
        twist.linear.x  = vx * lin * speed_scale
        twist.linear.y  = vy * lin * speed_scale
        twist.angular.z = oz * ang * speed_scale

        current = (twist.linear.x, twist.linear.y, twist.angular.z)
        # Always republish while any key is down (in case a subscriber
        # relies on a steady stream), but at minimum on every change.
        if pressed or current != self._last_published:
            self.pub.publish(twist)
            self._last_published = current

    def shutdown(self):
        self._quit.set()
        try:
            self.device.ungrab()
        except (OSError, IOError):
            pass
        self.pub.publish(Twist())  # stop on exit


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
