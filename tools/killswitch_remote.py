#!/usr/bin/env python3
"""Laptop end of the RoboRacer remote kill-switch.

Runs on the OPERATOR'S LAPTOP, not the car. Reads the handheld remote and streams
an armed/disarmed heartbeat to killswitch_node on the Jetson over UDP.

    remote --BT/USB--> laptop --WiFi/UDP @50Hz--> Jetson --> /estop + brake

Plain Python 3. No ROS, no WSL, no DDS - deliberately. A kill-switch wants
"newest datagram wins, silence means stop", which is what UDP already is, and it
means this runs on native Windows where the Bluetooth stack actually lives.

    pip install pygame

    python killswitch_remote.py --host 192.168.137.220                 # gamepad
    python killswitch_remote.py --host 192.168.137.220 --input mouse   # USB mouse

INPUT MODES

  gamepad  ARM by holding both shoulder buttons for 0.5 s. KILL with ANY other
           button or d-pad direction - "anything that is not the arm combo", so
           whatever controller you buy, mashing it stops the car. --list prints
           button numbers if you need to remap --arm-buttons.

  mouse    Opens a small window and CAPTURES the pointer inside it, so the mouse
           behaves like a handheld remote instead of a desktop pointer. LEFT
           CLICK arms; any other mouse button or Space kills; ESCAPE releases the
           mouse and quits. Alt-Tabbing away also kills, since you cannot click a
           window you are not focused on. Intended for bench testing the full
           laptop->WiFi->car path before the real remote arrives; it is not a
           competition-legal remote.

THE CAR ALSO STOPS IF THIS SCRIPT STOPS. Closing the window, Ctrl+C, the laptop
sleeping, the WiFi dropping and the remote's battery dying are all the same event
from the car's side - packets stop arriving and it brakes within the node's
timeout_ms. That is the fallback this whole thing exists for.

RULES NOTE: this is a KILL-SWITCH, not a teleop. Do not add throttle or steering
here. "During the race, the teams MUST NOT control the car manually. Violating
this rule MAY lead to disqualification regardless of the number of warnings."
"""

import argparse
import os
import socket
import struct
import sys
import time

# Must match killswitch.cpp exactly: magic, version, token, seq, armed.
WIRE = struct.Struct("!4sBIIB")
MAGIC = b"RRKS"
VERSION = 1
DEFAULT_TOKEN = 0x5252534B

SEND_HZ = 50.0
ARM_HOLD_SEC = 0.5


def build_packet(seq: int, armed: bool, token: int) -> bytes:
    return WIRE.pack(MAGIC, VERSION, token, seq & 0xFFFFFFFF, 1 if armed else 0)


class Quit(Exception):
    """The input source asked to shut down (window closed)."""


class Status:
    """One self-overwriting status line, so the operator can glance at it."""

    def __init__(self):
        self._last = None

    def show(self, text: str) -> None:
        if text == self._last:
            return
        self._last = text
        sys.stdout.write("\r" + text.ljust(78))
        sys.stdout.flush()

    def note(self, text: str) -> None:
        self._last = None
        sys.stdout.write("\r" + text.ljust(78) + "\n")
        sys.stdout.flush()


class GamepadSource:
    """Hold both shoulder buttons to arm; any other button kills."""

    needs_display = False

    def __init__(self, pygame, arm_buttons, status, list_only=False):
        self.pg = pygame
        self.arm_buttons = arm_buttons
        self.status = status
        self.list_only = list_only
        self.armed = False
        self.arm_since = None
        self.pad = None
        self._open()

    def _open(self):
        self.pg.joystick.quit()
        self.pg.joystick.init()
        if self.pg.joystick.get_count() == 0:
            self.pad = None
            return
        self.pad = self.pg.joystick.Joystick(0)
        self.pad.init()
        self.status.note(
            "controller: %s  (%d buttons, %d hats)"
            % (self.pad.get_name(), self.pad.get_numbuttons(), self.pad.get_numhats())
        )

    def banner(self):
        return [
            "ARM   hold buttons %s together for %.1fs" % (self.arm_buttons, ARM_HOLD_SEC),
            "KILL  any other button, or any d-pad direction",
        ]

    def poll(self):
        for event in self.pg.event.get():
            if event.type == self.pg.JOYDEVICEREMOVED:
                if self.armed:
                    self.status.note("!! controller disconnected - STOPPING")
                self.armed, self.pad = False, None
            elif event.type == self.pg.JOYDEVICEADDED and self.pad is None:
                self._open()

        if self.pad is None:
            self.armed = False
            return False, "no controller - plug one in"

        try:
            pressed = [i for i in range(self.pad.get_numbuttons()) if self.pad.get_button(i)]
            hats = [self.pad.get_hat(i) for i in range(self.pad.get_numhats())]
        except self.pg.error:
            # The pad vanished between the event pump and here.
            self.armed, self.pad = False, None
            return False, "no controller - plug one in"

        if self.list_only:
            self.status.show("buttons %s  hats %s" % (pressed or "-", hats or "-"))
            return False, "--list"

        arm_held = all(b in pressed for b in self.arm_buttons)
        other = [b for b in pressed if b not in self.arm_buttons]
        hat_moved = any(h != (0, 0) for h in hats)

        if other or hat_moved:
            if self.armed:
                self.status.note("!! KILL (button %s) - STOPPING" % (other or "d-pad"))
            self.armed, self.arm_since = False, None
        elif arm_held and not self.armed:
            # Hold, not tap, so it cannot be armed by a knock in transit.
            self.arm_since = self.arm_since or time.monotonic()
            if time.monotonic() - self.arm_since >= ARM_HOLD_SEC:
                self.armed = True
                self.status.note(">> ARMED - car is allowed to drive")
        elif not arm_held:
            self.arm_since = None

        return self.armed, ""


class MouseSource:
    """Bench stand-in: a clickable window. Left click arms, anything else kills.

    Needs a real window - mouse events only reach an app that owns one - so this
    will not run headless over SSH. Use --hold-armed for that.
    """

    needs_display = True
    WIDTH, HEIGHT = 460, 190

    def __init__(self, pygame, status):
        self.pg = pygame
        self.status = status
        self.armed = False
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("RoboRacer kill-switch")
        # Confine the pointer to the window so the mouse acts like a handheld
        # remote rather than a desktop pointer - no clicking into another app by
        # accident mid-test. Escape is the way out, and send_loop releases the
        # grab on EVERY exit path so a crash cannot leave the mouse trapped.
        self._grab(True)
        self.big = pygame.font.SysFont(None, 56)
        self.small = pygame.font.SysFont(None, 22)

    def _grab(self, on):
        self.pg.event.set_grab(on)
        self.pg.mouse.set_visible(True)

    def release(self):
        """Give the pointer back. Safe to call more than once."""
        try:
            self._grab(False)
        except self.pg.error:
            pass

    def banner(self):
        return [
            "ARM   left click in the window",
            "KILL  any other mouse button, Space, or Alt-Tab away",
            "QUIT  Escape - releases the mouse and stops the car",
        ]

    def _kill(self, why):
        if self.armed:
            self.status.note("!! KILL (%s) - STOPPING" % why)
        self.armed = False

    def poll(self):
        pg = self.pg
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self._kill("window closed")
                raise Quit
            elif event.type == pg.MOUSEBUTTONDOWN:
                if event.button == 1:
                    if not self.armed:
                        self.armed = True
                        self.status.note(">> ARMED - car is allowed to drive")
                elif event.button not in (4, 5):  # 4/5 are wheel on some setups
                    self._kill("mouse button %d" % event.button)
            elif event.type == pg.KEYDOWN:
                if event.key == pg.K_ESCAPE:
                    self._kill("escape")
                    raise Quit
                if event.key == pg.K_SPACE:
                    self._kill("space")
            elif event.type == getattr(pg, "WINDOWFOCUSLOST", -1):
                # With the pointer grabbed this means Alt-Tab. You cannot click a
                # window you are not focused on, so staying armed here would leave
                # the car running with no way to stop it.
                self._kill("window lost focus")
            elif event.type == getattr(pg, "WINDOWFOCUSGAINED", -2):
                # SDL drops the grab when focus goes; take it back on return.
                self._grab(True)

        self._draw()
        return self.armed, ""

    def _draw(self):
        bg = (24, 96, 40) if self.armed else (120, 24, 24)
        self.screen.fill(bg)
        label = self.big.render("ARMED" if self.armed else "STOPPED", True, (255, 255, 255))
        self.screen.blit(label, label.get_rect(center=(self.WIDTH // 2, 58)))
        for i, line in enumerate(self.banner()):
            text = self.small.render(line, True, (235, 235, 235))
            self.screen.blit(text, (18, 108 + i * 26))
        self.pg.display.flip()


def send_loop(sock, dest, token, source, status):
    """Stream the heartbeat at SEND_HZ, and say STOP on the way out."""
    seq = 0
    period = 1.0 / SEND_HZ
    next_send = time.monotonic()
    try:
        while True:
            armed, note = source.poll()
            seq += 1
            try:
                sock.sendto(build_packet(seq, armed, token), dest)
            except OSError as exc:
                # WiFi down. Say so and keep looping - the car has already stopped
                # itself on the timeout, and this recovers when the link returns.
                status.show("NETWORK DOWN (%s) - car is stopping" % exc.strerror)
            else:
                state = "ARMED - driving" if armed else "*** STOPPED ***"
                status.show(("%s   %s   seq %d" % (state, note, seq)).replace("   ", "  ", 1))
            next_send += period
            time.sleep(max(0.0, next_send - time.monotonic()))
    except (KeyboardInterrupt, Quit):
        pass
    finally:
        # Give the pointer back FIRST: whatever else fails below, the operator
        # must not be left with a captured mouse.
        release = getattr(source, "release", None)
        if release is not None:
            release()

        # Say STOP explicitly rather than letting the car find out by timeout.
        for _ in range(10):
            seq += 1
            try:
                sock.sendto(build_packet(seq, False, token), dest)
            except OSError:
                break
        status.note("kill-switch closed - car is STOPPED.")


def main() -> int:
    ap = argparse.ArgumentParser(description="RoboRacer remote kill-switch (laptop side)")
    ap.add_argument("--host", default="192.168.137.220", help="Jetson IP")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument(
        "--input",
        choices=("gamepad", "mouse"),
        default="gamepad",
        help="gamepad (competition) or mouse (bench testing, needs a display)",
    )
    ap.add_argument(
        "--token",
        type=lambda v: int(v, 0),
        default=DEFAULT_TOKEN,
        help="must match killswitch_node's 'token' param",
    )
    ap.add_argument(
        "--arm-buttons",
        default="4,5",
        help="the two buttons that arm the car when held together (default LB,RB)",
    )
    ap.add_argument("--list", action="store_true", help="print gamepad button presses")
    ap.add_argument(
        "--hold-armed",
        action="store_true",
        help="BENCH ONLY: send ARMED continuously with no remote at all. Car on blocks.",
    )
    args = ap.parse_args()

    status = Status()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)

    if args.hold_armed:
        print("=" * 72)
        print("  BENCH MODE - sending ARMED with no remote attached.")
        print("  The ONLY way to stop the car is Ctrl+C. Keep it on blocks.")
        print("=" * 72)

        class _Always:
            def poll(self):
                return True, "BENCH"

        send_loop(sock, dest, args.token, _Always(), status)
        return 0

    if args.input == "gamepad":
        # Joystick-only: no window, so SDL must not try to open a video device.
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    try:
        import pygame
    except ImportError:
        print("pygame is missing.  pip install pygame", file=sys.stderr)
        return 1
    pygame.init()

    try:
        if args.input == "mouse":
            source = MouseSource(pygame, status)
        else:
            arm_buttons = [int(b) for b in args.arm_buttons.split(",") if b.strip()]
            source = GamepadSource(pygame, arm_buttons, status, list_only=args.list)
    except pygame.error as exc:
        print("could not open the %s input: %s" % (args.input, exc), file=sys.stderr)
        if args.input == "mouse":
            print("mouse mode needs a desktop - it cannot run headless over SSH.",
                  file=sys.stderr)
        return 1

    print("=" * 72)
    print("  ROBORACER KILL-SWITCH  ->  %s:%d   (%s)" % (args.host, args.port, args.input))
    for line in source.banner():
        print("    " + line)
    print("  Stopped by default. Quitting stops the car.")
    print("=" * 72)

    send_loop(sock, dest, args.token, source, status)
    return 0


if __name__ == "__main__":
    sys.exit(main())
