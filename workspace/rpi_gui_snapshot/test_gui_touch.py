# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# ruff: noqa: I001, E402
import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pygame
import gui_fb

from gui_fb import (
    EvdevTouchReader,
    copy_surface_to_rgb565,
    force_opaque_kms_framebuffer,
    is_widget_visible_on_manual_tab,
    scale_evdev_axis,
    touch_event_to_mouse_event,
)


class KmsFramebufferTest(unittest.TestCase):
    def test_xrgb_padding_is_forced_opaque_before_kms_flip(self) -> None:
        surface = pygame.Surface(
            (2, 1),
            depth=32,
            masks=(0x00FF0000, 0x0000FF00, 0x000000FF, 0),
        )
        surface.fill((24, 26, 32))
        raw = memoryview(surface.get_view("0")).cast("B")
        self.assertEqual(list(raw[3::4]), [0, 0])
        del raw

        changed = force_opaque_kms_framebuffer(surface)

        raw = memoryview(surface.get_view("0")).cast("B")
        self.assertTrue(changed)
        self.assertEqual(list(raw[3::4]), [255, 255])
        del raw


class LinuxFramebufferTest(unittest.TestCase):
    def test_surface_is_converted_to_little_endian_rgb565(self) -> None:
        source = pygame.Surface((4, 1), depth=32)
        source.set_at((0, 0), (255, 0, 0))
        source.set_at((1, 0), (0, 255, 0))
        source.set_at((2, 0), (0, 0, 255))
        source.set_at((3, 0), (255, 255, 255))
        target = pygame.Surface((4, 1), depth=16, masks=(0xF800, 0x07E0, 0x001F, 0))

        pixels = copy_surface_to_rgb565(source, target)

        self.assertEqual(bytes(pixels), b"\x00\xf8\xe0\x07\x1f\x00\xff\xff")
        del pixels

    def test_evdev_axis_is_scaled_and_clamped(self) -> None:
        self.assertEqual(scale_evdev_axis(0, 0, 1024, 1024), 0)
        self.assertEqual(scale_evdev_axis(512, 0, 1024, 1024), 512)
        self.assertEqual(scale_evdev_axis(1024, 0, 1024, 1024), 1023)
        self.assertEqual(scale_evdev_axis(1200, 0, 1024, 1024), 1023)


class ManualMenuTest(unittest.TestCase):
    def test_widget_without_tab_is_visible_on_every_manual_tab(self) -> None:
        back_button = SimpleNamespace(_tab=None)
        joint_button = SimpleNamespace(_tab="joint")

        self.assertTrue(is_widget_visible_on_manual_tab(back_button, "joint"))
        self.assertTrue(is_widget_visible_on_manual_tab(back_button, "xyz"))
        self.assertTrue(is_widget_visible_on_manual_tab(joint_button, "joint"))
        self.assertFalse(is_widget_visible_on_manual_tab(joint_button, "xyz"))


class EvdevProtocolBTest(unittest.TestCase):
    def test_tracking_id_without_btn_touch_generates_down_and_up(self) -> None:
        reader = EvdevTouchReader.__new__(EvdevTouchReader)
        reader.width, reader.height = (1024, 600)
        reader.x_range = (0, 1000)
        reader.y_range = (0, 1000)
        reader.raw_x, reader.raw_y = (0, 0)
        reader._btn_touch_pressed = False
        reader._tracking_active = False
        reader.pressed = False
        reader.reported_pressed = False
        reader.last_pos = (0, 0)
        reader.dirty = False
        reader.pending = bytearray()
        reader.offline = False
        reader.descriptor = 123
        packet = gui_fb._INPUT_EVENT.pack
        down = b"".join(
            (
                packet(0, 0, gui_fb._EV_ABS, gui_fb._ABS_MT_POSITION_X, 250),
                packet(0, 0, gui_fb._EV_ABS, gui_fb._ABS_MT_POSITION_Y, 500),
                packet(0, 0, gui_fb._EV_ABS, gui_fb._ABS_MT_TRACKING_ID, 7),
                packet(0, 0, gui_fb._EV_SYN, gui_fb._SYN_REPORT, 0),
            )
        )
        up = b"".join(
            (
                packet(0, 0, gui_fb._EV_ABS, gui_fb._ABS_MT_TRACKING_ID, 0xFFFFFFFF),
                packet(0, 0, gui_fb._EV_SYN, gui_fb._SYN_REPORT, 0),
            )
        )
        reads = [down, BlockingIOError(), up, BlockingIOError()]
        original_read = gui_fb.os.read

        def fake_read(*_args):
            value = reads.pop(0)
            if isinstance(value, BaseException):
                raise value
            return value

        try:
            gui_fb.os.read = fake_read
            first = reader.poll()
            second = reader.poll()
        finally:
            gui_fb.os.read = original_read
        self.assertEqual(first[0].type, pygame.MOUSEBUTTONDOWN)
        self.assertEqual(first[0].pos, (256, 300))
        self.assertEqual(second[0].type, pygame.MOUSEBUTTONUP)


class TouchEventMappingTest(unittest.TestCase):
    def test_finger_down_maps_normalized_coordinates_to_left_click(self) -> None:
        event = pygame.event.Event(
            pygame.FINGERDOWN,
            {"x": 0.25, "y": 0.5, "finger_id": 1, "touch_id": 2},
        )

        mapped = touch_event_to_mouse_event(event, (1024, 600))

        self.assertEqual(mapped.type, pygame.MOUSEBUTTONDOWN)
        self.assertEqual(mapped.pos, (256, 300))
        self.assertEqual(mapped.button, 1)
        self.assertTrue(mapped.touch)

    def test_finger_motion_is_clamped_and_maps_relative_motion(self) -> None:
        event = pygame.event.Event(
            pygame.FINGERMOTION,
            {
                "x": 1.1,
                "y": -0.1,
                "dx": 0.1,
                "dy": -0.2,
                "finger_id": 1,
                "touch_id": 2,
            },
        )

        mapped = touch_event_to_mouse_event(event, (1024, 600))

        self.assertEqual(mapped.type, pygame.MOUSEMOTION)
        self.assertEqual(mapped.pos, (1023, 0))
        self.assertEqual(mapped.rel, (102, -120))
        self.assertEqual(mapped.buttons, (1, 0, 0))

    def test_finger_up_maps_to_left_button_release(self) -> None:
        event = pygame.event.Event(
            pygame.FINGERUP,
            {"x": 0.75, "y": 0.25, "finger_id": 1, "touch_id": 2},
        )

        mapped = touch_event_to_mouse_event(event, (1024, 600))

        self.assertEqual(mapped.type, pygame.MOUSEBUTTONUP)
        self.assertEqual(mapped.pos, (768, 150))
        self.assertEqual(mapped.button, 1)


if __name__ == "__main__":
    unittest.main()
