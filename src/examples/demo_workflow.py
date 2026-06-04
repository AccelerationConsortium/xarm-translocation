#!/usr/bin/env python3
"""
Workflow Demo

Slow-speed orchestration of a few moves. Extend by adding more steps to
run_workflow().

Usage:
    python src/examples/demo_workflow.py
    python src/examples/demo_workflow.py --auto             # skip Enter prompts
    python src/examples/demo_workflow.py --no-open-gripper  # don't open gripper at start
"""

import argparse
import sys

from core.xarm_controller import XArmController

JOINT_SPEED = 10    # deg/s, slow
TRACK_SPEED = 100   # mm/s, slow
LINEAR_SPEED = 50   # mm/s, slow Cartesian (TCP) speed for linear moves


def confirm(message: str, auto: bool = False) -> bool:
    """Pause for operator confirmation. Returns False on Ctrl+C."""
    print(f"\n{message}")
    if auto:
        return True
    try:
        input("Press Enter to continue (Ctrl+C to abort)...")
        return True
    except KeyboardInterrupt:
        print("\nAborted by user.")
        return False


def run_workflow(
    controller: XArmController,
    auto: bool = False,
    open_gripper_at_start: bool = True,
) -> bool:
    """Execute the workflow sequence. Returns True if all steps succeeded."""

    if open_gripper_at_start:
        if not confirm("Step: open gripper", auto):
            return False
        if not controller.open_gripper(wait=True):
            print("Failed to open gripper.")
            return False

    if not confirm(f"Step: move linear track to 'Deck' at {TRACK_SPEED} mm/s", auto):
        return False
    if not controller.move_track_to_named_location("Deck", speed=TRACK_SPEED, wait=True):
        print("Failed to move track to Deck.")
        return False

    if not confirm(f"Step: move arm to 'deck_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_home", speed=JOINT_SPEED):
        print("Failed to move to deck_home.")
        return False

    if not confirm(f"Step: move arm to 'deck_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_high", speed=JOINT_SPEED):
        print("Failed to move to deck_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_slot1_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_slot1_high", speed=JOINT_SPEED):
        print("Failed to move to deck_slot1_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'deck_slot1_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("deck_slot1_low", speed=LINEAR_SPEED):
        print("Failed to move to deck_slot1_low.")
        return False

    if not confirm("Step: close gripper to 120 mm", auto):
        return False
    if not controller.move_gripper_to_stroke(120, wait=True):
        print("Failed to close gripper to 120 mm.")
        return False

    if not confirm(f"Step: move arm linearly back to 'deck_slot1_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("deck_slot1_high", speed=LINEAR_SPEED):
        print("Failed to move back to deck_slot1_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_high", speed=JOINT_SPEED):
        print("Failed to move to deck_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_solid_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_solid_high", speed=JOINT_SPEED):
        print("Failed to move to deck_solid_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'deck_solid_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("deck_solid_low", speed=LINEAR_SPEED):
        print("Failed to move linearly to deck_solid_low.")
        return False

    if not confirm("Step: open gripper fully", auto):
        return False
    if not controller.open_gripper(wait=True):
        print("Failed to open gripper.")
        return False

    if not confirm(f"Step: move arm linearly to 'deck_solid_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("deck_solid_high", speed=LINEAR_SPEED):
        print("Failed to move linearly to deck_solid_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_high", speed=JOINT_SPEED):
        print("Failed to move to deck_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_solid_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_solid_high", speed=JOINT_SPEED):
        print("Failed to move to deck_solid_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'deck_solid_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("deck_solid_low", speed=LINEAR_SPEED):
        print("Failed to move linearly to deck_solid_low.")
        return False

    if not confirm("Step: close gripper to 120 mm", auto):
        return False
    if not controller.move_gripper_to_stroke(120, wait=True):
        print("Failed to close gripper to 120 mm.")
        return False

    if not confirm(f"Step: move arm linearly to 'deck_solid_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("deck_solid_high", speed=LINEAR_SPEED):
        print("Failed to move linearly to deck_solid_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_high", speed=JOINT_SPEED):
        print("Failed to move to deck_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_home", speed=JOINT_SPEED):
        print("Failed to move to deck_home.")
        return False

    if not confirm(f"Step: move linear track to 'Home' at {TRACK_SPEED} mm/s", auto):
        return False
    if not controller.move_track_to_named_location("Home", speed=TRACK_SPEED, wait=True):
        print("Failed to move track to Home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_2_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_2_high", speed=JOINT_SPEED):
        print("Failed to move to opentrons_2_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_2_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_2_low", speed=LINEAR_SPEED):
        print("Failed to move to opentrons_2_low.")
        return False

    if not confirm("Step: open gripper fully", auto):
        return False
    if not controller.open_gripper(wait=True):
        print("Failed to open gripper.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_2_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_2_high", speed=JOINT_SPEED):
        print("Failed to move to opentrons_2_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_2_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_2_low", speed=LINEAR_SPEED):
        print("Failed to move to opentrons_2_low.")
        return False

    if not confirm("Step: close gripper to 120 mm", auto):
        return False
    if not controller.move_gripper_to_stroke(120, wait=True):
        print("Failed to close gripper to 120 mm.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_2_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_2_high", speed=LINEAR_SPEED):
        print("Failed to move to opentrons_2_high.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move linear track to 'Hood' at {TRACK_SPEED} mm/s", auto):
        return False
    if not controller.move_track_to_named_location("Hood", speed=TRACK_SPEED, wait=True):
        print("Failed to move track to Hood.")
        return False

    if not confirm(f"Step: move arm to 'hood_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_home", speed=JOINT_SPEED):
        print("Failed to move to hood_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_shaker_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_shaker_high", speed=JOINT_SPEED):
        print("Failed to move to hood_shaker_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_shaker_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_shaker_low", speed=LINEAR_SPEED):
        print("Failed to move linearly to hood_shaker_low.")
        return False

    if not confirm("Step: open gripper fully", auto):
        return False
    if not controller.open_gripper(wait=True):
        print("Failed to open gripper.")
        return False

    if not confirm(f"Step: move arm to 'hood_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_home", speed=JOINT_SPEED):
        print("Failed to move to hood_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_shaker_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_shaker_high", speed=JOINT_SPEED):
        print("Failed to move to hood_shaker_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_shaker_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_shaker_low", speed=LINEAR_SPEED):
        print("Failed to move to hood_shaker_low.")
        return False

    if not confirm("Step: close gripper to 120 mm", auto):
        return False
    if not controller.move_gripper_to_stroke(120, wait=True):
        print("Failed to close gripper to 120 mm.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_shaker_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_shaker_high", speed=LINEAR_SPEED):
        print("Failed to move to hood_shaker_high.")
        return False

    if not confirm(f"Step: move arm to 'hood_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_home", speed=JOINT_SPEED):
        print("Failed to move to hood_home.")
        return False

    if not confirm(f"Step: move linear track to 'Home' at {TRACK_SPEED} mm/s", auto):
        return False
    if not controller.move_track_to_named_location("Home", speed=TRACK_SPEED, wait=True):
        print("Failed to move track to Home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_2_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_2_high", speed=JOINT_SPEED):
        print("Failed to move to opentrons_2_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_2_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_2_low", speed=LINEAR_SPEED):
        print("Failed to move to opentrons_2_low.")
        return False

    if not confirm("Step: open gripper fully", auto):
        return False
    if not controller.open_gripper(wait=True):
        print("Failed to open gripper.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_2_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_2_high", speed=JOINT_SPEED):
        print("Failed to move to opentrons_2_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_2_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_2_low", speed=LINEAR_SPEED):
        print("Failed to move to opentrons_2_low.")
        return False

    if not confirm("Step: close gripper to 120 mm", auto):
        return False
    if not controller.move_gripper_to_stroke(120, wait=True):
        print("Failed to close gripper to 120 mm.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_2_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_2_high", speed=LINEAR_SPEED):
        print("Failed to move to opentrons_2_high.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move linear track to 'Hood' at {TRACK_SPEED} mm/s", auto):
        return False
    if not controller.move_track_to_named_location("Hood", speed=TRACK_SPEED, wait=True):
        print("Failed to move track to Hood.")
        return False

    if not confirm(f"Step: move arm to 'hood_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_home", speed=JOINT_SPEED):
        print("Failed to move to hood_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_shaker_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_shaker_high", speed=JOINT_SPEED):
        print("Failed to move to hood_shaker_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_shaker_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_shaker_low", speed=LINEAR_SPEED):
        print("Failed to move to hood_shaker_low.")
        return False

    if not confirm("Step: open gripper fully", auto):
        return False
    if not controller.open_gripper(wait=True):
        print("Failed to open gripper.")
        return False

    if not confirm(f"Step: move arm to 'hood_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_home", speed=JOINT_SPEED):
        print("Failed to move to hood_home.")
        return False

    if not confirm(f"Step: move linear track to 'Home' at {TRACK_SPEED} mm/s", auto):
        return False
    if not controller.move_track_to_named_location("Home", speed=TRACK_SPEED, wait=True):
        print("Failed to move track to Home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_6_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_6_high", speed=JOINT_SPEED):
        print("Failed to move to opentrons_6_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_6_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_6_low", speed=LINEAR_SPEED):
        print("Failed to move to opentrons_6_low.")
        return False

    if not confirm("Step: close gripper to 120 mm", auto):
        return False
    if not controller.move_gripper_to_stroke(120, wait=True):
        print("Failed to close gripper to 120 mm.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_6_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_6_high", speed=LINEAR_SPEED):
        print("Failed to move linearly to opentrons_6_high.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move linear track to 'Hood' at {TRACK_SPEED} mm/s", auto):
        return False
    if not controller.move_track_to_named_location("Hood", speed=TRACK_SPEED, wait=True):
        print("Failed to move track to Hood.")
        return False

    if not confirm(f"Step: move arm to 'hood_filter_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_filter_home", speed=JOINT_SPEED):
        print("Failed to move to hood_filter_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_filter_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_filter_high", speed=JOINT_SPEED):
        print("Failed to move to hood_filter_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_filter_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_filter_low", speed=LINEAR_SPEED):
        print("Failed to move to hood_filter_low.")
        return False

    if not confirm("Step: open gripper fully", auto):
        return False
    if not controller.open_gripper(wait=True):
        print("Failed to open gripper.")
        return False

    if not confirm(f"Step: move arm to 'hood_filter_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_filter_home", speed=JOINT_SPEED):
        print("Failed to move to hood_filter_home.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_filter_top_plate' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_filter_top_plate", speed=LINEAR_SPEED):
        print("Failed to move to hood_filter_top_plate.")
        return False

    if not confirm("Step: close gripper to 120 mm", auto):
        return False
    if not controller.move_gripper_to_stroke(120, wait=True):
        print("Failed to close gripper to 120 mm.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_filter_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_filter_high", speed=LINEAR_SPEED):
        print("Failed to move to hood_filter_high.")
        return False

    if not confirm(f"Step: move arm to 'hood_filter_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_filter_home", speed=JOINT_SPEED):
        print("Failed to move to hood_filter_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_home", speed=JOINT_SPEED):
        print("Failed to move to hood_home.")
        return False

    if not confirm(f"Step: move arm to 'deck_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_home", speed=JOINT_SPEED):
        print("Failed to move to deck_home.")
        return False

    if not confirm(f"Step: move arm to 'deck_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_high", speed=JOINT_SPEED):
        print("Failed to move to deck_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_slot2_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_slot2_high", speed=JOINT_SPEED):
        print("Failed to move to deck_slot2_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'deck_slot2_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("deck_slot2_low", speed=LINEAR_SPEED):
        print("Failed to move linearly to deck_slot2_low.")
        return False

    if not confirm("Step: open gripper fully", auto):
        return False
    if not controller.open_gripper(wait=True):
        print("Failed to open gripper.")
        return False

    if not confirm(f"Step: move arm linearly to 'deck_slot2_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("deck_slot2_high", speed=LINEAR_SPEED):
        print("Failed to move linearly to deck_slot2_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_high", speed=JOINT_SPEED):
        print("Failed to move to deck_high.")
        return False

    if not confirm(f"Step: move arm to 'deck_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_home", speed=JOINT_SPEED):
        print("Failed to move to deck_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_home", speed=JOINT_SPEED):
        print("Failed to move to hood_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_filter_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_filter_home", speed=JOINT_SPEED):
        print("Failed to move to hood_filter_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_filter_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_filter_high", speed=JOINT_SPEED):
        print("Failed to move to hood_filter_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_filter_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_filter_low", speed=LINEAR_SPEED):
        print("Failed to move linearly to hood_filter_low.")
        return False

    if not confirm("Step: close gripper to 120 mm", auto):
        return False
    if not controller.move_gripper_to_stroke(120, wait=True):
        print("Failed to close gripper to 120 mm.")
        return False

    if not confirm(f"Step: move arm linearly to 'hood_filter_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("hood_filter_high", speed=LINEAR_SPEED):
        print("Failed to move linearly to hood_filter_high.")
        return False

    if not confirm(f"Step: move arm to 'hood_filter_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_filter_home", speed=JOINT_SPEED):
        print("Failed to move to hood_filter_home.")
        return False

    if not confirm(f"Step: move arm to 'hood_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("hood_home", speed=JOINT_SPEED):
        print("Failed to move to hood_home.")
        return False

    if not confirm(f"Step: move linear track to 'Home' at {TRACK_SPEED} mm/s", auto):
        return False
    if not controller.move_track_to_named_location("Home", speed=TRACK_SPEED, wait=True):
        print("Failed to move track to Home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_6_high' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_6_high", speed=JOINT_SPEED):
        print("Failed to move to opentrons_6_high.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_6_low' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_6_low", speed=LINEAR_SPEED):
        print("Failed to move linearly to opentrons_6_low.")
        return False

    if not confirm("Step: open gripper fully", auto):
        return False
    if not controller.open_gripper(wait=True):
        print("Failed to open gripper.")
        return False

    if not confirm(f"Step: move arm linearly to 'opentrons_6_high' at {LINEAR_SPEED} mm/s", auto):
        return False
    if not controller.move_plate_linear("opentrons_6_high", speed=LINEAR_SPEED):
        print("Failed to move linearly to opentrons_6_high.")
        return False

    if not confirm(f"Step: move arm to 'opentrons_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("opentrons_home", speed=JOINT_SPEED):
        print("Failed to move to opentrons_home.")
        return False

    if not confirm(f"Step: move arm to 'deck_home' at {JOINT_SPEED} deg/s", auto):
        return False
    if not controller.move_to_named_location("deck_home", speed=JOINT_SPEED):
        print("Failed to move to deck_home.")
        return False

    print("\nWorkflow complete.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the deck-home workflow.")
    parser.add_argument("--profile", default="robot", help="Connection profile from xarm_config.yaml")
    parser.add_argument("--auto", action="store_true", help="Skip Enter prompts between steps")
    parser.add_argument(
        "--no-open-gripper",
        action="store_true",
        help="Skip opening the gripper at the start",
    )
    args = parser.parse_args()

    controller = XArmController(profile_name=args.profile, auto_enable=True)

    try:
        ok = run_workflow(
            controller,
            auto=args.auto,
            open_gripper_at_start=not args.no_open_gripper,
        )
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\nInterrupted - stopping robot.")
        controller.stop_motion()
        controller.disconnect()
        return 130
    finally:
        controller.disconnect()


if __name__ == "__main__":
    sys.exit(main())
