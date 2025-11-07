#!/usr/bin/env python3
"""
Plate Pickup Demo

This demo demonstrates picking up a well plate from the deck using the linear track
and gripper system. The sequence performs a safe pickup operation.

Usage:
    python demo_plate_pickup.py --simulate    # Use simulation mode
    python demo_plate_pickup.py --real        # Use real hardware
    
Optional flags:
    --auto              # Skip user confirmations
    --slow              # Use 0.5x speed multiplier  
    --fast              # Use 2.0x speed multiplier
    --speed-multiplier  # Custom speed multiplier (e.g., --speed-multiplier 1.5)
"""

import argparse
import sys
import time
from core.xarm_controller import XArmController


def move_with_confirmation(controller, movement_func, description, auto_confirm=False, speed_info=None):
    """
    Execute a movement with optional user confirmation.
    
    Args:
        controller: XArmController instance
        movement_func: Function that performs the movement
        description: Description of the movement for user
        auto_confirm: If True, skip user confirmation
        speed_info: Dictionary with speed information to display
    
    Returns:
        bool: True if movement successful, False otherwise
    """
    print(f"\n{description}")
    
    if speed_info:
        if 'joint_speed' in speed_info:
            print(f"   Joint speed: {speed_info['joint_speed']}°/s")
        if 'tcp_speed' in speed_info:
            print(f"   TCP speed: {speed_info['tcp_speed']} mm/s")
        if 'track_speed' in speed_info:
            print(f"   Track speed: {speed_info['track_speed']} mm/s")
    
    if not auto_confirm:
        try:
            input("Press Enter to continue (Ctrl+C to abort)...")
        except KeyboardInterrupt:
            print("\nMovement aborted by user")
            controller.stop_motion()  # Stop robot immediately
            return False
    
    print("Executing movement...")
    try:
        success = movement_func()
        
        if success:
            print("Movement completed successfully")
            time.sleep(1)  # Brief pause between movements
            return True
        else:
            print("Movement failed")
            return False
    except KeyboardInterrupt:
        print("\nMovement interrupted - stopping robot immediately!")
        controller.stop_motion()  # Stop robot immediately
        return False


def get_speed_config():
    """
    Define speed configurations for each step of the plate pickup demo.
    These speeds are optimized for safe plate handling operations.
    
    Returns:
        dict: Speed configurations for each movement step
    """
    return {
        'robot_home': {
            'joint_speed': 20,   # °/s - Standard joint speed for homing
            'description': 'Robot joint homing speed with gripper open'
        },
        'move_to_local_1': {
            'track_speed': 150,  # mm/s - Moderate track speed to Local_1
            'description': 'Linear track movement to Local_1 position'
        },
        'deck_high': {
            'joint_speed': 15,   # °/s - Careful approach to deck area
            'description': 'Joint movement to deck high position'
        },
        'deck_low': {
            'joint_speed': 8,    # °/s - Very slow approach to plate level
            'description': 'Slow descent to plate pickup level'
        },
        'plate_pickup': {
            'description': 'Close gripper to pick up well plate'
        },
        'deck_high_return': {
            'joint_speed': 10,   # °/s - Careful lift with plate
            'description': 'Slow lift to safe height with plate'
        },
        'final_home': {
            'joint_speed': 15,   # °/s - Safe return to home with plate
            'description': 'Return to home position with plate'
        }
    }


def demo_plate_pickup(controller, auto_confirm=False, custom_speeds=None):
    """
    Execute the complete plate pickup sequence.
    
    Args:
        controller: XArmController instance
        auto_confirm: If True, skip user confirmations between movements
        custom_speeds: Optional dictionary to override default speeds
    
    Returns:
        bool: True if all movements successful, False otherwise
    """
    try:
        # Get speed configurations
        speeds = get_speed_config()
        if custom_speeds:
            speeds.update(custom_speeds)
    
        print("\n" + "=" * 60)
        print("PLATE PICKUP DEMO")
        print("=" * 60)
        print("This demo will execute the following sequence:")
        print("1. Robot joints → Home position + Open gripper")
        print("2. Linear track → Local_1 position")
        print("3. Joint movement → deck_high position") 
        print("4. Joint movement → deck_low position")
        print("5. Close gripper (grip well plate)")
        print("6. Joint movement → deck_high position (with plate)")
        print("7. Robot joints → Home position (with plate)")
        print("\nSpeed Configuration:")
        for step, config in speeds.items():
            if 'joint_speed' in config:
                print(f"   {step}: {config['joint_speed']}°/s (joint)")
            elif 'track_speed' in config:
                print(f"   {step}: {config['track_speed']} mm/s (track)")
            else:
                print(f"   {step}: {config['description']}")
        print("=" * 60)
        
        if not auto_confirm:
            try:
                input("Press Enter to start the demo (Ctrl+C to abort)...")
            except KeyboardInterrupt:
                print("\nDemo aborted by user")
                controller.stop_motion()
                return False
    
        # Get predefined positions
        positions = controller.position_config.get('positions', {})
        
        # Check that all required positions exist
        required_positions = [
            'robot_home', 'deck_high', 'deck_low'
        ]
        
        for pos_name in required_positions:
            if pos_name not in positions:
                print(f"Error: Position '{pos_name}' not found in position_config.yaml")
                return False
        
        # Check if linear track is enabled (the configuration will be validated by the movement function)
        if not controller.is_component_enabled('track'):
            print("Error: Linear track is not enabled")
            return False
    
        # Step 1: Robot joints to home + open gripper
        joint_speed = speeds['robot_home']['joint_speed']
        def move_home_and_open_gripper():
            success = controller.move_to_named_location('robot_home', speed=joint_speed)
            if success:
                controller.open_gripper()  # Open gripper when going home
            return success
        if not move_with_confirmation(
            controller,
            move_home_and_open_gripper,
            "Step 1: Moving robot joints to home position + opening gripper",
            auto_confirm,
            speeds['robot_home']
        ):
            return False
        
        # Step 2: Linear track to Local_1
        track_speed = speeds['move_to_local_1']['track_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_track_to_named_location('Local_1', speed=track_speed),
            "Step 2: Moving linear track to Local_1 position",
            auto_confirm,
            speeds['move_to_local_1']
        ):
            return False
        
        # Step 3: Joint movement to deck_high
        joint_speed = speeds['deck_high']['joint_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_to_named_location('deck_high', speed=joint_speed),
            "Step 3: Joint movement to deck_high position",
            auto_confirm,
            speeds['deck_high']
        ):
            return False
        
        # Step 4: Joint movement to deck_low (approach plate)
        joint_speed = speeds['deck_low']['joint_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_to_named_location('deck_low', speed=joint_speed),
            "Step 4: Joint movement to deck_low position (approach plate)",
            auto_confirm,
            speeds['deck_low']
        ):
            return False
        
        # Step 5: Close gripper to pick up plate
        if not move_with_confirmation(
            controller,
            lambda: controller.close_gripper(),
            "Step 5: Closing gripper to pick up well plate",
            auto_confirm,
            speeds['plate_pickup']
        ):
            return False
        
        # Step 6: Joint movement back to deck_high (with plate)
        joint_speed = speeds['deck_high_return']['joint_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_to_named_location('deck_high', speed=joint_speed),
            "Step 6: Joint movement to deck_high position (with plate)",
            auto_confirm,
            speeds['deck_high_return']
        ):
            return False
        
        # Step 7: Robot joints to home (with plate)
        joint_speed = speeds['final_home']['joint_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_to_named_location('robot_home', speed=joint_speed),
            "Step 7: Joint movement back to robot home position (with plate)",
            auto_confirm,
            speeds['final_home']
        ):
            return False
        
        print("\nPlate Pickup Demo completed successfully!")
        print("Well plate has been successfully picked up and moved to home position!")
        print("=" * 60)
        return True
    
    except KeyboardInterrupt:
        print("\nDemo sequence interrupted by user")
        controller.stop_motion()
        return False
    except Exception as e:
        print(f"\nDemo sequence failed with error: {e}")
        controller.stop_motion()
        return False


def main():
    parser = argparse.ArgumentParser(description='Plate Pickup Demo')
    parser.add_argument('--simulate', action='store_true', help='Simulation mode')
    parser.add_argument('--real', action='store_true', help='Real hardware mode')
    parser.add_argument('--auto', action='store_true', help='Auto-confirm all movements (no user prompts)')
    parser.add_argument('--slow', action='store_true', help='Use slow speed (0.5x multiplier)')
    parser.add_argument('--fast', action='store_true', help='Use fast speed (2.0x multiplier)')
    parser.add_argument('--speed-multiplier', type=float, default=1.0, help='Custom speed multiplier (default: 1.0)')
    
    args = parser.parse_args()
    
    # Determine mode
    if args.simulate and args.real:
        print("Cannot specify both --simulate and --real")
        sys.exit(1)
    
    if not args.simulate and not args.real:
        print("Must specify either --simulate or --real")
        sys.exit(1)
    
    simulate = args.simulate
    auto_confirm = args.auto
    
    # Process speed options
    custom_speeds = None
    speed_description = "Default"
    
    if args.slow and args.fast:
        print("Cannot specify both --slow and --fast")
        sys.exit(1)
    
    if args.slow:
        speed_multiplier = 0.5
        speed_description = "Slow (0.5x)"
    elif args.fast:
        speed_multiplier = 2.0
        speed_description = "Fast (2.0x)"
    elif args.speed_multiplier != 1.0:
        speed_multiplier = args.speed_multiplier
        speed_description = f"Custom ({speed_multiplier}x)"
    else:
        speed_multiplier = 1.0
    
    # Apply speed multiplier if specified
    if speed_multiplier != 1.0:
        base_speeds = get_speed_config()
        custom_speeds = {}
        for step, config in base_speeds.items():
            custom_speeds[step] = config.copy()
            if 'joint_speed' in config:
                custom_speeds[step]['joint_speed'] = config['joint_speed'] * speed_multiplier
            if 'track_speed' in config:
                custom_speeds[step]['track_speed'] = config['track_speed'] * speed_multiplier
    
    print("Plate Pickup Demo")
    print("=" * 50)
    print(f"Mode: {'SIMULATION' if simulate else 'REAL HARDWARE'}")
    print(f"Confirmation: {'AUTO' if auto_confirm else 'MANUAL'}")
    print(f"Speed: {speed_description}")
    print("=" * 50)
    
    try:
        # Initialize controller
        if simulate:
            controller = XArmController(
                simulation_mode=True,
                auto_enable=True
            )
        else:
            controller = XArmController(
                profile_name='real_hw',
                simulation_mode=False,
                auto_enable=True
            )
        
        if not controller.initialize():
            print("Failed to initialize controller")
            return
        
        print("Controller initialized successfully")
        
        # Check if linear track is available
        if not controller.is_component_enabled('track'):
            print("Warning: Linear track not enabled. Some movements may fail.")
        
        # Check if gripper is available
        if not controller.has_gripper():
            print("Warning: No gripper configured. Plate pickup will not work.")
        
        # Run the demo
        success = demo_plate_pickup(controller, auto_confirm, custom_speeds)
        
        if success:
            print("\nDemo completed successfully!")
        else:
            print("\nDemo completed with some failures")
            
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
        if 'controller' in locals() and controller:
            print("Stopping robot motion immediately...")
            controller.stop_motion()
    except Exception as e:
        print(f"Demo failed with error: {e}")
        if 'controller' in locals() and controller:
            print("Stopping robot motion due to error...")
            controller.stop_motion()
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.disconnect()
            print("Controller disconnected")


if __name__ == "__main__":
    main()
