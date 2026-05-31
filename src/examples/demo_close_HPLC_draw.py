#!/usr/bin/env python3
"""
HPLC Drawer Closing Demo

This demo closes an HPLC drawer that is assumed to be already open.
The sequence performs the closing operation and returns to home position.

Usage:
    python demo_close_HPLC_draw.py --simulate    # Use simulation mode
    python demo_close_HPLC_draw.py --real        # Use real hardware
    
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
    print(f"\n🎯 {description}")
    
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
            print("\n⏹️ Movement aborted by user")
            controller.stop_motion()  # Stop robot immediately
            return False
    
    print("🔄 Executing movement...")
    try:
        success = movement_func()
        
        if success:
            print("✅ Movement completed successfully")
            time.sleep(1)  # Brief pause between movements
            return True
        else:
            print("❌ Movement failed")
            return False
    except KeyboardInterrupt:
        print("\n⏹️ Movement interrupted - stopping robot immediately!")
        controller.stop_motion()  # Stop robot immediately
        return False


def get_speed_config():
    """
    Define speed configurations for each step of the closing demo.
    These speeds are optimized for safe drawer closing operations.
    
    Returns:
        dict: Speed configurations for each movement step
    """
    return {
        'linear_home': {
            'track_speed': 100,  # mm/s - Normal track speed for homing
            'description': 'Linear motor homing speed'
        },
        'robot_home': {
            'joint_speed': 20,   # °/s - Standard joint speed for homing
            'description': 'Robot joint homing speed'
        },
        'uplc_draw_home': {
            'joint_speed': 20,   # °/s - Standard joint speed
            'description': 'Joint movement to drawer area'
        },
        'approach_open_drawer': {
            'joint_speed': 10,   # °/s - Slower approach to open drawer
            'description': 'Careful approach to open drawer position'
        },
        'position_for_closing': {
            'tcp_speed': 30,     # mm/s - Moderate linear speed for positioning
            'description': 'Linear positioning for drawer closing'
        },
        'close_drawer': {
            'tcp_speed': 15,     # mm/s - Slow and controlled closing
            'description': 'Very slow drawer closing action'
        },
        'retract_from_closed': {
            'tcp_speed': 20,     # mm/s - Slow retraction from closed drawer
            'description': 'Careful retraction from closed drawer'
        },
        'move_away_from_drawer': {
            'tcp_speed': 40,     # mm/s - Moderate speed moving away
            'description': 'Move away from drawer area'
        },
        'final_home': {
            'joint_speed': 20,   # °/s - Standard return to home
            'description': 'Return to home position'
        }
    }


def demo_hplc_drawer_closing(controller, auto_confirm=False, custom_speeds=None):
    """
    Execute the complete HPLC drawer closing sequence.
    
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
        print("🔬 HPLC DRAWER CLOSING DEMO")
        print("=" * 60)
        print("This demo will execute the following sequence:")
        print("1. Linear motor → Home position")
        print("2. Robot joints → Home position + Open gripper") 
        print("3. Joint movement → uplc_draw_home + Close gripper")
        print("4. Joint movement → uplc_draw_open_max (approach open drawer)")
        print("5. Linear movement → uplc_draw_open_min (position for closing)")
        print("6. Linear movement → uplc_draw_open_close (close drawer)")
        print("7. Linear movement → uplc_draw_open_min (retract from closed)")
        print("8. Linear movement → uplc_draw_open_max (move away)")
        print("9. Joint movement → Robot home")
        print("\n📊 Speed Configuration:")
        for step, config in speeds.items():
            if 'joint_speed' in config:
                print(f"   {step}: {config['joint_speed']}°/s (joint)")
            elif 'track_speed' in config:
                print(f"   {step}: {config['track_speed']} mm/s (track)")
            elif 'tcp_speed' in config:
                print(f"   {step}: {config['tcp_speed']} mm/s (linear)")
        print("=" * 60)
        
        if not auto_confirm:
            try:
                input("Press Enter to start the demo (Ctrl+C to abort)...")
            except KeyboardInterrupt:
                print("\n⏹️ Demo aborted by user")
                controller.stop_motion()
                return False
    
        # Get predefined positions
        positions = controller.position_config.get('positions', {})
        
        # Check that all required positions exist
        required_positions = [
            'robot_home', 'uplc_draw_home', 'uplc_draw_open_max', 
            'uplc_draw_open_min', 'uplc_draw_open_close'
        ]
        
        for pos_name in required_positions:
            if pos_name not in positions:
                print(f"❌ Error: Position '{pos_name}' not found in position_config.yaml")
                return False
    
        # Step 1: Linear motor to home
        track_speed = speeds['linear_home']['track_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_track_to_position(0, speed=track_speed),  # Home position is 0
            "Step 1: Moving linear motor to home position",
            auto_confirm,
            speeds['linear_home']
        ):
            return False
        
        # Step 2: Robot joints to home + open gripper
        joint_speed = speeds['robot_home']['joint_speed']
        def move_home_and_open_gripper():
            success = controller.move_to_named_location('robot_home', speed=joint_speed)
            if success:
                controller.open_gripper()  # Open gripper when going home
            return success
        if not move_with_confirmation(
            controller,
            move_home_and_open_gripper,
            "Step 2: Moving robot joints to home position + opening gripper",
            auto_confirm,
            speeds['robot_home']
        ):
            return False
        
        # Step 3: Joint movement to uplc_draw_home + close gripper
        joint_speed = speeds['uplc_draw_home']['joint_speed']
        def move_to_draw_home_and_close_gripper():
            success = controller.move_to_named_location('uplc_draw_home', speed=joint_speed)
            if success:
                controller.close_gripper()  # Close gripper when reaching drawer area
            return success
        if not move_with_confirmation(
            controller,
            move_to_draw_home_and_close_gripper,
            "Step 3: Joint movement to uplc_draw_home position + closing gripper",
            auto_confirm,
            speeds['uplc_draw_home']
        ):
            return False
        
        # Step 4: Joint movement to uplc_draw_open_max (approach open drawer)
        joint_speed = speeds['approach_open_drawer']['joint_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_to_named_location('uplc_draw_open_max', speed=joint_speed),
            "Step 4: Joint movement to approach open drawer position",
            auto_confirm,
            speeds['approach_open_drawer']
        ):
            return False
        
        # Step 5: Linear movement to uplc_draw_open_min (position for closing)
        tcp_speed = speeds['position_for_closing']['tcp_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_plate_linear('uplc_draw_open_min', num_steps=1, speed=tcp_speed),
            "Step 5: Linear movement to position for drawer closing",
            auto_confirm,
            speeds['position_for_closing']
        ):
            return False
        
        # Step 6: Linear movement to uplc_draw_open_close (close the drawer)
        tcp_speed = speeds['close_drawer']['tcp_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_plate_linear('uplc_draw_open_close', num_steps=1, speed=tcp_speed),
            "Step 6: Linear movement to close the drawer",
            auto_confirm,
            speeds['close_drawer']
        ):
            return False
        
        # Step 7: Linear movement back to uplc_draw_open_min (retract from closed)
        tcp_speed = speeds['retract_from_closed']['tcp_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_plate_linear('uplc_draw_open_min', num_steps=1, speed=tcp_speed),
            "Step 7: Linear movement to retract from closed drawer",
            auto_confirm,
            speeds['retract_from_closed']
        ):
            return False
        
        # Step 8: Linear movement to uplc_draw_open_max (move away from drawer)
        tcp_speed = speeds['move_away_from_drawer']['tcp_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_plate_linear('uplc_draw_open_max', num_steps=1, speed=tcp_speed),
            "Step 8: Linear movement away from drawer area",
            auto_confirm,
            speeds['move_away_from_drawer']
        ):
            return False
        
        # Step 9: Joint movement back to robot home
        joint_speed = speeds['final_home']['joint_speed']
        if not move_with_confirmation(
            controller,
            lambda: controller.move_to_named_location('robot_home', speed=joint_speed),
            "Step 9: Joint movement back to robot home position",
            auto_confirm,
            speeds['final_home']
        ):
            return False
        
        print("\n🎉 HPLC Drawer Closing Demo completed successfully!")
        print("=" * 60)
        return True
    
    except KeyboardInterrupt:
        print("\n⏹️ Demo sequence interrupted by user")
        controller.stop_motion()
        return False
    except Exception as e:
        print(f"\n❌ Demo sequence failed with error: {e}")
        controller.stop_motion()
        return False


def main():
    parser = argparse.ArgumentParser(description='HPLC Drawer Closing Demo')
    parser.add_argument('--simulate', action='store_true', help='Simulation mode')
    parser.add_argument('--real', action='store_true', help='Real hardware mode')
    parser.add_argument('--auto', action='store_true', help='Auto-confirm all movements (no user prompts)')
    parser.add_argument('--slow', action='store_true', help='Use slow speed (0.5x multiplier)')
    parser.add_argument('--fast', action='store_true', help='Use fast speed (2.0x multiplier)')
    parser.add_argument('--speed-multiplier', type=float, default=1.0, help='Custom speed multiplier (default: 1.0)')
    
    args = parser.parse_args()
    
    # Determine mode
    if args.simulate and args.real:
        print("❌ Cannot specify both --simulate and --real")
        sys.exit(1)
    
    if not args.simulate and not args.real:
        print("❌ Must specify either --simulate or --real")
        sys.exit(1)
    
    simulate = args.simulate
    auto_confirm = args.auto
    
    # Process speed options
    custom_speeds = None
    speed_description = "Default"
    
    if args.slow and args.fast:
        print("❌ Cannot specify both --slow and --fast")
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
            if 'tcp_speed' in config:
                custom_speeds[step]['tcp_speed'] = config['tcp_speed'] * speed_multiplier
    
    print("🔬 HPLC Drawer Closing Demo")
    print("=" * 50)
    print(f"Mode: {'SIMULATION' if simulate else 'REAL HARDWARE'}")
    print(f"Confirmation: {'AUTO' if auto_confirm else 'MANUAL'}")
    print(f"Speed: {speed_description}")
    print("=" * 50)
    
    try:
        # Initialize controller
        controller = XArmController(
            profile_name='robot',
            auto_enable=True,
        )


        if not controller.initialize():
            print("❌ Failed to initialize controller")
            return
        
        print("✅ Controller initialized successfully")
        
        # Check if linear track is available
        if not controller.is_component_enabled('track'):
            print("⚠️ Warning: Linear track not enabled. Some movements may fail.")
        
        # Run the demo
        success = demo_hplc_drawer_closing(controller, auto_confirm, custom_speeds)
        
        if success:
            print("\n🎊 Demo completed successfully!")
        else:
            print("\n⚠️ Demo completed with some failures")
            
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted by user")
        if 'controller' in locals() and controller:
            print("🛑 Stopping robot motion immediately...")
            controller.stop_motion()
    except Exception as e:
        print(f"❌ Demo failed with error: {e}")
        if 'controller' in locals() and controller:
            print("🛑 Stopping robot motion due to error...")
            controller.stop_motion()
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.disconnect()
            print("🔌 Controller disconnected")


if __name__ == "__main__":
    main()
