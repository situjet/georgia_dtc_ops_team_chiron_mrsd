#!/usr/bin/env python3
"""
Gimbal Pitch Test Script

This script demonstrates gimbal control using the PayloadSDK:
1. Pitches the gimbal down by 30 degrees
2. Returns the gimbal to zero position

The script provides detailed timing information and status updates.
"""

import subprocess
import time
import sys
import os

def print_timestamped(message):
    """Print message with timestamp"""
    timestamp = time.strftime("%H:%M:%S", time.localtime())
    print(f"[{timestamp}] {message}")

def run_gimbal_test():
    """Run the gimbal pitch test"""
    print_timestamped("=" * 60)
    print_timestamped("GIMBAL PITCH TEST STARTING")
    print_timestamped("This test will:")
    print_timestamped("  1. Pitch gimbal DOWN by 30 degrees")
    print_timestamped("  2. Return gimbal to ZERO position")
    print_timestamped("=" * 60)
    
    # Build path to the executable
    # The executable is in the PayloadSDK build directory
    executable_path = "/root/ros_ws/src/PayloadSdk/build_examples/examples/gimbal_pitch_test"
    
    # Check if the executable exists
    if not os.path.exists(executable_path):
        print_timestamped(f"ERROR: Executable not found at {executable_path}")
        print_timestamped("Make sure the PayloadSDK examples have been built!")
        
        # Try alternative paths
        alt_paths = [
            "/workspace/airstack/ros_ws/build/PayloadSdk/examples/gimbal_pitch_test",
            "/workspace/build/PayloadSdk/examples/gimbal_pitch_test",
            "./gimbal_pitch_test",
            "../build/examples/gimbal_pitch_test"
        ]
        
        for alt_path in alt_paths:
            if os.path.exists(alt_path):
                executable_path = alt_path
                print_timestamped(f"Found executable at alternative path: {executable_path}")
                break
        else:
            print_timestamped("CRITICAL: Cannot locate gimbal_pitch_test executable!")
            return False
    
    try:
        print_timestamped(f"Executing: {executable_path}")
        print_timestamped("=" * 60)
        
        # Run the C++ executable
        process = subprocess.Popen(
            [executable_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        # Real-time output
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                print(output.strip())
        
        # Wait for completion
        return_code = process.wait()
        
        print_timestamped("=" * 60)
        if return_code == 0:
            print_timestamped("GIMBAL PITCH TEST COMPLETED SUCCESSFULLY!")
        else:
            print_timestamped(f"GIMBAL PITCH TEST FAILED WITH CODE: {return_code}")
        print_timestamped("=" * 60)
        
        return return_code == 0
        
    except FileNotFoundError:
        print_timestamped(f"ERROR: Could not execute {executable_path}")
        print_timestamped("Make sure the PayloadSDK has been built and the executable exists!")
        return False
    except subprocess.SubprocessError as e:
        print_timestamped(f"ERROR: Subprocess failed: {e}")
        return False
    except KeyboardInterrupt:
        print_timestamped("User interrupted the test!")
        if process:
            process.terminate()
        return False

def main():
    """Main function"""
    print_timestamped("Starting Python wrapper for gimbal pitch test...")
    
    success = run_gimbal_test()
    
    if success:
        print_timestamped("Test completed successfully!")
        sys.exit(0)
    else:
        print_timestamped("Test failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()