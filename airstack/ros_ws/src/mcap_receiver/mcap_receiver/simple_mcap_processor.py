#!/usr/bin/env python3
"""
simple_mcap_processor.py - Simplified MCAP processor
Designed for single-file single-target scenarios, removing unnecessary complexity
"""

import sys
import subprocess
import os
import gzip
from pathlib import Path

def process_mcap_file(filepath):
    """Simple and direct MCAP file processing"""
    filepath = Path(filepath)
    
    # Unified processing completion marker directory
    processed_dir = filepath.parent / '.processed'
    processed_dir.mkdir(exist_ok=True)
    
    # Marker file name based on original file
    marker_file = processed_dir / f"{filepath.name}.done"
    
    # Check if already processed
    if marker_file.exists():
        print(f"Already processed: {filepath}")
        return True
    
    try:
        print(f"Processing: {filepath}")
        
        # 1. Verify gzip integrity
        if filepath.suffix == '.gz':
            result = subprocess.run(['gzip', '-t', str(filepath)], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                print(f"ERROR: Corrupted gzip file: {filepath}")
                return False
        
        # 2. Decompress (if needed)
        mcap_file = filepath
        if filepath.suffix == '.gz':
            output_path = filepath.with_suffix('')
            print(f"Decompressing to: {output_path}")
            
            with gzip.open(filepath, 'rb') as f_in:
                with open(output_path, 'wb') as f_out:
                    f_out.write(f_in.read())
            
            mcap_file = output_path
        
        # 3. Process using bridge system (synchronously wait for completion)
        print(f"Starting bridge processing for: {mcap_file}")
        
        # Get script directory
        script_dir = Path(__file__).parent
        bridge_script = script_dir / "mcap_to_udp_bridge.py"
        
        # Get UDP target from environment variables
        udp_ip = os.environ.get('UDP_TARGET_IP', '10.3.1.106')
        udp_port = os.environ.get('UDP_TARGET_PORT', '5005')

        cmd = [
            'bash', '-c',
            f'set +u && source /opt/ros/humble/setup.bash && '
            f'source ../../../install/setup.bash && '
            f'python3 {bridge_script} {mcap_file} --udp-ip {udp_ip} --udp-port {udp_port}'
        ]
        
        # Execute synchronously, wait for bridge completion
        result = subprocess.run(cmd)
        
        if result.returncode == 0:
            print(f"ROS processing completed successfully: {mcap_file}")
        else:
            print(f"ROS processing failed with exit code: {result.returncode}")
            return False
        
        # 4. Mark as processed
        marker_file.touch()
        print(f"Processing completed: {filepath}")
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {e}")
        return False
    except Exception as e:
        print(f"ERROR processing {filepath}: {e}")
        return False

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Simple MCAP Processor with Bridge')
    parser.add_argument('mcap_file', nargs='?', help='Path to MCAP file (.mcap.gz)')
    parser.add_argument('--version', action='version', version='Simple MCAP Processor 1.0')
    
    args = parser.parse_args()
    
    if not args.mcap_file:
        parser.print_help()
        sys.exit(1)
    
    filepath = args.mcap_file
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        sys.exit(1)
    
    success = process_mcap_file(filepath)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
