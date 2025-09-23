#!/usr/bin/env python3
"""
mcap_receiver_node.py - Main node, coordinates MCAP receiving and processing
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import subprocess
import threading
import time
import os
from pathlib import Path

class McapReceiverNode(Node):
    """Main node, manages MCAP file receiving and processing workflow"""
    
    def __init__(self):
        super().__init__('mcap_receiver_node')
        
        # Declare parameters
        self.declare_parameter('remote_user', 'dtc',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                               description='Remote SSH user'))
        self.declare_parameter('remote_ip', '10.3.1.32',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                               description='Remote machine IP'))
        self.declare_parameter('remote_mcap_dir', '/home/dtc/tmp/mcap_files/',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                               description='Remote MCAP directory'))
        self.declare_parameter('local_mcap_dir', '/home/lance/mcap_received/',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                               description='Local MCAP directory'))
        self.declare_parameter('udp_target_ip', '10.3.1.106',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                               description='UDP target IP'))
        self.declare_parameter('udp_target_port', 5005,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                               description='UDP target port'))
        self.declare_parameter('sync_interval', 30.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                               description='Sync interval in seconds'))
        
        # Get parameters
        self.remote_user = self.get_parameter('remote_user').value
        self.remote_ip = self.get_parameter('remote_ip').value
        self.remote_mcap_dir = self.get_parameter('remote_mcap_dir').value
        self.local_mcap_dir = self.get_parameter('local_mcap_dir').value
        self.udp_target_ip = self.get_parameter('udp_target_ip').value
        self.udp_target_port = self.get_parameter('udp_target_port').value
        self.sync_interval = self.get_parameter('sync_interval').value
        
        # create necessary directories
        self.local_mcap_path = Path(self.local_mcap_dir)
        self.processed_dir = self.local_mcap_path / '.processed'
        self.partial_dir = self.local_mcap_path / '.rsync-partial'
        
        self.local_mcap_path.mkdir(parents=True, exist_ok=True)
        self.processed_dir.mkdir(exist_ok=True)
        self.partial_dir.mkdir(exist_ok=True)
        
        self.get_logger().info(f'MCAP Receiver Node initialized')
        self.get_logger().info(f'Remote: {self.remote_user}@{self.remote_ip}:{self.remote_mcap_dir}')
        self.get_logger().info(f'Local: {self.local_mcap_dir}')
        self.get_logger().info(f'UDP Target: {self.udp_target_ip}:{self.udp_target_port}')
        
        # start sync timer
        self.create_timer(self.sync_interval, self.sync_and_process)
        
        # execute once immediately
        self.sync_and_process()
    
    def sync_and_process(self):
        """execute sync and processing workflow"""
        self.get_logger().info('Starting sync cycle...')
        
        # 1. sync files
        if self.sync_files():
            # 2. find new files
            new_files = self.find_new_files()
            
            # 3. process new files
            for file_path in new_files:
                self.process_file(file_path)
    
    def sync_files(self):
        """sync files using rsync"""
        try:
            cmd = [
                'rsync', '-av', '--progress',
                '--partial', f'--partial-dir={self.partial_dir}',
                '--timeout=60',
                '-e', 'ssh -o ConnectTimeout=10 -o ServerAliveInterval=10',
                f'{self.remote_user}@{self.remote_ip}:{self.remote_mcap_dir}*.mcap.gz',
                str(self.local_mcap_path)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info('Sync completed successfully')
                return True
            else:
                self.get_logger().error(f'Sync failed: {result.stderr}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Sync error: {e}')
            return False
    
    def find_new_files(self):
        """find new unprocessed files"""
        new_files = []
        
        for mcap_file in self.local_mcap_path.glob('*.mcap.gz'):
            marker_file = self.processed_dir / f'{mcap_file.name}.done'
            
            if not marker_file.exists():
                # Verify gzip integrity
                if self.verify_gzip(mcap_file):
                    new_files.append(mcap_file)
                    self.get_logger().info(f'Found new file: {mcap_file.name}')
                else:
                    self.get_logger().warning(f'Corrupted file skipped: {mcap_file.name}')
        
        return new_files
    
    def verify_gzip(self, file_path):
        """verify gzip file integrity"""
        try:
            result = subprocess.run(['gzip', '-t', str(file_path)], 
                                  capture_output=True)
            return result.returncode == 0
        except:
            return False
    
    def process_file(self, file_path):
        """process single MCAP file"""
        self.get_logger().info(f'Processing: {file_path.name}')
        
        try:
            # call processor
            cmd = [
                'ros2', 'run', 'mcap_receiver', 'simple_mcap_processor',
                str(file_path)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                # create processing marker
                marker_file = self.processed_dir / f'{file_path.name}.done'
                marker_file.touch()
                self.get_logger().info(f'Successfully processed: {file_path.name}')
            else:
                self.get_logger().error(f'Processing failed: {result.stderr}')
                
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = McapReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
