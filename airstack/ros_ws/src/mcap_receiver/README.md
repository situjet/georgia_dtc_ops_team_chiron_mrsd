# MCAP Receiver Package

## Overview

MCAP Receiver is a ROS 2 package for receiving compressed MCAP files from remote machines over unstable radio connections and forwarding the ROS messages via UDP to specified targets.

## System Architecture

```
Remote Machine (10.3.1.32)
    ↓ (rsync over SSH)
Local Receive Directory (/home/lance/mcap_received/)
    ↓ (file monitoring)
MCAP Processor
    ↓ (decompress + verify)
ROS 2 Replayer
    ↓ (message replay)
UDP Publisher (dell_publisher)
    ↓ (UDP protocol)
Remote Receiver (rostopic_subscriber)
```

## Features

- **Reliable File Transfer**: Uses rsync with resume capability for unstable connections
- **Automatic Processing**: Monitors for new files and processes them automatically
- **ROS 2 Integration**: Native ROS 2 nodes and launch files
- **UDP Broadcasting**: Compatible with existing rostopic_subscriber.py
- **Production Ready**: Includes logging, error handling, and monitoring

## Quick Start

### 1. Build the Package

```bash
cd your_ros_workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select mcap_receiver
source install/setup.bash
```

### 2. Configure Connection

Edit `src/mcap_receiver/config/simple_config.env`:

```bash
REMOTE_IP="10.3.1.32"
UDP_TARGET_IP="10.3.1.106"
UDP_TARGET_PORT="5005"
```

### 3. Run the System

#### Option 1: Main Node
```bash
ros2 run mcap_receiver mcap_receiver_node
```

#### Option 2: Launch File
```bash
ros2 launch mcap_receiver mcap_bridge.launch.py
```

#### Option 3: Shell Scripts
```bash
cd src/mcap_receiver
./scripts/simple_main_loop.sh
```

## Package Structure

```
mcap_receiver/
├── config/
│   └── simple_config.env          # Configuration file
├── launch/
│   └── mcap_bridge.launch.py      # Launch file
├── mcap_receiver/
│   ├── mcap_receiver_node.py      # Main coordination node
│   ├── simple_mcap_processor.py   # File processor
│   ├── mcap_to_rostopic_replayer.py # MCAP replayer
│   └── mcap_to_udp_bridge.py      # UDP bridge
├── scripts/
│   ├── mcap_sync.sh               # File sync script
│   ├── mcap_watcher.sh            # File monitoring
│   └── simple_main_loop.sh        # Main loop
└── test/
    └── test_installation.sh       # Installation test
```

## Nodes

### mcap_receiver_node
Main coordination node that manages the entire pipeline:
- Syncs files from remote machine
- Monitors for new files
- Processes MCAP files
- Coordinates with UDP publisher

**Parameters:**
- `remote_ip`: Remote machine IP address
- `remote_user`: SSH username
- `local_mcap_dir`: Local directory for received files
- `udp_target_ip`: UDP target IP
- `udp_target_port`: UDP target port
- `sync_interval`: Sync interval in seconds

### simple_mcap_processor
Processes individual MCAP files:
- Verifies gzip integrity
- Decompresses files
- Starts bridge processing
- Marks files as processed

### mcap_to_rostopic_replayer
Replays MCAP messages to ROS topics:
- Analyzes MCAP file structure
- Creates publishers for each topic
- Replays messages with timing

### mcap_to_udp_bridge
Bridges ROS topics to UDP:
- Coordinates replayer and UDP publisher
- Manages process lifecycle
- Handles signal cleanup

## Configuration

### SSH Setup

For passwordless operation, set up SSH keys:

```bash
ssh-keygen -t ed25519 -C "mcap_receiver"
ssh-copy-id dtc@10.3.1.32
```

### Environment Variables

The system uses these environment variables (from config file):

- `REMOTE_USER`: SSH username (default: "dtc")
- `REMOTE_IP`: Remote machine IP
- `REMOTE_MCAP_DIR`: Remote MCAP directory
- `LOCAL_MCAP_DIR`: Local receive directory
- `UDP_TARGET_IP`: UDP target IP
- `UDP_TARGET_PORT`: UDP target port
- `SYNC_INTERVAL_SECONDS`: Sync interval

## Dependencies

### ROS 2 Packages
- `rclpy`
- `std_msgs`
- `dell_publisher` (for UDP publishing)

### System Dependencies
- `rsync`
- `sshpass` (optional, for password auth)
- `gzip`
- `python3-mcap` (for MCAP processing)

### Python Dependencies
- `mcap`
- `pathlib`
- `subprocess`

## Usage Examples

### Test Individual Components

```bash
# Test SSH connection
ssh dtc@10.3.1.32 "ls ~/tmp/mcap_files/"

# Test file sync
./scripts/mcap_sync.sh

# Test file monitoring
./scripts/mcap_watcher.sh

# Test file processing
ros2 run mcap_receiver simple_mcap_processor /path/to/file.mcap.gz
```

### Monitor System

```bash
# Check running nodes
ros2 node list

# Check topics
ros2 topic list
ros2 topic echo /mcap_replay/sensor_data

# Check parameters
ros2 param list /mcap_receiver_node
```

## Troubleshooting

### Common Issues

1. **SSH Connection Failed**
   - Check network connectivity: `ping 10.3.1.32`
   - Verify SSH credentials
   - Check firewall settings

2. **Package Not Found**
   - Rebuild package: `colcon build --packages-select mcap_receiver`
   - Source workspace: `source install/setup.bash`

3. **File Processing Errors**
   - Check file permissions
   - Verify gzip integrity: `gzip -t file.mcap.gz`
   - Check disk space

4. **UDP Not Working**
   - Test UDP connection: `echo "test" | nc -u 10.3.1.106 5005`
   - Check port availability: `netstat -ulnp | grep 5005`

### Debug Commands

```bash
# Enable verbose logging
export RCUTILS_LOGGING_SEVERITY=DEBUG

# Check system resources
df -h /home/lance/mcap_received/
ps aux | grep mcap

# View logs
tail -f ~/.local/log/simple_mcap_receiver.log
```

## Performance Tuning

### Network Optimization
- Adjust rsync parameters for your network conditions
- Tune sync intervals based on data volume
- Use compression for large files

### System Resources
- Monitor disk space regularly
- Clean old processed files
- Use SSD storage for better I/O performance

### ROS 2 Optimization
- Adjust QoS settings for topics
- Use appropriate message buffer sizes
- Monitor CPU and memory usage

## Contributing

1. Follow ROS 2 coding standards
2. Add tests for new features
3. Update documentation
4. Test with real hardware setup

## License

This package is licensed under the Apache 2.0 License.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review log files
3. Test individual components
4. Check system dependencies

## Version History

- **0.0.1**: Initial release
  - Basic MCAP receiving and processing
  - ROS 2 integration
  - UDP broadcasting support