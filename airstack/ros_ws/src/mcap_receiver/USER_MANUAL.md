# MCAP Receiver System User Manual

## 📋 Table of Contents

1. [System Overview](#system-overview)
2. [Quick Start](#quick-start)
3. [Detailed Configuration](#detailed-configuration)
4. [Usage Methods](#usage-methods)
5. [Troubleshooting](#troubleshooting)
6. [Maintenance Guide](#maintenance-guide)

---

## 🎯 System Overview

MCAP Receiver is a complete ROS 2 system for receiving compressed MCAP files from remote machines over unstable radio connections and forwarding the ROS messages via UDP to specified targets.

### System Architecture

```
Remote Machine (dtc@10.3.1.32)
    ↓ rsync over SSH
Local Receive Directory (/home/lance/mcap_received/)
    ↓ File monitoring + gzip verification
MCAP Processor (decompress + processing markers)
    ↓ ROS 2 bridge
MCAP Replayer (message replay to ROS topics)
    ↓ ROS message subscription
UDP Publisher (dell_publisher)
    ↓ UDP protocol (__start__ + data blocks + __end__)
Remote Receiver (rostopic_subscriber.py)
```

### Core Features

- ✅ **Reliable Transfer**: rsync + resume capability + integrity verification
- ✅ **Smart Processing**: automatic monitoring + duplicate prevention + error recovery
- ✅ **ROS Integration**: native ROS 2 nodes + launch files
- ✅ **UDP Compatible**: fully compatible with existing rostopic_subscriber.py
- ✅ **Production Ready**: automated scripts + logging + monitoring

---

## 🚀 Quick Start

### 1. Environment Setup

```bash
# Enter ROS workspace
cd georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws

# Activate ROS environment
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 2. Configure Connection

Edit configuration file:
```bash
nano src/mcap_receiver/config/simple_config.env
```

Update these key configurations:
```bash
REMOTE_IP="10.3.1.32"                  # Your remote machine IP
UDP_TARGET_IP="10.3.1.106"             # UDP target IP
```

### 3. Test SSH Connection

```bash
# Test connection (requires password input)
ssh dtc@10.3.1.32 "ls ~/tmp/mcap_files/"
```

### 4. Start System

```bash
# Method 1: Start main node
ros2 run mcap_receiver mcap_receiver_node

# Method 2: Use launch file
ros2 launch mcap_receiver mcap_bridge.launch.py
```

---

## ⚙️ Detailed Configuration

### SSH Configuration

#### Option 1: SSH Keys (Recommended)
```bash
# Generate keys
ssh-keygen -t ed25519 -C "mcap_receiver"

# Copy to remote machine
ssh-copy-id dtc@10.3.1.32

# Test passwordless connection
ssh dtc@10.3.1.32 "echo 'SSH connection successful'"
```

#### Option 2: Use Password
System will prompt for password when needed, or you can configure sshpass.

### Configuration File Details

`src/mcap_receiver/config/simple_config.env`:

```bash
# Remote connection configuration
REMOTE_USER="dtc"                       # SSH username
REMOTE_IP="10.3.1.32"                  # Remote machine IP
REMOTE_MCAP_DIR="/home/dtc/tmp/mcap_files/"     # Remote MCAP directory

# Local storage configuration
LOCAL_MCAP_DIR="/home/lance/mcap_received/"     # Local receive directory
PARTIAL_DIR="/home/lance/mcap_received/.rsync-partial/"  # rsync temp directory
PROCESSED_DIR="/home/lance/mcap_received/.processed/"    # Processing marker directory

# UDP target configuration
UDP_TARGET_IP="10.3.1.106"              # UDP target IP
UDP_TARGET_PORT="5005"                  # UDP target port

# System behavior
SYNC_INTERVAL_SECONDS=30                # Sync interval (seconds)
```

---

## 🎮 Usage Methods

### Method 1: ROS Node Approach (Recommended)

#### Start Main Node
```bash
cd georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start main coordination node
ros2 run mcap_receiver mcap_receiver_node
```

#### Use Launch File
```bash
# Start complete bridge system
ros2 launch mcap_receiver mcap_bridge.launch.py

# Custom parameters
ros2 launch mcap_receiver mcap_bridge.launch.py \
    udp_ip:=10.3.1.100 \
    udp_port:=5007
```

### Method 2: Shell Script Approach

#### Single Execution
```bash
cd src/mcap_receiver

# 1. Sync files
./scripts/mcap_sync.sh

# 2. Find new files
./scripts/mcap_watcher.sh

# 3. Process files
ros2 run mcap_receiver simple_mcap_processor /path/to/file.mcap.gz
```

#### Continuous Operation
```bash
# Run main loop
./scripts/simple_main_loop.sh
```

### Method 3: Individual Component Testing

#### Test File Sync
```bash
cd src/mcap_receiver
source config/simple_config.env
./scripts/mcap_sync.sh
```

#### Test File Monitoring
```bash
./scripts/mcap_watcher.sh
```

#### Test File Processing
```bash
ros2 run mcap_receiver simple_mcap_processor /path/to/test.mcap.gz
```

---

## 🔧 Troubleshooting

### Common Issues

#### Issue 1: SSH Connection Failed
**Symptoms**: `Permission denied (publickey,password)`

**Solutions**:
```bash
# Check network connection
ping 10.3.1.32

# Test SSH connection
ssh -v dtc@10.3.1.32

# If password needed, enter manually or configure SSH keys
```

#### Issue 2: Package Not Found
**Symptoms**: `Package 'mcap_receiver' not found`

**Solutions**:
```bash
# Ensure in correct directory
cd georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws

# Rebuild
source /opt/ros/humble/setup.bash
colcon build --packages-select mcap_receiver

# Reactivate environment
source install/setup.bash
```

#### Issue 3: File Permission Issues
**Symptoms**: Scripts cannot execute

**Solutions**:
```bash
# Set script permissions
chmod +x src/mcap_receiver/scripts/*.sh

# Check directory permissions
ls -la /home/lance/mcap_received/
```

#### Issue 4: UDP Data Not Received
**Symptoms**: Remote receiver has no data

**Solutions**:
```bash
# Test UDP connection
echo "test" | nc -u 10.3.1.106 5005

# Check if port is occupied
netstat -ulnp | grep 5005

# Test local UDP reception
nc -ul 5005
```

### Debug Commands

```bash
# View ROS node status
ros2 node list
ros2 node info /mcap_receiver_node

# View topics
ros2 topic list
ros2 topic echo /mcap_replay/sensor_data

# View parameters
ros2 param list /mcap_receiver_node
ros2 param get /mcap_receiver_node remote_ip
```

---

## 🛠️ Maintenance Guide

### Daily Checks

```bash
# Check system status
cd georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if nodes are running
ros2 node list | grep mcap

# Check disk space
df -h /home/lance/mcap_received/

# Check processing status
ls -la /home/lance/mcap_received/.processed/
```

### Cleanup Maintenance

```bash
# Clean old processing markers (7 days old)
find /home/lance/mcap_received/.processed/ -name "*.done" -mtime +7 -delete

# Clean temporary files
rm -rf /home/lance/mcap_received/.rsync-partial/*

# Check log size
ls -lh ~/.local/log/simple_mcap_receiver.log
```

### Performance Monitoring

```bash
# Monitor network connection
ping -c 5 10.3.1.32

# Monitor disk usage
du -sh /home/lance/mcap_received/

# Monitor processes
ps aux | grep mcap
```

---

## 📚 Command Reference

### ROS Commands

```bash
# Start nodes
ros2 run mcap_receiver mcap_receiver_node
ros2 run mcap_receiver simple_mcap_processor <file>

# Start launch files
ros2 launch mcap_receiver mcap_bridge.launch.py

# Parameter operations
ros2 param list /mcap_receiver_node
ros2 param set /mcap_receiver_node sync_interval 60.0
ros2 param get /mcap_receiver_node remote_ip
```

### Shell Scripts

```bash
# File sync
./scripts/mcap_sync.sh

# File monitoring
./scripts/mcap_watcher.sh

# Main loop
./scripts/simple_main_loop.sh
```

### Test Commands

```bash
# Test installation
./test_installation.sh

# Test SSH connection
ssh dtc@10.3.1.32 "echo test"

# Test UDP sending
echo "test" | nc -u 10.3.1.106 5005

# Test gzip files
gzip -t /path/to/file.mcap.gz
```

---

## 🎯 Best Practices

### Production Environment Deployment

1. **Configure SSH keys** to avoid password authentication
2. **Set appropriate sync intervals** to balance real-time performance and network load
3. **Monitor disk space** and regularly clean old files
4. **Configure log rotation** to prevent log files from becoming too large
5. **Test network interruption recovery** to ensure system robustness

### Security Considerations

1. **Limit SSH access** to only necessary users and IPs
2. **Use firewall** to restrict UDP port access
3. **Regular system updates** to keep security patches current
4. **Monitor abnormal activity** to detect issues promptly

### Performance Optimization

1. **Adjust rsync parameters** based on network conditions
2. **Use SSD storage** to improve file I/O performance
3. **Adjust sync intervals** based on data volume and network conditions
4. **Monitor system resources** to ensure no impact on other services

---

## 📞 Support Information

### Version Information
- **Version**: 0.0.1
- **ROS Version**: Humble
- **Python Version**: 3.10+
- **Supported Platform**: Ubuntu 22.04

### File Locations
- **Configuration file**: `src/mcap_receiver/config/simple_config.env`
- **Scripts directory**: `src/mcap_receiver/scripts/`
- **Log file**: `~/.local/log/simple_mcap_receiver.log`
- **Data directory**: `/home/lance/mcap_received/`

### Related Documentation
- **README.md**: Basic introduction and quick start
- **package.xml**: ROS package dependency information
- **Launch file**: `launch/mcap_bridge.launch.py`

---

*This manual provides a complete usage guide for the MCAP Receiver system. If you encounter issues, please follow the troubleshooting section or check the relevant log files.*