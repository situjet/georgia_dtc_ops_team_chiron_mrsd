# MCAP Receiver Troubleshooting Guide

## 🚨 Common Issues Quick Reference

### Issue Categories
- [SSH Connection Issues](#ssh-connection-issues)
- [ROS Environment Issues](#ros-environment-issues)  
- [File Processing Issues](#file-processing-issues)
- [UDP Communication Issues](#udp-communication-issues)
- [Performance Issues](#performance-issues)

---

## SSH Connection Issues

### Issue: Permission denied (publickey,password)

**Symptoms:**
```
Permission denied, please try again.
dtc@10.3.1.32: Permission denied (publickey,password).
```

**Diagnosis:**
```bash
# Test basic connectivity
ping 10.3.1.32

# Test SSH with verbose output
ssh -v dtc@10.3.1.32

# Check SSH config
cat ~/.ssh/config
```

**Solutions:**

1. **Set up SSH keys (recommended):**
```bash
ssh-keygen -t ed25519 -C "mcap_receiver"
ssh-copy-id dtc@10.3.1.32
ssh dtc@10.3.1.32 "echo 'Connection successful'"
```

2. **Use password authentication:**
```bash
# Install sshpass if not available
sudo apt install sshpass

# Test with sshpass
sshpass -p 'your_password' ssh dtc@10.3.1.32 "ls"
```

3. **Check firewall settings:**
```bash
# On remote machine
sudo ufw status
sudo iptables -L

# On local machine
telnet 10.3.1.32 22
```

### Issue: Connection timeout

**Symptoms:**
```
ssh: connect to host 10.3.1.32 port 22: Connection timed out
```

**Solutions:**
```bash
# Check network connectivity
ping -c 5 10.3.1.32
traceroute 10.3.1.32

# Try different SSH port
ssh -p 2222 dtc@10.3.1.32

# Check if SSH service is running on remote
nmap -p 22 10.3.1.32
```

---

## ROS Environment Issues

### Issue: Package 'mcap_receiver' not found

**Symptoms:**
```
Package 'mcap_receiver' not found
```

**Diagnosis:**
```bash
# Check if package is built
ls install/mcap_receiver/

# Check ROS environment
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH

# List available packages
ros2 pkg list | grep mcap
```

**Solutions:**
```bash
# Rebuild package
cd your_ros_workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select mcap_receiver

# Source workspace
source install/setup.bash

# Verify installation
ros2 pkg list | grep mcap_receiver
```

### Issue: Python import errors

**Symptoms:**
```
ModuleNotFoundError: No module named 'mcap'
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```

**Solutions:**
```bash
# Install missing Python packages
pip3 install mcap

# For ROS Python issues, check Python version
python3 --version  # Should be 3.10 for ROS Humble

# Install ROS Python dependencies
sudo apt install python3-rclpy python3-std-msgs
```

### Issue: Node fails to start

**Symptoms:**
```
[ERROR] [mcap_receiver_node]: Failed to create node
```

**Diagnosis:**
```bash
# Check ROS daemon
ros2 daemon status
ros2 daemon stop
ros2 daemon start

# Check node directly
ros2 run mcap_receiver mcap_receiver_node --ros-args --log-level DEBUG
```

---

## File Processing Issues

### Issue: Files not syncing

**Symptoms:**
- No new files appear in local directory
- rsync command fails

**Diagnosis:**
```bash
# Test rsync manually
rsync -av --dry-run dtc@10.3.1.32:/home/dtc/tmp/mcap_files/ /home/lance/mcap_received/

# Check remote directory
ssh dtc@10.3.1.32 "ls -la /home/dtc/tmp/mcap_files/"

# Check local permissions
ls -la /home/lance/mcap_received/
```

**Solutions:**
```bash
# Create directories if missing
mkdir -p /home/lance/mcap_received/.rsync-partial
mkdir -p /home/lance/mcap_received/.processed

# Fix permissions
chmod 755 /home/lance/mcap_received/
chown $USER:$USER /home/lance/mcap_received/

# Test with verbose rsync
rsync -avv --progress dtc@10.3.1.32:/home/dtc/tmp/mcap_files/ /home/lance/mcap_received/
```

### Issue: Corrupted GZIP files

**Symptoms:**
```
ERROR: Corrupted gzip file: recording.mcap.gz
```

**Diagnosis:**
```bash
# Test gzip integrity
gzip -t /home/lance/mcap_received/recording.mcap.gz

# Check file size
ls -lh /home/lance/mcap_received/recording.mcap.gz

# Compare with remote
ssh dtc@10.3.1.32 "ls -lh /home/dtc/tmp/mcap_files/recording.mcap.gz"
```

**Solutions:**
```bash
# Re-download corrupted file
rm /home/lance/mcap_received/recording.mcap.gz
./scripts/mcap_sync.sh

# Use rsync with checksum verification
rsync -avc --progress dtc@10.3.1.32:/home/dtc/tmp/mcap_files/ /home/lance/mcap_received/
```

### Issue: Processing markers not created

**Symptoms:**
- Files processed repeatedly
- No .done files in .processed directory

**Solutions:**
```bash
# Check processed directory permissions
ls -la /home/lance/mcap_received/.processed/
chmod 755 /home/lance/mcap_received/.processed/

# Manually create marker for testing
touch /home/lance/mcap_received/.processed/test_file.mcap.gz.done

# Check disk space
df -h /home/lance/mcap_received/
```

---

## UDP Communication Issues

### Issue: UDP data not received

**Symptoms:**
- Remote receiver shows no data
- No network traffic on UDP port

**Diagnosis:**
```bash
# Test UDP port locally
nc -ul 5006 &
echo "test message" | nc -u localhost 5006

# Check if port is in use
netstat -ulnp | grep 5006
ss -ulnp | grep 5006

# Monitor network traffic
sudo tcpdump -i any port 5006
```

**Solutions:**
```bash
# Test UDP connectivity to remote
echo "test" | nc -u 10.3.1.106 5006

# Check firewall rules
sudo ufw status
sudo iptables -L OUTPUT | grep 5006

# Verify dell_publisher is running
ros2 node list | grep publisher
ros2 topic list | grep sensor_data
```

### Issue: UDP publisher not starting

**Symptoms:**
```
Failed to start UDP publisher
```

**Diagnosis:**
```bash
# Check dell_publisher package
ros2 pkg list | grep dell_publisher

# Test publisher directly
ros2 run dell_publisher rostopic_publisher --help

# Check topic availability
ros2 topic list
ros2 topic info /mcap_replay/sensor_data
```

---

## Performance Issues

### Issue: Slow file processing

**Symptoms:**
- Long delays between file sync and processing
- High CPU usage

**Diagnosis:**
```bash
# Monitor system resources
top
htop
iotop

# Check disk I/O
iostat -x 1

# Monitor network usage
iftop
nethogs
```

**Solutions:**
```bash
# Adjust sync interval
ros2 param set /mcap_receiver_node sync_interval 60.0

# Use faster storage
# Move to SSD if using HDD
sudo mount -o remount,noatime /home/lance/mcap_received/

# Optimize rsync
# Add --compress flag for slow networks
rsync -avz --compress-level=1 ...
```

### Issue: Memory usage growing

**Symptoms:**
- Increasing RAM usage over time
- System becomes unresponsive

**Solutions:**
```bash
# Monitor memory usage
watch -n 1 'free -h'
ps aux --sort=-%mem | head

# Restart nodes periodically
# Add to crontab for automatic restart
0 */6 * * * /usr/bin/pkill -f mcap_receiver_node

# Clean up old files
find /home/lance/mcap_received/ -name "*.mcap" -mtime +7 -delete
```

---

## Debug Commands

### System Information
```bash
# ROS environment
printenv | grep ROS
ros2 doctor

# System resources
df -h
free -h
lscpu
```

### Network Diagnostics
```bash
# Connectivity
ping -c 5 10.3.1.32
traceroute 10.3.1.32

# Port testing
nmap -p 22,5006 10.3.1.32
telnet 10.3.1.106 5006
```

### ROS Diagnostics
```bash
# Node information
ros2 node list
ros2 node info /mcap_receiver_node

# Topic information
ros2 topic list
ros2 topic hz /mcap_replay/sensor_data
ros2 topic echo /mcap_replay/sensor_data --once

# Parameter information
ros2 param list /mcap_receiver_node
ros2 param dump /mcap_receiver_node
```

### Log Analysis
```bash
# ROS logs
ros2 log level /mcap_receiver_node DEBUG

# System logs
journalctl -u your_service_name -f
tail -f ~/.local/log/simple_mcap_receiver.log

# Process monitoring
ps aux | grep mcap
pgrep -f mcap_receiver
```

---

## Recovery Procedures

### Complete System Reset
```bash
# Stop all processes
pkill -f mcap_receiver
pkill -f rostopic_publisher

# Clean temporary files
rm -rf /home/lance/mcap_received/.rsync-partial/*
rm -rf /home/lance/mcap_received/.processed/*

# Restart ROS daemon
ros2 daemon stop
ros2 daemon start

# Rebuild and restart
cd your_ros_workspace
colcon build --packages-select mcap_receiver
source install/setup.bash
ros2 run mcap_receiver mcap_receiver_node
```

### Emergency File Recovery
```bash
# Recover from backup
rsync -av backup_location/ /home/lance/mcap_received/

# Force re-download all files
rm /home/lance/mcap_received/*.mcap.gz
./scripts/mcap_sync.sh

# Reset processing markers
rm /home/lance/mcap_received/.processed/*.done
```

---

## Prevention Tips

1. **Regular Monitoring:**
   - Set up log rotation
   - Monitor disk space daily
   - Check network connectivity regularly

2. **Automated Recovery:**
   - Use systemd services for auto-restart
   - Implement health checks
   - Set up alerts for failures

3. **Maintenance Schedule:**
   - Weekly: Clean old files and logs
   - Monthly: Update system packages
   - Quarterly: Review and optimize configuration

4. **Documentation:**
   - Keep configuration changes documented
   - Maintain troubleshooting logs
   - Update this guide with new issues

---

## Getting Help

If issues persist after following this guide:

1. **Collect Information:**
   ```bash
   # System info
   uname -a
   ros2 doctor
   
   # Logs
   ros2 log level /mcap_receiver_node DEBUG
   journalctl -u your_service --since "1 hour ago"
   
   # Configuration
   cat src/mcap_receiver/config/simple_config.env
   ros2 param dump /mcap_receiver_node
   ```

2. **Check Documentation:**
   - USER_MANUAL.md for usage instructions
   - README.md for system overview
   - Launch files for parameter details

3. **Test Components Individually:**
   - SSH connection
   - File sync
   - ROS nodes
   - UDP communication

Remember: Most issues are related to network connectivity, file permissions, or ROS environment setup. Start with the basics and work systematically through each component.