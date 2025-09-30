# Operator Directory

This directory contains various operator tools and systems for the Chiron project.

## Subdirectories

- `tak/` - **Atlanta CASEVAC CoT Test System** - Complete TAK integration testing suite
- `chiron-operator-extension/` - Chiron operator extension components
- `foxglove_extensions/` - Foxglove Studio extensions
- `gimbal-foxglove-controller/` - Gimbal control via Foxglove
- `gimbal-joy-controller/` - Gimbal joystick controller
- `ros_ws/` - ROS workspace for operator tools

## TAK System (tak/)

The `tak/` directory contains a comprehensive test system for CASEVAC scenarios:
- **Atlanta-focused scenarios**: 6 realistic emergency situations across Atlanta metro
- **PyTAK integration**: Full CoT message generation and transmission
- **WinTAK compatibility**: Ready for tactical awareness testing
- **Real-time updates**: Status progression every 45 seconds

### Quick Start TAK System

```bash
cd tak
pip install -r requirements_casevac.txt
python casevac_cot_test.py
```

See `tak/README.md` for complete documentation.

## Features

- **Clustered GPS Coordinates**: 5 CASEVAC scenarios positioned around Fort Benning, Georgia
- **Realistic Scenarios**: Combat trauma, IED blasts, heat exhaustion, vehicle accidents, training injuries
- **Medical Details**: Injury types, precedence levels, equipment needed, casualty counts
- **Status Updates**: Periodic updates showing medical team progress
- **TAK Integration**: Full compatibility with ATAK, WinTAK, iTAK, and TAK Server

## CASEVAC Scenarios

The script includes 5 pre-configured CASEVAC scenarios:

1. **CASEVAC-001**: Gunshot wound (Urgent, 1 casualty)
2. **CASEVAC-002**: IED blast (Urgent Surgical, 3 casualties) 
3. **CASEVAC-003**: Heat exhaustion (Priority, 1 casualty)
4. **CASEVAC-004**: Vehicle rollover (Urgent, 1 casualty)
5. **CASEVAC-005**: Training accident (Priority, 1 casualty)

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements_casevac.txt
```

2. Or install PyTAK directly:
```bash
pip install pytak
```

## Configuration

### Option 1: WinTAK Server Mode

1. Start WinTAK
2. Go to Settings → Network Preferences
3. Enable "TCP Server" on port 8087
4. Note your local IP address
5. Update `TAK_SERVER_URL` in `casevac_cot_test.py` if needed

### Option 2: External TAK Server

1. Update `TAK_SERVER_URL` in `casevac_cot_test.py`:
```python
TAK_SERVER_URL = "tcp://your-tak-server:8087"
```

### Option 3: FreeTAKServer (Local Testing)

1. Install FreeTAKServer:
```bash
pip install FreeTAKServer
```

2. Start the server:
```bash
python -m FreeTAKServer.controllers.services.FTS
```

3. Use default configuration in the script (localhost:8087)

## Usage

### 1. Test TAK Server Configuration

First, test your TAK server connection:

```bash
python tak_server_config.py
```

This will:
- Test connections to common TAK server configurations
- Provide connection instructions for WinTAK
- Create a basic TAK server configuration file

### 2. Run CASEVAC CoT Test

```bash
python casevac_cot_test.py
```

The script will:
- Send initial CASEVAC requests for all 5 scenarios
- Send periodic status updates every 30 seconds
- Display detailed logging information
- Continue until stopped with Ctrl+C

### 3. Verify in WinTAK

1. Ensure WinTAK is connected to the TAK server
2. Look for CASEVAC markers on the map around Fort Benning, GA coordinates
3. Click on markers to see detailed casualty information
4. Monitor for status updates appearing as new events

## CoT Message Format

The script generates military-standard CoT messages with:

- **Event Type**: `b-r-f-h-c` (CASEVAC marker)
- **GPS Coordinates**: Latitude/longitude with precision
- **Medical Details**: Injury type, precedence, casualty count
- **Equipment Needs**: Required medical equipment
- **Status Updates**: Real-time progress updates

## Troubleshooting

### Connection Issues

1. **"Cannot connect to TAK server"**:
   - Verify TAK server is running
   - Check firewall settings
   - Confirm port 8087 is open
   - Try different server URL formats

2. **"PyTAK import error"**:
   - Install PyTAK: `pip install pytak`
   - Ensure Python 3.7+ is installed
   - Check virtual environment activation

3. **No markers in WinTAK**:
   - Verify WinTAK is connected to the server
   - Check that the script is sending messages (look for log output)
   - Ensure coordinates are within map view area
   - Try zooming out to see the Fort Benning area

### Configuration Issues

1. **Wrong GPS area**:
   - Modify `BASE_LAT` and `BASE_LON` in the script
   - Update scenario coordinates as needed

2. **Server address changes**:
   - Update `TAK_SERVER_URL` at the top of the script
   - Use format: `tcp://hostname:port`

## Customization

### Adding New Scenarios

Edit the `CASEVAC_SCENARIOS` list in `casevac_cot_test.py`:

```python
{
    "id": "CASEVAC-006",
    "lat": 32.3700,
    "lon": -84.9500,
    "description": "New scenario description",
    "urgency": "PRIORITY",
    "precedence": "2",
    "equipment_needed": "Required equipment",
    "casualties": 1,
    "injury_type": "Injury category"
}
```

### Changing Update Frequency

Modify the sleep intervals in the `run()` method:

```python
await asyncio.sleep(30)  # Change from 30 seconds to desired interval
```

### Different GPS Areas

Update base coordinates and scenario locations:

```python
BASE_LAT = 40.7589  # New York Central Park
BASE_LON = -73.9851
```

## Testing Checklist

- [ ] PyTAK installed successfully
- [ ] TAK server connection verified
- [ ] WinTAK connected to server
- [ ] Script runs without errors
- [ ] CASEVAC markers appear in WinTAK
- [ ] Marker details display correctly
- [ ] Status updates received
- [ ] All 5 scenarios visible

## Next Steps

Once the basic test is working:

1. **Integration Testing**: Connect to real TAK infrastructure
2. **Scenario Expansion**: Add more diverse CASEVAC scenarios  
3. **Automation**: Schedule automated testing scenarios
4. **Monitoring**: Add metrics and logging for system performance
5. **Security**: Implement TLS/SSL for encrypted communications

## Support

For issues related to:
- **PyTAK**: Check [PyTAK Documentation](https://pytak.readthedocs.io/)
- **WinTAK**: Consult WinTAK user manual
- **TAK Server**: See TAK Server documentation
- **This Script**: Check the inline comments and logging output