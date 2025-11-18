# Hardware Test Procedure

Complete step-by-step procedure for running hardware tests with the satellite testbed.

**SAFETY FIRST**: Always follow proper safety protocols when working with pressurized air systems and moving hardware.

---

## Table of Contents

1. [Pre-Flight Checklist](#pre-flight-checklist)
2. [Thruster Calibration](#thruster-calibration)
3. [Battery Setup](#battery-setup)
4. [Floor Preparation](#floor-preparation)
5. [Air Cylinder Verification](#air-cylinder-verification)
6. [Satellite Balancing](#satellite-balancing)
7. [Air System Setup](#air-system-setup)
8. [ESP32 Gateway Connection](#esp32-gateway-connection)
9. [OptiTrack Setup](#optitrack-setup)
10. [Motive Data Server](#motive-data-server)
11. [Camera Configuration](#camera-configuration)
12. [Final Checks and Launch](#final-checks-and-launch)
13. [Troubleshooting](#troubleshooting)

---

## Pre-Flight Checklist

Before starting, ensure you have:

- [ ] Both 12V batteries fully charged
- [ ] Battery adapter/mount ready
- [ ] ESP32 gateway and USB cable
- [ ] Computer with all software/packages installed
- [ ] OptiTrack cameras powered
- [ ] Satellite platform accessible
- [ ] Three scales for balancing
- [ ] Soft cloths for protecting air bearings
- [ ] Sweep for floor
- [ ] Air cylinder with sufficient pressure (at least 500 psi)

---

## Thruster Calibration

Thruster calibration measures the **thrust output** of each thruster and generates calibration values for the MPC controller. This is essential for accurate attitude control.

**For detailed calibration procedures**, see [THRUSTER_CALIBRATION.md](THRUSTER_CALIBRATION.md).

---

## Battery Setup

### 3.1 Battery Installation

1. **Verify charge level** on both 12V batteries:
   - Connect batteries to charger if needed
   - Ensure full charge before installation

2. **Install batteries**:
   - Turn OFF both power switches
   - Plug both batteries into the battery adapters
   - Ensure secure connections
   - Turn ON both power switches

---

## Floor Preparation

### 4.1 Clean the Epoxy Floor

**Critical for proper operation!** Dust and debris can cause:
- Unwanted friction affecting motion
- External torques causing rotation
- Damage to air bearings

**Cleaning procedure:**

1. **First sweep**:
   - Use a clean Swiffer cloth
   - Sweep entire 6×6 meter test area
   - Pay attention to corners and edges

2. **Second sweep**:
   - Sweep again in perpendicular direction
   - Ensures all debris and dust is removed

3. **Visual inspection**:
   - Check for any remaining dust, hair, or particles
   - The floor should be spotless

> **Important**: Clean the floor **immediately before** placing the satellite. Don't clean hours in advance.

---

## Air Cylinder Verification

### 5.1 Verify Air Cylinder Pressure

Before proceeding with any tests, verify that your air cylinder has sufficient pressure:

1. **Screw the main regulator** onto the air cylinder
   - Turn the regulator fully to the left (so no air gets out)

2. **Open the valve** on the air cylinder
   - Check the pressure gauge

3. **Verify pressure level**:
   - Meter should read **500 PSI or higher**
   - If pressure is low, replace air cylinder

---

## Satellite Balancing

### 6.1 Equipment Setup

1. **Prepare three scales**:
   - Place scales on the epoxy floor in the corner off negative x negative y
   - Zero each scale

2. **Protect air bearings**:
   - Place a soft cloth on each scale
   - **Never** place air bearings directly on hard surfaces
   - This prevents scratches and damage

### 6.2 Initial Placement

   **Carefully place satellite** on the three scales
   - One air bearing per scale
   - Ensure satellite is level
   - Place air cylinder in the mount


### 6.3 Balancing Procedure

**Goal**: Get all three air bearings within **±50 grams** of each other.

#### Method 1: Rotate Air Cylinder

1. **Loosen air cylinder mount**

2. **Rotate cylinder slowly**:
   - Rotate in small increments
   - Check weights after each adjustment
   - Continue until balanced

3. **Tighten mount** when balanced

#### Method 2: Adjust Balance Weights

For fine-tuning:

1. **Locate small weights** mounted on the satellite body

2. **Move weights** to redistribute mass:

3. **Re-measure** until balanced

#### Method 3: Adjust Air Bearing Positions (Least recommended)

If Method 1 and 2 doesn't achieve balance:

1. **Loosen air bearing nuts**
   - Air bearings can slide along grooves in the base

2. **Move heavy-side bearing** slightly outward
   - OR move light-side bearing slightly inward

3. **Re-measure and repeat** until balanced

### 6.4 Final Balance Check

- [ ] All three scales within ±50g of each other
- [ ] Weights recorded for reference
- [ ] All mounts and screws tightened
- [ ] Update weights in code

---

## Air System Setup

### 7.1 Pressure Settings

| System | Pressure | Purpose |
|--------|----------|---------|
| Main Regulator | 80 PSI | Primary air supply |
| Air Bearings | 60 PSI | Creates hover cushion |
| Thrusters | 40 PSI | Thruster actuation force |

**Important**: If you change thruster pressure from 40 PSI, you **must** recalibrate all 8 thrusters and update thrust values in the code.

### 7.2 Air System Startup Sequence

1. **Open main air cylinder valve** (fully open)

2. **Set main regulator** to 80 PSI

3. **Set air bearing regulator** to 60 PSI
   - This is the "float" pressure
   - Too low = satellite won't hover
   - Too high = unstable hover (max 80 psi, higher values will damage the air bearings)

4. **Make sure thruster regulator is set to** to 40 PSI
   - Standard operating pressure
   - Calibrated thrust values are for 40 PSI
   - If adjusted new calibration is needed

5. **Open air bearing valve**

6. **Open thruster valve**

### 7.3 Pressure Verification

- [ ] Main regulator: 80 PSI ±2 PSI
- [ ] Air bearing regulator: 60 PSI ±2 PSI
- [ ] Thruster regulator: 40 PSI ±2 PSI
- [ ] No air leaks detected (audible check)
- [ ] All valves in correct positions

---

## ESP32 Gateway Connection

### 8.1 Connect Gateway

1. **Plug in ESP32 gateway** to computer via USB

2. **Identify the port**:

   **On macOS/Linux:**
   ```bash
   ls /dev/tty.*
   # or
   ls /dev/ttyUSB*
   ```

   **On Windows:**
   - Open Device Manager → Ports (COM & LPT)
   - Note the COM port number (e.g., COM20)

### 8.2 Update Serial Port Configuration

Edit `config/constants.py`:

```python
# Line 55
SERIAL_PORT = '/dev/tty.usbserial-XXXXX'  # macOS
# or
SERIAL_PORT = '/dev/ttyUSB0'              # Linux
# or
SERIAL_PORT = 'COM20'                      # Windows
```

> **File location**: `/config/constants.py` (line 55)

---

## OptiTrack Setup

### 9.1 Power On Cameras

1. **Turn on all OptiTrack cameras**
   - Wait for cameras to fully boot (LED indicators stable)
   - Typically takes 30-60 seconds

### 9.2 Launch Motive Software

1. **Open Motive** on the computer

2. **Load your calibration**:
   - File → Open → Select your calibrated project (create one if not done already)
   - Verify all cameras are active

3. **Verify tracking**:
   - Place a **dummy marker set** on the floor
   - Confirm Motive displays the rigid body
   - Check position values are stable and reasonable

### 9.3 Tracking Verification Checklist

- [ ] All cameras show as active in Motive
- [ ] Dummy rigid body appears in 3D view
- [ ] Position values are stable (not jumping)
- [ ] Frame rate shows ~240 fps
- [ ] No tracking errors or warnings

---

## Motive Data Server

### 10.1 Start Data Streaming

The Motive data server bridges Motive tracking data to your Python control system.

**Start the server:**

```bash
python3 Motive/motive_data_server.py
```

> **File location**: `Motive/motive_data_server.py`

### 10.2 Verify Data Transfer

You should see output similar to:
```
[INFO] Connected to Motive
[INFO] Streaming rigid body data...
[INFO] Rigid Body 0: x=0.123, y=0.456, z=0.789
```

### 10.3 Troubleshooting Data Stream

If no data appears:
1. Check Motive is in "Streaming" mode
2. Verify network settings (usually localhost)
3. Ensure rigid body is defined in Motive
4. Check firewall settings

---

## Camera Configuration

### 11.1 Ground Station Hotspot

1. **Enable hotspot** on the ground station computer (make sure to name you hotspot "MittNet" and set the password to "Satellite123" or reflash the esp32s3 sense cameras with new name and password)
   - Hotspot name: MittNet
   - Password: Satellite123

2. **Wait for cameras to connect** (30-60 seconds)
   - The 4 ESP32s3 sense cameras will auto-connect

### 11.2 Find Camera IP Addresses

**Check connected devices:**

- **On macOS/Linux**: Check System Preferences → Network → Hotspot
- **On Windows**: Settings → Network → Mobile hotspot → Connected devices
- **Or use**: `arp -a` command to list connected devices

You should see 4 devices with IPs like: `192.168.137.XX`

### 11.3 Update Camera URLs

Edit `config/camera.py`:

```python
# Lines 79-84
CAMERA_URLS = {
    "Front": "http://192.168.137.XX:81/stream",  # Update XX with actual IP
    "Right": "http://192.168.137.YY:81/stream",  # Update YY with actual IP
    "Back":  "http://192.168.137.ZZ:81/stream",  # Update ZZ with actual IP
    "Left":  "http://192.168.137.WW:81/stream",  # Update WW with actual IP
}
```

> **File location**: `config/camera.py` (lines 79-84)

### 11.4 Test Camera Streams

You can test individual camera URLs in a web browser:
```
http://192.168.137.XX:81/stream
```

Each stream should show live video from the camera.

---

## Final Checks and Launch

### 12.1 Pre-Launch Checklist

Go through the complete system:

**Power:**
- [ ] Both battery switches ON
- [ ] ESP32 gateway connected and responding
- [ ] All LEDs showing expected states (OFF when inactive, ON when active)

**Tracking:**
- [ ] Motive showing satellite position
- [ ] motive_data_server.py running
- [ ] Position data streaming correctly

**Cameras:**
- [ ] All 4 cameras connected to hotspot
- [ ] Camera streams accessible
- [ ] Camera URLs updated in config

**Air System:**
- [ ] Air bearings pressurized (60 PSI)
- [ ] Thrusters pressurized (40 PSI)
- [ ] Main regulator at 80 PSI
- [ ] No air leaks

**Software:**
- [ ] Serial port configured correctly
- [ ] Thrusters tested and verified
- [ ] Mission parameters set
- [ ] Emergency stop procedure reviewed

### 12.2 Place Satellite on Floor

1. **Carefully lift satellite** from scales
   - Support from multiple points
   - Keep level during transfer

2. **Place on cleaned floor**
   - Verify hover is stable
   - Satellite should float smoothly

3. **Verify hover quality**:
   - Satellite should spin freely with gentle push
   - No scraping sounds

### 12.3 Launch Test Mission

**Start the control system:**

```bash
python3 real.py
```

> **File location**: `real.py`

**What happens:**
1. System initializes connections
2. Cameras start streaming
3. Position tracking begins
4. Control loop starts
5. Mission executes

### 12.4 Monitor During Test

**Watch for:**
- Position tracking in Motive
- Camera feeds showing motion
- Terminal output for errors
- Satellite staying within bounds
- Smooth, controlled motion

**Emergency Stop:**
- Press `Ctrl+C` in terminal
- Close air supply valve if needed
- Cut OFF power using the switches if needed

---

## Troubleshooting

### Satellite Won't Hover

**Possible causes:**
- Air bearing pressure too low (increase to 60 PSI)
- Air bearings clogged (inspect and clean)
- Main valve not fully open
- Leak in air bearing lines

### Thrusters Not Firing

**Possible causes:**
- Battery power OFF
- Wrong serial port configured
- Thruster solenoid failure (test individually)
- Loose wires

### Tracking Lost

**Possible causes:**
- Markers obscured from cameras
- Lighting too bright/dim
- Rigid body lost in Motive
- motive_data_server.py crashed (restart it)

### Unstable Motion

**Possible causes:**
- Floor not clean (re-sweep)
- Satellite not balanced (re-balance on scales)
- Air bearing pressure inconsistent
- External air currents (close doors)

### Camera Streams Not Working

**Possible causes:**
- Cameras not connected to hotspot (check hotspot)
- Wrong IP addresses in config (verify IPs)
- Camera offline (power cycle camera)

### Unexpected Rotation

**Possible causes:**
- Satellite not balanced (check ±50g tolerance)
- Debris on floor (clean floor again)
- COM offset not calibrated correctly
- Thruster forces not calibrated

---

## Emergency Procedures

### Emergency Stop

1. **Press Ctrl+C** in control terminal
2. **Close thruster valve** immediately
3. **Close air bearing valve** if needed
4. **Turn OFF battery switches**

### Air System Failure

1. **Close main valve** on air cylinder
2. **Release pressure** from all regulators
3. **Turn OFF battery power**
4. **Do not attempt to restart** until issue resolved

### Collision Risk

1. **Emergency stop** (Ctrl+C)
2. **Manually stop satellite**
3. **Review mission parameters** before restarting

---

## Post-Test Shutdown

### Proper Shutdown Sequence

1. **Stop control program** (Ctrl+C)

2. **Close thruster valve**

3. **Close main air cylinder valve**

4. **Leave air bearing valve open to depressurize the system**

5. **Turn OFF battery switches**

6. **Stop motive_data_server.py** (Ctrl+C)

7. **Close Motive software**

8. **Disconnect ESP32 gateway**

9. **Turn OFF hotspot**

### Data Collection

After each test, collect:
- [ ] CSV data files from `Data/Real_Test/`
- [ ] Camera recordings (if enabled)
- [ ] Motive tracking data (if saved)
- [ ] Any error logs

---

## Quick Reference

### Key File Locations

| Configuration | File | Line |
|---------------|------|------|
| Serial Port | `config/constants.py` | 55 |
| Camera URLs | `config/camera.py` | 79-84 |
| Thruster Calibration | `Thruster_Test/thrust_calibration.py` | - |
| Motive Server | `Motive/motive_data_server.py` | - |
| Hardware Control | `real.py` | - |

### Standard Operating Pressures

- Main: **80 PSI**
- Air Bearings: **60 PSI**
- Thrusters: **40 PSI**
