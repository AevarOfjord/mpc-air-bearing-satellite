---
name: Hardware Issue
about: Report a problem with the physical satellite hardware or air system
title: "[HARDWARE] "
labels: hardware
assignees: ''
---

## Hardware Component
**What hardware is affected?**
- [ ] Satellite body
- [ ] Air bearings
- [ ] Thrusters (which ones: _________)
- [ ] Air system / regulators / valves
- [ ] Batteries
- [ ] Solenoid valves
- [ ] Load cell (calibration bench)
- [ ] OptiTrack markers
- [ ] Cameras
- [ ] Other: ________

---

## Problem Description
**What is happening?**

A clear description of the hardware issue.

---

## Observable Symptoms
**What do you observe?**
- [ ] Component won't respond
- [ ] Unusual sounds / vibrations
- [ ] Physical damage visible
- [ ] Tracking lost
- [ ] Performance degraded
- [ ] Other: ________

---

## When Did This Start?
- After how many test runs?
- After which mission type? (Waypoint / Shape Following)
- Suddenly or gradually?
- First time or recurring?

---

## Pre-Issue Checklist
- [ ] I've reviewed [HARDWARE_TEST_PROCEDURE.md](https://github.com/AevarOfjord/mpc-air-bearing-satellite/blob/main/HARDWARE_TEST_PROCEDURE.md)
- [ ] I've checked air pressure levels (80/60/40 PSI)
- [ ] I've visually inspected the hardware
- [ ] I've performed a basic test (e.g., `python3 testing_environment.py`)
- [ ] Component is fully connected and secured
- [ ] Batteries are charged

---

## Recent Actions
**What was happening when this occurred?**
- Last successful operation: ________
- What changed since then: ________
- Have you recalibrated thrusters? When: ________

---

## Troubleshooting Attempted
**What have you already tried?**
- [ ] Restarted the system
- [ ] Checked all connections
- [ ] Recalibrated (if applicable)
- [ ] Tested individual components
- [ ] Cleaned components (markers, floor, bearings)
- [ ] Other: ________

---

## Data / Logs
**Please attach if available:**
- Most recent CSV data file
- Terminal output from last run
- Photos of the hardware issue (if applicable)
- Air pressure readings at time of issue

---

## Safety Status
- [ ] System has been shut down safely
- [ ] Air supply is OFF
- [ ] Batteries are disconnected
- [ ] No ongoing hazard

---

## Recommended Next Steps
What would you like to happen?
- Debug instructions
- Replacement part guidance
- Calibration assistance
- Other: ________
