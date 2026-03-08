---
name: autotest
description: Run ArduPilot SITL autotests (integration/behavior tests). Use when the user asks to run autotests, vehicle tests, or specific test methods.
argument-hint: "<vehicle> [test_name]"
disable-model-invocation: true
allowed-tools: Bash(python3 *autotest*), Bash(./waf *), Read, Grep
---

# Run ArduPilot Autotests

Autotests are Python-based integration tests that run vehicles in SITL simulation.

## Argument parsing

Parse `$ARGUMENTS` for vehicle and optional test name:
- `/autotest Copter` — build and run all Copter tests
- `/autotest Copter AltHold` — run specific Copter test
- `/autotest Plane QuadPlane` — run specific Plane test
- `/autotest --list Copter` — list available Copter tests

## Workflow

### Step 1: Build the vehicle first

**Always build separately using `/build` or `./waf` — do NOT use `build.<Vehicle>` in autotest.py:**

```bash
./waf configure --board sitl    # if not already configured
./waf copter                    # or plane, rover, sub, etc.
```

### Step 2: List available tests (optional)

```bash
python3 Tools/autotest/autotest.py --list-subtests-for-vehicle=<Vehicle>
```

Vehicle names: `Copter`, `Plane`, `Rover`, `Sub`, `Tracker`, `Helicopter`, `QuadPlane`, `BalanceBot`, `Sailboat`, `Blimp`

### Step 3: Run tests

```bash
# Run all tests for a vehicle
python3 Tools/autotest/autotest.py test.<Vehicle>

# Run a specific test method
python3 Tools/autotest/autotest.py test.<Vehicle>.<TestMethod>
```

### Useful options

```bash
# Show test timing info
python3 Tools/autotest/autotest.py --show-test-timings test.Copter.AltHold

# Run with debug build (build with --debug first via /build)
python3 Tools/autotest/autotest.py --debug test.Copter.AltHold
```

## Test file locations

| Vehicle | Test file |
|---------|-----------|
| Copter | `Tools/autotest/arducopter.py` |
| Plane | `Tools/autotest/arduplane.py` |
| Rover | `Tools/autotest/rover.py` |
| Sub | `Tools/autotest/ardusub.py` |
| Helicopter | `Tools/autotest/helicopter.py` |
| Tracker | `Tools/autotest/antennatracker.py` |

## Report results

- Show pass/fail for each test method
- For failures, show the relevant error message and timeout info
- Autotests produce logs in `logs/` — mention these for further analysis
- Test output includes MAVLink messages that can help diagnose issues

## Common failure patterns

- **Timeout waiting for message** — vehicle didn't reach expected state in time
- **Altitude/position check failed** — vehicle didn't hit waypoint or target
- **Mode change rejected** — arming checks or pre-conditions not met
- **Build failure** — fix build first before running tests
