# Adaptive Flight Control

Adaptive Flight Control is a flight controller for compact quadrotors whose body changes in ways it cannot measure directly: a different frame, a new payload, slower motors, a weak motor. It is the cascaded PID that flight stacks such as PX4 already fly, made adaptive in the three places where fixed gains were measured to fail. Everyday flight stays the tuned PID. It is designed for a 220 mm, 500 g quadrotor at 500 Hz, and every result here is from simulation.

The targets it is built against are in [docs/requirements.md](docs/requirements.md).

## How It Works

**Cascade.** Position sets a velocity, velocity an acceleration, the acceleration an attitude, the attitude a body rate, and the rate loop the torques the mixer turns into four motor commands. Gains are tuned once on the nominal drone, with a rate-loop integral, hover-thrust estimation and a mixer that gives up yaw first, as PX4 ships.

**Weight.** Hover-thrust estimation compares the thrust commanded with the vertical acceleration that follows, and updates the mass the cascade believes while the drone is near level.

**Dynamics.** Two estimators read motor RPM and the gyro. One fits each motor's lag from how fast its speed closes on its command. Once motors are slower than the gains were tuned for, the attitude and rate loops are slowed to match, keeping the tuned margin. The other fits how strongly each motor turns each axis and takes the median of the four, so a weak motor cannot bias the inertia the torque commands are scaled by.

**Motor health.** From measured RPM the controller knows the roll and pitch torque the motors should produce, and from the gyro the torque they actually produce. A steady gap is torque that has gone missing. Each motor sits on its own diagonal, so the direction of the gap names the motor and its size gives its remaining thrust.

**Output.** Four normalised motor commands, and a condition readout of mass, motor lag, inertia and motor health.

## How It Performs

Each scenario is 200 randomised trials with fixed seeds, against a fair PID baseline with the same cascade and fixed gains.

<p>
  <img src="reports/figures/comparison.png" alt="Share of trials passed in each scenario by the fair PID and the adaptive controller">
  <br>
  <sub><b>PID and adaptive.</b> The share of 200 trials that meet each scenario's target, for the fair PID (grey) and the adaptive controller (blue). S1 to S5 are everyday flight, where both pass every trial. S6 and S7 count trials that hold within 0.3 m of hover after a motor is damaged. S8 is an airframe beyond the training range. S9, a motor failing completely, is outside the envelope.</sub>
</p>

| Scenario | Fair PID | Adaptive |
| :-- | --: | --: |
| S1 Nominal flight, median RMS error | 0.016 m | 0.016 m |
| S5 One motor at 60 %, median deviation | 0.090 m | 0.069 m |
| S6 One motor at 20 to 40 %, held within 0.3 m | 16 % | 56 % |
| S7 Payload, damaged motor and wind, held within 0.3 m | 0 % | 13 % |
| S8 Extreme airframe, within 0.15 m RMS | 96 % | 100 % |
| S8 Extreme airframe, 95th percentile RMS error | 0.132 m | 0.030 m |

On everyday flight the adaptive controller matches the PID to the millimetre, which is the point: adaptation only acts where the drone has changed. Its clearest gain is at the tail of S8, where motors up to 60 ms slow left the fixed gains with too little margin and the slowed loops keep it.

### Condition Readout

The estimators are read at the end of each flight and compared with the drone the simulator flew.

| Quantity | Scenario | Median error | Within target |
| :-- | :-- | --: | --: |
| Mass after a 30 % payload | S3 | 0.02 % | 100 % within 10 % |
| Mass, unknown airframe | S2 | 10.2 % | 49 % within 10 % |
| Mass in units of motor thrust, unknown airframe | S2 | 0.01 % | 100 % within 10 % |
| Motor lag, 10 to 40 ms | S2 | 0.76 ms | 100 % within 5 ms |
| Inertia, yaw | S2 | 10 % | 48 % within 10 % |
| Inertia, pitch | S2 | 19 % | 28 % within 10 % |
| Inertia, roll | S2 | 136 % | 0 % within 10 % |
| Health of a motor at 60 % | S5 | 0.01 pp | 100 % within 10 pp |
| Health of a motor at 20 to 40 % | S6 | 0.02 pp | 85 % within 10 pp |

Hover thrust is measured through the motors, so what it finds exactly is mass in units of their thrust, which is what the cascade needs to command. Read as kilograms, it carries the motor-constant error the controller cannot see, up to 20 %. Inertia is read well enough on yaw, roughly on pitch, and not on roll. The motor health of S6 is wrong only in the trials where the drone was already lost.

## Engineering Decisions

### An adaptive PID, not a learned policy

The first plan replaced the PID with a learned policy. A teacher policy trained with reinforcement learning flew far worse than the fair PID, and measuring the PID showed that it fails in only three specific ways: weight, slow dynamics, and a failing motor. The controller keeps the PID and adapts it in exactly those three ways, so every other flight is the behaviour a tuned stack already trusts.

### A fair baseline first

Against a PID without hover-thrust estimation or a rate integral, adaptation looked like a large win. Once the baseline had what PX4 ships, it met every everyday target in every trial. The everyday scenarios are therefore held to no regression, and the hard scenarios are the ones where fixed gains were measured to fail.

### Slowing the loops instead of retuning them

Slow motors add delay, and delay is what erodes a tuned loop's margin. Rather than search for new gains in flight, the controller scales the attitude and rate gains by the ratio of the tuned lag to the measured one, which keeps the margin the original tuning was built with.

## Operating Envelope

The controller is suited to a healthy or partly weakened quadrotor whose mass, inertia and motor speed are unknown or drift, in wind up to 8 m/s. It holds the everyday scenarios to the fair PID and extends the airframe range the PID can fly.

A motor failing completely (S9) is outside the envelope. With one motor off, its diagonal partner has to idle too, and only a controlled spin about the yaw axis can keep the drone upright. A cascade cannot fly that, so failure detection and the three-motor landing are built but switched off (`FAILURE_MODE` in `src/control/adaptive.py`), and neither controller lands any S9 trial. A motor left with 20 to 40 % thrust (S6, S7) sits at the edge of the envelope, held in about half of trials and a tenth of combined trials.

## How to Use It

```bash
git clone https://github.com/iamabdirazak/adaptive-flight-control.git
cd adaptive-flight-control
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[test]"
```

```python
from src.control.adaptive import Adaptive
from src.plant.vehicle import Vehicle

controller = Adaptive(Vehicle(), count)        # count drones flown together
controller.drone = drone                       # source of motor RPM telemetry
command = controller(state, target, heading, dt)
controller.mass, controller.lag.estimate, controller.inertia, controller.fault.health
```

### Reproducing the Results

```bash
python -m pytest -q
python -m src.evaluation.tune
python -m src.evaluation.baseline
python -m src.evaluation.readout
python -m src.evaluation.comparison
```

## Limitations

- All results are from simulation. The C library, the compute budget and the hardware are not yet built or measured.
- A complete motor failure is not handled (see Operating Envelope).
- Roll inertia is not estimated reliably, and mass in kilograms is only as good as the motor constants.
- Wind is not estimated, so the readout does not report it.
- RPM loss (S10) and the guard and fallback (S11) are not built.

## License

Released under the MIT License.

## Author

Abdirazak Mubarak, November 2026
