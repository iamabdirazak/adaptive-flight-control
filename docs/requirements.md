# Requirements

Adaptive Flight Control · version 3 · 19 November 2026

## Purpose

A flight controller for compact quadrotors that flies well when the drone changes in ways it cannot measure: a different frame, a new payload, wind, a weak or failed motor. It is delivered as a small C library that runs on the flight controller's microcontroller, inside an existing flight stack or custom firmware.

Every number in this document is a target. Results are reported against these targets once measured.

## What adaptive means

The controller is a cascaded PID that watches how the drone responds, works out what has changed about it, and changes its own settings to match while it flies. Everyday flight stays the tuned PID. Three things adapt, each tied to a failure measured on the fixed PID.

| What changes on the drone | What the controller adapts | Scenarios |
|---|---|---|
| Weight, from payload or battery | The thrust it commands, through hover-thrust estimation | S2, S3 |
| Dynamics, from inertia, slow motors or a different frame | The PID gains | S8 |
| A motor failing, partly or completely | The control mode: a controlled spin and a landing | S6, S7, S9 |

## Scope

| In | Out |
|---|---|
| Turning state and target into four motor commands | State estimation (provided by the host, e.g. PX4 EKF2) |
| Adapting to frame, payload, wind and motor health | Sensor drivers, radio, telemetry transport |
| Reporting the drone's condition (mass, wind, motor health) | Mission logic, navigation, obstacle avoidance |
| Guarding against its own faults, with a tuning-free fallback | Hexacopters and other layouts (quadrotor X only in v1) |

## Vehicle

The reference drone, used to build the simulator and later the hardware.

| Item | Target |
|---|---|
| Layout | Quadrotor, X configuration |
| Size | Compact, about 220 mm motor-to-motor diagonal, 5-inch propellers |
| Take-off mass | About 500 g |
| Thrust-to-weight | At least 4 : 1 healthy, so three motors can still carry it after a failure |
| Battery | 4S lithium polymer |
| Flight controller | STM32H7 class (Cortex-M7, hardware floating point, 480 MHz) |
| ESCs | Bidirectional DShot, reporting motor RPM |
| IMU | ICM-42688-P or equivalent |

Exact components are fixed during hardware design. The simulator is built from their datasheets.

## Scenarios

Each scenario is run in simulation as 200 randomised trials. Every controller is compared against a fair PID baseline, a cascaded PID with what PX4 ships: a rate-loop integral, hover-thrust estimation, a mixer that gives up yaw first, and gains tuned on the nominal drone with a delay margin. RAPTOR is added as a second reference where it applies.

The scenarios fall into two groups. In the everyday group the fair PID already meets every target, so there the adaptive controller must not do worse. The hard group lies past what fixed gains can handle, and it is the reason this controller exists.

### Everyday: no regression

Pass criterion: the adaptive controller's median is at most 1.1 × the fair PID's median, and every trial meets the floor.

| ID | Scenario | Floor | Fair PID median |
|---|---|---|---|
| S1 | Nominal flight, figure-eight at 2 m/s | ≤ 0.10 m RMS | 0.016 m |
| S2 | Unknown airframe: mass ±30 %, inertia ±40 %, motor constants ±20 %, motor delay 10 to 40 ms | ≤ 0.15 m RMS | 0.017 m |
| S3 | Payload added mid-hover, +30 % mass | ≤ 0.30 m drop | 0.080 m |
| S4 | Wind, steady up to 8 m/s with gusts | ≤ 0.50 m from hover | 0.056 m |
| S5 | One motor weakened to 60 % thrust | ≤ 0.30 m from hover | 0.088 m |

### Hard: past fixed gains

Pass criterion: at least 90 % of trials meet the target.

| ID | Scenario | Target | Fair PID |
|---|---|---|---|
| S6 | Motor damage, one motor left with 20 to 40 % thrust | Detected, then lands within 2 m, under 1.5 m/s, tilted under 30° | 17 % held position |
| S7 | Combined changes, payload +30 % and one motor at 40 %, in wind up to 8 m/s | Detected, then lands within 2 m, under 1.5 m/s, tilted under 30° | 0 % held position |
| S8 | Extreme airframe: mass ±45 %, inertia ±60 %, motor constants ±30 %, motor delay up to 60 ms | ≤ 0.15 m RMS | 81 % |
| S9 | One motor fails completely in hover | Lands within 2 m, under 1.5 m/s, tilted under 30° | 0 % |

### Adaptive controller only

| ID | Scenario | Target |
|---|---|---|
| S10 | Motor RPM telemetry lost | S1 RMS ≤ 2 × its own S1 |
| S11 | Policy output invalid or conditions outside the trained range | Guard takes over within one tick, levels the drone and descends |

## Condition report

| Quantity | Accuracy target |
|---|---|
| Mass | Within 10 % |
| Wind speed | Within 1.5 m/s |
| Motor health, per motor | Within 10 percentage points |

## Controller budgets

| Budget | Target |
|---|---|
| Control rate | 500 Hz |
| Compute per tick | < 0.2 ms on the reference flight controller |
| Memory | < 200 KB, weights in flash |
| Behaviour | Deterministic: no dynamic allocation, no randomness, fixed order of operations |
| Language | C99, no operating system dependency |
| Training | Hours, not days, on an Apple silicon Mac |

## Interface

```c
afc_init(&ctx, &config);          // once: motor layout and limits
afc_step(&ctx, &input, &output);  // every tick
afc_report(&ctx, &report);        // any time
```

Conventions follow PX4: NED world frame, FRD body frame, SI units, PX4 motor order.

| Input | Unit |
|---|---|
| Attitude (quaternion), angular rate | –, rad/s |
| Position, velocity | m, m/s |
| Specific force from the IMU | m/s² |
| Motor RPM, four motors (may be missing) | rpm |
| Target position, velocity and heading | m, m/s, rad |

| Output | Unit |
|---|---|
| Motor commands, four motors | normalised 0 to 1 |
| Status | adaptive, fallback or land |
| Report | mass, wind, motor health |

## Operating envelope

Training covers a range of conditions. Testing goes beyond that range to find where the controller stops working.

| Condition | Training range | Test range |
|---|---|---|
| Mass | ±30 % | ±45 % |
| Inertia | ±40 % | ±60 % |
| Motor constants | ±20 % | ±30 % |
| Wind | 0 to 8 m/s | 0 to 12 m/s |
| Motor health, one motor | 50 to 100 % | 0 to 100 % |
| Sensor noise | datasheet × 0.5 to 2 | datasheet × 3 |
| Motor delay | 10 to 40 ms | up to 60 ms |

## Verification

Each scenario runs at least 200 randomised trials with fixed seeds, so every result can be reproduced. The reported metrics are position error, altitude drop, settling time, landing success rate, readout accuracy and compute time per tick. Compute time is measured on the reference flight controller, not estimated.

## Open decisions

The final airframe and component choice, which comes from the hardware design. The adaptation memory, a fixed history window or a recurrent state, which is chosen by timing both against the compute budget.

## Changes from version 1

Version 1 set absolute targets before anything was measured. Once the PID baseline was made fair, it met every target from S1 to S5 in all trials, so those targets could not show what adaptation adds. Version 2 holds the everyday scenarios to the fair PID's measured numbers, and adds hard scenarios S6 to S8 where fixed gains were measured to fail. Motor failure moves from S6 to S9, RPM loss from S7 to S10 and the guard from S8 to S11.

## Changes from version 2

Version 2 planned a learned policy that replaces the PID. A first teacher policy trained with reinforcement learning flew far worse than the fair PID, and the measurements showed the PID only fails in three specific ways. Version 3 keeps the PID and makes it adaptive in exactly those three ways. A motor left with 20 to 40 % thrust is treated as a partial failure, so S6 and S7 are now judged on detecting it and landing safely rather than on holding position.

