# Adaptive Flight Control

A modular adaptive flight controller for quadrotors. A learned policy adapts to the frame, payload, wind and motor health, a guard with a tuning-free fallback watches over it, and the drone reports its own condition. It is built to run as a small C library on the flight controller.

![Controller architecture](docs/architecture.png)

The targets it is built against are in [docs/requirements.md](docs/requirements.md).
