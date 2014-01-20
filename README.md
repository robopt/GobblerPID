GobblerPID
==========

Fully interrupt driven PID control for ENR259
Freescale Assembly (HCS12) and embedded C programming was used to implement 3 Different PID (Proportional Integral Derivative) control systems with 2 working in tandem all driven by interrupts. PID anti-lock braking system was created from scratch to eliminate the issue of our robot skidding in corners, even though we had nearly 2" of rubber grip.


Overview:
====
- Sensor readings are setup to run in background, when a series of conversions completes (the 3 main sensors) an interrupt is triggered to save the values into an array
- Every ~2-3ms interrupt happens to average the n (default 50) most recent sensor readings (this interrupt has higher priority, wont leave to read sensors)
- Every 3 interrupts anti lock brakes are applied if need instead of PID if front sensor reading is too high
- Calculates P (current error), I (sum of error) and D (rate of change)
- Translates the PID values into motor values within an 8bit 0-255 range
  - If a value of 0 happens for a motor value, the motor is set to "brake" by allowing current to flow into both leads of the motor to resist movement


