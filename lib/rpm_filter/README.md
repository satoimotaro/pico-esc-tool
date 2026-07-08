# rpm_filter

eRPM → mechanical RPM and signal conditioning.

- `rpm = erpm / pole_pairs` (pole_pairs = MOTOR_POLES / 2).
- Reject checksum-error / dropout packets; light smoothing for stable low-speed readout
  (thruster range 100–4000 RPM, where noise matters most).

Status: stub. A0 does the raw division inline.
