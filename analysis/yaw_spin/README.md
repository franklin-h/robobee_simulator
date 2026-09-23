# Yaw spin 3 and 4 diagnosis

The early yaw lag is not caused by WLQP failing to allocate the requested yaw
moment. The evidence points to a yaw feedback/feedforward design that does not
account for the moving vehicle's yaw dynamics. The static calibration has a
moderate local error, but no yaw sign inversion or large static gain error on
the inspected calibration slices. These recordings alone cannot uniquely
separate aerodynamic yaw damping, free-flight actuator gain changes and other
coupled dynamics.

## Recorded behavior

Both runs command 1.256637 rad/s (72 deg/s), with a 0.1 s rate ramp beginning at
0.5 s. This is below the controller's 6 rad/s heading-reference slew limit.
Reconstructing the controller equation from the logged row-major rotation,
reference, geometric angular-rate output and yaw torque recovers these gains
with approximately 1e-16 uNm RMS residual:

| Quantity | yaw_spin3 | yaw_spin4 |
|---|---:|---:|
| Kp [uNm/rad] | 0.1 | 0.1 |
| Kd [uNm/(rad/s)] | 0.009 | 0.009 |
| Ki [uNm/(rad s)] | 0.3 | 0 |
| Heading at 1.5 s | 16.33 deg | 10.12 deg |
| Reference at 1.5 s | 68.39 deg | 68.39 deg |
| Heading lag at 1.5 s | 52.06 deg | 58.27 deg |
| Yaw torque request at 1.5 s | 0.2405 uNm | 0.1114 uNm |
| First roll split <= -0.12 after spin starts | 1.9998 s | 2.9050 s |
| First tilt >30 deg after spin starts | 2.2990 s | 3.6600 s |

The 1.5 s values are means over +/-25 ms to reduce flap ripple. Spin 3 eventually
exceeds the reference yaw rate; spin 4 still has about 148 deg heading error at
3.5 s. The later attitude breakdown should not be used to tune a scalar yaw
loop.

## WLQP and map checks

Using `popts_fit_20260918_005015.mat`, selected by the current setup script,
recompute the full quadratic map at each logged actuator command. Account for
the actual causal 32-sample moving average applied to the wrench request inside
the SLX wrapper. Over 0.65–1.8 s, yaw map/request RMS errors are only
6.10e-7 and 5.53e-7 uNm respectively. This very close match also supports that
this is the map used in the recordings, although the Dataset files do not save
a complete parameter/source snapshot.

Over that window neither run reaches the yaw harmonic slew limit. h2 ranges
are 0.0064–0.0772 and 0.0059–0.0324, well inside the current +/-0.4 input box.
The 0.8 uNm yaw-request clamp is also inactive during the early lag. Changing
yaw allocation weights or increasing its limits will not resolve that phase.
Map/request agreement verifies allocation; it does NOT verify delivered torque.

The original static sweep provides an independent check. At h2=+0.1 and zero
pitch/roll inputs:

| Voltage | Measured yaw torque | Map yaw torque |
|---|---:|---:|
| 130 V | +0.5118 uNm | +0.4578 uNm |
| 150 V | +0.5084 uNm | +0.4490 uNm |

The negative-h2 points also have the correct negative sign. Near the flight
pitch offset of +0.05 with h2=udiff=0, the map's bias discrepancy is only about
0.0036–0.0052 uNm at 130–150 V. Thus the controller comments describing an old
0.03–0.08 uNm bias and a 12.5 uNm/h2 slope do not characterize this newer map.
These are calibration checks, not independent held-out flight validation.

## Interpretation and next change

The controller explicitly assumes no aerodynamic yaw damping. Its rate-reference
term supplies only Kd*r_ref = 0.01131 uNm at the commanded spin rate. It has no
identified yaw-drag compensation or desired-angular-acceleration torque term.
The map is input-only and the sweep configuration welds the airframe: it cannot
represent torque changes caused by body angular velocity. The slow yaw response
is consistent with missing rate-dependent loading/effective flight dynamics,
rather than the modest static fit error alone.

Disabling Ki explains why spin 4 continues to accumulate heading lag. Restoring
Ki=0.3 is not a complete fix: spin 3 builds torque, exceeds the desired rate and
encounters roll saturation earlier. Roll split saturation precedes the large
tilt excursion in both runs; exact causation of the roll instability requires
a separate dynamics analysis.

Recommended sequence:

1. Identify yaw rate response and roll coupling using small positive and
   negative yaw commands around the flight trim, while holding position and
   monitoring roll headroom. Log the actual aero wrench as well as body rates.
2. Use that identified model to add yaw-rate load compensation (and acceleration
   feedforward if warranted), either in the wrench bias or the yaw controller,
   with consistent signs/units and without double counting.
3. Retune yaw Kp/Kd and use a limited integral term for residual trim. Do not
   blindly multiply gains or raise Ki while roll is saturating.
4. Recheck the complete turn with roll input bounds and rate limits visible.

No controller, model, or plant tuning was changed for this diagnosis.

## Reproduce

In MATLAB, add this directory to the path and call `analyze_yaw_spin34`.
It loads the two existing Dataset files and the calibration artifacts, prints
metrics, and regenerates `yaw_spin34_diagnosis.png` and
`yaw_spin34_metrics.mat`. It does not simulate or modify the models.
