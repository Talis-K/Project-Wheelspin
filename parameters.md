# Parameter register — EMRAX 208 & simple traction control

Companion to [`parameters.m`](parameters.m). That file holds numeric values and short line comments; this document explains **what each symbol is**, **where the value came from**, and **how it is used** in a simple FSAE traction-control (TC) model.

**Project:** P304 — Traction Control System for an FSAE Car (UTS Motorsports)

**Configuration switches in `.m`:** set `winding_type` (`HV` / `MV` / `LV`), `cooling_type` (`air` / `liquid` / `combined`), and `gear_ratio_i` before relying on selected `K_T`, currents, `T_cont`, and reflected inertia.

**Convention:** `NaN` in code = unknown / to be measured / TBC. Corner order is always **FL, FR, RL, RR**.

---

## Sources

| ID | Reference |
|----|-----------|
| [1] | EMRAX. *EMRAX 208 technical data table* (v1.7). https://emrax.com/wp-content/uploads/2026/08/EMRAX_208_datasheet_v1.7.pdf |
| [2] | SAE International. *Formula SAE Rules 2026* (v1.0). Accumulator DC power limit. |
| Est. | Engineering estimate (not datasheet) |
| Calc. | Derived from other parameters in this register |
| TBD | Not yet known — measure, choose hardware, or tune |

Overview context from the team motor brief: EMRAX 208 is a compact axial-flux PMSM; peak ≈ 150 Nm, mass ≈ 9.4–10.3 kg, max 7000 rpm, position via resolver/encoder, torque lag estimated ≈ 2–5 ms, FSAE 80 kW DC accumulator limit.

---

## 1. Actuator — torque and power production

| Parameter | Symbol | Value | Source | Role / how obtained |
|-----------|--------|-------|--------|---------------------|
| Peak torque | \(T_{pk}\) | 150 Nm | [1] | Hard upper bound on torque command |
| Peak power | \(P_{pk}\) | 86 kW | [1] (overview) | Power envelope ceiling (shaft) |
| Continuous torque | \(T_{cont}\) | 54 / 84 / 90 Nm (air / liquid / combined) | [1]; cooling TBC | Long-run torque after thermal derate settles |
| Torque constant | \(K_T\) | 0.62 / 0.38 / 0.15 Nm/A\(_\mathrm{RMS}\) (HV / MV / LV) | [1]; winding TBC | \(T \approx K_T\,I_\mathrm{RMS}\); also inverse for current from torque |
| Torque-constant drift | \(\partial K_T/\partial T\) | Unknown (`dKT_dT`) | TBD | Scale \(K_T\) with magnet/winding temperature so hot torque is not over-reported |
| Torque response time | \(\tau_m\) | ≈ 2–5 ms (code uses 3.5 ms mid) | Est. | First-order lag inverter command → shaft torque |
| Current limits | \(I_{pk}\), \(I_{cont}\) | Peak 240 / 400 / 1000 A; cont. 140 / 220 / 560 A (HV / MV / LV) | [1]; inverter TBC | Saturate at **min**(motor, inverter) |
| Reverse / regen | — | Full negative torque available | [1] | Allows regen braking torque requests |
| Limiting speed | \(\omega_{max}\) | 7000 rpm → rad/s in `.m` | [1] | Top of speed range / torque map |
| Regulated power limit | \(P_{lim}\) | 80 kW at accumulator outlet (\(V_{DC}\times I_{DC}\)) | [2] | Limits **DC** power, not shaft power |
| Efficiency | \(\eta(\omega,T)\) | Peak 96 % near efficiency island; map TBD | [1] | \(P_{DC} \approx (T\,\omega)/\eta\) for the power limiter |
| Continuous power (cooling) | \(P_{cont}\) | 33 / 52 / 56 kW (air / liquid / combined) | [1] | Continuous shaft power where derate settles |

**Selected winding bundle (sets \(K_T\) and currents):**

| Variant | Nominal voltage | Peak current | \(K_T\) |
|---------|-----------------|--------------|---------|
| HV | 690 V | 240 A | 0.62 Nm/A\(_\mathrm{RMS}\) |
| MV | 420 V | 400 A | 0.38 Nm/A\(_\mathrm{RMS}\) |
| LV | 170 V | 1000 A | 0.15 Nm/A\(_\mathrm{RMS}\) |

**.m helpers:** `T_from_I_pk = K_T*I_pk`, `T_from_I_cont = K_T*I_cont` — compare with \(T_{pk}\) / \(T_{cont}\) and take the lower bound in a limiter.

---

## 2. Electrical — pack / inverter

| Parameter | Symbol | Value | Source | Role / how obtained |
|-----------|--------|-------|--------|---------------------|
| Pack OCV | \(V_{OC}\) | ______ V | TBD (measure) | Bus voltage without load; sag model input |
| Pack internal resistance | \(R_{int}\) | ______ Ω | TBD (measure) | Under load \(V_{DC} \approx V_{OC} - I_{DC} R_{int}\); early torque limit if voltage collapses |
| Inverter peak / cont. current | \(I_{inv,pk}\), \(I_{inv,cont}\) | ______ A | TBD | Effective limit = min with motor \(I_{pk}\), \(I_{cont}\) |
| Motor nominal bus (variant) | \(V_{mot}\) | 690 / 420 / 170 V | [1] | Context for winding choice; not the same as measured pack voltage |

---

## 3. Mechanical, temperature & motor sensing

| Parameter | Symbol | Value | Source | Role / how obtained |
|-----------|--------|-------|--------|---------------------|
| Rotor inertia | \(J_r\) | 0.01569 kg·m² | [1] | Motor-side inertia |
| Reflected rotor inertia | \(J_{r,\mathrm{refl}}\) | 0.141 / 0.192 / 0.251 / 0.318 kg·m² at \(i = 3 / 3.5 / 4 / 4.5\) | [1] + Calc. \(J_r i^2\) | Add to axle inertia; appears in \(J\dot{\omega}\) |
| Gear ratio | \(i\) (`gear_ratio_i`, `i_g`) | default 3.5 in `.m` | TBD / calc. table | Motor speed / wheel speed (define sign & chaining) |
| Motor mass | \(m_m\) | 9.4–10.3 kg (code mid 9.85) | [1] | Part of vehicle mass / CG → normal loads |
| Motor speed | \(\omega_m\) | resolver/encoder | [1]; fitted type TBD | Rotor speed — **not** wheel speed |
| Winding / rotor temp limits | \(T_{lim}\) | 100 °C winding sensor / 100 °C rotor surface | [1] | Start of thermal derate |
| Winding temperature | \(T_{wind}\) | runtime | TBD sensor | Input to \(\partial K_T/\partial T\) and derate |
| Cooling choice | — | air / liquid / combined | [1]; TBC | Selects \(T_{cont}\), \(P_{cont}\) |

---

## 4. Driveline (placeholders)

| Parameter | Symbol | Value | Source | Role |
|-----------|--------|-------|--------|------|
| Gearbox efficiency | \(\eta_g\) | TBD | TBD | Shaft power / motor power |
| Gearbox / driveline inertia | \(J_g\) | TBD | TBD | Rotating inertia referred to motor or axle (state clearly) |
| Shaft / CV torque limit | \(T_{shaft,max}\) | TBD | TBD | Hardware clip below motor peak if required |

---

## 5. Vehicle body (placeholders)

| Parameter | Symbol | Value | Source | Role |
|-----------|--------|-------|--------|------|
| Vehicle mass | \(m_v\) | TBD | Weigh / BOM | Longitudinal dynamics \(m_v \dot{v}_x = \sum F_x - F_{drag}-F_{rr}\) |
| Wheelbase | \(L\) | TBD | Geometry | \(L = a + b\) |
| CG → front / rear | \(a\), \(b\) | TBD | Measure / CAD | Static weight split |
| CG height | \(h_{cg}\) | TBD | Measure / CAD | Longitudinal load transfer |
| Gravity | \(g\) | 9.81 m/s² | Standard | Normal loads |
| Drag area | \(C_D A\) | TBD | Aero / coast-down | \(F_{drag} = \tfrac12 \rho C_D A v_x^2\) |
| Rolling resistance | \(C_{rr}\) | TBD | Tyre / coast-down | \(F_{rr} \approx C_{rr} m_v g\) |
| Air density | \(\rho_{air}\) | 1.225 kg/m³ | Approx. | Drag |
| Vehicle speed | \(v_x\) | runtime | GPS/encoder fusion TBD | Reference for slip |

---

## 6. Wheels & tyres — four corners (placeholders)

| Parameter | Symbol | Value | Source | Role |
|-----------|--------|-------|--------|------|
| Effective radius | \(R_e\) (and per-corner) | TBD | Measure / tyre data | \(u = \omega_w R_e\) |
| Wheel inertia | \(J_w\) | TBD | Measure / CAD | Wheel spin dynamics |
| Wheel angular speeds | \(\omega_{w,FL}\ldots\omega_{w,RR}\) | runtime | Wheel speed sensors | **Primary TC inputs** |
| Linear wheel speeds | \(u_{FL}\ldots u_{RR}\) | Calc. | \(u = \omega_w R_e\) | Slip numerator |
| Slip ratio | \(\kappa_{FL}\ldots\kappa_{RR}\) | Calc. | \(\kappa = (u - v_x)/\max(|v_x|, v_\varepsilon)\) | TC error signal |
| Standstill guard | \(v_\varepsilon\) | 0.5 m/s (default) | Est. | Avoid \(\kappa\) blow-up at \(v_x\approx 0\) |
| Normal loads | \(F_{z,FL}\ldots F_{z,RR}\) | TBD / Calc. | Static + transfer | Scales available friction force |
| Long. tyre forces | \(F_{x,\ldots}\) | TBD / model | Magic formula or simple \(\mu F_z\) | Plant / estimator |
| Peak long. µ | \(\mu_{x,max}\) | TBD | Track / tyre | \(\lvert F_x\rvert \le \mu F_z\) |
| Optimal slip | \(\kappa_{opt}\) | TBD | Tyre curve / test | Natural TC setpoint |
| Magic-formula coeffs | \(B,C,D,E\) | TBD | Fit / supplier | If using Pacejka-style tyre model |

**Minimal slip definition used in `.m` comments:**

\[
\kappa = \frac{u - v_x}{\max(|v_x|,\, v_\varepsilon)}, \qquad u = \omega_w R_e
\]

Driven-axle slips (e.g. RL & RR for RWD) are what a simple TC usually regulates.

---

## 7. Traction control — signals & gains (placeholders)

| Parameter | Symbol | Value | Source | Role |
|-----------|--------|-------|--------|------|
| Driver / VCU torque request | \(T_{cmd}\) | runtime | Pedal map | Pre-TC demand |
| Post-TC torque | \(T_{tc}\) | runtime | Controller output | Command to inverter / torque map |
| Regen ceiling | \(T_{regen,max}\) | TBD | Thermal / rules | Negative torque bound |
| Target slip | \(\kappa_{target}\) | TBD (often \(\approx\kappa_{opt}\)) | Tune | Setpoint for slip controller |
| TC gains | \(K_{p,tc}\), \(K_{i,tc}\) | TBD | Tune | PI (or similar) on slip / spin error |
| Sample time | \(T_s\) | TBD | ECU / Simulink fixed step | Discrete controller |
| Driven axle | `driven_axle` | default `"RWD"` | Vehicle TBC | Which corners TC acts on |

**Typical simple loop (sketch):** measure four \(\omega_w\) → form \(\kappa\) on driven wheels → compare to \(\kappa_{target}\) → reduce \(T_{cmd}\) to \(T_{tc}\) → enforce \(T \le T_{pk}\), current, thermal, and \(P_{DC}\le P_{lim}\).

---

## How values get into the model

1. Edit configuration switches at the top of `parameters.m`.
2. Replace `NaN` placeholders as hardware and vehicle data land.
3. Run `parameters.m` (or point the Simulink model **InitFcn** at it) so symbols exist in the base workspace.
4. Prefer keeping **numbers only in `.m`** and **definitions / sources in this `.md`** so the register stays auditable for P304.

---

## Quick status legend

| In `.m` | Meaning |
|---------|---------|
| Concrete number | Taken from [1], [2], calc., or stated estimate |
| `NaN` | Placeholder — fill before claiming closed-loop accuracy |
| `switch` on winding/cooling | Datasheet multi-variant; one choice active at a time |
