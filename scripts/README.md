# Scripts Folder

This directory contains utility scripts for preprocessing and unit normalization of aerospace simulation data.

The primary purpose of these scripts is to convert mixed Imperial-unit simulation exports into strict SI units for use in flight-dynamics, control and  validation workflows.

---

# convert_to_si.py

## Purpose

`convert_to_si.py` converts the file:

    Atmos_01_sim_01.csv

from Imperial / mixed aerospace units to fully consistent SI units.

The converted file is saved automatically as:

    Atmos_01_sim_01_si_units.csv

The script is tailored specifically to the structure of `Atmos_01_sim_01.csv`.

---

## Units Converted

The following conversions are performed:

### Length & Position
- ft → m

### Velocity
- ft/s → m/s
- ft/min → m/s
- nmi/h (knots) → m/s

### Acceleration
- ft/s² → m/s²

### Density
- slug/ft³ → kg/m³

### Pressure
- lbf/ft² → Pa

### Forces
- lbf → N

### Moments
- ft·lbf → N·m

### Temperature
- °Rankine → K

### Angles
- deg → rad
- deg/s → rad/s

All converted columns are renamed to reflect their SI units.

---

## Requirements

See `requirements.txt` at the root folder.

Minimal dependencies:

    pandas
    numpy

Install using pip:

    pip install -r requirements.txt

Or with conda:

    conda install pandas numpy

---

## Usage

From inside the `scripts/` directory:

    python convert_to_si.py

The output file will be generated automatically:

    Atmos_01_sim_01_si_units.csv

---

## Notes

- The script assumes the input CSV file is located in the same directory.
- No automatic unit detection is performed; conversions are explicitly defined.
- Designed for reproducible aerospace simulation workflows.
- Intended for SI-consistent flight dynamics and atmospheric modeling pipelines.

---

## Future Extensions (Optional)

Possible improvements include:

- CLI argument support (input/output file specification)
- Automated dimensional validation checks
- ISA atmosphere cross-validation
- Unit tests for regression safety

---

End of file.

---

# iekf_convention_mc.py

## Purpose

Monte Carlo prototype comparing the left- and right-invariant EKF on
SE2(3) x R^7 for the open error-convention decision of the sensor fusion work
(`TODO-sensor-fusion.md`, `doc/sensor_fusion.rst`). One filter with the
convention as a switch; both conventions see the same truth, sensor samples
and initial error (paired runs). NumPy only; matplotlib optional for the plot.

Model: launch-centred inertial frame, point-mass gravity with gradient, MEMS
IMU with random-walk biases, GNSS position and velocity, barometric height
with random-walk bias. Scenarios: nominal, large and huge initial heading
error, GNSS outage, far from the origin with weak yaw (float64 and float32
covariance), nominal in float32.

## Usage

    python scripts/iekf_convention_mc.py                       # all scenarios, 20 runs
    python scripts/iekf_convention_mc.py --runs 50 --scenarios large_heading outage
    python scripts/iekf_convention_mc.py --out results/iekf_mc --no-plots

Prints a table per scenario and convention (ANEES, final yaw and position
RMS, heading convergence time, failed runs) and writes
`iekf_convention_mc.png` (yaw RMS and ANEES against time) to `--out`.
