# Load SFBM Module (Stage 2b)

## 📌 Overview
This folder contains the MATLAB pre-processing scripts for **Stage 2b: SFBM Aerodynamic Load Calculation & Nodal Force Mapping**.

The methodology follows the 4-step sequential pipeline:
1. `vn_diagram.m` -> Generates `vn_load_cases.csv` (12 flight & gust envelope points).
2. `tail_load.m` -> Generates `tail_load_cases.csv` (24 balancing tail load cases for FWD & AFT CG).
3. `htp_sfbm.m` -> Generates `htp_sfbm.csv` (Schrenk spanwise load, shear $V(y)$, and bending moment $M(y)$).
4. `htp_nodal_forces.m` -> Reads `htp_sfbm.csv` and `htp_nodes.csv` (exported from ANSYS APDL `HTP_PARAMETRIC.mac`), then solves for external nodal forces via `lsqminnorm` and outputs `htp_loads_<case>.inp`.

---

## 🚀 Execution Instructions

### Step 1: Run Aerodynamic Pre-processing (MATLAB)
Open MATLAB in this directory and execute sequentially:
```matlab
run('vn_diagram.m');
run('tail_load.m');
run('htp_sfbm.m');
```

### Step 2: Export Active Node List from ANSYS APDL
Run `HTP_PARAMETRIC.mac` in ANSYS up to the node export block to generate `htp_nodes.csv`. Copy that file into this folder.

### Step 3: Map Nodal Loads (MATLAB)
Set `case_to_export = 'D_fwd';` (default) in `htp_nodal_forces.m` and run:
```matlab
run('htp_nodal_forces.m');
```
This generates `htp_loads_D_fwd.inp`.

### Step 4: Apply in ANSYS APDL
Copy `htp_loads_D_fwd.inp` to your ANSYS working directory and load it in Block 9:
```apdl
/INPUT, htp_loads_D_fwd, inp
```
