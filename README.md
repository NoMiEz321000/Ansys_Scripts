# Parametric APDL-Based Horizontal Tailplane Structural Modelling and Verification under EASA CS-25 Flight Loads

[![Course](https://img.shields.io/badge/Course-15016606%20Aircraft%20Structure-blue.svg)](https://iaai.kmitl.ac.th/)
[![Institution](https://img.shields.io/badge/Institution-IAAI%20KMITL-orange.svg)](https://iaai.kmitl.ac.th/)
[![Standard](https://img.shields.io/badge/Airworthiness-EASA%20CS--25-green.svg)](https://www.easa.europa.eu/)
[![Solver](https://img.shields.io/badge/FEA-ANSYS%20Mechanical%20APDL%202026%20R1-red.svg)](https://www.ansys.com/)
[![License](https://img.shields.io/badge/License-Academic%20Student%20Edition-lightgrey.svg)](https://www.ansys.com/academic/free-student-products)

---

## 📌 Project Overview

This repository contains the complete computational source code, verification macros, aerodynamic pre-processing pipelines, literature resources, and the final laboratory report for the **Aircraft Structure Mini-Project** (`15016606`), Semester 1, Academic Year 2026, Aeronautical Engineering Program, International Academy of Aviation Industry (IAAI), King Mongkut's Institute of Technology Ladkrabang (KMITL).

* **Course Instructor:** Assoc. Prof. Dr. Eng. Suwin Sleesongsom
* **Project Team:**
  * Archawin Chindarattanaporn (ID: 67130043)
  * Aryan Sajith (ID: 67130044)
  * Thanatchai Khunpiluke (ID: 67130036)
  * Sirasak Permsup (ID: 67130029)
  * Peeradol Singsamran (ID: 67130061)
* **Final Report Document:** [`reference/Stage_Docment/HTP_Final_Report_v7.pdf`](reference/Stage_Docment/HTP_Final_Report_v7.pdf)

---

## 🎯 Project Scope & Objectives

In conventional airframe design, every change of internal structural layout requires rebuilding, repairing, and remeshing CAD geometry. Following the foundational parametric workflow of **Palumbo et al. (2026)** (*"A Parametric APDL-Based Workflow for Efficient Structural Design of a Civil Aircraft Horizontal Tailplane"*, *Eng. Proc.* 2026, 133, 15), this project develops a fully script-driven workflow that builds, meshes, loads, and verifies multiple internal layouts of a horizontal tailplane (HTP) in **ANSYS Mechanical APDL (MAPDL)** without manual GUI interaction.

The project scope is focused on three certified core objectives:

### 🗺️ System Architecture & Workflow Diagram

```mermaid
flowchart TD

subgraph group_model["Parametric Modeling"]
  node_parametric["HTP Model Builder<br/>[HTP_PARAMETRIC.mac]"]
  node_configurations["HTP Configurations<br/>[HTP_PARAMETRIC.mac]"]
end

subgraph group_loads["Flight Load Processing"]
  node_load_analysis["Aerodynamic Load Analysis"]
  node_nodal_mapping["Nodal Force Mapping<br/>[htp_nodal_forces.m]"]
  node_node_export["Load Node Export"]
  node_load_cards["Mapped Load Inputs"]
end

subgraph group_verification["Model Verification"]
  node_verification["Acceptance Gates<br/>[VERIFY_MODAL.mac]"]
  node_modal_runs["Modal Verification"]
end

subgraph group_analysis["Advanced Analysis"]
  node_static_extension["Static and Buckling"]
  node_sensitivity["Sensitivity and Sizing"]
  node_optimization_results["Sizing Results"]
end

subgraph group_visual["Visual Inspection"]
  node_load_visuals["Load Visualization"]
  node_rib_visuals["Rib Inspection<br/>[RENDER_RIBS.mac]"]
  node_visual_check["Batch Visual Checks<br/>[VERIFY_VISUAL.mac]"]
end

node_engineer(("Engineer"))
node_ansys["ANSYS MAPDL"]
node_matlab["MATLAB"]

node_engineer -->|"runs"| node_parametric
node_parametric -->|"generates"| node_configurations
node_parametric -->|"builds model in"| node_ansys
node_load_analysis -->|"runs in"| node_matlab
node_load_analysis -->|"preprocesses loads for"| node_nodal_mapping
node_node_export -->|"exports nodes for"| node_nodal_mapping
node_nodal_mapping -->|"produces mapped forces"| node_load_cards
node_configurations -->|"undergoes"| node_verification
node_verification -->|"checks modes with"| node_modal_runs
node_configurations -->|"supports downstream analysis"| node_static_extension
node_static_extension -.->|"informs"| node_sensitivity
node_sensitivity -.->|"records results"| node_optimization_results
node_engineer -->|"runs"| node_load_visuals
node_engineer -->|"runs"| node_rib_visuals
node_engineer -->|"runs"| node_visual_check
node_load_visuals -->|"renders in"| node_ansys
node_rib_visuals -->|"inspects in"| node_ansys
node_visual_check -->|"verifies in"| node_ansys

classDef toneBlue fill:#dbeafe,stroke:#2563eb,stroke-width:1.5px,color:#172554
classDef toneAmber fill:#fef3c7,stroke:#d97706,stroke-width:1.5px,color:#78350f
classDef toneMint fill:#dcfce7,stroke:#16a34a,stroke-width:1.5px,color:#14532d
classDef toneRose fill:#ffe4e6,stroke:#e11d48,stroke-width:1.5px,color:#881337
classDef toneIndigo fill:#e0e7ff,stroke:#4f46e5,stroke-width:1.5px,color:#312e81
class node_parametric,node_configurations toneBlue
class node_load_analysis,node_nodal_mapping,node_node_export,node_load_cards toneAmber
class node_verification,node_modal_runs toneMint
class node_static_extension,node_sensitivity,node_optimization_results toneRose
class node_load_visuals,node_rib_visuals,node_visual_check,node_engineer,node_ansys,node_matlab toneIndigo
```
*(Vector architecture diagram rendered natively by GitHub. Raw SVG vector is also available at [`assets/architecture_diagram.svg`](assets/architecture_diagram.svg)).*

1. **Objective 1 (Parametric APDL Model):** Develop a unified APDL macro (`HTP_PARAMETRIC.mac`) that generates three complete internal airframe topologies from parameters alone:
   * **Configuration A (Reference Baseline):** 14 ribs, 3 spars, 8 stringers (2-cell box)
   * **Configuration B:** 9 ribs, 2 spars, 8 stringers (1-cell box)
   * **Configuration C:** 7 ribs, 2 spars, 6 stringers (1-cell box)
2. **Objective 2 (Flight Load Reconstruction & Mapping):** Reconstruct aerodynamic flight loads for the ATR 42-600 under EASA CS-25, identify the governing dive-speed download case $D_{fwd}$ ($V_D = 312.5\text{ kt EAS}$, half-tail limit load $P = -9,142.84\text{ N}$, root moment $M_{\text{root}} = -16,058\text{ N}\cdot\text{m}$), and map discrete nodal forces to conformal skin shells using MATLAB with root equilibrium error **$< 0.001\%$**.
3. **Objective 3 (Model Verification):** Enforce three rigorous acceptance gates (Free-Free Rigid-Body Modes, Mass Conservation, and 1g Gravity Equilibrium) to verify complete nodal connectivity, element formulation, and physical integrity before conducting downstream stress assessments.

*(Note: Advanced post-verification extensions including linear static stress, linear eigenvalue buckling, and sensitivity-guided sizing optimization are archived in `optimization/` and outlined as Future Work in Section 6.4 of the final report).*

---

## 📊 Summary of Certified Verification Results

### Acceptance Gate Performance Matrix (All 3 Configurations)

| Gate | Verification Check | Acceptance Criterion | Configuration A<br>`[14-3-8]` | Configuration B<br>`[9-2-8]` | Configuration C<br>`[7-2-6]` | Status |
| :---: | :--- | :--- | :---: | :---: | :---: | :---: |
| **Gate 1** | **Free-Free Rigid-Body Modes** | Exactly 6 zero-energy modes ($f_{1..6} \le 10^{-3}\text{ Hz}$) | $\max f = 2.46 \times 10^{-4}\text{ Hz}$ | $\max f = 1.97 \times 10^{-4}\text{ Hz}$ | $\max f = 2.41 \times 10^{-4}\text{ Hz}$ | **PASS** |
| | **Elastic Spectral Separation** | First elastic mode jumps cleanly ($f_7 > 1\text{ Hz}$, ratio $> 10^4$) | $f_7 = 55.99\text{ Hz}$<br>(ratio: $2.28 \times 10^5$) | $f_7 = 52.34\text{ Hz}$<br>(ratio: $2.66 \times 10^5$) | $f_7 = 45.95\text{ Hz}$<br>(ratio: $1.90 \times 10^5$) | **PASS** |
| **Gate 2** | **Mass Conservation** | Volume integral (`ETABLE`) equals solver mass matrix (`MTOT`) | $140.463\text{ kg}$<br>($\Delta = 0.0000\%$) | $121.890\text{ kg}$<br>($\Delta = 0.0000\%$) | $113.638\text{ kg}$<br>($\Delta = 0.0000\%$) | **PASS** |
| **Gate 3** | **1g Gravity Equilibrium** | Root reaction matches total weight within $0.001\%$ in X, Y, Z | Error $< 10^{-6}\%$<br>(Strength Ratio: $242.3$) | Error $< 10^{-6}\%$<br>(Strength Ratio: $274.3$) | Error $< 10^{-6}\%$<br>(Strength Ratio: $297.4$) | **PASS** |
| **Flight Load** | **Aerodynamic Equilibrium** | Root reaction matches applied half-tail load ($-9,142.84\text{ N}$) | $+9,142.84\text{ N}$<br>(Error $< 0.001\%$) | $+9,142.84\text{ N}$<br>(Error $< 0.001\%$) | $+9,142.82\text{ N}$<br>(Error $< 0.001\%$) | **PASS** |

### Key Physics & Modeling Highlights
* **Watertight Shared-Node Topology:** Thin-walled skins and webs are modeled using `SHELL181` (full integration), while spar caps, stringers, and rib flanges utilize `BEAM188` (Timoshenko beams). All components share geometric lines followed by `NUMMRG, NODE, 0.01`, eliminating artificial constraint equations or penalty contact elements.
* **Dual-Zone Flange Normal Blending:** Rib flanges (`SEC 6`, $40\times 1.0\text{ mm}$) follow the $7^\circ$ wing dihedral across the wingbox but blend smoothly to pure in-plane orientation ahead of $x/C = 0.025$, eliminating both the nose "beak" overhang and outer-edge skin puncture.
* **Stress Concentration Mitigation:** Stringer pass-through cutouts on ribs feature 8-keypoint circular tangent fillets ($R=10\text{ mm}$), lowering $K_t$ from $>2.4$ down to $\approx 1.4$. Dynamic pillar capping ($R_{hole} \le D_{adj} - 14\text{ mm}$) maintains safe web ligaments.
* **44% Internal-Node Load Resolution:** Selecting load nodes by raw spatial coordinates accidentally excited internal integration nodes of cubic `BEAM188` elements, causing $44\%$ of aerodynamic forces to vanish. Restricting node selection strictly to corner nodes of composite skin shells (`ESEL, S, MAT,, 2` + `NSLE`) restored $100.000\%$ force transmission.

---

## 📂 Repository Structure

```text
Ansys_Scripts/
│
├── HTP_PARAMETRIC.mac              # Master 10-Block parametric ANSYS APDL script (v1.4.5-stable)
├── htp_loads_D_fwd_cfg1.inp        # Governing limit flight load card (Config A 14-3-8)
├── htp_loads_D_fwd_cfg2.inp        # Governing limit flight load card (Config B 9-2-8)
├── htp_loads_D_fwd_cfg3.inp        # Governing limit flight load card (Config C 7-2-6)
│
├── assets/                         # Vector & raster system architecture diagrams for documentation
├── archive/                        # Auxiliary APDL macros (VERIFY_VISUAL, RENDER_RIBS, EXPORT_LOAD_NODES)
├── load_sfbm/                      # Stage 2b: Aerodynamic flight loads & SFBM pre-processing suite
│   ├── README.md                   # Complete mathematical derivations for V-n, BTL & Schrenk
│   ├── matlab/                     # MATLAB pipeline scripts (vn_diagram.m, tail_load.m, etc.)
│   ├── loads/                      # APDL input force cards (htp_loads_D_fwd_cfg1..3.inp)
│   ├── results/                    # Reaction audits, mass breakdown CSVs, and load summaries
│   └── visual_load/                # High-res contour plots and boundary condition diagrams
│
├── modal_analysis/                 # Stage 3b: Model verification & Free-Free modal suite
│   ├── README.md                   # Verification methodologies, acceptance gates & APDL commands
│   ├── note.md                     # Technical physics reference & Q&A defense cheat-sheet
│   ├── HTP_BUILD_CORE.mac          # Core geometry builder callable by batch drivers
│   ├── VERIFY_MODAL.mac            # Free-free modal and 1g gravity verification driver
│   ├── result/                     # Verification CSV tables and gate pass/fail logs
│   └── visual_modal/               # Mode shape 1–6 displacement contour plots
│
├── optimization/                   # Advanced Extensions (Future Work: Stage 3c & 3d sizing optimization)
│   ├── README.md                   # Zero-remeshing sizing optimization pipeline documentation
│   ├── BUILD_OPT_BASES.mac         # Frozen-mesh database builder for rapid evaluations (2.5-4.2s)
│   ├── UPDATE_SECTIONS_AND_SOLVE.mac # Fast section updater with recomputed beam offsets
│   ├── result/                     # Sizing histories, sensitivity matrices, and trade-off tables
│   └── visual_opt/                 # Buckling modes and post-sizing deflection plots
│
├── visual_log/                     # Multi-view visual verification logs, galleries, and snapshot suites
│
└── reference/                      # Official documentation, syllabus, and academic references
    ├── Stage_Docment/              # Milestone project reports and presentations
    │   ├── HTP_Final_Report_v7.pdf # ★ FINAL CERTIFIED REPORT (October 21, 2026) ★
    │   └── 00_Stage_Documents_Index.md # Index of stage documents
    │
    └── Resources/                  # Full-text textbooks and regulatory certification standards
        ├── [Reference] A simple approximation method for obtaining the spanwise lift.pdf
        ├── [Reference] ATR 42-600 Aircraft Characteristics Airport and Maintenance Planning.pdf
        ├── [Reference] Aircraft Design A Conceptual Approach.pdf
        ├── [Reference] Aircraft_Structures_for_Engineering_Students_6th_Edition.pdf
        ├── [Reference] Airframe_Structural_Design_by_Michael_Ch.pdf
        ├── [Reference] Dynamics of Structures.pdf
        ├── [Reference] Finite Element Procedures in Engineering Analysis Bathe K.J.pdf
        ├── [Reference] Metallic Materials Properties Development and.pdf
        └── [Refence Literature] A Parametric APDL-Based Workflow for Efficient Structural Design of...pdf
```

---

## 📖 Key Literature & Academic References

The modeling parameters, airworthiness regulations, and analytical methods implemented in this repository are based on the following textbooks and standards stored in [`reference/Resources/`](reference/Resources/):

1. **[1] Palumbo, C. et al. (2026):** *"A Parametric APDL-Based Workflow for Efficient Structural Design of a Civil Aircraft Horizontal Tailplane"*, *Engineering Proceedings*, 133(15). (Foundational parametric geometry & verification methodology).
2. **[2] EASA CS-25 (Amendment 27, 2023):** *Certification Specifications and Acceptable Means of Compliance for Large Aeroplanes*, European Union Aviation Safety Agency. (CS-25.333, CS-25.335, CS-25.337, CS-25.341).
3. **[3] ATR Aircraft (2020):** *ATR 42-600 Aircraft Characteristics: Airport and Maintenance Planning*, Blagnac, France. (Geometry, MTOW, MZFW, CG limits, operational speeds).
4. **[4] Jacobs, E. N., Ward, K. E., and Pinkerton, R. M. (1933):** *The Characteristics of 78 Related Airfoil Sections from Tests in the Variable-Density Wind Tunnel*, NACA Report No. 460.
5. **[5] ASM International (1990):** *ASM Handbook, Vol. 2: Properties and Selection: Nonferrous Alloys and Special-Purpose Materials*. (Aluminium alloy 7075-T6 mechanical properties).
6. **[6] Lundsgaard-Larsen, C. et al. (2012):** Quasi-isotropic carbon/epoxy laminate properties for civil aircraft structures.
7. **[7] Schrenk, O. (1940):** *A Simple Approximation Method for Obtaining the Spanwise Lift Distribution*, NACA Technical Memorandum No. 948.
8. **[8] Raymer, D. P. (2018):** *Aircraft Design: A Conceptual Approach*, 6th Edition, AIAA Education Series. (Longitudinal trim and flight envelopes).
9. **[9] Bruhn, E. F. (1973):** *Analysis and Design of Flight Vehicle Structures*, Jacobs Publishing.
10. **[10] Megson, T. H. G. (2017):** *Aircraft Structures for Engineering Students*, 6th Edition, Butterworth-Heinemann. (Semi-monocoque airframes and cutouts).
11. **[11] Niu, M. C. Y. (1999):** *Airframe Structural Design*, 2nd Edition, Conmilit Press.
12. **[12] Bathe, K. J. (2006):** *Finite Element Procedures*, Prentice Hall. (Block Lanczos eigenvalue extraction and stiffness null space).
13. **[13] Clough, R. W. and Penzien, J. (2003):** *Dynamics of Structures*, 3rd Edition, Computers & Structures. (Rigid-body mechanics and modal orthogonality).
14. **[14] Vanderplaats, G. N. (2007):** *Multidiscipline Design Optimization*, Vanderplaats R&D.
15. **[15] FAA / MMPDS-01 (2003):** *Metallic Materials Properties Development and Standardization*. (Aluminium design allowables).
16. **[16] SAE International / CMH-17 (2012):** *Composite Materials Handbook CMH-17, Vol. 3: Polymer Matrix Composites*.

---

## 💻 How to Run the Models

### Prerequisites
* **ANSYS Mechanical APDL (MAPDL):** Release 2026 R1 (or compatible release with SHELL181 / BEAM188).
* **MATLAB:** R2022b or later (Optimization Toolbox for `lsqminnorm`).
* **Python:** 3.10+ (for automated batch logging and result post-processing).

### 1. Build and Solve in Batch Mode (Recommended)
To execute the master parametric macro in headless batch mode:
```powershell
# Run Configuration A (Default CONFIG = 1)
& "C:\Program Files\ANSYS Inc\v261\ansys\bin\winx64\ansys261.exe" -b -i "HTP_PARAMETRIC.mac" -o "run_cfgA.out"
```

To switch between configurations, set the `CONFIG` parameter at the top of Block 1 in `HTP_PARAMETRIC.mac`:
* `CONFIG = 1`: Configuration A (14 ribs, 3 spars, 8 stringers)
* `CONFIG = 2`: Configuration B (9 ribs, 2 spars, 8 stringers)
* `CONFIG = 3`: Configuration C (7 ribs, 2 spars, 6 stringers)

### 2. Run Verification Suite (Free-Free Modal & Gravity Audit)
Execute the batch verification driver inside `modal_analysis/`:
```powershell
cd modal_analysis
& "C:\Program Files\ANSYS Inc\v261\ansys\bin\winx64\ansys261.exe" -b -i "VERIFY_MODAL.mac" -o "verify_run.out"
```
The results and pass/fail verdicts are automatically exported to `modal_analysis/result/STAGE3B_VERIFICATION_SUMMARY.txt`.

### 3. Re-generate Aerodynamic Nodal Forces (MATLAB)
If aircraft geometry or load conditions are updated, recalculate and re-map the flight loads:
```matlab
cd load_sfbm/matlab
run('vn_diagram.m');       % Generate V-n diagram and boundary points
run('tail_load.m');        % Evaluate pitch trim equilibrium across CG envelope
run('htp_sfbm.m');         % Calculate Schrenk spanwise load and SFBM curves
run('htp_nodal_forces.m');  % Map tributary loads to skin nodes and write .inp files
```

---

## 🏛️ Academic Integrity & Acknowledgements

This laboratory report and codebase were developed for academic evaluation under the supervision of **Assoc. Prof. Dr. Eng. Suwin Sleesongsom**. The authors extend their sincere gratitude for his guidance, technical suggestions, and valuable insights on airframe structural modeling.

*King Mongkut's Institute of Technology Ladkrabang (KMITL) — October 2026.*
