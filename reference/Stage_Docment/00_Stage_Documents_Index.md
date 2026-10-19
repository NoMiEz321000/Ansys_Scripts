# Parametric APDL-Based Horizontal Tailplane Structural Analysis and Optimization

## Master Stage Documentation Index & Project Synthesis

**Institution:** Department of Aerospace Engineering, School of Engineering, King Mongkut's Institute of Technology Ladkrabang (KMITL)  
**Course:** Aircraft Structure (Academic Year 3, Semester 1)  
**Aircraft Baseline:** ATR 42-600 Regional Turboprop / EASA CS-25 Reference  
**Author:** Structural Engineering Team (Antigravity Assistant Pair Programming)  
**Date:** October 2026  
**Project Workspace:** `Parametric APDL-Based Horizontal Tailplane Structural Analysis and Optimization\Ansys_Scripts`  

---

## 1. Project Overview & Executive Summary

This engineering documentation package provides a comprehensive, academically rigorous account of the design, aerodynamic load integration, finite element modeling, mathematical verification, baseline static/buckling analysis, and sizing optimization of an aircraft horizontal tailplane (HTP). Modeled on the ATR 42-600 twin-turboprop regional transport category aircraft and governed by European Aviation Safety Agency (EASA) CS-25 airworthiness specifications, the structural architecture is developed using a fully parametric, script-driven methodology within ANSYS Mechanical APDL (MAPDL).

Building upon the foundational parametric methodology proposed by Palumbo et al. (2026), the project investigates three distinct internal structural topologies (Configuration A: 14 Ribs, 3 Spars, 8 Stringers; Configuration B: 9 Ribs, 2 Spars, 8 Stringers; Configuration C: 7 Ribs, 2 Spars, 6 Stringers). Using a reconstructed aerodynamic flight limit load envelope at dive velocity ($V_D = 312.5\text{ kt EAS}$, $n_{\max} = +2.571$, limit downward shear $P = -9,142.84\text{ N}$), the structural responses are evaluated under five mathematical verification gates. Following baseline static stress decomposition and eigenvalue stability evaluation—which revealed an elastic skin panel buckling deficit in baseline Configuration C ($|\lambda_1| = 1.326$, $\text{MS}_{\text{buckle}} = -0.12$)—a zero-remeshing fast-path optimization pipeline was developed. Forward finite difference sensitivity analysis guided a multi-regime sizing procedure, resolving structural stability and establishing **Configuration C Regime 1A ($112.40\text{ kg}$)** as the optimal airframe design, achieving a verified **-19.98% mass reduction** relative to the 14-rib baseline while preserving full aeroelastic bending stiffness.

---

## 2. Master Stage Map & Document Index

The complete documentation suite comprises technical reports covering Stages 1 through 3d, accompanied by high-resolution visual figure archives in `figures/` and synthesis indices:

**Table 0.1a: Master project task breakdown, lifecycle milestones, and schedule**

| Task ID | Task Name | Category / Deliverable | Status | Start Date | End Date | Duration (Days) |
| :---: | :--- | :--- | :---: | :---: | :---: | :---: |
| **T-01** | Initial topic study (aircraft window structure) | Topic Selection & Literature | Completed | 2026-09-16 | 2026-09-22 | 7 |
| **T-02** | Topic change to horizontal tailplane; literature review | Topic Selection & Literature | Completed | 2026-09-23 | 2026-09-25 | 3 |
| **T-03** | Step 1: Data collection & parameter definition | Stage 1: Input Data | Completed | 2026-09-26 | 2026-09-27 | 2 |
| **T-04** | Step 2: Parametric APDL script development | Stage 2a: APDL Script | Completed | 2026-09-28 | 2026-10-15 | 18 |
| **T-05** | Proposal presentation for project approval | Major Milestone | Completed | 2026-09-30 | 2026-09-30 | 1 |
| **T-06** | Step 3: SFBM load pre-processing (MATLAB) | Stage 2b: Load Reconstruction | Completed | 2026-10-05 | 2026-10-17 | 13 |
| **T-07** | Step 4: Configuration setup & Step 5: Model verification | Stage 3a-b: Setup & Verification | Completed | 2026-10-17 | 2026-10-18 | 2 |
| **T-08** | Step 6: Static analysis & Step 7: Baseline validation | Stage 3c: Static FEM Analysis | Completed | 2026-10-17 | 2026-10-18 | 2 |
| **T-09** | Step 8: Evaluation & comparison (Config A, B, C) | Stage 3d: Optimization | Completed | 2026-10-19 | 2026-10-19 | 1 |
| **T-10** | Step 9: Final Report preparation | Documentation & Submission | In Progress | 2026-10-19 | 2026-10-20 | 2 |

**Table 0.1b: Master project stage documentation directory and delivery index**

| Stage Reference | Task ID | Document Title / Scope | Schedule Dates | Delivery Status | Document File / Relative Link |
| :--- | :--- | :--- | :---: | :---: | :--- |
| **Stage 1** | T-03 | Step 1: Data Collection & Parameter Definition | 26–27 Sep 2026 | Completed | `Finished Stage 1 Aircraft Baseline and Parameter.pdf` |
| **Stage 2a** | T-04 | Step 2: Parametric APDL Script Development | 28 Sep – 15 Oct 2026 | Completed | [Stage 2a: Parametric APDL Script Development](Stage%202a_%20Parametric%20APDL%20Script%20Development.md) |
| **Stage 2b** | T-06 | Step 3: SFBM Load Pre-processing and Nodal Force Mapping | 05–17 Oct 2026 | Completed | [Stage 2b: SFBM Load Pre-processing and Nodal Force Mapping](Stage%202b_%20SFBM%20Load%20Pre-processing%20and%20Nodal%20Force%20Mapping.md) |
| **Stage 3a** | T-07 (Part 1) | Step 4: Configuration Setup (Topologies A, B, C) | 17–18 Oct 2026 | Completed | [Stage 3a: Configuration Setup](Stage%203a_%20Configuration%20Setup.md) |
| **Stage 3b** | T-07 (Part 2) | Step 5: Model Verification (5-Gate Mathematical Qualification) | 17–18 Oct 2026 | Completed | [Stage 3b: Model Verification](Stage%203b_%20Model%20Verification.md) |
| **Stage 3c** | T-08 | Step 6: Static Analysis & Step 7: Baseline Validation | 17–18 Oct 2026 | Completed | [Stage 3c: Baseline Static and Buckling Analysis](Stage%203c_%20Baseline%20Static%20and%20Buckling%20Analysis.md) |
| **Stage 3d** | T-09 | Step 8: Evaluation & Comparison (Sensitivity & Sizing Optimization) | 19 Oct 2026 | Completed | [Stage 3d: Sensitivity Analysis and Sizing Optimization](Stage%203d_%20Sensitivity%20Analysis%20and%20Sizing%20Optimization.md) |
| **Synthesis** | T-10 | Step 9: Final Report Preparation (Master Documentation Index) | 19–20 Oct 2026 | In Progress | [Master Stage Documentation Index](00_Stage_Documents_Index.md) |

---

## 3. End-to-End Engineering Workflow & Interdependence

The project execution follows a structured, multi-disciplinary aircraft structural engineering progression featuring closed-loop feedback between aerodynamic loads, finite element meshing, and structural optimization:

```
[Stage 1: Aircraft Baseline & Parameterization]
       │
       ├────────────────────────────────────────┐
       ▼                                        ▼
[Stage 2a: Parametric APDL Scripting]    [Stage 2b: SFBM Load Mapping]
 (Modular Macro, Shell/Beam Mesh)         (Schrenk Theory, Nodal Forces)
       │                                        │
       └───────────────────┬────────────────────┘
                           ▼
               [Stage 3a: Configuration Setup]
                (Topologies A, B, C Assembled)
                           │
                           ▼
               [Stage 3b: Model Verification]
                (5-Gate Mathematical Qualification)
                           │
                           ▼
               [Stage 3c: Baseline Static & Buckling]
                (Stress Hotspots & Config C Deficit)
                           │
                           ▼
               [Stage 3d: Sizing Optimization]
                (Fast-Path APDL, Sensitivity, Regime 1A)
```

1. **Stage 1 (Baseline Definition & Geometry Formulation):** Establishes the ATR 42-600 lifting surface geometry (semi-span $b/2 = 3,978.0\text{ mm}$, root chord $c_r = 1,940.0\text{ mm}$, tip chord $c_t = 1,000.0\text{ mm}$, sweep angle $\Lambda = 4.7^\circ$, dihedral $\Gamma = 2.0^\circ$, NACA 0012 symmetric airfoil profile).
2. **Stage 2a (Parametric APDL Script Development):** Formulates the modular APDL script architecture (`MAIN_MACRO.mac` through Blocks 1 to 5). Generates keypoints, splined BTL chordwise lines, conformal aerodynamic skin areas, internal spar/rib webs, and stringers using high-order finite elements (`SHELL181` and `BEAM188`).
3. **Stage 2b (SFBM Aerodynamic Load Reconstruction & Mapping):** Reconstructs the EASA CS-25 symmetrical flight limit maneuver envelope. Using Schrenk's approximation, chordwise pressure integration, and continuous spanwise equilibrium equations, aerodynamic shear forces and bending moments are translated into discrete 3D nodal point forces mapped directly to conformal external skin nodes.
4. **Stage 3a (Configuration Setup & Topology Parameterization):** Implements structural topology switching parameters (`N_RIB`, `N_SPAR`, `N_STR`) to generate the three target airframe configurations (A: 14-3-8, B: 9-2-8, C: 7-2-6) with precise analytical rib pitches ($306.00\text{ mm}$, $497.25\text{ mm}$, $663.00\text{ mm}$).
5. **Stage 3b (Model Verification & Mathematical Qualification):** Subject all three finite element models to a rigorous 5-Gate qualification suite: verifying 6 zero-frequency rigid-body modes, cantilever natural vibration frequencies, mass conservation ($0.0000\%$ discrepancy), $1g$ multi-axis gravity equilibrium (reaction errors $< 1.0 \times 10^{-6}\%$), and unit displacement rigid translation (zero parasitic strain).
6. **Stage 3c (Baseline Static and Stability Analysis):** Solves linear static equilibrium under limit flight load case $D_{\text{fwd}}$ ($-9,142.84\text{ N}$). Decomposes shell vs. beam stresses (Full Graphics vs. PowerGraphics), identifies local chordwise bending concentrations in interior rib flanges (`SEC 6`, $29.54 - 59.15\text{ MPa}$), and identifies the critical stability deficit in Configuration C ($|\lambda_1| = 1.326 < 1.50$, $\text{MS}_{\text{buckle}} = -0.12$).
7. **Stage 3d (Sensitivity Analysis & Sizing Optimization):** Constructs a zero-remeshing fast-path evaluation architecture within MAPDL ($2.5 - 4.2\text{ s}$ per solve). Evaluates forward finite difference gradients, executes sensitivity-guided sizing across three regimes, resolves the Configuration C buckling deficiency ($|\lambda_1|: 1.326 \to 1.601$), and confirms **Configuration C Regime 1A ($112.40\text{ kg}$)** as the optimal, fully-compliant airframe structure.

---

## 4. Key Results at a Glance

A consolidated summary of structural masses, static deflections, peak stresses, stability margins, and optimization outcomes across all configurations is presented below:

**Table 0.2: Master project results synthesis across baseline and optimized structural configurations**

| Structural Metric / Parameter | Configuration A (14-3-8) | Configuration B (9-2-8) | Configuration C (7-2-6) | Governing Specification / Benchmark |
| :--- | :---: | :---: | :---: | :--- |
| **Rib Bays & Analytical Pitch** | 13 bays @ $306.00\text{ mm}$ | 8 bays @ $497.25\text{ mm}$ | 6 bays @ $663.00\text{ mm}$ | Precise geometric layout |
| **Lightening Holes ($N_{\text{cut}}$)** | 7 (1 LE + 6 box) | 6 (1 LE + 5 box) | 5 (1 LE + 4 box) | Standardized web cutouts |
| **Loaded Skin Nodes ($N_{\text{nodes}}$)**| 1,053 nodes | 648 nodes | 486 nodes | Stage 2b SFBM mapping grid |
| **Baseline Half-Span Mass ($M_0$)** | **140.463 kg** (Reference) | **121.890 kg** (-13.22%) | **113.637 kg** (-19.10%) | Dual-method mass audit ($0.0000\%$) |
| **5-Gate Verification Status** | **PASS** (Gates 1 to 5) | **PASS** (Gates 1 to 5) | **PASS** (Gates 1 to 5) | Mathematical qualification |
| **Free-Free RBM Frequencies** | $\le 2.46 \times 10^{-4}\text{ Hz}$ | $\le 2.15 \times 10^{-4}\text{ Hz}$ | $\le 2.92 \times 10^{-4}\text{ Hz}$ | True unconstrained kinematic state |
| **First Elastic Cantilever Mode ($f_7$)**| $55.99\text{ Hz}$ (Bending) | $57.17\text{ Hz}$ (Bending) | $54.24\text{ Hz}$ (Bending) | First out-of-plane flexure |
| **Applied Flight Limit Shear** | $-9,142.84\text{ N}$ | $-9,142.84\text{ N}$ | $-9,142.84\text{ N}$ | Flight case $D_{\text{fwd}}$ ($V_D = 312.5\text{ kt}$) |
| **Root Reaction Balance ($\sum R_z$)** | **-9,142.84 N** ($100.00\%$) | **-9,142.84 N** ($100.00\%$) | **-9,142.82 N** ($100.00\%$) | Static equilibrium error $< 0.001\%$ |
| **Baseline Tip Deflection (`TIP_UZ` / $u_{z,\text{tip}}$)**| **-6.716 mm** | **-6.898 mm** (+2.71%) | **-7.042 mm** (+4.85%) | Vertical out-of-plane flexure |
| **Vector Displacement Max ($\text{DMX}$)**| **6.768 mm** | **6.951 mm** | **7.096 mm** | $\vert u_{z,\text{tip}} \vert / \text{DMX} = 99.24\%$ |
| **Baseline Peak Stress (PowerGraphics)**| **29.54 MPa** (`SEC 6`) | **46.78 MPa** (`SEC 6`) | **59.15 MPa** (`SEC 6`) | Interior rib flange chordwise bending |
| **Baseline Minimum Safety Factor** | $\text{FS} = 16.25$ | $\text{FS} = 10.26$ | $\text{FS} = 8.11$ | Enormous static strength reserve |
| **Baseline Buckling Factor ($\vert \lambda_1 \vert$)** | **2.557** ($\text{MS} = +0.70$) | **1.978** ($\text{MS} = +0.32$) | **1.326** ($\text{MS} = -0.12$) | **Config C Fails Buckling ($< 1.50$)** |
| **Regime 1A Sized Mass ($M_{\text{opt}}$)** | **126.11 kg** (-10.22%) | **114.83 kg** (-18.25%) | **112.40 kg** (**-19.98%**) | **Config C Regime 1A Selected Optimum** |
| **Regime 1A Sized Tip Deflection** | $-6.72\text{ mm}$ | $-6.72\text{ mm}$ | **-6.69 mm** | Matches baseline stiffness ($\le 6.716\text{ mm}$) |
| **Regime 1A Sized Buckling Factor** | $\vert \lambda_1 \vert = 2.489$ | $\vert \lambda_1 \vert = 2.207$ | **$\vert \lambda_1 \vert = 1.601$** | **Buckling Restored ($\text{MS} = +0.07$)** |
| **Sized Peak Stress (PowerGraphics)** | $29.94\text{ MPa}$ ($\text{FS} = 16.03$) | $44.64\text{ MPa}$ ($\text{FS} = 10.75$) | $51.13\text{ MPa}$ ($\text{FS} = 9.39$) | Interior rib flange hotspot |
| **Structural Mass Saving** | $-14.35\text{ kg}$ (-10.22%) | $-25.63\text{ kg}$ (-18.25%) | **-28.06 kg** (**-19.98%**) | Half-span structural saving vs Base A |

---

## 5. Stage 1 Technical Errata & Workflow Refinements

During the progressive verification and rigorous finite element auditing conducted across Stages 2a through 3d, several preliminary assumptions, rough approximations, and legacy descriptions originating from early conceptual notes were updated to ensure absolute academic fidelity:

**Table 0.3: Reconciliation and errata between initial conceptual assumptions and verified finite element models**

| Parameter / Aspect | Preliminary / Legacy Description | Verified Numerical Reality | Engineering Justification & Scientific Context |
| :--- | :--- | :--- | :--- |
| **Al 7075-T6 Design Yield Strength** | Handbook typical yield $\sigma_y = 503.0\text{ MPa}$ ($73.0\text{ ksi}$) | Minimum design yield allowable $F_{ty} = 480.0\text{ MPa}$; $\sigma_{\text{allow}} = 320.0\text{ MPa}$ ($\text{FS} = 1.50$) | Adopts conservative aerospace specification minimums (MMPDS-01 A-basis) rather than typical handbook values, ensuring statutory CS-25 airworthiness compliance. |
| **Composite Skin Allowable Limit** | Ultimate tensile strength $F_{tu} = 600 - 800\text{ MPa}$ | Design allowable $\sigma_{\text{allow}} = 200.0\text{ MPa}$ ($F_{tu} = 300.0\text{ MPa}$, $\text{FS} = 1.50$) | Enforces open-hole compression (OHC), environmental knockdown, and impact damage tolerance envelopes required for transport aircraft composite skins. |
| **Analytical Rib Pitch Values** | Rounded nominal pitches: $300\text{ mm}$ and $525\text{ mm}$ | Exact analytical pitches: **306.00 mm** (A, 13 bays), **497.25 mm** (B, 8 bays), **663.00 mm** (C, 6 bays) | Dividing structural span ($b/2 = 3,978.0\text{ mm}$) by integer rib bay counts yields exact analytical pitches. Approximate rounded values were legacy shorthand. |
| **Rib Web Lightening Hole Counts** | Described casually as $N_{\text{cut}} = 6, 6, 4$ | Exact hole counts: **7** (Config A: 1 LE + 6 box), **6** (Config B: 1 LE + 5 box), **5** (Config C: 1 LE + 4 box) | Exact hole counts reflect the geometry generator in Block 5, maintaining safe edge margins and dynamic pillar capping. |
| **Conformal Loaded Skin Nodes** | Referred to loosely as an unverified "7,000+ grid points" | Exact loaded node counts: **1,053** (Config A), **648** (Config B), **486** (Config C) | "7,000+" was an unverified conceptual estimate; actual conformal skin locations receiving mapped SFBM aerodynamic point forces are precisely $1,053$, $648$, and $486$. |
| **Optimization Method Characterization** | Described in preliminary notes as an "automated optimizer" | **Sensitivity-guided manual sizing (2 targeted iterations per regime)** | The optimization was executed via forward finite difference gradient calculation followed by structured, sensitivity-informed manual sizing iterations. |
| **Critical Stress Hotspot Location** | Assumed to be root spar caps under global wingbox bending | **Interior Rib Flanges (`SEC 6`, BEAM188)** under chordwise camber bending transfer | Transverse airfoil pressure differentials induce localized chordwise bending across thin rib caps, generating peaks ($29.54 - 59.15\text{ MPa}$) higher than spar caps. |
| **PowerGraphics vs Full Graphics Stresses** | Initial discrepancy observed between GUI plots and batch reports | **Element-isolated shell extraction vs 3D beam facet rendering** | Divergence is resolved: Full Graphics ignores BEAM188 elements in nodal averaging and blends shell T-junctions, while PowerGraphics renders true 3D beam facets. |

---

## 6. Nomenclature, Coordinate Frames, & Engineering Units

### 6.1 Global Coordinate System
The finite element model adheres strictly to the canonical aircraft Cartesian coordinate system:
- **$X$-Axis (Chordwise):** Directed positively aft along the tailplane chord line, from leading edge to trailing edge ($X = 0\text{ mm}$ at root leading edge).
- **$Y$-Axis (Spanwise):** Directed positively outboard along the tailplane semi-span from root to wingtip ($Y = 0\text{ mm}$ at root rib; $Y = 3,978.0\text{ mm}$ at tip rib).
- **$Z$-Axis (Vertical / Normal):** Directed positively upwards perpendicular to the chordwise-spanwise plane ($Z = 0\text{ mm}$ at the airfoil chord reference plane).

### 6.2 Structural Symbols and Engineering Acronyms

**Table 0.4: Master nomenclature and mathematical symbols**

| Symbol / Acronym | Engineering Definition | Dimension / Units |
| :--- | :--- | :---: |
| **APDL** | ANSYS Parametric Design Language | — |
| **BTL** | Base Thickness Line (Chordwise parametric grid curve) | — |
| **CFRP** | Carbon Fiber Reinforced Polymer | — |
| **DMX** | Displacement Vector Sum Maximum ($\max \sqrt{U_x^2 + U_y^2 + U_z^2}$) | $\text{mm}$ |
| **EAS** | Equivalent Airspeed | $\text{kt}$ |
| **$EI$** | Flexural / Bending Rigidity | $\text{N}\cdot\text{mm}^2$ |
| **$\text{FS}$** | Factor of Safety ($\sigma_{\text{ref}} / \sigma_{\text{limit}}$) | Dimensionless |
| **$F_{ty}$ / $F_{tu}$** | Material Tensile Yield Strength / Ultimate Tensile Strength | $\text{MPa}$ |
| **HTP** | Horizontal Tailplane | — |
| **$M$ / $M_{\text{tot}}$** | Total Half-Span Structural Mass | $\text{kg}$ |
| **$\text{MS}_{\text{yield}}$** | Margin of Safety against Material Yield ($\text{FS} / 1.50 - 1.0$) | Dimensionless |
| **$\text{MS}_{\text{buckle}}$** | Margin of Safety against Linear Eigenvalue Buckling ($\vert \lambda_1 \vert / 1.50 - 1.0$) | Dimensionless |
| **$N_{\text{cut}}$** | Number of Lightening Cutouts per Rib Web | Integer |
| **RBM** | Rigid-Body Mode | — |
| **SEC ID** | Structural Section Identifier (`SEC 1` to `SEC 8`) | Integer |
| **SEQV / $\sigma_{\text{vM}}$** | Von Mises Equivalent Stress | $\text{MPa}$ |
| **SFBM** | Shear Force and Bending Moment | — |
| **$T_{\text{skin}}, T_{\text{spar}}, T_{\text{rib}}$** | Shell Component Thicknesses (Skin, Spar Web, Rib Web) | $\text{mm}$ |
| **`TIP_UZ`** | Vertical Out-of-Plane Tip Deflection ($U_z$ at $y = 3,978.0\text{ mm}$) | $\text{mm}$ |
| **$V_D$** | Design Dive Airspeed | $\text{kt}$ |
| **$\vert \lambda_1 \vert$** | First Linear Eigenvalue Buckling Load Factor (Bifurcation Multiplier) | Dimensionless |

---

## 7. Software Environment & Computational Specifications

The finite element analyses and optimization procedures were conducted in the following standardized computational environment:
- **Finite Element Solver:** ANSYS Mechanical APDL 2026 R1 (v26.1) / Batch Mode (`ansys261.exe -b`).
- **Post-Processing & Graphic Tools:** MAPDL Vector Graphics Engine (`/GRAPHICS, FULL` and `/GRAPHICS, POWER`), High-Resolution Bottom-Bar Palette Format.
- **Data Analysis & Automation:** Python 3.12, Windows PowerShell 5.1.
- **License Capacity Audit:**
  - The analyses were executed under the ANSYS Student license tier, which enforces an element and node ceiling of **128,000 nodes**.
  - The largest structural model generated in this project (Configuration A) contains **40,607 nodes** and **40,432 elements**, utilizing **$31.7\%$** of the available node limit.
  - Configurations B and C utilize **28,141 nodes** ($22.0\%$) and **22,963 nodes** ($17.9\%$), respectively, confirming complete feasibility and headroom within academic licensing constraints.

---

## 8. Summary of Project Achievements

1. **Fully Script-Driven Parametric Architecture:** Generated a modular, robust APDL workflow capable of automatically modeling three distinct aircraft structural configurations without manual GUI interaction.
2. **Aerodynamic Load Equilibrium:** Successfully mapped limit flight loads derived from CS-25 maneuver requirements, balancing a half-tail downward shear load of $-9,142.84\text{ N}$ with numerical equilibrium errors under $0.001\%$.
3. **Five-Gate Mathematical Qualification:** Qualified all finite element models across five independent verification gates, verifying zero-frequency rigid-body modes, exact structural weight equilibrium, and rigid-body unit displacement invariance.
4. **Physical Stress & Stability Mapping:** Uncovered the non-governing nature of static yield strength ($\text{FS} > 8.0$), localized chordwise bending concentrations in interior rib flanges ($29.54 - 59.15\text{ MPa}$), and diagnosed the linear eigenvalue buckling deficit in baseline Configuration C ($|\lambda_1| = 1.326$, $\text{MS}_{\text{buckle}} = -0.12$).
5. **High-Performance Sizing Optimization:** Implemented a zero-remeshing fast-path evaluation pipeline ($2.5 - 4.2\text{ s}$ per solve), quantified 18 finite difference sensitivity gradients, and established a 3-tier multi-criteria decision framework.
6. **Definitive Airframe Design:** Confirmed **Configuration C Regime 1A ($112.40\text{ kg}$)** as the optimal horizontal tailplane design, delivering a **-19.98% mass reduction (saving 28.06 kg per half-span / 56.12 kg total)** while maintaining complete aeroelastic bending stiffness equivalence and resolving buckling stability ($|\lambda_1| = 1.601$, $\text{MS}_{\text{buckle}} = +0.07$).

---

## References

1. Palumbo, C. et al. (2026). *A Parametric APDL-Based Workflow for Efficient Structural Design of a Civil Aircraft Horizontal Tailplane*. Engineering Proceedings, 133, 15.
2. European Aviation Safety Agency (EASA). (2023). *Certification Specifications for Large Aeroplanes (CS-25)*, Amendment 27.
3. Jacobs, E. N., Ward, K. E., & Pinkerton, R. M. (1933). *The Characteristics of 78 Related Airfoil Sections from Tests in the Variable-Density Wind Tunnel*. National Advisory Committee for Aeronautics, NACA Report No. 460.
4. Bruhn, E. F. (1973). *Analysis and Design of Flight Vehicle Structures*. Jacobs Publishing.
5. Megson, T. H. G. (2017). *Aircraft Structures for Engineering Students*. Butterworth-Heinemann, 6th Edition.
6. Niu, M. C. Y. (1999). *Airframe Structural Design: Practical Design Information and Data on Aircraft Structures*. Conmilit Press, 2nd Edition.
7. Bathe, K. J. (2006). *Finite Element Procedures*. Prentice Hall.
8. Clough, R. W., & Penzien, J. (2003). *Dynamics of Structures*. Computers & Structures, Inc., 3rd Edition.
9. Vanderplaats, G. N. (2007). *Multidiscipline Design Optimization*. VR&D Inc.
10. Metallic Materials Properties Development and Standardization (MMPDS-01). (2003). *Aluminum Alloy 7075-T6 Technical Specifications*. Federal Aviation Administration (FAA).
11. Composite Materials Handbook (CMH-17-3G). (2012). *Polymer Matrix Composites Materials Usage, Design, and Analysis*. SAE International.
12. Avions de Transport Régional (ATR). (2016). *ATR 42-600 Aircraft Technical Specification and Manufacturer Structural Baseline Drawings*. Blagnac, France.
