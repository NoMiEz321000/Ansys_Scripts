# Parametric APDL-Based Horizontal Tailplane Structural Analysis and Optimization

This repository contains an advanced parametric ANSYS APDL macro (`HTP_PARAMETRIC.mac`) for the structural analysis and optimization of a composite Horizontal Tailplane (HTP).

## 🚀 Version 1.3.3-beta

### Features & Updates
- **3D Surface Normal Orientation for Rib Flanges:** Resolved the rib flange inverted orientation and skin penetration defect. The flange beam orientation keypoints ($K_{OR}$) are now calculated using the true 3D surface normal $\vec{n} = \hat{T} \times \vec{S}$ (where $\hat{T}$ is the chordwise line tangent and $\vec{S}$ is the spanwise skin tangent accounting for both dihedral $7^\circ$ and planform taper). This forces the 40 mm flange width to lie 100% parallel to the inclined aerodynamic skin without tilt.
- **Inner-Skin Flush Offset:** Adjusted the section offset to `FL_OFFZ_RIB = -1*(RIB_FT/2 + T_SKIN/2)`, ensuring the outer face of the rib flange aligns mathematically flush with the inner surface of the skin with zero penetration/clipping in 3D solid rendering (`/ESHAPE, 1`).
- **Single-Sided Inward Flange for Base & Tip Ribs:** Resolved the outward flange extension defect at Root ($Y=0$) and Tip ($Y=SPAN$). Introduced single-sided inward L-profile sections (`SEC 7` and `SEC 8`) with width reduced to 20 mm (`RIB_FW_EDGE = 20.0`), strictly eliminating all overhang beyond $Y < 0$ and $Y > SPAN$.
- **Enhanced Multi-Scale Verification Suite (`VERIFY_VISUAL.mac`):** Expanded the automated diagnostic suite to 11 auto-framed snapshots (adding ultra-close-up cross-sections for mid-rib upper/lower flanges, root rib, and tip rib), alongside full quantitative metric tracking across `SEC 4`, `SEC 5`, `SEC 6`, `SEC 7`, and `SEC 8`.
- **Zero-Regression Verification:** Confirmed 0% regression on untouched structural components (Stringers: 416 elements, Spar Flanges: 312 elements, Total Rib Flanges: 1,134 elements [972 interior + 81 root + 81 tip]). Static solve completes cleanly with 0 errors ($S_{MAX} = 19.77\text{ MPa}$, $\delta_{TIP} = -0.93\text{ mm}$).

### ⚠️ Future Fixes & Enhancements (Pending Fixes)
- **Rib Cutouts for Stringers & Spar Flanges (Mouse Holes / Notches) ตาม Literature:** เจาะรูเว้า (Cutouts / Mouse holes / Notches) บริเวณแนวขอบ Rib ทั้งผิวด้านบนและผิวด้านล่าง เพื่อให้ Stringers (Hat-section) และ Spar Flanges วิ่งทะลุผ่านระนาบ Rib Web ได้อย่างถูกต้องสมบูรณ์แบบตามงานวิจัยอ้างอิง (Literature Reference) ตามรูปภาพโครงสร้าง Rib Web Layout (a)

---
## 🚀 Version 1.3.2-beta

---
## 🚀 Version 1.3.1-beta

### Features & Updates
- **Spar Intersection Protrusion Fix:** Resolved the vertical flange protrusion defect at spar-rib junctions. Replaced keypoint-based (`LSLK`) selection with chordwise skin boundary line filtering (`COMP_SKINS` with $|Y_2 - Y_1| < 1.0$), completely preventing vertical spar web lines from being selected and meshed with `SEC 6`.
- **Tip Chord Closed Flange Perimeter:** Resolved missing rib flange elements at the tip chord (Rib 14) trailing edge and lower surface. Fixed keypoint compression indexing issue (`NUMCMP, ALL`) by migrating to topology-driven boundary line extraction directly from skin components (`LSLA, S`), ensuring 100% complete closed perimeters across all 14 ribs.
- **Dihedral-Aware Beam Normal Orientation:** Resolved rib flange sinking and inverted offset. Beam orientation keypoints `K_OR` are now dynamically calculated along the local chord and camber line accounting for wing dihedral ($Z_I = Y_1 \cdot \tan(7^\circ)$), guaranteeing that beam normals point inward into the structure consistently from root to tip.
- **Zero-Regression Verification:** Confirmed 0% regression on untouched structural components (Stringers: 416 elements, Spar Flanges: 312 elements), while Rib Flanges increased from 954 to 1,134 elements (+180 elements), reflecting the full restoration of missing tip rib segments.

---
## 🚀 Version 1.3.0-beta

### Features & Updates
- **Literature-based Topology & Hole Packing:** Implemented max-ratio mathematical packing for lightening holes bounded by the NACA 0012 thickness distribution. The LE Bay now fits a single perfectly centered max-ratio hole.
- **Swept-back Planform (Straight Trailing Edge):** Modified the global macro-geometry to feature a Straight Trailing Edge, shifting the Leading Edge backward proportionally towards the tip.
- **Robust Rib Flange Generation:** Added fully parametric 360-degree Rib Flanges using BEAM188 elements. Solved the APDL ID recycling bug by using dynamic topological selection (LSLK, S, 1) post-boolean operations.
- **Enhanced Structural Parameters:** Upgraded element dimensions to robust sizes (RIB_FW=40mm, RIB_FT=1mm, SPAR_FW=80mm) and optimized T_SKIN down to 2.0mm.

### ⚠️ Known Issues (Pending Fixes)
- **Spar Intersection Protrusion:** Rib flanges inappropriately protrude at intersections with spars due to unhandled boolean overlaps.
- **Tip Chord Trailing Edge Defect:** Rib flanges are missing at the trailing edge of the tip chord.
- **Skin Sinking (Offset Issue):** Certain rib flange sections appear to sink into the skin incorrectly, requiring an orientation/offset review.

---
## 🚀 Version 1.2.0-beta

### Features & Updates
- **Stringer Hat Section (Omega Profile):** Replaced legacy I-beam stringers with parametric Hat sections (HATS). The dimensions are set to STR_BRIM=15, STR_W3=40, and STR_H=50.
- **Unified 3D Beam Orientation:** Applied the exact 3D normal vector calculation ($\vec{n} = \vec{L} \times \vec{T}$) to the stringers, eliminating the bug where stringers would point outside the aerodynamic skin. All stiffeners (Spars and Stringers) now point inward perfectly.
- **Centered Symmetric Offset:** Implemented *GET, CGY to dynamically locate the geometric center of the Hat section and applied SECOFFSET, USER, CGY, 0. This resolves an issue where the flat stringer brims would asymmetrically dig into the curved airfoil shell.
- **Run Directory Migration:** Upgraded the execution environment to main_run_3/ for strict separation of source code and output files.

---
## 🚀 Version 1.1.0-beta

### Features & Updates
- **3D Spar Flange Orientation:** Re-wrote the spar flange BEAM188 meshing logic to use a precise 3D surface normal calculation ($\vec{n} = \vec{L} \times \vec{T}$). This forces the rectangular spar flanges to lie completely flush and parallel to the curved aerodynamic skin.
- **Section Offsets:** Implemented SECOFFSET for the spar flanges in the local Z-axis (FL_OFFZ = -1*(SPAR_FT/2 + T_SKIN/2)) so the outer face of the flange aligns mathematically perfectly with the inner face of the skin.
- **Project Restructuring:** Moved all .out, .err, and MAPDL junk files into an isolated main_run_2/ directory to keep the repository clean.

### 🚧 Pending Fixes (To-Do)
- **Flange-Skin Gap Visualization:** Currently, the skin uses a default MID offset, meaning the nodes (cyan mesh) lie at the center of the skin thickness. Because Ansys MAPDL draws shells as zero-thickness planes by default, the correct T_SKIN/2 inward shift of the flange appears as a visual gap in the GUI. Waiting for a decision to either keep the mathematically accurate mid-plane, or change the skin to an Outer Mold Line (OML) TOP offset for 1:1 visual alignment.

---
## 🚀 Version 1.0.0-beta

### Features & Capabilities
- **Parametric Geometry Generation:** Automatically generates a NACA 0012 HTP structure with Ribs, Spars, Stringers, and Skins based on parametric inputs (Span, Root/Tip Chord, Dihedral, etc.).
- **Topological Layout Selector:** Easily switch between predefined structural configurations:
  - **Config A:** 14 Ribs, 3 Spars, 8 Stringers, 6 Cutouts per Rib (Added LE bay hole)
  - **Config B:** 9 Ribs, 2 Spars, 8 Stringers, 6 Cutouts per Rib (Added LE bay hole)
  - **Config C:** 7 Ribs, 2 Spars, 6 Stringers, 4 Cutouts per Rib (Added LE bay hole)
- **Robust Geometric Cutouts:** Implements bulletproof Component-based boolean subtraction for Lightening Holes, seamlessly handling Ansys Area ID recycling.
- **Flawless Meshing:** Uses consistent `ESIZE` and topology tracking to avoid mesh crashes, singular matrices, or unmeshed skin panels.
- **Structural Solving:** Fully configured to apply boundary conditions, evaluate structural loads, and extract maximum stresses and tip deflections.

---

## 🛠️ The Road to v1.0.0-beta: Comprehensive Bug Fix Review
The journey from the initial script to `v1.0.0-beta` involved hunting down several highly obscure geometric and meshing bugs in Ansys APDL. Here is a summary of the major fixes:

### 1. The Rib Meshing Crash (`SIG$SEGV` & 1.E+20 Aspect Ratio)
- **Symptom:** Ansys would abruptly crash or abort meshing with severe aspect ratio errors when trying to mesh the cut Ribs.
- **Root Cause:** Splitting the highly faceted airfoil splines using `ASBL` and letting `SMRTSIZE` handle the local mesh density caused microscopic sliver elements near the Leading Edge.
- **Fix:** Dropped `SMRTSIZE` entirely and enforced a global `ESIZE`. Abandoned line-partitioning (`ASBL`) and allowed the robust free-mesher to map the unpartitioned area dynamically.

### 2. The Flying Holes Bug (`WPOFFS` Axis Mapping)
- **Symptom:** Holes were generated far below the wing surface instead of piercing the Ribs.
- **Root Cause:** The Working Plane offset command `WPOFFS, 0, 0, -Y_STA(I)` inadvertently translated the WP along the Global Z-axis instead of the Spanwise Y-axis. 
- **Fix:** Corrected the coordinate mapping to `WPOFFS, 0, Y_STA(I), 0`.

### 3. The "First Rib Only" Cut Bug (Area ID Recycling)
- **Symptom:** The script perfectly cut lightening holes in Rib 1, but for Ribs 2 to N, the holes were drawn but not subtracted, leaving solid ribs.
- **Root Cause:** When Rib 1's holes were subtracted, Ansys freed up Area IDs 1 to 6. When `AL, ALL` generated Rib 2, it recycled ID 1. However, the script used `MAXD` (Maximum Defined ID) to select the new Rib, which accidentally grabbed ID 7 (the cut version of Rib 1). Thus, `ASBA` attempted to subtract Rib 2's holes from Rib 1, failing geometrically.
- **Fix:** Abolished `MAXD` tracking for Rib area generation. Wrapped the generation in `CM` (Component) logic (`CMSEL, U, TEMP_PRE_RIB_AREAS`) to robustly isolate the newly generated Area ID regardless of recycling.

### 4. The Missing LE Skin Mesh Bug (`AMESH` Skin ID Recycling)
- **Symptom:** The upper leading edge skin elements for the first few bays were entirely missing in `EPLOT`, despite `APLOT` showing the areas existed and 0 warnings in the log.
- **Root Cause:** Because `ASBA` deleted the 5 circular holes per rib, Ansys recycled Area IDs 1 to 6. The Skin generation loop (`A, KB1...`) naturally reused IDs 1 to 6 for the very first panels it generated (the upper leading edge). Because the skin selection logic relied on `*GET, A_MAX_PRE, AREA, 0, NUM, MAXD` (which was ~13), the `ASEL, S, AREA, , 14, 157` command completely skipped the recycled skin IDs.
- **Fix:** Completely purged `MAXD` tracking from Skin, Spar, and Stringer generation. Replaced it with Component tracking (`TEMP_PRE_SKINS`), guaranteeing all generated topologies are flawlessly captured and meshed.

### 5. Singular Matrix / Meba License Errors (Unattached Nodes)
- **Symptom:** Ansys would abruptly abort the sparse solver, output a "Singular Matrix" warning, and erroneously request an enterprise `meba` license.
- **Root Cause:** Floating, unattached nodes left behind by boolean operations or orphaned geometry created a zero-stiffness matrix block.
- **Fix:** Ensured proper node sharing between Stringers/Flanges and Skins. Restored the critical missing Orientation Keypoint `K, 90000` for `BEAM188`. Moved the `NDELE` (node cleanup) command into the `/PREP7` block where it safely executes before the solver starts.

### 6. Addition of the Leading Edge Bay Holes
- **Enhancement:** The original logic left the entire Leading Edge bay solid. Successfully parameterized and introduced an extra `CYL4` geometric hole at `X = 0.08*C` for all ribs, elevating the Config A/B layout to 6 cutouts, and Config C to 4 cutouts.

---

## 📌 Next Steps
- Implement single-sided inward flange offset for Base (Root) and Tip Ribs (v1.3.2-beta).
- Verify the structural boundary conditions and applied pressure loads.
- Run the Parametric Optimization Loop to find the optimal thicknesses and sizing for Config A, B, and C.
- Extract the structural mass, Von Mises stresses, and Tip Deflection for the final Literature Report.

## 📁 Archive & Legacy Documentation
Historical notes and early development tracking files have been moved to `archive/`:
- [`archive/progress.md`](archive/progress.md): Historical milestone tracker and initial bugfix notes.
- [`archive/experiment.md`](archive/experiment.md): Early experiments on beam orientation formulations.

## 🛠️ Usage
To run the analysis, execute the macro in ANSYS MAPDL (Batch Mode recommended for speed):
```bash
"C:\Program Files\ANSYS Inc\vXXX\ansys\bin\winx64\ANSYS.exe" -b -i "HTP_PARAMETRIC.mac" -o "HTP_run.out"
```
Check `HTP_Results.txt` for the final optimized outputs.
