# Parametric APDL-Based Horizontal Tailplane Structural Analysis and Optimization

This repository contains an advanced parametric ANSYS APDL macro (`HTP_PARAMETRIC.mac`) for the structural analysis and optimization of a composite Horizontal Tailplane (HTP).

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
- Verify the structural boundary conditions and applied pressure loads.
- Run the Parametric Optimization Loop to find the optimal thicknesses and sizing for Config A, B, and C.
- Extract the structural mass, Von Mises stresses, and Tip Deflection for the final Literature Report.

## 🛠️ Usage
To run the analysis, execute the macro in ANSYS MAPDL (Batch Mode recommended for speed):
```bash
"C:\Program Files\ANSYS Inc\vXXX\ansys\bin\winx64\ANSYS.exe" -b -i "HTP_PARAMETRIC.mac" -o "HTP_run.out"
```
Check `HTP_Results.txt` for the final optimized outputs.
