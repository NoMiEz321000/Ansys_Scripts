# Ansys HTP Parametric Model - Experimental Log

## Experiment: Attempt to Fix Spar Flange Alignment with Skin (Post v1.0.0-beta)

### Background & Objective
In 1.0.0-beta, it was observed that all Stringers and Spar Flanges (modeled with BEAM188) were oriented horizontally (pointing towards a fixed global orientation node K_OR = 90000). The goal of this experiment was to mathematically re-orient the beam elements so that their cross-sections would align flush with the curved NACA airfoil skin.

### Theoretical Logic Attempts
1. **Adjacent Keypoint Topology (K_OR = KP1 + 1)**
   - **Theory:** The mathematically perfect tangent vector of the skin at any rib can be found by simply drawing a vector to the adjacent keypoint on the same rib.
   - **Result:** **FAILED.** The NUMCMP, ALL command used after boolean area operations (ASBA) scrambled the Keypoint IDs. Therefore, KP1 + 1 pointed to random locations in space, causing severe geometric distortion in the beam orientations.

2. **Calculus Derivative with Corrected Thickness Scale (DYDX)**
   - **Theory:** By calculating the exact derivative of the NACA thickness equation (dZ/dX), we can establish an exact outward normal vector for any point on the upper or lower skin. Using this normal, we offset a custom orientation node (K_OR) to force the local Y-axis of the beam to be perpendicular to the skin.
   - **Result:** **FAILED (Geometrically Conflicting).** While this successfully aligned the flanges parallel to the skin, it caused a critical misalignment with the **Spar Web**. 
     - A Spar Flange (I-beam or Rectangular section) must rivet flat against the Spar Web.
     - The Spar Web drops vertically (or straight along a chordwise line) from the upper skin to the lower skin.
     - Because the skin is curved, the outward normal is **not** collinear with the Spar Web.
     - Aligning the Flange to the Skin caused the Flange's web to tilt away from the Spar Web, resulting in an unphysical intersecting joint.

3. **Split Logic (Stringers vs. Spar Flanges)**
   - **Theory:** Stringers should be perpendicular to the skin (using DYDX). Spar Flanges should be parallel to the Spar Web (using the vector KP2_OPP - KP1).
   - **Result:** **FAILED (Cross-Section Limitations).** Ansys BEAM188 elements maintain rigid 90-degree internal cross-sections. If the Spar Flange is forced to align its web with the Spar Web, its horizontal flange width will no longer be flush with the curved skin (creating a V-shaped gap). In real life, flanges are machined or bent at an angle, but standard APDL beams do not support variable-angle flanges without defining custom arbitrary sections (SECTYPE, ASEC).

### Conclusion & Rollback
Due to the geometric conflicts between standard beam element orientations, the curved airfoil surface, and the vertical spar web, attempting to rigidly align the flanges caused more topological issues than it solved (including intersecting elements and meshing order bugs where AMESH would prematurely mesh the beam lines). 

**Decision:** The project has been strictly rolled back to 1.0.0-beta. All beam orientation codes tested after this version are deemed unworkable for the current standard BEAM188 library approach.
- **Date**: 2026-10-06
- **Objective**: Fix BEAM188 spar flange orientation so elements lay flush against the skin profile. Focus on Spar only, skipping stringers to isolate potential errors.
- **Hypothesis**: The previous orientation vector incorrectly assumed the elements were in the X-Y plane rather than the K-node defining the X-Z plane. Also the analytical tangent derivative (DYDX) had scaling issues. Fixing the math using a full 3D cross product 
 = L x T and using local Z offset (Option A: FL_OFFZ = -1*(SPAR_FT/2 + T_SKIN/2)) will correct the orientation.
- **Action**:
  - Implemented 3D cross-product normal calculation L x T specifically for COMP_FLANGE_UPPER and COMP_FLANGE_LOWER.
  - Ignored Stringers as requested.
  - Set SECOFFSET for BEAM188 rectangular section to local Z direction instead of Y.
  - Fixed "Too many expressions" APDL syntax error during testing by breaking the equation down.
  - Resolved license server limits by properly clearing out zombie MAPDL processes and lock files.
  - Ran MAPDL batch mode.
- **Result**: HTP_PARAMETRIC.mac compiled and ran without errors. Node generation counts and solver completed successfully. SMAX = 17.38 MPa, TIP_UZ = -0.67 mm.
- **Conclusion**: The exact analytical 3D normal vector formulation and Z-axis SECOFFSET works beautifully for spar flanges. 
