# Ansys HTP Parametric Model - Progress Tracker

## 🟢 Completed Achievements

### 1. Geometric & Topology Stability (v1.0.0-beta)
- **Rib Lightening Holes (Boolean Operations):** Successfully implemented circular lightening holes across the ribs. 
- **Fixed ID Recycling Bug:** Resolved a critical Ansys APDL topological bug where Boolean subtraction (ASBA) was deleting original area IDs and recycling them. The old MAXD tracking method was picking up recycled IDs, causing the upper skin and rib elements to vanish during meshing. Fixed by implementing robust **Component-based tracking** (CMSEL, U and CM, ..., AREA) to strictly isolate newly generated topologies.
- **Leading Edge (LE) Bay Holes:** Expanded the hole-cutting logic to include the Leading Edge bay (X = 0.08*C_I), bringing the total cuts to 6 for standard configurations and 4 for Config C.
- **Script Robustness:** Cleaned up area groupings (COMP_SKINS, COMP_RIBS, COMP_SPARS) ensuring 100% element generation without any missing mesh sections.

---

## 🟡 Work in Progress / Pending

- **Beam Element Orientation (Spar Flanges & Stringers):** 
  - *Status:* Reverted to standard default behavior (1.0.0-beta).
  - *Note:* Attempts to mathematically align BEAM188 flanges perfectly tangent to the curved skin while simultaneously keeping the beam's web flush with the vertical Spar Web hit structural geometry limitations inherent to standard 90-degree orthogonal beam cross-sections. Refer to experiment.md for full details.

---

## 🔴 Future Goals & Optimization
- Conduct mesh convergence studies (ESIZE optimization).
- Parametric structural weight optimization vs. Tip deflection.
- Extracting stress concentration factors (Kt) around the newly meshed lightening holes.

### [2026-10-06] Fix Spar Flange BEAM188 Orientation
- **Status:** Completed
- **Details:** Re-wrote the spar flange meshing block to accurately define the orientation keypoint (K) using a full 3D normal calculation (
 = L x T). The section offset was updated to Option A (flush inner face) using local Z translation (-SPAR_FT/2 - T_SKIN/2). Stringer logic was kept untouched for now to isolate fixes.

### [2026-10-06] Release v1.2.0-beta: Stringer Hat Section & 3D Orientation
- **Status:** Completed
- **Details:** 
  - Converted the Stringer cross-sections from I-Beams to Hat sections (HATS).
  - Parameterized the Hat section using STR_BRIM=15, STR_W3=40, STR_H=50, and STR_T=3.0.
  - Unified the 3D normal vector logic (
 = L x T) for both Spar Flanges and Stringers, guaranteeing all stiffeners point perfectly inward.
  - Dynamically fetched the Centroid Y (CGY) of the Hat section using *GET to perfectly center the Stringer along the node. This resolves the asymmetric "sinking" graphical bug.
  - Migrated run outputs to main_run_3/ for better repository organization.

### [2026-10-08] Release v1.3.0-beta: Swept-back Planform & Rib Flanges
- **Status:** Completed (with known issues)
- **Details:**
  - Implemented max-ratio hole packing bounded by the airfoil thickness equation.
  - Adjusted global geometry to a **Straight Trailing Edge (Swept-back Leading Edge)** by dynamically shifting local X-coordinates.
  - Increased structural dimensions: RIB_FW=40, RIB_FT=1, SPAR_FW=80. Reduced T_SKIN to 2.0.
  - Implemented Rib Flanges along the entire airfoil perimeter using robust topological selection (LSLK, S, 1).
- **Known Issues (Pending Fixes):**
  - รอยต่อ Rib Flange กับ Spar: มีส่วนของ Flange โผล่ทะลุออกมา (อาจเกิดจากโค้ด Rib Flange ยังไม่ครอบคลุมบริเวณจุดตัดนี้)
  - ปลายปีก (Tip Chord): Rib Flange ฝั่งท้าย (Trailing Edge) มีชิ้นส่วนที่ขาดหายไป
  - การทับซ้อน: มีจังหวะที่ Rib Flange บางส่วนจมลงไปใน Skin ผิดปกติ (รอตรวจสอบการคำนวณ Orientation K_OR)
