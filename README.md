# HTP Parametric - Debug Log & Task List

**เป้าหมายหลัก:** แก้ไขสคริปต์ `HTP_PARAMETRIC.mac` ให้สามารถรันผ่านใน Ansys Student (ลิมิต 128k nodes) และให้ผลลัพธ์ที่ถูกต้องสำหรับการทำ Optimization

## 📋 ปัญหาที่รอการแก้ไข (To-Do List)

1. [ ] **Rib Meshing Crash (SIG$SEGV):** แม้จะปิดการเจาะรูแล้ว Rib ตันๆ ก็ยัง Mesh ไม่ได้ สาเหตุน่าจะมาจาก Airfoil profile ที่มุมแหลมจัดตรง LE/TE ทำให้ Mesher พัง
2. [ ] **Lightening Holes (ยังไม่ได้ทดสอบ):** ระบบ CYL4 + ASBA ที่เขียนไว้ยังไม่ได้รันจริง เพราะ Rib Mesh ยังพังอยู่
3. [ ] **BEAM188/SHELL181 Node Sharing:** Stringer/Flange อาจจะยังไม่ share node กับ Skin
4. [ ] **Load Application (4.5G):** ยังเป็น Uniform Pressure อยู่ ยังไม่ได้ทำเป็น Distribution
5. [ ] **Solve & Post-Processing:** รอให้ Geometry + Mesh ผ่านก่อน (License ไม่ให้รัน Batch ต้องรันผ่าน GUI)

## 🔧 Debug Flags ในสคริปต์
| Flag | ตำแหน่ง | ค่าปัจจุบัน | ความหมาย |
|------|---------|------------|----------|
| `SKIP_HOLES` | Block 5 | `1` (ปิด) | ข้ามการเจาะรู Lightening Holes |
| `SKIP_RIB_MESH` | Block 8 | `1` (ปิด) | ข้ามการ Mesh Ribs |
| Block 9 & 10 | ท้ายไฟล์ | Commented | BC, Loads, Solve, Post-process |

---

## 🛠️ โซนทดลอง (Hypothesis & Testing)

### รอบที่ 1: ทดสอบด้วย `*GO, SKIP_HOLES`
- **สมมติฐาน:** ปัญหามาจากการเจาะรู (ASBA boolean)
- **ผลลัพธ์:** ❌ APDL ไม่ยอมให้ `*GO` กระโดดข้าม `*DO` loop → เปลี่ยนมาใช้ `*IF` flag แทน

### รอบที่ 2: ปิดเจาะรู (SKIP_HOLES=1), เปิด Rib Mesh
- **สมมติฐาน A:** ปัญหามาจากรูเจาะ → ถ้าปิดรูแล้ว Mesh ผ่าน
- **สมมติฐาน B:** ปัญหามาจาก Airfoil profile แหลม → ถ้าปิดรูแล้วยังพัง
- **ผลลัพธ์:** ❌ **ทฤษฎี B ถูก** — Crash ด้วย `SIG$SEGV` ตรง `AMESH, ALL` แม้ Rib ไม่มีรู

### รอบที่ 3: ปิดทั้ง Hole + Rib Mesh (SKIP_HOLES=1, SKIP_RIB_MESH=1)
- **สมมติฐาน:** ส่วนที่เหลือ (Skin, Spar, Beams) น่าจะรันผ่านได้
- **ผลลัพธ์:** ✅ Geometry + Skin/Spar/Beam Mesh ผ่านหมด! Solve พังเพราะ License (ไม่ใช่บัคโค้ด)
