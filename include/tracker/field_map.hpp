// field_map.hpp -- where every fiducial is, in FIELD coordinates.
// Source: Field.step (BEST, Aug 23 2026), converted mm -> inches.
// Field origin = center of the Dining Room, Z = up, axes = the STEP file's X/Y.
// Tag IDs are NOT in the STEP file. Fill the id fields from the fiducial
// placement map on page 7 of the Field Drawings, then set IDS_FILLED = true.
#pragma once

namespace fieldmap {

constexpr bool IDS_FILLED = false;

struct TableTag { int id; double x, y, z; };
// z = 3.625 in (12x12 hardboard top on 3.5 in 2x4 leg blocks)
constexpr TableTag TABLES[9] = {
  {-1,   0.0,   0.0, 3.625},  // Table:1 (center)
  {-1,  36.0,  36.0, 3.625},  // Table:2
  {-1,   0.0,  36.0, 3.625},  // Table:3
  {-1, -36.0,  36.0, 3.625},  // Table:4
  {-1,  36.0,   0.0, 3.625},  // Table:5
  {-1, -36.0,   0.0, 3.625},  // Table:6
  {-1,  36.0, -36.0, 3.625},  // Table:7
  {-1,   0.0, -36.0, 3.625},  // Table:8
  {-1, -36.0, -36.0, 3.625},  // Table:9
};

struct WallTag { int id; double x, y, z; double nx, ny; }; // (nx,ny) = direction the tag FACES
// All wall tags: center 9.0 in above the floor, on the inside faces (|x| or |y| = 66.0 in)
constexpr WallTag WALLS[12] = {
  {-1,  18.0,  66.0, 9.0,  0, -1}, {-1, -18.0,  66.0, 9.0,  0, -1},  // short wall, +Y side
  {-1,  18.0, -66.0, 9.0,  0,  1}, {-1, -18.0, -66.0, 9.0,  0,  1},  // short wall, -Y side
  {-1, -66.0,  18.0, 9.0,  1,  0}, {-1, -66.0,  54.0, 9.0,  1,  0},  // long wall, -X side
  {-1, -66.0, -18.0, 9.0,  1,  0}, {-1, -66.0, -54.0, 9.0,  1,  0},
  {-1,  66.0,  18.0, 9.0, -1,  0}, {-1,  66.0,  54.0, 9.0, -1,  0},  // long wall, +X side
  {-1,  66.0, -18.0, 9.0, -1,  0}, {-1,  66.0, -54.0, 9.0, -1,  0},
};

// Dining Room interior is x, y in [-66, 66]. The short walls (y = +/-66) run
// x in [-42, 42], so the four 24 in doorways are the corner gaps
// x in [42, 66] or [-66, -42] at y = +/-66.

inline bool isTableId(int id) {
  if (!IDS_FILLED) return true;     // fall back to the geometric side-length check
  for (const auto& t : TABLES) if (t.id == id) return true;
  return false;
}

} // namespace fieldmap
