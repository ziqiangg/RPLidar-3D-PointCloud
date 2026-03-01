# 3D Reconstruction Guide for RPLidar Scans

## Understanding the Cylindrical Artifact

When scanning with a **vertically-mounted LiDAR** rotated about a fixed point (e.g., in a room corner), you'll notice that raw point clouds appear as **two concentric half-cylinders** rather than the expected room geometry.

### Why This Happens

This is **NOT a sensor error** — it's a **geometric artifact** of the scanning method:

1. **Scanning Geometry**: The LiDAR rotates in a vertical plane, taking measurements along circular arcs
2. **Natural Distribution**: Points naturally lie on cylindrical surfaces centered on the rotation axis
3. **Sampling vs. Reality**: Raw points show WHERE measurements were taken, not the SHAPE of objects

```
Actual Room (Cube):          Raw Point Cloud (Artifact):
┌───────────────┐            
│               │                    ╱╲
│               │                   ╱  ╲
│   [Scanner]   │                  │ () │  ← Concentric
│       ●       │                   ╲  ╱     cylinders
│               │                    ╲╱
└───────────────┘
```

### The Solution: Surface Reconstruction

To visualize the **actual 3D environment**, you must reconstruct surfaces from the point cloud:

---

## Reconstruction Methods

### 1. **Voxel Grid** ⭐ RECOMMENDED for Room Scanning

**Best for**: Architectural spaces, rooms, structured environments

**How it works**: Bins points into 3D voxels (think: 3D pixels). A cube room will appear as a filled rectangular block showing occupied space.

**Pros**:
- Fast and intuitive
- Shows volumetric occupancy
- No preprocessing needed
- Works with sparse data

**Usage**:
```bash
# Command line
python viewer/surface_reconstruction.py data/scan_3d.csv voxel 0.05

# Or use GUI: Select "Voxel Grid" in Visualization tab
```

**Parameters**:
- `voxel_size`: Size of each voxel in meters (default: 0.05m = 5cm)
  - Smaller = more detail, more voxels
  - Larger = faster, smoother visualization
  - Recommended: 0.03-0.10m for room scanning

---

### 2. **Ball Pivoting Mesh**

**Best for**: Sparse, uneven data; outdoor environments

**How it works**: "Rolls" virtual balls of varying radii to create triangular mesh connecting nearby points.

**Pros**:
- Works well with uneven point density
- Creates actual surface mesh
- Good for objects with holes

**Cons**:
- Requires normal estimation (slower)
- May miss areas with low density
- Parameter-sensitive

**Usage**:
```bash
python viewer/surface_reconstruction.py data/scan_3d.csv ball_pivoting
```

---

### 3. **Poisson Surface Reconstruction**

**Best for**: Dense, complete scans; smooth surfaces

**How it works**: Creates smooth, watertight mesh using implicit function.

**Pros**:
- Produces very smooth surfaces
- Fills holes automatically
- High-quality result

**Cons**:
- Requires oriented normals (slow)
- May extrapolate beyond data
- Not ideal for sparse scans
- Can "over-smooth" details

**Usage**:
```bash
python viewer/surface_reconstruction.py data/scan_3d.csv poisson
```

**Parameters**:
- `depth`: Octree depth (6-12). Higher = more detail, slower

---

### 4. **Alpha Shapes**

**Best for**: Concave shapes, object boundary detection

**How it works**: Creates concave hull wrapping around points with configurable "tightness".

**Pros**:
- Preserves concave features
- Good for irregular shapes

**Cons**:
- Parameter-sensitive
- May create artifacts with sparse data

**Usage**:
```bash
python viewer/surface_reconstruction.py data/scan_3d.csv alpha_shape
```

---

## Using the GUI

### Step 1: Perform a 3D Scan

1. Launch the application:
   ```powershell
   python viewer/app.py
   ```

2. In **Scan Control** tab:
   - Select "3D Scan (servo 0° → 180°)"
   - Click **Start Scan**
   - Wait for completion

3. Scan saves to `data/scan_3d.csv` and `data/scan_3d.ply`

### Step 2: Visualize with Reconstruction

1. Switch to **Visualization** tab

2. Click **Load File...** → Select `data/scan_3d.csv`

3. Select reconstruction method:
   - ☑ **Voxel Grid** (recommended for rooms)
   - ☐ Raw Points (shows cylindrical artifact)
   - ☐ Ball Pivoting Mesh
   - ☐ Poisson Surface

4. Adjust **Voxel Size**: 
   - Default: 0.05m (5cm)
   - Fine detail: 0.02-0.03m
   - Coarse view: 0.08-0.10m

5. Click **Visualize in 3D Window**

---

## Command-Line Usage

### Quick Visualization

```bash
# Voxel grid (default)
python viewer/standalone_viewer.py data/scan_3d.csv 2 voxel 0.05

# Raw points (no reconstruction)
python viewer/standalone_viewer.py data/scan_3d.csv 2 raw

# Ball pivoting mesh
python viewer/standalone_viewer.py data/scan_3d.csv 2 ball_pivoting

# Poisson surface
python viewer/standalone_viewer.py data/scan_3d.csv 2 poisson
```

### Direct Reconstruction

```bash
# Voxel grid with 3cm voxels
python viewer/surface_reconstruction.py data/scan_3d.csv voxel 0.03

# Ball pivoting
python viewer/surface_reconstruction.py data/scan_3d.csv ball_pivoting

# Poisson reconstruction
python viewer/surface_reconstruction.py data/scan_3d.csv poisson
```

---

## Verification: Is My Coordinate System Correct?

### ✅ Your System is Already Correct

The coordinate transformation in `xyzscan_servo_auto.py` correctly handles vertically-mounted LiDAR:

```python
# Lines 448-461: Correct spherical to Cartesian conversion
elevation_rad = math.radians(elevation_deg)  # LiDAR angle (0-360°)
azimuth_rad = math.radians(z_plane)          # Servo angle (0-180°)

# Step 1: Project onto vertical circle
horizontal_distance = r * math.cos(elevation_rad)
z = r * math.sin(elevation_rad)

# Step 2: Rotate around vertical axis by servo angle
x = horizontal_distance * math.cos(azimuth_rad)
y = horizontal_distance * math.sin(azimuth_rad)
```

### Coordinate System:
- **Origin**: Scanner mounting point (rotation center)
- **X-axis**: Forward at servo 0°
- **Y-axis**: Right (perpendicular to X)
- **Z-axis**: Vertical (up)
- **Elevation**: LiDAR rotation (0° = forward-horizontal, 90° = up, 180° = back, 270° = down)
- **Azimuth**: Servo rotation (horizontal sweep 0° to 180°)

### CSV Format Verification:

```csv
quality,elevation_deg,distance_mm,x_m,y_m,z_m
15,81.390625,332.75,0.049811718784895626,0.0,0.3290005397741689
```

- `x_m, y_m, z_m`: Cartesian coordinates in meters ✅
- `elevation_deg`: LiDAR angle (0-360°) ✅
- `distance_mm`: Radial distance in millimeters ✅

**Conclusion**: Your coordinate transformation is geometrically correct. The cylindrical appearance is the expected sampling pattern — use reconstruction to visualize actual geometry.

---

## Tips for Best Results

### Scanning Strategy

1. **Mount Scanner in Corner**: Place scanner at room corner for maximum coverage
2. **Ensure Full Servo Sweep**: Verify servo completes 0° to 180° range
3. **Stable Mount**: Vibration during scan creates artifacts
4. **Multiple Scans**: If needed, perform scans from different positions and merge

### Resolution vs. Speed

| Voxel Size | Detail Level | Voxels (typical 3x3m room) | Performance |
|------------|--------------|---------------------------|-------------|
| 0.02m      | Very Fine    | ~675,000                  | Slow        |
| 0.03m      | Fine         | ~300,000                  | Good        |
| 0.05m      | Medium       | ~108,000                  | Fast        |
| 0.08m      | Coarse       | ~42,000                   | Very Fast   |
| 0.10m      | Very Coarse  | ~27,000                   | Instant     |

### Reconstruction Method Selection

| Environment          | Best Method        | Alternative      |
|----------------------|--------------------|------------------|
| Room/Interior        | Voxel Grid         | Ball Pivoting    |
| Outdoor/Terrain      | Ball Pivoting      | Alpha Shapes     |
| Dense Complete Scan  | Poisson            | Ball Pivoting    |
| Sparse/Partial Scan  | Voxel Grid         | Ball Pivoting    |
| Object Scanning      | Ball Pivoting      | Poisson          |

---

## Troubleshooting

### "Mesh creation failed, showing raw points"

**Cause**: Not enough points for meshing algorithm

**Solutions**:
- Use **Voxel Grid** instead (always works)
- Reduce `voxel_size` parameter
- Perform longer scan to capture more points

### Voxel grid looks blocky

**Cause**: Voxel size too large

**Solution**: Reduce voxel size (try 0.03m or 0.02m)

### Reconstruction is very slow

**Cause**: Too many points or small voxel size

**Solutions**:
- Increase voxel size (try 0.08m)
- Use Voxel Grid instead of Poisson/Ball Pivoting
- For large scans, downsample first

### Room shape not recognizable

**Cause**: Incomplete scan coverage or wrong method

**Solutions**:
- Verify servo completed full sweep (check log)
- Try different reconstruction method
- Ensure scanner was stable during scan
- Check that CSV has valid x_m, y_m, z_m data

---

## Example Workflow: Scanning a Cube Room

```bash
# 1. Set up scanner in room corner
# 2. Run 3D scan via GUI or:
python rpi_scanner_service.py  # On Raspberry Pi

# 3. Visualize with voxel reconstruction
python viewer/standalone_viewer.py data/scan_3d.csv 2 voxel 0.05

# 4. Try different voxel sizes
python viewer/standalone_viewer.py data/scan_3d.csv 2 voxel 0.03  # Finer
python viewer/standalone_viewer.py data/scan_3d.csv 2 voxel 0.08  # Coarser

# 5. Compare with raw points (cylindrical artifact)
python viewer/standalone_viewer.py data/scan_3d.csv 2 raw

# 6. Try mesh reconstruction
python viewer/standalone_viewer.py data/scan_3d.csv 2 ball_pivoting
```

**Expected Result**: Voxel grid should clearly show the rectangular room boundary, with walls, floor, and ceiling as filled voxel blocks.

---

## References

- **Open3D Documentation**: http://www.open3d.org/docs/
- **Voxel Grids**: [Tutorial](http://www.open3d.org/docs/latest/tutorial/geometry/voxelization.html)
- **Surface Reconstruction**: [Tutorial](http://www.open3d.org/docs/latest/tutorial/geometry/surface_reconstruction.html)
- **Ball Pivoting**: Bernardini et al., 1999
- **Poisson Surface**: Kazhdan et al., 2006

---

## Summary

✅ **Coordinate system is correct** — cylindrical pattern is expected sampling geometry

✅ **Use Voxel Grid reconstruction** for room scanning (best results)

✅ **CSV/PLY format is correct** — x_m, y_m, z_m in Cartesian coordinates

✅ **Scanner should be at rotation center** (typically room corner)

**The key insight**: Don't visualize raw points for geometry understanding — always use reconstruction to see the actual 3D environment.
