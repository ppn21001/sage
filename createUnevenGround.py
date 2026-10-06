import numpy as np
from scipy.ndimage import gaussian_filter

WIDTH = 40.0
LENGTH = 40.0

NX = 81
NY = 81

MAX_HEIGHT = 1.8
NOISE_STRENGTH = 1.35
SMOOTHING_SIGMA = 2.0

OUTPUT_FILE = "src/fleet_simulation/terrain/forest_terrain.obj"

x = np.linspace(-WIDTH / 2, WIDTH / 2, NX)
y = np.linspace(-LENGTH / 2, LENGTH / 2, NY)

X, Y = np.meshgrid(x, y)

# Smooth rolling terrain
Z = (
    0.25 * np.sin(X / 4.0)
    + 0.20 * np.cos(Y / 5.0)
    + 0.15 * np.sin((X + Y) / 6.0)
)

rng = np.random.default_rng(seed=42)

noise = rng.normal(size=(NY, NX))
noise = gaussian_filter(noise, sigma=SMOOTHING_SIGMA)

noise -= noise.mean()
noise /= np.max(np.abs(noise))

Z += NOISE_STRENGTH * noise

# Scale elevation range
Z -= Z.mean()

height_range = Z.max() - Z.min()

if height_range > 0:
    Z *= MAX_HEIGHT / height_range

# Make center approximately z=0
Z -= Z[NY // 2, NX // 2]


# --------------------------------------------------
# Compute vertex normals
# --------------------------------------------------

dx = x[1] - x[0]
dy = y[1] - y[0]

# Height gradients
dZ_dy, dZ_dx = np.gradient(Z, dy, dx)

# Surface normal for z = f(x,y):
#
# n = (-dz/dx, -dz/dy, 1)
#
NX_normal = -dZ_dx
NY_normal = -dZ_dy
NZ_normal = np.ones_like(Z)

length = np.sqrt(
    NX_normal**2
    + NY_normal**2
    + NZ_normal**2
)

NX_normal /= length
NY_normal /= length
NZ_normal /= length


with open(OUTPUT_FILE, "w") as f:

    f.write("# Procedurally generated forest terrain\n")
    f.write(f"# Size: {WIDTH} x {LENGTH} m\n")
    f.write(f"# Grid: {NX} x {NY}\n\n")

    # -------------------------
    # Vertices
    # -------------------------
    for j in range(NY):
        for i in range(NX):
            f.write(
                f"v {X[j, i]:.6f} "
                f"{Y[j, i]:.6f} "
                f"{Z[j, i]:.6f}\n"
            )

    f.write("\n")

    # -------------------------
    # Vertex normals
    # -------------------------
    for j in range(NY):
        for i in range(NX):
            f.write(
                f"vn {NX_normal[j, i]:.6f} "
                f"{NY_normal[j, i]:.6f} "
                f"{NZ_normal[j, i]:.6f}\n"
            )

    f.write("\n")

    # -------------------------
    # Faces
    #
    # OBJ syntax:
    #
    # vertex_index//normal_index
    #
    # We have one normal per vertex, so the
    # vertex and normal indices are identical.
    # -------------------------
    for j in range(NY - 1):
        for i in range(NX - 1):

            v00 = j * NX + i + 1
            v10 = j * NX + (i + 1) + 1
            v01 = (j + 1) * NX + i + 1
            v11 = (j + 1) * NX + (i + 1) + 1

            f.write(
                f"f "
                f"{v00}//{v00} "
                f"{v10}//{v10} "
                f"{v11}//{v11}\n"
            )

            f.write(
                f"f "
                f"{v00}//{v00} "
                f"{v11}//{v11} "
                f"{v01}//{v01}\n"
            )


print(f"Generated: {OUTPUT_FILE}")
print(f"Elevation min: {Z.min():.3f} m")
print(f"Elevation max: {Z.max():.3f} m")
print(f"Vertices: {NX * NY}")
print(f"Triangles: {(NX - 1) * (NY - 1) * 2}")