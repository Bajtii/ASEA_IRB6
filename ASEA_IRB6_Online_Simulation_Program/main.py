import numpy as np
import matplotlib.pyplot as plt

from matplotlib.widgets import Slider, TextBox, Button
from functools import reduce

from utils.matrix_transformation import DH_elementary_matrix

# ======================================================
# Robot Parameters
# ======================================================
D1 = 0.7
L2 = 0.45
L3 = 0.67
D4 = 0.095

# ======================================================
# Trajectory and Position Storage
# ======================================================
trajectory = []

positions = []
positions_xyz = []


# ======================================================
# Create NumPy Array
# ======================================================
def create_array(*args):
    return np.array(*args, dtype=np.dtype("f8"))


# ======================================================
# Calculate Transformation Matrix
# ======================================================
def calculate_T_matrices(thetas):

    T00 = DH_elementary_matrix(theta=thetas[0], alpha=90)
    T03 = DH_elementary_matrix(theta=90)
    T04 = DH_elementary_matrix(theta=-thetas[1], a=L2)
    T05 = DH_elementary_matrix(theta=-90, alpha=-90)

    T10 = DH_elementary_matrix(theta=thetas[0], alpha=90)
    T16 = DH_elementary_matrix(theta=thetas[2], a=L3, alpha=-90)

    T21 = DH_elementary_matrix(theta=thetas[0], d=D1, alpha=90)
    T23 = DH_elementary_matrix(theta=-thetas[3] + 90, a=D4)
    T29 = DH_elementary_matrix(theta=-90, alpha=-90)
    T210 = DH_elementary_matrix(theta=thetas[4])

    mask = np.zeros((4, 4))

    mask[0, -1] = 1
    mask[1, -1] = 1
    mask[2, -1] = 1

    T0 = reduce(np.dot, (T00, T03, T04, T05))
    T1 = reduce(np.dot, (T10, T16))
    T2 = reduce(np.dot, (T21, T23, T29, T210))

    T_combined = ((T0 + T1) * mask) + T2

    return T_combined


# ======================================================
# Extract XYZ Coordinates
# ======================================================
def extract_xyz(T):
    return T[0, -1], T[1, -1], T[2, -1]


# ======================================================
# Compute Joint Positions
# ======================================================
def compute_positions(theta1, theta2, theta3, theta4):

    theta2 = -theta2 + np.pi / 2
    theta4 = -theta4 + np.pi / 2

    x = np.zeros(5)
    y = np.zeros(5)
    z = np.zeros(5)

    x[0], y[0], z[0] = 0, 0, 0

    x[1] = x[0]
    y[1] = y[0]
    z[1] = z[0] + D1

    x[2] = x[1] + L2 * np.cos(theta2)
    y[2] = y[1]
    z[2] = z[1] + L2 * np.sin(theta2)

    x[3] = x[2] + L3 * np.cos(theta3)
    y[3] = y[2]
    z[3] = z[2] + L3 * np.sin(theta3)

    x[4] = x[3] + D4 * np.cos(theta4)
    y[4] = y[3]
    z[4] = z[3] + D4 * np.sin(theta4)

    x_rot = np.cos(theta1) * x - np.sin(theta1) * y
    y_rot = np.sin(theta1) * x + np.cos(theta1) * y
    z_rot = z

    return x_rot, y_rot, z_rot


# ======================================================
# Inverse Kinematics
# ======================================================
def inverse_kinematics(px, py, pz):

    Beta = 0
    Alpha = 1.38
    Gamma = 1.579

    As = np.sin(Alpha)
    Ac = np.cos(Alpha)

    Bs = np.sin(Beta)
    Bc = np.cos(Beta)

    Gs = np.sin(Gamma)
    Gc = np.cos(Gamma)

    theta4 = np.arccos(Ac * Bc)

    if Alpha > 0 and Beta > 0:
        theta4 = -theta4

    elif Alpha < 0 and Beta > 0:
        theta4 = -theta4

    theta5 = np.arcsin((As * Bs / -np.sin(theta4)))
    theta5_deg = np.degrees(theta5)

    theta1 = np.arctan2(py, px)

    r = np.sqrt(px**2 + py**2)

    a = D4 * np.sin(theta4)
    b = D4 * np.cos(theta4)

    pz_modified = pz - D1

    c = pz_modified - b
    d = r - a

    e = np.sqrt(c**2 + d**2)

    beta = np.arccos(d / e)

    lambda_ = np.arccos(
        (L2**2 + e**2 - L3**2) / (2 * L2 * e)
    )

    if pz < 0.74:
        theta2 = 90 + np.degrees(beta) - np.degrees(lambda_)
    else:
        theta2 = 90 - np.degrees(beta) - np.degrees(lambda_)

    f = L2 * np.sin(np.deg2rad(theta2))

    g = d - f

    theta3 = np.arccos(g / L3)

    theta3_deg = np.degrees(theta3)

    calculated_thetas = np.array(
        [
            np.degrees(theta1),
            theta2,
            np.degrees(theta3),
            np.degrees(theta4)
        ]
    )

    x, y, z = compute_positions(
        *map(np.radians, calculated_thetas)
    )

    px_C, py_C, pz_C = x[4], y[4], z[4]

    tolerance = 0.01

    if (
        abs(px - px_C) > tolerance
        or abs(py - py_C) > tolerance
        or abs(pz - pz_C) > tolerance
    ):

        calculated_thetas[2] = -calculated_thetas[2]

        x, y, z = compute_positions(
            *map(np.radians, calculated_thetas)
        )

        px_C, py_C, pz_C = x[4], y[4], z[4]

    print(f"Calculated angles: {calculated_thetas}")

    return calculated_thetas


# ======================================================
# Plot Robot Configuration
# ======================================================
def plot_robot(theta1_deg, theta2_deg, theta3_deg, theta4_deg):

    theta1 = np.radians(theta1_deg)
    theta2 = np.radians(theta2_deg)
    theta3 = np.radians(theta3_deg)
    theta4 = np.radians(theta4_deg)

    x, y, z = compute_positions(
        theta1,
        theta2,
        theta3,
        theta4
    )

    ax.cla()

    ax.plot(
        x,
        y,
        z,
        marker="o",
        linestyle="-",
        color="b"
    )

    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1.5])

    end_effector_coords.set_val(
        f"X: {x[4]:.2f}  Y: {y[4]:.2f}  Z: {z[4]:.2f}"
    )

    plt.draw()


# ======================================================
# Slider Update Callback
# ======================================================
def update(val):

    plot_robot(
        slider1.val,
        slider2.val,
        slider3.val,
        slider4.val
    )

    text_box1.set_val(f"{slider1.val:.1f}")
    text_box2.set_val(f"{slider2.val:.1f}")
    text_box3.set_val(f"{slider3.val:.1f}")
    text_box4.set_val(f"{slider4.val:.1f}")


# ======================================================
# Text Input Callbacks
# ======================================================
def submit1(text):
    try:
        slider1.set_val(float(text))
    except ValueError:
        pass


def submit2(text):
    try:
        slider2.set_val(float(text))
    except ValueError:
        pass


def submit3(text):
    try:
        slider3.set_val(float(text))
    except ValueError:
        pass


def submit4(text):
    try:
        slider4.set_val(float(text))
    except ValueError:
        pass


def submit_x(text):
    try:
        val = float(text)
        text_box_x.set_val(f"{val:.2f}")
    except ValueError:
        pass


def submit_y(text):
    try:
        val = float(text)
        text_box_y.set_val(f"{val:.2f}")
    except ValueError:
        pass


def submit_z(text):
    try:
        val = float(text)
        text_box_z.set_val(f"{val:.2f}")
    except ValueError:
        pass


# ======================================================
# Reset Robot State
# ======================================================
def reset_robot(event):

    global trajectory

    slider1.set_val(0)
    slider2.set_val(0)
    slider3.set_val(0)
    slider4.set_val(0)

    trajectory = []

    plot_robot(0, 0, 0, 0)


# ======================================================
# Reset Trajectory
# ======================================================
def reset_trajectory(event):

    global trajectory

    trajectory = []

    line.set_data([], [])

    coords_text.set_text("")

    fig_draw.canvas.draw()

    plot_robot(
        slider1.val,
        slider2.val,
        slider3.val,
        slider4.val
    )


# ======================================================
# Placeholder Calculation Function
# ======================================================
def calculate_values(event):
    pass


# ======================================================
# Draw 2D Trajectory
# ======================================================
def draw_trajectory(event):

    if event.inaxes == ax_draw:

        x, y = event.xdata, event.ydata

        trajectory.append((x, y))

        xdata, ydata = zip(*trajectory)

        line.set_data(xdata, ydata)

        coords_text.set_text(
            "\n".join(
                [f"({x:.2f}, {y:.2f})" for x, y in trajectory]
            )
        )

        fig_draw.canvas.draw()


# ======================================================
# Add Position Using Joint Angles
# ======================================================
def add_position(event):

    positions.append(
        (
            slider1.val,
            slider2.val,
            slider3.val,
            slider4.val
        )
    )

    print(f"Added position: {positions[-1]}")


# ======================================================
# Add Position Using XYZ Coordinates
# ======================================================
def add_position_xyz(event):

    try:

        x = float(text_box_x.text)
        y = float(text_box_y.text)
        z = float(text_box_z.text)

        thetas = inverse_kinematics(x, y, z)

        positions.append(thetas)
        positions_xyz.append((x, y, z))

        print(
            f"Added XYZ position: ({x}, {y}, {z}) "
            f"with thetas: {thetas}"
        )

    except ValueError:

        print("Invalid XYZ coordinates")


# ======================================================
# Remove Last Position
# ======================================================
def remove_position(event):

    if positions:

        removed_position = positions.pop()

        print(f"Removed position: {removed_position}")


# ======================================================
# Interpolate Joint Positions
# ======================================================
def interpolate_positions(start, end, steps=20):

    return [
        np.linspace(s, e, steps)
        for s, e in zip(start, end)
    ]


# ======================================================
# Play Full Trajectory
# ======================================================
def play_trajectory(event):

    global trajectory

    trajectory = []

    for i in range(len(positions) - 1):

        start_thetas = positions[i]
        end_thetas = positions[i + 1]

        x_start, y_start, z_start = compute_positions(
            *map(np.radians, start_thetas)
        )

        x_end, y_end, z_end = compute_positions(
            *map(np.radians, end_thetas)
        )

        x_start, y_start, z_start = (
            x_start[-1],
            y_start[-1],
            z_start[-1]
        )

        x_end, y_end, z_end = (
            x_end[-1],
            y_end[-1],
            z_end[-1]
        )

        interp_x = np.linspace(x_start, x_end, 20)
        interp_y = np.linspace(y_start, y_end, 20)
        interp_z = np.linspace(z_start, z_end, 20)

        for x, y, z in zip(
            interp_x,
            interp_y,
            interp_z
        ):

            thetas = inverse_kinematics(x, y, z)

            plot_robot(*thetas)

            trajectory.append((x, y, z))

            plt.pause(0.1)

    plot_robot(*positions[-1])


# ======================================================
# Main Robot Visualization Window
# ======================================================
fig = plt.figure(figsize=(14, 8))

ax = fig.add_subplot(111, projection="3d")


# ======================================================
# Slider Configuration
# ======================================================
ax_slider1 = plt.axes(
    [0.1, 0.01, 0.65, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_slider2 = plt.axes(
    [0.1, 0.03, 0.65, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_slider3 = plt.axes(
    [0.1, 0.05, 0.65, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_slider4 = plt.axes(
    [0.1, 0.07, 0.65, 0.03],
    facecolor="lightgoldenrodyellow"
)

slider1 = Slider(
    ax_slider1,
    "Theta1",
    -180.0,
    180.0,
    valinit=0
)

slider2 = Slider(
    ax_slider2,
    "Theta2",
    -180.0,
    180.0,
    valinit=0
)

slider3 = Slider(
    ax_slider3,
    "Theta3",
    -180.0,
    180.0,
    valinit=0
)

slider4 = Slider(
    ax_slider4,
    "Theta4",
    -180.0,
    180.0,
    valinit=0
)


# ======================================================
# Joint Angle Text Boxes
# ======================================================
ax_text1 = plt.axes(
    [0.87, 0.01, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_text2 = plt.axes(
    [0.87, 0.05, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_text3 = plt.axes(
    [0.87, 0.09, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_text4 = plt.axes(
    [0.87, 0.13, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

text_box1 = TextBox(ax_text1, "Theta1", initial="0.0")
text_box2 = TextBox(ax_text2, "Theta2", initial="0.0")
text_box3 = TextBox(ax_text3, "Theta3", initial="0.0")
text_box4 = TextBox(ax_text4, "Theta4", initial="0.0")


# ======================================================
# XYZ Input Fields
# ======================================================
ax_text_x = plt.axes(
    [0.87, 0.45, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_text_y = plt.axes(
    [0.87, 0.50, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_text_z = plt.axes(
    [0.87, 0.55, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

text_box_x = TextBox(ax_text_x, "X", initial="0.0")
text_box_y = TextBox(ax_text_y, "Y", initial="0.0")
text_box_z = TextBox(ax_text_z, "Z", initial="0.0")


# ======================================================
# Button Configuration
# ======================================================
ax_button_reset = plt.axes(
    [0.87, 0.22, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_button_calculate = plt.axes(
    [0.87, 0.27, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_button_add_position = plt.axes(
    [0.87, 0.32, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_button_add_position_xyz = plt.axes(
    [0.87, 0.37, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_button_remove_position = plt.axes(
    [0.87, 0.42, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

ax_button_play_trajectory = plt.axes(
    [0.87, 0.60, 0.1, 0.03],
    facecolor="lightgoldenrodyellow"
)

button_reset = Button(ax_button_reset, "Reset")

button_calculate = Button(
    ax_button_calculate,
    "Calculate"
)

button_add_position = Button(
    ax_button_add_position,
    "Add Position"
)

button_add_position_xyz = Button(
    ax_button_add_position_xyz,
    "Add Position XYZ"
)

button_remove_position = Button(
    ax_button_remove_position,
    "Remove Position"
)

button_play_trajectory = Button(
    ax_button_play_trajectory,
    "Play Trajectory"
)


# ======================================================
# End Effector Coordinate Display
# ======================================================
ax_end_effector_coords = plt.axes(
    [0.84, 0.65, 0.14, 0.05],
    facecolor="lightgoldenrodyellow"
)

end_effector_coords = TextBox(
    ax_end_effector_coords,
    "",
    initial="X: 0.0  Y: 0.0  Z: 0.0"
)


# ======================================================
# Widget Event Connections
# ======================================================
slider1.on_changed(update)
slider2.on_changed(update)
slider3.on_changed(update)
slider4.on_changed(update)

text_box1.on_submit(submit1)
text_box2.on_submit(submit2)
text_box3.on_submit(submit3)
text_box4.on_submit(submit4)

text_box_x.on_submit(submit_x)
text_box_y.on_submit(submit_y)
text_box_z.on_submit(submit_z)

button_reset.on_clicked(reset_robot)
button_calculate.on_clicked(calculate_values)

button_add_position.on_clicked(add_position)

button_add_position_xyz.on_clicked(add_position_xyz)

button_remove_position.on_clicked(remove_position)

button_play_trajectory.on_clicked(play_trajectory)


# ======================================================
# Initial Robot Rendering
# ======================================================
plot_robot(0, 0, 0, 0)


# ======================================================
# 2D Trajectory Drawing Window
# ======================================================
fig_draw, ax_draw = plt.subplots()

ax_draw.set_title("2D Trajectory Drawing")

ax_draw.set_xlim([-1.6, 1.6])
ax_draw.set_ylim([-1.6, 1.6])

(line,) = ax_draw.plot([], [], "r-")


coords_text = ax_draw.text(
    1.05,
    0.5,
    "",
    transform=ax_draw.transAxes,
    verticalalignment="center"
)

fig_draw.canvas.mpl_connect(
    "button_press_event",
    draw_trajectory
)


# ======================================================
# Trajectory Reset Button
# ======================================================
ax_button_reset_traj = plt.axes([0.8, 0.9, 0.1, 0.075])

button_reset_traj = Button(
    ax_button_reset_traj,
    "Reset Trajectory"
)

button_reset_traj.on_clicked(reset_trajectory)


# ======================================================
# Start Application
# ======================================================
plt.show()
