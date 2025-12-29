import csv
import math
import matplotlib.pyplot as plt

def wrap_theta(th):
    two_pi = 2.0 * math.pi
    th = (th + math.pi) % two_pi - math.pi
    if th >= math.pi / 2.0 + 0.15:
        th -= two_pi
    if th < -1.5 * math.pi - 0.15:
        th += two_pi
    return th

t, theta_raw, omega = [], [], []
with open("pendulum_path.csv", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row["t"]))
        theta_raw.append(float(row["theta"]))
        omega.append(float(row["omega"]))

theta = [wrap_theta(th) for th in theta_raw]
print(theta)

x = [math.cos(th) for th in theta]
y = [math.sin(th) for th in theta]

plt.figure("ω vs θ", figsize=(6, 5))
plt.plot(theta, omega, linewidth=1.5)
plt.scatter([math.pi / 2, -1.5 * math.pi], [0, 0], color="red", s=60, zorder=5)
plt.xlabel("θ")
plt.ylabel("θ̇ dot")
plt.grid(True)

# # --- 2. Position plot (x, y) ---
# plt.figure("Pendulum trajectory", figsize=(6, 6))
# plt.plot(x, y, linewidth=1.5)
# circle = plt.Circle((0, 0), L, fill=False, linestyle="--", alpha=0.3)
# plt.gca().add_artist(circle)
# plt.gca().set_aspect("equal", adjustable="box")
# pad = 1.2 * L
# plt.xlim(-pad, pad)
# plt.ylim(-pad, pad)
# plt.xlabel("x")
# plt.ylabel("y")
# plt.title("Pendulum Swing Path")
# plt.grid(True)

plt.show()
