import numpy as np
import matplotlib.pyplot as plt

def generate_arc(start_pos, arc_angle_deg, arc_length=6.0, num_points=100):
    theta = np.radians(abs(arc_angle_deg))
    radius = arc_length / theta

    if arc_angle_deg > 0:
        center = (start_pos[0], start_pos[1] - radius)
        angles = np.linspace(np.pi / 2, np.pi / 2 - theta, num_points)
    else:
        center = (start_pos[0], start_pos[1] + radius)
        angles = np.linspace(-np.pi / 2, -np.pi / 2 + theta, num_points)

    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)

    x_rot = -y
    y_rot = x

    return x_rot, y_rot, x_rot[-1]

def generate_straight_line(length=6.0, num_points=100):
    x = np.linspace(0, length, num_points)
    y = np.zeros_like(x)
    x_rot = -y
    y_rot = x
    return x_rot, y_rot

def save_paths_txt(path_list, filename="paths.txt"):
    with open(filename, "w") as f:
        for idx, (x_list, y_list) in enumerate(path_list):
            line = [f"{idx}"]
            for x, y in zip(x_list, y_list):
                line.append(f"{x:.3f}")
                line.append(f"{y:.3f}")
            f.write(" ".join(line) + "\n")
    print(f"✅ Saved {len(path_list)} paths to {filename}")

def main():
    # ===== 参数配置 =====
    arc_length = 6.0
    angle_resolution_deg = 15
    min_angle_deg = -180
    max_angle_deg = 180
    points_per_path = 30
    start = (0.0, 0.0)

    arc_angles_deg = list(range(min_angle_deg, max_angle_deg + 1, angle_resolution_deg))

    plt.figure(figsize=(8, 8))
    path_list = []

    # 生成圆弧路径
    for angle in arc_angles_deg:
        x_arc, y_arc, end_x = generate_arc(start, angle, arc_length, num_points=points_per_path)
        x_all = np.insert(x_arc, 0, 0.0)
        y_all = np.insert(y_arc, 0, 0.0)
        path_list.append((x_all, y_all))
        label = f"{angle:+}° {'Right' if end_x > 0 else 'Left'}"
        plt.plot(x_all, y_all, label=label)

    # 添加直线路径
    x_line, y_line = generate_straight_line(length=arc_length, num_points=points_per_path)
    x_line = np.insert(x_line, 0, 0.0)
    y_line = np.insert(y_line, 0, 0.0)
    path_list.append((x_line, y_line))
    plt.plot(x_line, y_line, label="0° Line", color="black", linestyle="--")

    # 保存为 txt 文件
    save_paths_txt(path_list, filename="paths.txt")

    # 坐标轴与图形
    ax = plt.gca()
    ax.set_aspect('equal')
    ax.axhline(0, color='black', linewidth=1)
    ax.axvline(0, color='black', linewidth=1)
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend(loc='upper left', fontsize='small', ncol=2)
    plt.title(f"Paths from Origin | L={arc_length}, ΔAngle={angle_resolution_deg}°, Range=[{min_angle_deg}°, {max_angle_deg}°]")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

