import math
import random
import tkinter as tk
from tkinter import ttk

ITEM_TEMPLATES = [
    {
        "name": "Laptop(open notebook)",
        "w_min": 320, "w_max": 400,
        "d_min": 220, "d_max": 280,
        "h_min": 140, "h_max": 160
    },
    {
        "name": "Water Cup",
        "w_min": 70, "w_max": 90,
        "d_min": 70, "d_max": 90,
        "h_min": 110, "h_max": 150
    },
    {
        "name": "Power Bank",
        "w_min": 130, "w_max": 170,
        "d_min": 70, "d_max": 95,
        "h_min": 18, "h_max": 28
    },
    {
        "name": "Pen Holder",
        "w_min": 80, "w_max": 110,
        "d_min": 80, "d_max": 110,
        "h_min": 100, "h_max": 130
    },
    {
        "name": "Textbook",
        "w_min": 210, "w_max": 260,
        "d_min": 270, "d_max": 310,
        "h_min": 22, "h_max": 35
    }
]


class ObstacleItem:
    def __init__(self, name, cx, cy, width, depth, height):
        self.name = name
        self.cx = cx
        self.cy = cy
        self.w = width
        self.d = depth
        self.h = height
        self.x1 = cx - width / 2.0
        self.x2 = cx + width / 2.0
        self.y1 = cy - depth / 2.0
        self.y2 = cy + depth / 2.0

    def aabb_intersect(self, other):
        return not (
            self.x2 < other.x1 or
            self.x1 > other.x2 or
            self.y2 < other.y1 or
            self.y1 > other.y2
        )

    def point_inside(self, px, py):
        return (self.x1 <= px <= self.x2) and (self.y1 <= py <= self.y2)

    def get_label_text(self):
        return f"{self.name}\nW:{self.w:.0f} D:{self.d:.0f} H:{self.h:.0f}"


class MobileManipulatorPrototype:
    def __init__(self):
        self.table_length = 1800.0
        self.table_width = 800.0
        self.table_x_min = 0.0
        self.table_x_max = self.table_length
        self.table_y_min = 0.0
        self.table_y_max = self.table_width

        self.base_world_x = self.table_length / 2.0
        self.base_world_y = 0.0
        self.safety_margin = 50.0
        self.chassis_radius = 120.0

        self.L1 = 300.0
        self.L2 = 600.0
        self.arm_inner_r = abs(self.L1 - self.L2)
        self.arm_outer_r = self.L1 + self.L2

        self.dof0_min = -120.0
        self.dof0_max = 120.0
        self.dof1_min = -140.0
        self.dof1_max = 140.0

        self.z_retracted = True
        self.z_max_down = 150.0
        self.current_arm_joint = [0.0, 0.0]

        # 3D‑Z规划参数
        self.base_safe_z = 220.0
        self.z_obstacle_gap = 60.0   # 在障碍物上方预留安全间隙
        self.grasp_z = 0.0

        self.obstacle_list = self.generate_random_obstacles()

        self.last_task_data = {
            "obstacles": [],
            "target_table": None,
            "detour_waypoints": [],
            "arm_trajectory": [],
            "arm_end_points": [],
            "arm_end_3d": [],  # 存储 (x,y,z) 完整3D轨迹，供X‑Z主视图
            "final_dof": None,
            "base_start": None,
            "base_end": None
        }

    def generate_random_obstacles(self):
        obs_list = []
        count = random.randint(2, 5)
        for _ in range(count):
            template = random.choice(ITEM_TEMPLATES)
            w = random.randint(template["w_min"], template["w_max"])
            d = random.randint(template["d_min"], template["d_max"])
            h = random.randint(template["h_min"], template["h_max"])
            cx = random.uniform(w / 2 + 40, self.table_x_max - w / 2 - 40)
            cy = random.uniform(max(d/2+80, 40), self.table_y_max - d / 2 - 20)
            new_obs = ObstacleItem(template["name"], cx, cy, w, d, h)
            conflict = False
            for exist in obs_list:
                if new_obs.aabb_intersect(exist):
                    conflict = True
                    break
            if not conflict:
                obs_list.append(new_obs)
        return obs_list

    def print_obstacle_info(self):
        print("\n==== Pre‑generated static table obstacles ====")
        for idx, obs in enumerate(self.obstacle_list):
            print(f"  Obstacle{idx+1}: {obs.get_label_text()}  center({obs.cx:.1f},{obs.cy:.1f})")
        print("============================================\n")

    def update_table_geometry(self, table_len: float, table_wid: float):
        self.table_length = table_len
        self.table_width = table_wid
        self.table_x_max = self.table_length
        self.table_y_max = self.table_width
        print(f"[Table Updated] Length(X)={self.table_length} mm , Width(Y)={self.table_width} mm")

    def update_arm_link(self, L1: float, L2: float):
        self.L1 = L1
        self.L2 = L2
        self.arm_inner_r = abs(self.L1 - self.L2)
        self.arm_outer_r = self.L1 + self.L2
        print(f"[Arm Link Updated] L1={self.L1:.1f} mm, L2={self.L2:.1f} mm")
        print(f"Arm work annulus: inner radius={self.arm_inner_r:.1f} mm , outer radius={self.arm_outer_r:.1f} mm")

    def auto_select_arm_link(self, req_min_reach):
        """自动连杆寻优 L1 ∈300‑600 mm (30‑60cm)，L2∈300‑600；内环死区≤350；优先总臂展最小"""
        print(f"\n==== Auto arm‑link selection ON, required minimal reach: {req_min_reach:.1f} mm ====")
        candidates = []
        step = 30
        for l1 in range(300, 601, step):
            for l2 in range(300, 601, step):
                inner = abs(l1 - l2)
                outer = l1 + l2
                if outer < req_min_reach:
                    continue
                if inner > 350:
                    continue
                cos_th2 = (req_min_reach**2 - l1**2 - l2**2) / (2*l1*l2)
                if not (-1.0 <= cos_th2 <= 1.0):
                    continue
                cost = outer
                candidates.append({"L1":l1, "L2":l2, "inner":inner, "outer":outer, "cost":cost})
        if not candidates:
            print("[AutoSelect] NO valid arm combination found, keep current L1/L2")
            return
        candidates.sort(key=lambda c: c["cost"])
        best = candidates[0]
        print(f"✅ Auto‑selected arm: L1={best['L1']} mm(30‑60cm), L2={best['L2']} mm")
        print(f"  inner dead‑zone={best['inner']} mm , total max reach={best['outer']} mm")
        self.update_arm_link(float(best["L1"]), float(best["L2"]))

    def table_to_robot_base(self, table_x: float, table_y: float, base_x, base_y):
        dx = table_x - base_x
        dy = table_y - base_y
        return dx, dy

    def two_r_inverse_kinematics(self, x: float, y: float):
        r_sq = x**2 + y**2
        r = math.sqrt(r_sq)
        if not (self.arm_inner_r - 1e-6 <= r <= self.arm_outer_r + 1e-6):
            return False, 0.0, 0.0

        cos_theta2 = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
        theta2_pos_rad = math.acos(cos_theta2)
        theta2_neg_rad = -math.acos(cos_theta2)

        def solve(th2_rad):
            alpha = math.atan2(y, x)
            beta = math.atan2(self.L2 * math.sin(th2_rad), self.L1 + self.L2 * math.cos(th2_rad))
            th1_rad = alpha - beta
            d0 = math.degrees(th1_rad)
            d1 = math.degrees(th2_rad)
            valid = (self.dof0_min <= d0 <= self.dof0_max) and (self.dof1_min <= d1 <= self.dof1_max)
            return valid, d0, d1

        valid1, d0_1, d1_1 = solve(theta2_pos_rad)
        valid2, d0_2, d1_2 = solve(theta2_neg_rad)

        if valid1:
            return True, d0_1, d1_1
        elif valid2:
            return True, d0_2, d1_2
        else:
            return False, d0_1, d1_1

    def generate_robot_detour_path(self, start_base_x, start_base_y, final_base_x, final_base_y):
        wp = []
        wp.append((round(start_base_x, 1), round(start_base_y, 1)))
        step1_y = min(start_base_y - self.safety_margin - 120, final_base_y - 80)
        wp.append((round(start_base_x, 1), round(step1_y, 1)))
        wp.append((round(final_base_x, 1), round(step1_y, 1)))
        wp.append((round(final_base_x, 1), round(final_base_y, 1)))
        return wp

    def check_chassis_path_collision(self, waypoints, obstacle_list):
        for obs in obstacle_list:
            for (px, py) in waypoints:
                if py < self.safety_margin:
                    continue
                dist = math.hypot(px - obs.cx, py - obs.cy)
                if dist < (self.chassis_radius + max(obs.w, obs.d) / 2 + self.safety_margin * 0.5):
                    return True, f"Chassis path collide with obstacle [{obs.name}]"
        return False, "No chassis collision"

    def check_stop_pose_safe(self, bx, by, obs_list):
        for obs in obs_list:
            dist_stop = math.hypot(bx - obs.cx, by - obs.cy)
            if dist_stop < (self.chassis_radius + max(obs.w, obs.d)/2 + self.safety_margin):
                return False, f"Too close to [{obs.name}]"
        return True, "Safe stop"

    def get_obstacle_height_at_xy(self, px, py, obs_list):
        """查询该XY点下最高障碍物高度，无障碍物返回0"""
        max_h = 0.0
        for obs in obs_list:
            if obs.point_inside(px, py):
                if obs.h > max_h:
                    max_h = obs.h
        return max_h

    def check_arm_pose_collision_3d(self, base_x, base_y, dof0_deg, dof1_deg, z_arm, obs_list):
        """3D碰撞检测：连杆+Z高度"""
        d0r = math.radians(dof0_deg)
        d1r = math.radians(dof1_deg)
        j1x = base_x + self.L1 * math.cos(d0r)
        j1y = base_y + self.L1 * math.sin(d0r)
        j2x = j1x + self.L2 * math.cos(d0r + d1r)
        j2y = j1y + self.L2 * math.sin(d0r + d1r)

        sample_cnt = 10
        for i in range(sample_cnt + 1):
            t = i / sample_cnt
            sx = base_x + t * (j1x - base_x)
            sy = base_y + t * (j1y - base_y)
            obs_h = self.get_obstacle_height_at_xy(sx, sy, obs_list)
            if obs_h > 1e-3 and z_arm <= obs_h + 1e-3:
                return True, f"Arm link‑1 3D collide, obs H={obs_h:.1f}, arm Z={z_arm:.1f}"
        for i in range(sample_cnt + 1):
            t = i / sample_cnt
            sx = j1x + t * (j2x - j1x)
            sy = j1y + t * (j2y - j1y)
            obs_h = self.get_obstacle_height_at_xy(sx, sy, obs_list)
            if obs_h > 1e-3 and z_arm <= obs_h + 1e-3:
                return True, f"Arm link‑2 3D collide, obs H={obs_h:.1f}, arm Z={z_arm:.1f}"
        return False, "No 3D arm collision"

    @staticmethod
    def calc_arm_endpoint(base_x, base_y, L1, L2, dof0_deg, dof1_deg):
        d0r = math.radians(dof0_deg)
        d1r = math.radians(dof1_deg)
        j1x = base_x + L1 * math.cos(d0r)
        j1y = base_y + L1 * math.sin(d0r)
        j2x = j1x + L2 * math.cos(d0r + d1r)
        j2y = j1y + L2 * math.sin(d0r + d1r)
        return j2x, j2y

    def sample_candidate_base_poses(self, target_x, target_y, original_base_x, original_base_y):
        candidates = []
        sample_y_list = [-120, -180, -240, -300, -360]
        for sy in sample_y_list:
            candidates.append( (target_x, sy) )
        return candidates

    def replan_search_best_base(self, tbx, tby, obs_list, start_bx, start_by):
        print("\n==== Auto Replan Activated: Try multiple base candidates ====")
        candidates = self.sample_candidate_base_poses(tbx, tby, start_bx, start_by)
        valid_options = []
        for (cand_bx, cand_by) in candidates:
            print(f"\n-- Testing candidate base pose: ({cand_bx:.1f}, {cand_by:.1f})")
            safe_stop, msg_stop = self.check_stop_pose_safe(cand_bx, cand_by, obs_list)
            if not safe_stop:
                print(f"    Skip: {msg_stop}")
                continue
            waypoints = self.generate_robot_detour_path(start_bx, start_by, cand_bx, cand_by)
            chassis_collide, msg_chassis = self.check_chassis_path_collision(waypoints, obs_list)
            if chassis_collide:
                print(f"    Skip: {msg_chassis}")
                continue
            dx_loc, dy_loc = self.table_to_robot_base(tbx, tby, cand_bx, cand_by)
            ik_ok, d0, d1 = self.two_r_inverse_kinematics(dx_loc, dy_loc)
            if not ik_ok:
                print("    Skip: IK no valid joint solution")
                continue
            arm_collide, msg_arm = self.check_arm_pose_collision_3d(cand_bx, cand_by, d0, d1, self.base_safe_z, obs_list)
            if arm_collide:
                print(f"    Skip: {msg_arm}")
                continue
            dist_cost = math.hypot(cand_bx - start_bx, cand_by - start_by)
            valid_options.append( {"base":(cand_bx,cand_by), "waypoints":waypoints, "dof":(d0,d1), "cost":dist_cost} )
            print(f"    ✅ This candidate is feasible, movement cost={dist_cost:.1f}")
        if len(valid_options) == 0:
            print("\n❌ All sampled base candidates failed. No feasible plan.")
            return None
        valid_options.sort(key=lambda x: x["cost"])
        best = valid_options[0]
        print(f"\n✅ Selected BEST candidate base pose {best['base']}, minimal movement cost={best['cost']:.1f}")
        return best

    def execute_mobile_manip_task(self, tbx: float, tby: float, tbz: float, obj_id="obj_test"):
        print("=" * 70)
        print(f"[Mission Start] ObjectID={obj_id}")
        print(f"Target TABLE‑FRAME X={tbx:.1f} mm Y={tby:.1f} mm Z={tbz:.1f} mm")
        print(f"Current mobile base world pos: X={self.base_world_x:.1f}, Y={self.base_world_y:.1f}")

        obs_list = self.obstacle_list
        start_bx, start_by = self.base_world_x, self.base_world_y
        dx_loc0, dy_loc0 = self.table_to_robot_base(tbx, tby, start_bx, start_by)
        r_need = math.hypot(dx_loc0, dy_loc0)

        self.auto_select_arm_link(r_need)

        dx_loc, dy_loc = self.table_to_robot_base(tbx, tby, start_bx, start_by)
        r_local = math.hypot(dx_loc, dy_loc)
        print(f"\nTarget relative to current robot base: local X={dx_loc:.1f}, Y={dy_loc:.1f}, radius r={r_local:.1f} mm")
        print(f"Arm annulus work space: inner={self.arm_inner_r:.1f} mm , outer={self.arm_outer_r:.1f} mm")

        best_plan = None
        detour_waypoints = []
        final_dof = None

        if self.arm_inner_r -1e-6 <= r_local <= self.arm_outer_r +1e-6:
            print(f"[Condition] Target inside arm annulus workspace, FIRST TRY base stationary")
            ik_ok, d0, d1 = self.two_r_inverse_kinematics(dx_loc, dy_loc)
            if ik_ok:
                arm_collide, msg_arm = self.check_arm_pose_collision_3d(start_bx, start_by, d0, d1, self.base_safe_z, obs_list)
                if not arm_collide:
                    print("✅ Stationary base plan works, use this plan")
                    best_plan = {"base":(start_bx, start_by), "waypoints":[], "dof":(d0,d1), "cost":0.0}
                else:
                    print(f"⚠ Stationary base plan FAILED: {msg_arm} → Trigger Auto Replan")
                    best_plan = self.replan_search_best_base(tbx, tby, obs_list, start_bx, start_by)
            else:
                print("⚠ Stationary base IK failed → Trigger Auto Replan")
                best_plan = self.replan_search_best_base(tbx, tby, obs_list, start_bx, start_by)
        elif r_local < self.arm_inner_r:
            print(f"[Condition] Target inside inner dead‑zone r < {self.arm_inner_r:.1f} mm → Trigger Auto Replan")
            best_plan = self.replan_search_best_base(tbx, tby, obs_list, start_bx, start_by)
        else:
            print(f"[Condition] Target beyond max arm reach r > {self.arm_outer_r:.1f} mm → Trigger Auto Replan")
            best_plan = self.replan_search_best_base(tbx, tby, obs_list, start_bx, start_by)

        if best_plan is None:
            print("[FINAL FAIL] No feasible plan found after replanning. Task abort.")
            self.last_task_data = {
                "obstacles": obs_list,
                "target_table": (tbx, tby, tbz),
                "detour_waypoints": [],
                "arm_trajectory": [],
                "arm_end_points": [],
                "arm_end_3d": [],
                "final_dof": None,
                "base_start": (start_bx, start_by),
                "base_end": (start_bx, start_by)
            }
            self.draw_single_window()
            self.draw_xz_view()
            return {"status":"global_replan_fail", "obj_id":obj_id}

        final_bx, final_by = best_plan["base"]
        detour_waypoints = best_plan["waypoints"]
        dof0, dof1 = best_plan["dof"]
        self.base_world_x, self.base_world_y = final_bx, final_by

        if len(detour_waypoints) > 0:
            print("\n[Mobile base detour waypoints (world X,Y)]")
            for idx, (wx, wy) in enumerate(detour_waypoints):
                print(f"    waypoint{idx}: world X={wx:.1f} mm , world Y={wy:.1f} mm")
            print(f"[Mobile base finished detour. New base world pos X={self.base_world_x:.1f}, Y={self.base_world_y:.1f}]")

        print(f"\n[2R‑IK Result] DOF0(base rotate)={dof0:.2f} ° ; DOF1(relative elbow)={dof1:.2f} °")
        step_cnt = 8
        arm_traj = []
        arm_end_pts = []
        arm_end_3d = []

        print("\n==== 3D continuous Z‑profile planning (follow obstacle contour) ====")
        for i in range(step_cnt + 1):
            ratio = i / step_cnt
            d0 = self.current_arm_joint[0] + (dof0 - self.current_arm_joint[0]) * ratio
            d1 = self.current_arm_joint[1] + (dof1 - self.current_arm_joint[1]) * ratio
            arm_traj.append([round(d0, 2), round(d1, 2)])
            ex, ey = self.calc_arm_endpoint(self.base_world_x, self.base_world_y, self.L1, self.L2, d0, d1)
            arm_end_pts.append((ex, ey))
            # 动态Z：查询XY下方障碍物高度
            obs_here = self.get_obstacle_height_at_xy(ex, ey, obs_list)
            z_dyn = max(self.base_safe_z, obs_here + self.z_obstacle_gap)
            arm_end_3d.append((ex, ey, z_dyn))
            print(f"  traj{i}: X={ex:.1f} Y={ey:.1f}, obs_h={obs_here:.1f}, dynamic_Z={z_dyn:.1f}")

        print(f"[Arm joint‑space trajectory waypoints count={len(arm_traj)}]")
        for idx, pt in enumerate(arm_traj):
            print(f"    arm_wp{idx}: DOF0={pt[0]:6.2f}°, DOF1={pt[1]:6.2f}°")

        print("\n[Phase1: Move arm joints, Z dynamically follows obstacle upper contour]")
        self.current_arm_joint = [dof0, dof1]
        print(f"[Phase2: Z‑axis descend to grasp Z={tbz:.1f} mm → grasp object]")
        print("[Grasp complete → retract Z‑axis]")

        if random.random() < 0.8:
            print("[Verification PASS: grasp mission complete]")
            stat = "success"
        else:
            print("[Verification FAIL: grasp offset over tolerance]")
            stat = "verify_fail"
        print(f"[Mission End] status = {stat}")

        self.last_task_data = {
            "obstacles": obs_list,
            "target_table": (tbx, tby, tbz),
            "detour_waypoints": detour_waypoints,
            "arm_trajectory": arm_traj,
            "arm_end_points": arm_end_pts,
            "arm_end_3d": arm_end_3d,
            "final_dof": (dof0, dof1),
            "base_start": (start_bx, start_by),
            "base_end": (final_bx, final_by)
        }
        self.draw_single_window()
        self.draw_xz_view()
        return {"status": stat, "obj_id": obj_id,
                "final_base_pos": [self.base_world_x, self.base_world_y],
                "final_arm_joint": [dof0, dof1]}

    def draw_single_window(self):
        data = self.last_task_data
        obs_list = data["obstacles"]
        tgt = data["target_table"]
        detour_wp = data["detour_waypoints"]
        arm_end_pts = data["arm_end_points"]
        final_dof = data["final_dof"]
        table_w = self.table_length
        table_h = self.table_width

        win = tk.Toplevel()
        win.title("Top‑Down XY View (Auto‑Replan + Auto‑Arm‑Select)")
        c = tk.Canvas(win, bg="white", width=1400, height=950)
        c.pack(padx=12, pady=12)

        scale = 0.52
        off_x = 110
        off_y = 720

        def to_canvas(px, py):
            cx = off_x + px * scale
            cy = off_y - py * scale
            return cx, cy

        axis_extend_x = 400
        axis_extend_y_up = 200
        axis_extend_y_down = 900
        x_axis_min = -axis_extend_x
        x_axis_max = table_w + axis_extend_x
        y_axis_min = -axis_extend_y_down
        y_axis_max = table_h + axis_extend_y_up

        ax0, ay0 = to_canvas(x_axis_min, 0)
        ax1, ay1 = to_canvas(x_axis_max, 0)
        c.create_line(ax0, ay0, ax1, ay1, fill="blue", width=2, arrow=tk.LAST)
        c.create_text(ax1 + 18, ay0 + 14, text="X (mm)", fill="blue", font=("Arial", 11, "bold"))

        bx0, by0 = to_canvas(0, y_axis_min)
        bx1, by1 = to_canvas(0, y_axis_max)
        c.create_line(bx0, by0, bx1, by1, fill="red", width=2, arrow=tk.LAST)
        c.create_text(bx0 - 35, by1 - 14, text="Y (mm)", fill="red", font=("Arial", 11, "bold"))

        tx1, ty1 = to_canvas(0, 0)
        tx2, ty2 = to_canvas(table_w, table_h)
        c.create_rectangle(tx1, ty1, tx2, ty2, outline="black", width=3)
        c.create_text((tx1 + tx2) / 2, ty1 + 24,
                      text=f"Table: Length={table_w:.0f} mm | Width={table_h:.0f} mm",
                      font=("Arial",10,"bold"))

        for obs in obs_list:
            x1c, y1c = to_canvas(obs.x1, obs.y1)
            x2c, y2c = to_canvas(obs.x2, obs.y2)
            c.create_rectangle(x1c, y1c, x2c, y2c, fill="#ffeebb", outline="#882200", width=2)
            mx = (x1c + x2c) / 2
            my = (y1c + y2c) / 2
            c.create_text(mx, my, text=obs.get_label_text(), font=("Arial", 8, "bold"), fill="black")

        if len(detour_wp) >= 1:
            pts_canvas = []
            for (wx, wy) in detour_wp:
                ccx, ccy = to_canvas(wx, wy)
                pts_canvas.append((ccx, ccy))
            c.create_line(pts_canvas, fill="#002b80", width=3, dash=(6, 3))
            for idx, (ccx, ccy) in enumerate(pts_canvas):
                c.create_oval(ccx-5, ccy-5, ccx+5, ccy+5, fill="#002b80")
                c.create_text(ccx+12, ccy-10, text=f"WP{idx}",
                              font=("Arial",8,"bold"), fill="#002b80")
            end_x, end_y = pts_canvas[-1]
            c.create_text(end_x + 22, end_y - 28,
                          text="Chassis Path", fill="#002b80", font=("Arial", 10, "bold"))

        if len(arm_end_pts) > 1:
            arm_pts_canvas = []
            for (ex, ey) in arm_end_pts:
                ecx, ecy = to_canvas(ex, ey)
                arm_pts_canvas.append((ecx, ecy))
            c.create_line(arm_pts_canvas, fill="#00cccc", width=3)
            end_x, end_y = arm_pts_canvas[-1]
            c.create_text(end_x + 22, end_y - 22,
                          text="Arm‑End Trajectory(Top‑view XY proj.)", fill="#00cccc", font=("Arial", 10, "bold"))

        if tgt is not None:
            tgx, tgy, _ = tgt
            gx, gy = to_canvas(tgx, tgy)
            c.create_oval(gx-8, gy-8, gx+8, gy+8, outline="#dd2222", width=3)
            c.create_text(gx+14, gy, text=f"Target({tgx:.0f},{tgy:.0f})",
                          fill="#dd2222", font=("Arial",9,"bold"))

        base_cx, base_cy = to_canvas(self.base_world_x, self.base_world_y)
        c.create_oval(base_cx-10, base_cy-10, base_cx+10, base_cy+10, fill="black")
        c.create_text(base_cx, base_cy-16, text="Base", font=("Arial",9,"bold"), fill="black")

        if final_dof is not None:
            d0, d1 = final_dof
            d0r = math.radians(d0)
            d1r = math.radians(d1)
            j1x = self.base_world_x + self.L1 * math.cos(d0r)
            j1y = self.base_world_y + self.L1 * math.sin(d0r)
            j2x = j1x + self.L2 * math.cos(d0r + d1r)
            j2y = j1y + self.L2 * math.sin(d0r + d1r)
            j1c_x, j1c_y = to_canvas(j1x, j1y)
            j2c_x, j2c_y = to_canvas(j2x, j2y)
            c.create_line(base_cx, base_cy, j1c_x, j1c_y, fill="#227722", width=4)
            c.create_line(j1c_x, j1c_y, j2c_x, j2c_y, fill="#cc4422", width=4)
            c.create_oval(j1c_x-6, j1c_y-6, j1c_x+6, j1c_y+6, fill="#227722")
            c.create_oval(j2c_x-8, j2c_y-8, j2c_x+8, j2c_y+8, fill="#cc4422")
            c.create_text(j2c_x+14, j2c_y, text="End‑effector",
                          fill="#cc4422", font=("Arial",9,"bold"))

        c.create_text(700, 24, text=f"L1={self.L1:.0f} mm  L2={self.L2:.0f} mm | 3D‑Z contour‑follow ON",
                      font=("Arial",11,"bold"))

    def draw_xz_view(self):
        """新增主视图 X‑Z 侧视弹窗，观察Z高度与障碍物"""
        data = self.last_task_data
        obs_list = data["obstacles"]
        arm3d = data["arm_end_3d"]
        tgt = data["target_table"]

        win = tk.Toplevel()
        win.title("Side X‑Z Main View (Obstacle Height & Z trajectory)")
        c = tk.Canvas(win, bg="white", width=1300, height=750)
        c.pack(padx=10, pady=10)

        scale_xz = 0.45
        off_x = 80
        off_z = 620

        def to_xz(px, pz):
            cx = off_x + px * scale_xz
            cz = off_z - pz * scale_xz
            return cx, cz

        # 坐标轴
        x_min = -200
        x_max = 2100
        z_min = -50
        z_max = 450
        ax0, az0 = to_xz(x_min, 0)
        ax1, az1 = to_xz(x_max, 0)
        c.create_line(ax0, az0, ax1, az1, fill="black", width=2, arrow=tk.LAST)
        c.create_text(ax1+15, az0+12, text="X (mm)", font=("Arial",11,"bold"))

        zx0, zz0 = to_xz(0, z_min)
        zx1, zz1 = to_xz(0, z_max)
        c.create_line(zx0, zz0, zx1, zz1, fill="black", width=2, arrow=tk.LAST)
        c.create_text(zx0-35, zz0-12, text="Z (mm)", font=("Arial",11,"bold"))

        # 绘制所有障碍物：X范围简化投影为矩形（高度）
        for obs in obs_list:
            x1c, z1c = to_xz(obs.x1, 0)
            x2c, z2c = to_xz(obs.x2, obs.h)
            c.create_rectangle(x1c,z1c,x2c,z2c,fill="#ffddaa",outline="#aa3300",width=2)
            cx_txt = (x1c+x2c)/2
            cz_txt = (z1c+z2c)/2
            c.create_text(cx_txt, cz_txt, text=f"H={obs.h:.0f}",font=("Arial",8))

        # 绘制3D末端Z轨迹
        if len(arm3d) >1:
            pts = []
            for (ex, ey, ez) in arm3d:
                cx, cz = to_xz(ex, ez)
                pts.append((cx, cz))
            c.create_line(pts, fill="#00cccc", width=3)
            for (cx,cz) in pts:
                c.create_oval(cx-4,cz-4,cx+4,cz+4,fill="#00cccc")
            last_cx, last_cz = pts[-1]
            c.create_text(last_cx+15, last_cz-10, text="End‑effector Z‑Traj",fill="#00cccc",font=("Arial",10,"bold"))

        if tgt is not None:
            tx, ty, tz = tgt
            gx, gz = to_xz(tx, tz)
            c.create_oval(gx-7,gz-7,gx+7,gz+7,outline="#dd2222",width=3)
            c.create_text(gx+12,gz,text=f"Target Z={tz:.0f}",fill="#dd2222",font=("Arial",9))

        c.create_text(600,22,text="X‑Z Side View: Cyan = end‑effector dynamic Z trajectory; orange boxes = obstacle height",font=("Arial",10))


def main():
    try:
        mm = MobileManipulatorPrototype()
        mm.print_obstacle_info()
        print("==== IC382 Mobile‑Manipulator Prototype | 3D‑Z Contour‑Follow + X‑Z Side View ====")
        print("1: Input target TABLE‑FRAME(X,Y,Z mm) → auto‑arm‑select + auto‑replan + 2‑GUI windows")
        print("2: Modify robot parameters(arm link length / joint limits)")
        print("3: Modify rectangular table length & width")
        print("q: quit")
        while True:
            sel = input("\nPlease select mode (1/2/3/q):")
            if sel == "q":
                break
            elif sel == "1":
                try:
                    tx = float(input("Input target TABLE‑FRAME X(mm):"))
                    ty = float(input("Input target TABLE‑FRAME Y(mm):"))
                    tz = float(input("Input target TABLE‑FRAME Z(mm):"))
                    oid = input("Object ID(enter default):") or "obj_test"
                    mm.execute_mobile_manip_task(tx, ty, tz, oid)
                except ValueError:
                    print("Invalid numeric input")
            elif sel == "2":
                print("\nModify arm physical parameters")
                l1_in = input(f"L1 first link mm(current {mm.L1}):")
                l2_in = input(f"L2 second link mm(current {mm.L2}):")
                d0min_in = input(f"DOF0 min angle({mm.dof0_min}):")
                d0max_in = input(f"DOF0 max angle({mm.dof0_max}):")
                d1min_in = input(f"DOF1 min angle({mm.dof1_min}):")
                d1max_in = input(f"DOF1 max angle({mm.dof1_max}):")
                if l1_in.strip() != "" and l2_in.strip() != "":
                    mm.update_arm_link(float(l1_in), float(l2_in))
                if d0min_in.strip() != "" and d0max_in.strip() != "":
                    mm.dof0_min = float(d0min_in)
                    mm.dof0_max = float(d0max_in)
                if d1min_in.strip() != "" and d1max_in.strip() != "":
                    mm.dof1_min = float(d1min_in)
                    mm.dof1_max = float(d1max_in)
                print(f"Joint limits updated: DOF0 {mm.dof0_min}~{mm.dof0_max}°, DOF1 {mm.dof1_min}~{mm.dof1_max}°")
            elif sel == "3":
                tl_in = input(f"Table length X mm(current {mm.table_length}):")
                tw_in = input(f"Table width Y mm(current {mm.table_width}):")
                tl = mm.table_length
                tw = mm.table_width
                if tl_in.strip() != "":
                    tl = float(tl_in)
                if tw_in.strip() != "":
                    tw = float(tw_in)
                mm.update_table_geometry(tl, tw)
            else:
                print("Invalid option")
    except Exception as e:
        import traceback
        print("\n===== PROGRAM CRASH ERROR =====")
        print(traceback.format_exc())


if __name__ == "__main__":
    main()
