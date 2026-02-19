def control_loop(self):
    if not self.odom_received or not self.running:
        return

    # Initialize start_time for circle mode timing
    if self.start_time is None:
        self.start_time = time.time()

    # -----------------------
    # MODE 1: Go to (x, y)
    # -----------------------
    if self.mode == 'goto':
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.hypot(dx, dy)

        # Stop condition
        if dist < self.goal_tol:
            self.stop_robot()
            self.mode = 'idle'
            self.get_logger().info("Goal reached.")
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = self.wrap_to_pi(desired_heading - self.current_theta)

        # Simple proportional go-to-goal controller
        kp_lin = float(self.kp)
        kp_ang = float(2.0 * self.kp)

        # Reduce forward motion if you're not facing the goal
        if abs(heading_error) > (np.pi / 2.0):
            v = 0.0
        else:
            v = kp_lin * dist

        omega = kp_ang * heading_error

        # Limits
        v = float(np.clip(v, -self.max_v, self.max_v))
        omega = float(np.clip(omega, -self.max_omega, self.max_omega))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)

        # Optional logging data (treat goal as "ref")
        t = time.time() - self.start_time
        self.data['time'].append(t)
        self.data['ref_x'].append(self.goal_x)
        self.data['ref_y'].append(self.goal_y)
        self.data['actual_x'].append(self.current_x)
        self.data['actual_y'].append(self.current_y)
        self.data['error_x'].append(dx)
        self.data['error_y'].append(dy)
        return

    # -----------------------
    # MODE 2: Circle tracking
    # -----------------------
    t = time.time() - self.start_time
    if t > self.duration:
        self.running = False
        self.stop_robot()
        self.save_results()
        return

    ref_x, ref_y, ref_vx, ref_vy = self.get_reference_trajectory(t)

    error_x = ref_x - self.current_x
    error_y = ref_y - self.current_y

    vx_des = self.kp * error_x + ref_vx
    vy_des = self.kp * error_y + ref_vy

    theta = self.current_theta
    v = vx_des * math.cos(theta) + vy_des * math.sin(theta)

    desired_heading = math.atan2(vy_des, vx_des)
    heading_error = self.wrap_to_pi(desired_heading - theta)
    omega = 2.0 * self.kp * heading_error

    v = float(np.clip(v, -self.max_v, self.max_v))
    omega = float(np.clip(omega, -self.max_omega, self.max_omega))

    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = omega
    self.cmd_vel_pub.publish(cmd)

    # Store data
    self.data['time'].append(t)
    self.data['ref_x'].append(ref_x)
    self.data['ref_y'].append(ref_y)
    self.data['actual_x'].append(self.current_x)
    self.data['actual_y'].append(self.current_y)
    self.data['error_x'].append(error_x)
    self.data['error_y'].append(error_y)
