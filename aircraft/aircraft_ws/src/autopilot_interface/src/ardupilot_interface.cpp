#include "ardupilot_interface.hpp"

ArdupilotInterface::ArdupilotInterface() : Node("ardupilot_interface"),
    active_srv_or_act_flag_(false), aircraft_fsm_state_(ArdupilotInterfaceState::STARTED), 
    offboard_flag_frequency(10), offboard_flag_count_(0), last_offboard_flag_count_(0),
    target_system_id_(-1), mav_state_(-1), mav_type_(-1),
    armed_flag_(false), ardupilot_mode_(""),
    lat_(NAN), lon_(NAN), alt_(NAN), alt_ellipsoid_(NAN),
    x_(NAN), y_(NAN), z_(NAN),  vx_(NAN), vy_(NAN), vz_(NAN), ref_lat_(NAN), ref_lon_(NAN), ref_alt_(NAN),
    true_airspeed_m_s_(NAN), heading_(NAN),
    home_lat_(NAN), home_lon_(NAN), home_alt_(NAN)
{
    RCLCPP_INFO(this->get_logger(), "ArduPilot interfacing!");
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    // Check and log whether simulation time is enabled or not
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Simulation time is enabled.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Simulation time is disabled.");
    }
    last_offboard_flag_rate_check_time_ = this->get_clock()->now(); // Monitor the rate of offboard flag
    // Initialize the arrays
    position_.fill(NAN);
    q_.fill(NAN);
    velocity_.fill(NAN);
    angular_velocity_.fill(NAN);

    // MAVROS Publishers
    rclcpp::QoS qos_profile_pub(10);  // Depth of 10
    qos_profile_pub.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Or rclcpp::DurabilityPolicy::Volatile
    setpoint_pos_pub_= this->create_publisher<GeoPoseStamped>("mavros/setpoint_position/global", qos_profile_pub);

    // Offboard flag publisher
    offboard_flag_pub_ = this->create_publisher<autopilot_interface_msgs::msg::OffboardFlag>("offboard_flag", qos_profile_pub);

    // Create callback groups (Reentrant or MutuallyExclusive)
    callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Timed callbacks in parallel
    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Listen to subscribers in parallel
    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Services are parallel but refused if active_srv_or_act_flag_ is true
    callback_group_action_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Actions are parallel but refused if active_srv_or_act_flag_ is true

    // Timers
    ardupilot_interface_printout_timer_ = this->create_wall_timer(
        3s, // Timer period of 3 seconds
        std::bind(&ArdupilotInterface::ardupilot_interface_printout_callback, this),
        callback_group_timer_
    );
    offboard_flag_timer_ = this->create_wall_timer(
        std::chrono::nanoseconds(1000000000 / offboard_flag_frequency),
        std::bind(&ArdupilotInterface::offboard_flag_callback, this),
        callback_group_timer_
    );

    // Subscribers configuration
    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.callback_group = callback_group_subscriber_;
    rclcpp::QoS qos_profile_sub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile_sub.keep_last(10);  // History: KEEP_LAST with depth 10
    qos_profile_sub.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // MAVROS subscribers
    mavros_global_position_global_sub_= this->create_subscription<NavSatFix>(
        "mavros/global_position/global", qos_profile_sub, // 4Hz
        std::bind(&ArdupilotInterface::global_position_global_sub_callback, this, std::placeholders::_1), subscriber_options);
    mavros_local_position_odom_sub_= this->create_subscription<Odometry>(
        "mavros/local_position/odom", qos_profile_sub, // 4Hz
        std::bind(&ArdupilotInterface::local_position_odom_callback, this, std::placeholders::_1), subscriber_options);
    mavros_global_position_local_sub_ = this->create_subscription<Odometry>(
        "mavros/global_position/local", qos_profile_sub, // 4Hz
        std::bind(&ArdupilotInterface::global_position_local_callback, this, std::placeholders::_1), subscriber_options);
    mavros_vfr_hud_sub_ = this->create_subscription<VfrHud>(
        "mavros/vfr_hud", qos_profile_sub, // 4Hz
        std::bind(&ArdupilotInterface::vfr_hud_callback, this, std::placeholders::_1), subscriber_options);
    mavros_home_position_home_sub_ = this->create_subscription<HomePosition>(
        "mavros/home_position/home", qos_profile_sub, // 1Hz
        std::bind(&ArdupilotInterface::home_position_home_callback, this, std::placeholders::_1), subscriber_options);
    mavros_state_sub_ = this->create_subscription<State>(
        "mavros/state", qos_profile_sub, //1Hz
        std::bind(&ArdupilotInterface::state_callback, this, std::placeholders::_1), subscriber_options);

    // MAVROS service clients
    vehicle_info_client_ = this->create_client<VehicleInfoGet>("mavros/vehicle_info_get");
    arming_client_ = this->create_client<CommandBool>("mavros/cmd/arming");
    command_long_client_ = this->create_client<CommandLong>("mavros/cmd/command");
    takeoff_client_ = this->create_client<CommandTOL>("mavros/cmd/takeoff");
    landing_client_ = this->create_client<CommandTOL>("mavros/cmd/land");
    set_param_client_ = this->create_client<ParamSetV2>("mavros/param/set");
    set_mode_client_ = this->create_client<SetMode>("mavros/set_mode");
    wp_push_client_ = this->create_client<WaypointPush>("mavros/mission/push");
    set_wp_client_ = this->create_client<WaypointSetCurrent>("mavros/mission/set_current");

    // Services
    set_speed_service_ = this->create_service<autopilot_interface_msgs::srv::SetSpeed>(
        "set_speed", std::bind(&ArdupilotInterface::set_speed_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_);
    set_reposition_service_ = this->create_service<autopilot_interface_msgs::srv::SetReposition>(
        "set_reposition", std::bind(&ArdupilotInterface::set_reposition_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_);

    // Actions
    land_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Land>(this, "land_action",
            std::bind(&ArdupilotInterface::land_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArdupilotInterface::land_handle_cancel, this, std::placeholders::_1),
            std::bind(&ArdupilotInterface::land_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    takeoff_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Takeoff>(this, "takeoff_action",
            std::bind(&ArdupilotInterface::takeoff_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArdupilotInterface::takeoff_handle_cancel, this, std::placeholders::_1),
            std::bind(&ArdupilotInterface::takeoff_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    orbit_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Orbit>(this, "orbit_action",
            std::bind(&ArdupilotInterface::orbit_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArdupilotInterface::orbit_handle_cancel, this, std::placeholders::_1),
            std::bind(&ArdupilotInterface::orbit_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    offboard_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Offboard>(this, "offboard_action",
            std::bind(&ArdupilotInterface::offboard_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArdupilotInterface::offboard_handle_cancel, this, std::placeholders::_1),
            std::bind(&ArdupilotInterface::offboard_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
}

// Callbacks for subscribers (reentrant group)
void ArdupilotInterface::global_position_global_sub_callback(const NavSatFix::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    lat_ = msg->latitude;
    lon_ = msg->longitude;
    alt_ellipsoid_ = msg->altitude; // Positive is above the WGS 84 ellipsoid
}
void ArdupilotInterface::local_position_odom_callback(const Odometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    position_[0] = msg->pose.pose.position.x; // ENU
    position_[1] = msg->pose.pose.position.y;
    position_[2] = msg->pose.pose.position.z;
    q_[0] = msg->pose.pose.orientation.w;
    q_[1] = msg->pose.pose.orientation.x;
    q_[2] = msg->pose.pose.orientation.y;
    q_[3] = msg->pose.pose.orientation.z;
    velocity_[0] = msg->twist.twist.linear.x; // Body frame
    velocity_[1] = msg->twist.twist.linear.y;
    velocity_[2] = msg->twist.twist.linear.z;
    angular_velocity_[0] = msg->twist.twist.angular.x; // TODO: double check
    angular_velocity_[1] = msg->twist.twist.angular.y;
    angular_velocity_[2] = msg->twist.twist.angular.z;
    // See also topics /mavros/local_position/velocity_body, /mavros/local_position/velocity_local
}
void ArdupilotInterface::global_position_local_callback(const Odometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    // Position (ENU -> NED)
    x_ = msg->pose.pose.position.y;  // N <- E
    y_ = msg->pose.pose.position.x;  // E <- N
    z_ = -msg->pose.pose.position.z; // D <- -U
    // Velocity (ENU -> NED)
    vx_ = msg->twist.twist.linear.y;  // N <- E
    vy_ = msg->twist.twist.linear.x;  // E <- N
    vz_ = -msg->twist.twist.linear.z; // D <- -U
}
void ArdupilotInterface::vfr_hud_callback(const VfrHud::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    alt_ = msg->altitude; // MSL
    heading_ = msg->heading; // degrees 0..360, also in /mavros/global_position/compass_hdg
    true_airspeed_m_s_ = msg->airspeed; // m/s
}
void ArdupilotInterface::home_position_home_callback(const HomePosition::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    ref_lat_ = msg->geo.latitude; // geodetic coordinates in WGS-84 datum
    ref_lon_ = msg->geo.longitude; // geodetic coordinates in WGS-84 datum
    ref_alt_ = msg->geo.altitude; // ellipsoid altitude
}
void ArdupilotInterface::state_callback(const State::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    //add string for msg->mode, remove in_transition_mode_; in_transition_to_fw_
    armed_flag_ = msg->armed;
    mav_state_ = msg->system_status; // MAV_STATE: MAV_STATE_CALIBRATING = 2, MAV_STATE_STANDBY = 3, MAV_STATE_ACTIVE = 4 (0: unkown, 5,6: failsafe, 8: flight termination, 1,7: boot)
    ardupilot_mode_ = msg->mode; // See https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/State.msg
    if ((aircraft_fsm_state_ == ArdupilotInterfaceState::LANDED) && (mav_state_ == 3)) {
        aircraft_fsm_state_ = ArdupilotInterfaceState::STARTED; // Reset ArduPilot interface state when in standby after landing
    }
}

// Callbacks for timers (reentrant group)
void ArdupilotInterface::ardupilot_interface_printout_callback()
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads

    // Once the vehicle is in standby, retrieve for SYSID_THISMAV and MAV_TYPE if they are not already set
    if ((mav_state_ == 3) && ((target_system_id_ == -1) || (mav_type_ == -1))) {
        auto request = std::make_shared<VehicleInfoGet::Request>();
        vehicle_info_client_->async_send_request(request,
            [this](rclcpp::Client<VehicleInfoGet>::SharedFuture future) {
                auto response = future.get();
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                target_system_id_ = response->vehicles[0].sysid; // SYSID_THISMAV
                mav_type_ = response->vehicles[0].type; // MAV_TYPE (1: fixed-wing/vtol, 2: quad)
            });
    }

    auto now = this->get_clock()->now();
    double elapsed_sec = (now - last_offboard_flag_rate_check_time_).seconds();
    double actual_rate = NAN;
    if (elapsed_sec > 0) {
        actual_rate = (offboard_flag_count_ - last_offboard_flag_count_) / elapsed_sec;
    }
    last_offboard_flag_count_.store(offboard_flag_count_.load());
    last_offboard_flag_rate_check_time_ = now;
    RCLCPP_INFO(get_logger(),
                "Vehicle status:\n"
                "  target_system_id: %d\n"
                "  mav_type (1: fw/vtol, 2: quad): %d\n"
                "  mav_state (3: standby, 4: active): %d\n"
                "  armed: %s\n"
                "  ardupilot flight mode: %s\n"
                "Global position:\n"
                "  lat: %.5f, lon: %.5f,\n"
                "  alt AMSL: %.2f, alt ell: %.2f\n"
                "Local position:\n"
                "  NED pos: %.2f %.2f %.2f\n"
                "  NED vel: %.2f %.2f %.2f\n"
                "  heading: %.2f\n"
                "  ref lat: %.5f, lon: %.5f, alt ell: %.2f\n"
                "Odometry:\n"
                "  ENU position: %.2f %.2f %.2f\n"
                "  quaternion: %.2f %.2f %.2f %.2f\n"
                "  frame vel: %.2f %.2f %.2f\n"
                "  angular_vel: %.2f %.2f %.2f\n"
                "Airspeed:\n"
                "  true_airspeed_m_s: %.2f\n"
                "Current node time:\n"
                "  %.2f seconds\n"
                "Current FSM State:\n"
                "  %s\n"
                "Offboard flag rate:\n"
                "  %.2f Hz\n\n",
                //
                target_system_id_, mav_type_, mav_state_,
                (armed_flag_ ? "true" : "false"),
                ardupilot_mode_.c_str(),
                lat_, lon_, alt_, alt_ellipsoid_,
                x_, y_, z_,
                vx_, vy_, vz_,
                heading_,
                ref_lat_, ref_lon_, ref_alt_,
                position_[0], position_[1], position_[2],
                q_[0], q_[1], q_[2], q_[3],
                velocity_[0], velocity_[1], velocity_[2],
                angular_velocity_[0] * 180.0 / M_PI, angular_velocity_[1] * 180.0 / M_PI, angular_velocity_[2] * 180.0 / M_PI,
                true_airspeed_m_s_,
                this->get_clock()->now().seconds(),
                fsm_state_to_string(aircraft_fsm_state_).c_str(),
                actual_rate
            );
}
void ArdupilotInterface::offboard_flag_callback()
{
    offboard_flag_count_++; // Counter to monitor the rate of the offboard flag (no lock, atomic variable)

    auto msg = autopilot_interface_msgs::msg::OffboardFlag();
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    if (aircraft_fsm_state_ == ArdupilotInterfaceState::OFFBOARD_VELOCITY) {
        msg.offboard_flag = 7;
    } else if (aircraft_fsm_state_ == ArdupilotInterfaceState::OFFBOARD_ACCELERATION) {
        msg.offboard_flag = 8;
    } else {
        msg.offboard_flag = 0; // Inactive
    }
    offboard_flag_pub_->publish(msg);
}

// Callbacks for non-blocking services (reentrant callback group, active_srv_or_act_flag_ acting as semaphore)
void ArdupilotInterface::set_speed_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Request> request,
                        std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Response> response)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    if (((mav_type_ == 2) && aircraft_fsm_state_ != ArdupilotInterfaceState::MC_HOVER) || ((mav_type_ == 1) && aircraft_fsm_state_ != ArdupilotInterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Set speed rejected, ArdupilotInterface is not in hover/cruise state");
        response->success = false;
        return;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        response->success = false;
        return;
    }
    auto command_request = std::make_shared<CommandLong::Request>();
    command_request->command = 178; // MAV_CMD_DO_CHANGE_SPEED
    if (mav_type_ == 1) { // Fixed-wing/VTOL
        command_request->param1 = 0.0; // Airspeed
    } else if (mav_type_ == 2) { // Multicopter
        command_request->param1 = 1.0; // Ground Speed
    }
    command_request->param2 = request->speed; // The desired speed in m/s
    command_long_client_->async_send_request(command_request, // Asynchronously send the service request
        [this](rclcpp::Client<CommandLong>::SharedFuture future) {
                auto result = future.get(); // Check result asynchronously
                RCLCPP_INFO(this->get_logger(), "set_speed sent, success: %s, result: %d", result->success ? "true" : "false", result->result);
        });
    response->success = true;
    active_srv_or_act_flag_.store(false);
}
void ArdupilotInterface::set_reposition_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Request> request,
                        std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Response> response)
{
    {
        std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
        if ((mav_type_ == 1) || ((mav_type_ == 2) && !(aircraft_fsm_state_ == ArdupilotInterfaceState::MC_HOVER || aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT))) {
            RCLCPP_ERROR(this->get_logger(), "Set reposition rejected, ArdupilotInterface is not in a quad hover/orbit state (for VTOLs, use /orbit_action)");
            response->success = false;
            return;
        }
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        response->success = false;
        return;
    }

    // Change mode to GUIDED if in CIRCLE mode from a orbit
    while (aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT) { // TODO: lock variable read
        auto set_mode_request = std::make_shared<SetMode::Request>();
        set_mode_request->custom_mode = "GUIDED";
        set_mode_client_->async_send_request(set_mode_request,
            [this](rclcpp::Client<SetMode>::SharedFuture future) {
                if (future.get()->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "Request mode success");
                    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                    aircraft_fsm_state_ = ArdupilotInterfaceState::MC_HOVER;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Request mode failed");
                }
            });
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Add a small delay between service requests
    }

    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    double desired_east = request->east;
    double desired_north = request->north;
    double desired_alt = request->altitude;
    RCLCPP_INFO(this->get_logger(), "New requested reposition East-North %.2f %.2f Alt. %.2f", desired_east, desired_north, desired_alt);
    auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);
    double distance, heading;
    geod.Inverse(lat_, lon_, des_lat, des_lon, distance, heading);
    double yaw_enu_deg = 90.0 - heading;
    if (yaw_enu_deg > 180.0) {
        yaw_enu_deg -= 360.0;
    } else if (yaw_enu_deg < -180.0) {
        yaw_enu_deg += 360.0;
    }
    double yaw_enu_rad = yaw_enu_deg * M_PI / 180.0;
    for (int i = 0; i < 3; ++i) { // HARDCODED: send N times for robustness
        auto msg = geographic_msgs::msg::GeoPoseStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.position.latitude = des_lat;
        msg.pose.position.longitude = des_lon;
        msg.pose.position.altitude = home_alt_ + desired_alt; // Altitude in MSL/relative to takeoff
        msg.pose.orientation.w = cos(yaw_enu_rad / 2.0);
        msg.pose.orientation.z = sin(yaw_enu_rad / 2.0);
        setpoint_pos_pub_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Add a small delay between publishes
    }
    response->success = true;
    active_srv_or_act_flag_.store(false);
}

// Callbacks for actions (reentrant callback group, to be able to handle_goal and handle_cancel at the same time)
rclcpp_action::GoalResponse ArdupilotInterface::land_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Land::Goal> goal)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    RCLCPP_INFO(this->get_logger(), "land_handle_goal");
    if (((mav_type_ == 2) && !(aircraft_fsm_state_ == ArdupilotInterfaceState::MC_HOVER || aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT)) || ((mav_type_ == 1) && aircraft_fsm_state_ != ArdupilotInterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Landing rejected, ArdupilotInterface is not in hover/orbit/cruise state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ArdupilotInterface::land_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "land_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ArdupilotInterface::land_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "land_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Land::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Land::Feedback>();

    double landing_altitude = goal->landing_altitude;
    double vtol_transition_heading = goal->vtol_transition_heading;

    double pre_landing_loiter_distance = 300.0; // HARDCODED
    double pre_landing_loiter_radius = 150.0; // HARDCODED
    double angle_correction_deg = atan(pre_landing_loiter_radius/pre_landing_loiter_distance) * 180.0 / M_PI;
    auto [exit_lat, exit_lon] = lat_lon_from_polar(home_lat_, home_lon_, pre_landing_loiter_distance, vtol_transition_heading + 180.0);

    bool landing = true;
    uint64_t time_of_last_srv_req_us_ = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    ArdupilotInterfaceState current_fsm_state;
    rclcpp::Rate landing_loop_rate(100);
    while (landing) {
        landing_loop_rate.sleep();

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_ORBIT or FW_CRUISE
            feedback->message = "Canceling landing";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Landing canceled");
            return;
        }

        {
            std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
            current_fsm_state = aircraft_fsm_state_;
        }
        uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds

        if (mav_type_ == 2) { // Multicopter
            if (((current_fsm_state == ArdupilotInterfaceState::MC_HOVER) || (current_fsm_state == ArdupilotInterfaceState::MC_ORBIT)) 
                    && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_param_request = std::make_shared<ParamSetV2::Request>();
                set_param_request->param_id = "RTL_ALT"; // This is ineffective is the vehicle is already above this altitude
                set_param_request->value.type = 2; // Integer
                set_param_request->value.integer_value = static_cast<int64_t>(landing_altitude*100); // cm
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<ParamSetV2, autopilot_interface_msgs::action::Land>(
                    set_param_client_, set_param_request, goal_handle, 
                    "Request param set", ArdupilotInterfaceState::MC_RTL_PARAM_SET);
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_RTL_PARAM_SET) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "RTL";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Land>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::MC_RTL);
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_RTL) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                double distance_from_home_in_meters;
                geod.Inverse(lat_, lon_, home_lat_, home_lon_, distance_from_home_in_meters);
                if (distance_from_home_in_meters < 5.0) { // HARDCODED: 5m distance threshold
                    auto set_mode_request = std::make_shared<SetMode::Request>();
                    set_mode_request->custom_mode = "GUIDED";
                    time_of_last_srv_req_us_ = current_time_us;
                    call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Land>(
                        set_mode_client_, set_mode_request, goal_handle, 
                        "Request mode", ArdupilotInterfaceState::MC_RETURNED_READY_TO_LAND);
                }
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_RETURNED_READY_TO_LAND) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto landing_request = std::make_shared<CommandTOL::Request>();
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandTOL, autopilot_interface_msgs::action::Land>(
                    landing_client_, landing_request, goal_handle, 
                    "Request landing", ArdupilotInterfaceState::MC_LANDING);
            } else if (current_fsm_state == ArdupilotInterfaceState::MC_LANDING && (std::abs(alt_ - home_alt_) < 2.0)) { // HARDCODED: 2m altitude threshold
                feedback->message = "MC landing completed";
                goal_handle->publish_feedback(feedback);
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                aircraft_fsm_state_ = ArdupilotInterfaceState::LANDED;
                landing = false;
            }
        } else if (mav_type_ == 1) { // Fixed-wing/VTOL
            if ((current_fsm_state == ArdupilotInterfaceState::FW_CRUISE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto mission_request = std::make_shared<WaypointPush::Request>();
                mavros_msgs::msg::Waypoint wp1; // Create the first waypoint (dummy)
                wp1.frame = 3;
                wp1.command = 16; // NAV_WAYPOINT
                wp1.is_current = true;
                wp1.autocontinue = true;
                wp1.x_lat = 0.0;
                wp1.y_long = 0.0;
                wp1.z_alt = 0.0;
                mission_request->waypoints.push_back(wp1);
                mavros_msgs::msg::Waypoint wp2; // Create the second waypoint (return near home based on desired heading)
                auto [des_lat, des_lon] = lat_lon_from_polar(home_lat_, home_lon_, pre_landing_loiter_distance, vtol_transition_heading + 180.0 - angle_correction_deg);
                wp2.frame = 3;
                wp2.command = 16; // MAV_CMD_NAV_WAYPOINT
                wp2.is_current = false;
                wp2.autocontinue = true;
                wp2.x_lat = des_lat;
                wp2.y_long = des_lon;
                wp2.z_alt = 150.0; // HARDCODED: waypoint altitude
                mission_request->waypoints.push_back(wp2);
                mavros_msgs::msg::Waypoint wp3; // Create the third waypoint (loiter descend)
                wp3.frame = 3;
                wp3.command = 17; // NAV_LOITER_UNLIM
                wp3.is_current = false;
                wp3.autocontinue = true;
                wp3.param3 = pre_landing_loiter_radius;
                wp3.x_lat = des_lat;
                wp3.y_long = des_lon;
                wp3.z_alt = landing_altitude;
                mission_request->waypoints.push_back(wp3);
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<WaypointPush, autopilot_interface_msgs::action::Land>(
                    wp_push_client_, mission_request, goal_handle, 
                    "Request mission upload", ArdupilotInterfaceState::VTOL_LANDING_MISSION_UPLOADED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_LANDING_MISSION_UPLOADED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "AUTO";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Land>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::VTOL_LANDING_MISSION_MODE);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_LANDING_MISSION_MODE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto command_request = std::make_shared<CommandLong::Request>();
                command_request->command = 300; // MAV_CMD_MISSION_START
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandLong, autopilot_interface_msgs::action::Land>(
                    command_long_client_, command_request, goal_handle, 
                    "Request mission start", ArdupilotInterfaceState::VTOL_LANDING_MISSION_STARTED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_LANDING_MISSION_STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_current_request = std::make_shared<WaypointSetCurrent::Request>();
                set_current_request->wp_seq = 1;
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<WaypointSetCurrent, autopilot_interface_msgs::action::Land>(
                    set_wp_client_, set_current_request, goal_handle, 
                    "Request to set current waypoint ", ArdupilotInterfaceState::VTOL_LANDING_READY_FOR_QRTL);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_LANDING_READY_FOR_QRTL) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                double distance_from_exit_in_meters;
                geod.Inverse(lat_, lon_, exit_lat, exit_lon, distance_from_exit_in_meters);
                if ((distance_from_exit_in_meters < 30.0) && (std::abs(alt_ - (home_alt_ + landing_altitude)) < 10.0)
                        && (std::abs(heading_ - vtol_transition_heading) < 10.0)) { // HARDCODED: thresholds of 30m xy, 10m z, 10deg heading to start QRTL
                    auto set_param_request = std::make_shared<ParamSetV2::Request>();
                    set_param_request->param_id = "Q_RTL_ALT";
                    set_param_request->value.type = 2; // Integer
                    set_param_request->value.integer_value = static_cast<int64_t>(landing_altitude);
                    time_of_last_srv_req_us_ = current_time_us;
                    call_service_and_update_fsm<ParamSetV2, autopilot_interface_msgs::action::Land>(
                        set_param_client_, set_param_request, goal_handle, 
                        "Request param set", ArdupilotInterfaceState::VTOL_QRTL_PARAM_SET);
                }
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_QRTL_PARAM_SET) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "QRTL";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Land>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::VTOL_QRTL);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_QRTL) && (std::abs(alt_ - home_alt_) < 2.0)) { // HARDCODED: 2m altitude threshold
                feedback->message = "VTOL QRTL completed";
                goal_handle->publish_feedback(feedback);
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                aircraft_fsm_state_ = ArdupilotInterfaceState::LANDED;
                landing = false;
            }
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse ArdupilotInterface::offboard_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Offboard::Goal> goal)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    RCLCPP_INFO(this->get_logger(), "offboard_handle_goal");
    if (mav_type_ == 1) {
        RCLCPP_ERROR(this->get_logger(), "Offboard (GUIDED mode) rejected, not supported for ArduPlane VTOLs");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if ((mav_type_ == 2) && !(aircraft_fsm_state_ == ArdupilotInterfaceState::MC_HOVER || aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT)) {
        RCLCPP_ERROR(this->get_logger(), "Offboard (GUIDED mode) rejected, ArdupilotInterface is not in hover/orbit");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ArdupilotInterface::offboard_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "offboard_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ArdupilotInterface::offboard_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "offboard_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Offboard::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Offboard::Feedback>();

    int offboard_setpoint_type = goal->offboard_setpoint_type;
    double max_duration_sec = goal->max_duration_sec;

    offboard_flag_count_ = 0;
    bool offboarding = true;
    uint64_t time_of_offboard_start_us_ = -1;
    rclcpp::Rate offboard_loop_rate(100);
    while (offboarding) {
        offboard_loop_rate.sleep();

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_ORBIT or FW_CRUISE
            feedback->message = "Canceling offboard (GUIDED mode)";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Offboard (GUIDED mode) canceled");
            return;
        }

        uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
        if (time_of_offboard_start_us_ == -1) {
            // This is only sent once but it could be implemented with a FSM to verify the mode is changed
            auto set_mode_request = std::make_shared<SetMode::Request>();
            set_mode_request->custom_mode = "GUIDED";
            set_mode_client_->async_send_request(set_mode_request,
                [this](rclcpp::Client<SetMode>::SharedFuture future) {
                    if (future.get()->mode_sent) {
                        RCLCPP_INFO(this->get_logger(), "Request mode success");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Request mode failed");
                    }
                });
            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Reading data written by subs but also writing the FSM state
            if (offboard_setpoint_type == autopilot_interface_msgs::action::Offboard::Goal::VELOCITY) {
                aircraft_fsm_state_ = ArdupilotInterfaceState::OFFBOARD_VELOCITY;
                feedback->message = "Offboarding (GUIDED mode) with VELOCITY setpoints";
            } 
            else if (offboard_setpoint_type == autopilot_interface_msgs::action::Offboard::Goal::ACCELERATION) {
                aircraft_fsm_state_ = ArdupilotInterfaceState::OFFBOARD_ACCELERATION;
                feedback->message = "Offboarding (GUIDED mode) with ACCELERATION setpoints";
            }
            else {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_ERROR(this->get_logger(), "Offboard (GUIDED mode) type is not supported by ArdupilotInterface (only VELOCITY and ACCELERATION)");
                return;
            }
            goal_handle->publish_feedback(feedback);
            time_of_offboard_start_us_ = current_time_us;
            feedback->message = "Starting offboard control at t=" + std::to_string(time_of_offboard_start_us_) + " us";
            goal_handle->publish_feedback(feedback);
        }
        if (current_time_us >= (time_of_offboard_start_us_ + max_duration_sec * 1000000)) {
            time_of_offboard_start_us_ = -1;
            offboarding = false;
            // This is only sent once but it could be implemented with a FSM to verify the mode is changed
            auto set_mode_request = std::make_shared<SetMode::Request>();
            set_mode_request->custom_mode = "LOITER";
            set_mode_client_->async_send_request(set_mode_request,
                [this](rclcpp::Client<SetMode>::SharedFuture future) {
                    if (future.get()->mode_sent) {
                        RCLCPP_INFO(this->get_logger(), "Request mode success");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Request mode failed");
                    }
                });
            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Reading data written by subs but also writing the FSM state
            if (mav_type_ == 2) { // Multicopter
                aircraft_fsm_state_ = ArdupilotInterfaceState::MC_ORBIT; // This lets the reposition service change mode when called after offboard
            } else if (mav_type_ == 1) { // Fixed-wing/VTOL
                aircraft_fsm_state_ = ArdupilotInterfaceState::FW_CRUISE;
            }
            feedback->message = "Exiting offboard (GUIDED mode) control at t=" + std::to_string(current_time_us) + "us, entering LOITER mode";
            goal_handle->publish_feedback(feedback);
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse ArdupilotInterface::orbit_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Orbit::Goal> goal)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    RCLCPP_INFO(this->get_logger(), "orbit_handle_goal");
    if (((mav_type_ == 2) && !(aircraft_fsm_state_ == ArdupilotInterfaceState::MC_HOVER || aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT)) || ((mav_type_ == 1) && aircraft_fsm_state_ != ArdupilotInterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Landing rejected, ArdupilotInterface is not in hover/orbit/cruise state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ArdupilotInterface::orbit_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Orbit>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "orbit_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ArdupilotInterface::orbit_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Orbit>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "orbit_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Orbit::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Orbit::Feedback>();

    double desired_east = goal->east;
    double desired_north = goal->north;
    double desired_alt = goal->altitude;
    double desired_r = goal->radius;

    RCLCPP_INFO(this->get_logger(), "New requested orbit East-North %.2f %.2f Alt. %.2f Radius %.2f", desired_east, desired_north, desired_alt, desired_r);

    bool orbiting = true;
    uint64_t time_of_last_srv_req_us_ = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    ArdupilotInterfaceState current_fsm_state;
    rclcpp::Rate orbit_loop_rate(100);
    while (orbiting) {
        orbit_loop_rate.sleep();

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_ORBIT or FW_CRUISE
            feedback->message = "Canceling orbit";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Orbit canceled");
            return;
        }
        
        {
            std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
            current_fsm_state = aircraft_fsm_state_;
        }
        uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds

        if (mav_type_ == 2) { // Multicopter
            if (((current_fsm_state == ArdupilotInterfaceState::MC_HOVER) || (current_fsm_state == ArdupilotInterfaceState::MC_ORBIT)) 
                    && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_param_request = std::make_shared<ParamSetV2::Request>();
                set_param_request->param_id = "CIRCLE_RADIUS";
                set_param_request->value.type = 3; // Double
                set_param_request->value.double_value = desired_r * 100.0; // cm
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<ParamSetV2, autopilot_interface_msgs::action::Orbit>(
                    set_param_client_, set_param_request, goal_handle, 
                    "Request param1 set", ArdupilotInterfaceState::MC_ORBIT_PARAM1_SET);
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_ORBIT_PARAM1_SET) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_param_request = std::make_shared<ParamSetV2::Request>();
                set_param_request->param_id = "CIRCLE_RATE";
                set_param_request->value.type = 2; // Integer
                set_param_request->value.integer_value = static_cast<int64_t>(std::ceil((5.0 / desired_r) * (180.0 / M_PI))); // HARDCODED: ~5m/s orbit tangential speed for quads, because the parameter is an integer, actual speed will depend on radius
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<ParamSetV2, autopilot_interface_msgs::action::Orbit>(
                    set_param_client_, set_param_request, goal_handle, 
                    "Request param2 set", ArdupilotInterfaceState::MC_ORBIT_PARAM2_SET);
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_ORBIT_PARAM2_SET) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto mission_request = std::make_shared<WaypointPush::Request>();
                mavros_msgs::msg::Waypoint wp1; // Create the first waypoint (dummy)
                wp1.frame = 3;
                wp1.command = 16; // NAV_WAYPOINT
                wp1.is_current = true;
                wp1.autocontinue = true;
                wp1.x_lat = 0.0;
                wp1.y_long = 0.0;
                wp1.z_alt = 0.0;
                mission_request->waypoints.push_back(wp1);
                mavros_msgs::msg::Waypoint wp2; // Create the second waypoint on the orbit perimeter
                auto [center_lat, center_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);
                double distance_to_orbit, heading_to_orbit;
                geod.Inverse(center_lat, center_lon, lat_, lon_, distance_to_orbit, heading_to_orbit);
                auto [des_lat, des_lon] = lat_lon_from_polar(center_lat, center_lon, desired_r, heading_to_orbit);
                wp2.frame = 3;
                wp2.command = 16; // MAV_CMD_NAV_WAYPOINT
                wp2.is_current = false;
                wp2.autocontinue = true;
                wp2.x_lat = des_lat;
                wp2.y_long = des_lon;
                wp2.z_alt = desired_alt;          
                mission_request->waypoints.push_back(wp2);
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<WaypointPush, autopilot_interface_msgs::action::Orbit>(
                    wp_push_client_, mission_request, goal_handle, 
                    "Request mission upload", ArdupilotInterfaceState::MC_ORBIT_MISSION_UPLOADED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_ORBIT_MISSION_UPLOADED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "AUTO";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Orbit>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::MC_ORBIT_MISSION_MODE);
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_ORBIT_MISSION_MODE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto command_request = std::make_shared<CommandLong::Request>();
                command_request->command = 300; // MAV_CMD_MISSION_START
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandLong, autopilot_interface_msgs::action::Orbit>(
                    command_long_client_, command_request, goal_handle, 
                    "Request mission start", ArdupilotInterfaceState::MC_ORBIT_MISSION_STARTED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_ORBIT_MISSION_STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_current_request = std::make_shared<WaypointSetCurrent::Request>();
                set_current_request->wp_seq = 1;
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<WaypointSetCurrent, autopilot_interface_msgs::action::Orbit>(
                    set_wp_client_, set_current_request, goal_handle, 
                    "Requesting to set current waypoint", ArdupilotInterfaceState::MC_ORBIT_TRANSFER);
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_ORBIT_TRANSFER) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                double distance_from_orbit_center_in_meters;
                auto [center_lat, center_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);
                geod.Inverse(lat_, lon_, center_lat, center_lon, distance_from_orbit_center_in_meters);
                if ((std::abs(distance_from_orbit_center_in_meters - desired_r) < 1.0) && (std::abs(alt_ - (home_alt_ + desired_alt)) < 2.0)) { // HARDCODED: thresholds of 1m xy, 2m z
                    auto command_request = std::make_shared<CommandLong::Request>();
                    command_request->command = 195; // MAV_CMD_DO_SET_ROI_LOCATION
                    command_request->param5 = center_lat;
                    command_request->param6 = center_lon;
                    time_of_last_srv_req_us_ = current_time_us;
                    call_service_and_update_fsm<CommandLong, autopilot_interface_msgs::action::Orbit>(
                        command_long_client_, command_request, goal_handle, 
                        "Request ROI", ArdupilotInterfaceState::MC_ORBIT_ROI_SET);
                }
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_ORBIT_ROI_SET) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto [center_lat, center_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);
                double distance, required_heading_deg;
                geod.Inverse(lat_, lon_, center_lat, center_lon, distance, required_heading_deg);
                double normalized_current = fmod(heading_ + 360.0, 360.0);
                double normalized_required = fmod(required_heading_deg + 360.0, 360.0);
                double angle_diff = normalized_required - normalized_current;
                if (angle_diff > 180.0) {
                    angle_diff -= 360.0;
                } else if (angle_diff < -180.0) {
                    angle_diff += 360.0;
                }
                if (std::abs(angle_diff) >= 3.0) {
                    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                    aircraft_fsm_state_ = ArdupilotInterfaceState::MC_ORBIT_TRANSFER; // Step back to the previous service request until heading is reached
                } else if (std::abs(angle_diff) < 3.0) { // HARDCODED: threshold of 3deg on target heading
                    auto set_mode_request = std::make_shared<SetMode::Request>();
                    set_mode_request->custom_mode = "CIRCLE";
                    time_of_last_srv_req_us_ = current_time_us;
                    call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Orbit>(
                        set_mode_client_, set_mode_request, goal_handle, 
                        "Request mode", ArdupilotInterfaceState::MC_ORBIT_REACHED);
                }
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_ORBIT_REACHED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                feedback->message = "MC orbit completed";
                goal_handle->publish_feedback(feedback);
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                aircraft_fsm_state_ = ArdupilotInterfaceState::MC_ORBIT;
                orbiting = false;
            }
        } else if (mav_type_ == 1) { // Fixed-wing/VTOL
            if ((current_fsm_state == ArdupilotInterfaceState::FW_CRUISE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto mission_request = std::make_shared<WaypointPush::Request>();
                mavros_msgs::msg::Waypoint wp1; // Create the first waypoint (dummy)
                wp1.frame = 3;
                wp1.command = 16; // NAV_WAYPOINT
                wp1.is_current = true;
                wp1.autocontinue = true;
                wp1.x_lat = 0.0;
                wp1.y_long = 0.0;
                wp1.z_alt = 0.0;
                mission_request->waypoints.push_back(wp1);
                mavros_msgs::msg::Waypoint wp2; // Create the second waypoint (loiter)
                auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);
                wp2.frame = 3;
                wp2.command = 17; // NAV_LOITER_UNLIM
                wp2.is_current = false;
                wp2.autocontinue = true;
                wp2.param3 = desired_r; // TODO: radius
                wp2.x_lat = des_lat;
                wp2.y_long = des_lon;
                wp2.z_alt = desired_alt;
                mission_request->waypoints.push_back(wp2);
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<WaypointPush, autopilot_interface_msgs::action::Orbit>(
                    wp_push_client_, mission_request, goal_handle, 
                    "Request mission upload", ArdupilotInterfaceState::VTOL_ORBIT_MISSION_UPLOADED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_ORBIT_MISSION_UPLOADED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "AUTO";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Orbit>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::VTOL_ORBIT_MISSION_MODE);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_ORBIT_MISSION_MODE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto command_request = std::make_shared<CommandLong::Request>();
                command_request->command = 300; // MAV_CMD_MISSION_START
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandLong, autopilot_interface_msgs::action::Orbit>(
                    command_long_client_, command_request, goal_handle, 
                    "Request mission start", ArdupilotInterfaceState::VTOL_ORBIT_MISSION_STARTED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_ORBIT_MISSION_STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_current_request = std::make_shared<WaypointSetCurrent::Request>();
                set_current_request->wp_seq = 1;
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<WaypointSetCurrent, autopilot_interface_msgs::action::Orbit>(
                    set_wp_client_, set_current_request, goal_handle, 
                    "Request to set current waypoint", ArdupilotInterfaceState::VTOL_ORBIT_MISSION_COMPLETED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_ORBIT_MISSION_COMPLETED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                feedback->message = "VTOL orbit action completed";
                goal_handle->publish_feedback(feedback);
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                aircraft_fsm_state_ = ArdupilotInterfaceState::FW_CRUISE;
                orbiting = false;
            }
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse ArdupilotInterface::takeoff_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Takeoff::Goal> goal)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_goal");
    if (aircraft_fsm_state_ != ArdupilotInterfaceState::STARTED) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff rejected, ArdupilotInterface is not in STARTED state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (mav_state_ != 3) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff rejected, mav_state_ is not standby");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }    
    home_lat_ = lat_;
    home_lon_ = lon_;
    home_alt_ = alt_;
    RCLCPP_WARN(this->get_logger(), "Saved home_lat_: %.5f, home_lon_ %.5f, home_alt_ %.2f", home_lat_, home_lon_, home_alt_);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ArdupilotInterface::takeoff_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ArdupilotInterface::takeoff_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Takeoff::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Takeoff::Feedback>();

    double takeoff_altitude = goal->takeoff_altitude;
    // double vtol_transition_heading = goal->vtol_transition_heading; // Heading is handled by ArduPilot's Q_WVANE_ENABLE
    double vtol_loiter_nord = goal->vtol_loiter_nord;
    double vtol_loiter_east = goal->vtol_loiter_east;
    double vtol_loiter_alt = goal->vtol_loiter_alt;

    bool taking_off = true;
    uint64_t time_of_last_srv_req_us_ = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    ArdupilotInterfaceState current_fsm_state;
    rclcpp::Rate takeoff_loop_rate(100);
    while (taking_off) {
        takeoff_loop_rate.sleep();

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_ORBIT or FW_CRUISE
            feedback->message = "Canceling takeoff";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Takeoff canceled");
            return;
        }

        {
            std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
            current_fsm_state = aircraft_fsm_state_;
        }
        uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds

        if (mav_type_ == 2) { // Multicopter
            if ((current_fsm_state == ArdupilotInterfaceState::STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "GUIDED";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Takeoff>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::GUIDED_PRETAKEOFF);
            } else if ((current_fsm_state == ArdupilotInterfaceState::GUIDED_PRETAKEOFF) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto arm_request = std::make_shared<CommandBool::Request>();
                arm_request->value = true;
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandBool, autopilot_interface_msgs::action::Takeoff>(
                    arming_client_, arm_request, goal_handle, 
                    "Request arm", ArdupilotInterfaceState::ARMED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::ARMED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto takeoff_request = std::make_shared<CommandTOL::Request>();
                takeoff_request->altitude = takeoff_altitude;
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandTOL, autopilot_interface_msgs::action::Takeoff>(
                    takeoff_client_, takeoff_request, goal_handle, 
                    "Request takeoff", ArdupilotInterfaceState::MC_HOVER);
            } else if (current_fsm_state == ArdupilotInterfaceState::MC_HOVER) {
                if ((alt_ - home_alt_) > 0.9 * takeoff_altitude) { // HARDCODED: 90% threshold for takeoff altitude
                    feedback->message = "MC takeoff completed";
                    goal_handle->publish_feedback(feedback);
                    taking_off = false;
                }
            }
        } else if (mav_type_ == 1) { // Fixed-wing/VTOL
            if ((current_fsm_state == ArdupilotInterfaceState::STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "QLOITER";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Takeoff>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::VTOL_QLOITER_PRETAKEOFF);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_QLOITER_PRETAKEOFF) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto arm_request = std::make_shared<CommandBool::Request>();
                arm_request->value = true;
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandBool, autopilot_interface_msgs::action::Takeoff>(
                    arming_client_, arm_request, goal_handle, 
                    "Request arm", ArdupilotInterfaceState::ARMED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::ARMED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "GUIDED";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Takeoff>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::GUIDED_PRETAKEOFF);
            } else if ((current_fsm_state == ArdupilotInterfaceState::GUIDED_PRETAKEOFF) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto takeoff_request = std::make_shared<CommandTOL::Request>();
                takeoff_request->altitude = takeoff_altitude;
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandTOL, autopilot_interface_msgs::action::Takeoff>(
                    takeoff_client_, takeoff_request, goal_handle, 
                    "Request takeoff", ArdupilotInterfaceState::VTOL_TAKEOFF_MC);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_MC) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))
                        && (std::abs(alt_ - (home_alt_ + takeoff_altitude)) < 2.0)) { // HARDCODED: 2m altitude threshold
                // This block is not implemented because heading is handled by ArduPilot's Q_WVANE_ENABLE
                time_of_last_srv_req_us_ = current_time_us;
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_TAKEOFF_HEADING;
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_HEADING) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "CRUISE";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Takeoff>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::VTOL_TAKEOFF_TRANSITION);
            // HARDCODED: 10s wait in CRUISE mode before sending the VTOL takeoff loiter mission
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_TRANSITION) && (current_time_us > (time_of_last_srv_req_us_ + 10.0 * 1000000))) {
                auto mission_request = std::make_shared<WaypointPush::Request>();
                mavros_msgs::msg::Waypoint wp1; // Create the first waypoint (dummy)
                wp1.frame = 3;
                wp1.command = 16; // NAV_WAYPOINT
                wp1.is_current = true;
                wp1.autocontinue = true;
                wp1.x_lat = 0.0;
                wp1.y_long = 0.0;
                wp1.z_alt = 0.0;
                mission_request->waypoints.push_back(wp1);
                mavros_msgs::msg::Waypoint wp2; // Create the second waypoint (loiter)
                auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, vtol_loiter_east, vtol_loiter_nord);
                wp2.frame = 3;
                wp2.command = 17; // NAV_LOITER_UNLIM
                wp2.is_current = false;
                wp2.autocontinue = true;
                wp2.param3 = 200.0; // HARDCODED: 200m loiter radius
                wp2.x_lat = des_lat;
                wp2.y_long = des_lon;
                wp2.z_alt = vtol_loiter_alt;
                mission_request->waypoints.push_back(wp2);
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<WaypointPush, autopilot_interface_msgs::action::Takeoff>(
                    wp_push_client_, mission_request, goal_handle, 
                    "Request mission upload", ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_UPLOADED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_UPLOADED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<SetMode::Request>();
                set_mode_request->custom_mode = "AUTO";
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<SetMode, autopilot_interface_msgs::action::Takeoff>(
                    set_mode_client_, set_mode_request, goal_handle, 
                    "Request mode", ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_MODE);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_MODE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto command_request = std::make_shared<CommandLong::Request>();
                command_request->command = 300; // MAV_CMD_MISSION_START
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<CommandLong, autopilot_interface_msgs::action::Takeoff>(
                    command_long_client_, command_request, goal_handle, 
                    "Request mission start", ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_STARTED);
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_current_request = std::make_shared<WaypointSetCurrent::Request>();
                set_current_request->wp_seq = 1;
                time_of_last_srv_req_us_ = current_time_us;
                call_service_and_update_fsm<WaypointSetCurrent, autopilot_interface_msgs::action::Takeoff>(
                    set_wp_client_, set_current_request, goal_handle, 
                    "Request to set current waypoint", ArdupilotInterfaceState::FW_CRUISE);
            } else if ((current_fsm_state == ArdupilotInterfaceState::FW_CRUISE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                feedback->message = "VTOL takeoff completed";
                goal_handle->publish_feedback(feedback);
                taking_off = false;
            }
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}

void ArdupilotInterface::abort_action()
{
    // This is only sent once to be non-blocking but it could be implemented with a FSM to ensure the mode is changed
    auto set_mode_request = std::make_shared<SetMode::Request>();
    set_mode_request->custom_mode = "LOITER";
    set_mode_client_->async_send_request(set_mode_request,
        [this](rclcpp::Client<SetMode>::SharedFuture future) {
            if (future.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Request mode success");
            } else {
                RCLCPP_WARN(this->get_logger(), "Request mode failed");
            }
        });

    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    if (mav_type_ == 2) { // Multicopter
        aircraft_fsm_state_ = ArdupilotInterfaceState::MC_ORBIT; // This lets the reposition service change mode when called after an abort
    } else if (mav_type_ == 1) { // Fixed-wing/VTOL
        aircraft_fsm_state_ = ArdupilotInterfaceState::FW_CRUISE;
    }
    active_srv_or_act_flag_.store(false);
}

std::pair<double, double> ArdupilotInterface::lat_lon_from_cartesian(double ref_lat, double ref_lon, double x_offset, double y_offset)
{
    double temp_lat, temp_lon;
    double bearing_ns = (y_offset >= 0) ? 0 : 180; // North-South offset (bearing 0 for north, 180 for south)
    geod.Direct(ref_lat, ref_lon, bearing_ns, std::abs(y_offset), temp_lat, temp_lon);
    double return_lat, return_lon;
    double bearing_ew = (x_offset >= 0) ? 90 : 270; // East-West offset (bearing 90 for east, 270 for west)
    geod.Direct(temp_lat, temp_lon, bearing_ew, std::abs(x_offset), return_lat, return_lon);
    return {return_lat, return_lon};
}

std::pair<double, double> ArdupilotInterface::lat_lon_from_polar(double ref_lat, double ref_lon, double dist, double bear)
{
    double return_lat, return_lon;
    geod.Direct(ref_lat, ref_lon, bear, dist, return_lat, return_lon);
    return {return_lat, return_lon};
}

std::string ArdupilotInterface::fsm_state_to_string(ArdupilotInterfaceState state)
{
    switch (state) {
        case ArdupilotInterfaceState::STARTED: return "STARTED";
        case ArdupilotInterfaceState::GUIDED_PRETAKEOFF: return "GUIDED_PRETAKEOFF";
        case ArdupilotInterfaceState::ARMED: return "ARMED";
        case ArdupilotInterfaceState::MC_HOVER: return "MC_HOVER";
        case ArdupilotInterfaceState::VTOL_QLOITER_PRETAKEOFF: return "VTOL_QLOITER_PRETAKEOFF";
        case ArdupilotInterfaceState::VTOL_TAKEOFF_MC: return "VTOL_TAKEOFF_MC";
        case ArdupilotInterfaceState::VTOL_TAKEOFF_HEADING: return "VTOL_TAKEOFF_HEADING";
        case ArdupilotInterfaceState::VTOL_TAKEOFF_TRANSITION: return "VTOL_TAKEOFF_TRANSITION";
        case ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_UPLOADED: return "VTOL_TAKEOFF_MISSION_UPLOADED";
        case ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_MODE: return "VTOL_TAKEOFF_MISSION_MODE";
        case ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_STARTED: return "VTOL_TAKEOFF_MISSION_STARTED";
        case ArdupilotInterfaceState::FW_CRUISE: return "FW_CRUISE";
        case ArdupilotInterfaceState::MC_ORBIT: return "MC_ORBIT";
        case ArdupilotInterfaceState::MC_ORBIT_PARAM1_SET: return "MC_ORBIT_PARAM1_SET";
        case ArdupilotInterfaceState::MC_ORBIT_PARAM2_SET: return "MC_ORBIT_PARAM2_SET";
        case ArdupilotInterfaceState::MC_ORBIT_MISSION_UPLOADED: return "MC_ORBIT_MISSION_UPLOADED";
        case ArdupilotInterfaceState::MC_ORBIT_MISSION_MODE: return "MC_ORBIT_MISSION_MODE";
        case ArdupilotInterfaceState::MC_ORBIT_MISSION_STARTED: return "MC_ORBIT_MISSION_STARTED";
        case ArdupilotInterfaceState::MC_ORBIT_TRANSFER: return "MC_ORBIT_TRANSFER";
        case ArdupilotInterfaceState::MC_ORBIT_REACHED: return "MC_ORBIT_REACHED";
        case ArdupilotInterfaceState::MC_ORBIT_ROI_SET: return "MC_ORBIT_ROI_SET";
        case ArdupilotInterfaceState::VTOL_ORBIT_MISSION_UPLOADED: return "VTOL_ORBIT_MISSION_UPLOADED";
        case ArdupilotInterfaceState::VTOL_ORBIT_MISSION_MODE: return "VTOL_ORBIT_MISSION_MODE";
        case ArdupilotInterfaceState::VTOL_ORBIT_MISSION_STARTED: return "VTOL_ORBIT_MISSION_STARTED";
        case ArdupilotInterfaceState::VTOL_ORBIT_MISSION_COMPLETED: return "VTOL_ORBIT_MISSION_COMPLETED";
        case ArdupilotInterfaceState::MC_RTL_PARAM_SET: return "MC_RTL_PARAM_SET";
        case ArdupilotInterfaceState::MC_RTL: return "MC_RTL";
        case ArdupilotInterfaceState::MC_RETURNED_READY_TO_LAND: return "MC_RETURNED_READY_TO_LAND";
        case ArdupilotInterfaceState::MC_LANDING: return "MC_LANDING";
        case ArdupilotInterfaceState::VTOL_LANDING_MISSION_UPLOADED: return "VTOL_LANDING_MISSION_UPLOADED";
        case ArdupilotInterfaceState::VTOL_LANDING_MISSION_MODE: return "VTOL_LANDING_MISSION_MODE";
        case ArdupilotInterfaceState::VTOL_LANDING_MISSION_STARTED: return "VTOL_LANDING_MISSION_STARTED";
        case ArdupilotInterfaceState::VTOL_LANDING_READY_FOR_QRTL: return "VTOL_LANDING_READY_FOR_QRTL";
        case ArdupilotInterfaceState::VTOL_QRTL_PARAM_SET: return "VTOL_QRTL_PARAM_SET";
        case ArdupilotInterfaceState::VTOL_QRTL: return "VTOL_QRTL";
        case ArdupilotInterfaceState::LANDED: return "LANDED";
        case ArdupilotInterfaceState::OFFBOARD_VELOCITY: return "OFFBOARD_VELOCITY";
        case ArdupilotInterfaceState::OFFBOARD_ACCELERATION: return "OFFBOARD_ACCELERATION";
        default: return "UNKNOWN";
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; // Or set num_threads with executor(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<ArdupilotInterface>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
