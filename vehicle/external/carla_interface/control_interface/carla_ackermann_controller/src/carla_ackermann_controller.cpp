// carla_control_interface의 carla_ackermann_controller는 ros-bridge의 ackermanncontrol 패키지를 쓰지 않고,
//     바로 AckermannControlCommand형식으로부터 CarlaEgoVehicleControl 메세지 만드는 패키지임. 
//     ==> 이것도 나중에 수정해서 사용해보고 싶음. (두 노드 쓰는 것보다 이게 편리하니까? && cpp니까?)

#include "carla_ackermann_controller.hpp"

#include <stdio.h>
#include <ctime>

CarlaAckermannController::CarlaAckermannController(const rclcpp::NodeOptions & node_options) : Node("CarlaAckermannController", node_options)
{
    using std::placeholders::_1;
    //PID controller (the controller has to run with the simulation time, not with real-time)
    std::vector<double>speed_gains = {0.0, 0.0, 0.0};
    speed_gains.at(K_P) = declare_parameter<double>("Kp_speed", 0.05);
    speed_gains.at(K_I) = declare_parameter<double>("Ki_speed", 0.0);
    speed_gains.at(K_D) = declare_parameter<double>("Kd_speed", 0.50);
    std::vector<double>accel_gains = {0.0, 0.0, 0.0};
    accel_gains.at(K_P) = declare_parameter<double>("Kp_accel", 0.05);
    accel_gains.at(K_I) = declare_parameter<double>("Ki_accel", 0.0);
    accel_gains.at(K_D) = declare_parameter<double>("Kd_accel", 0.05);
    RCLCPP_INFO(get_logger(), "Kp_accel: %f", accel_gains.at(K_P));
    RCLCPP_INFO(get_logger(), "Ki_accel: %f", accel_gains.at(K_I));
    RCLCPP_INFO(get_logger(), "Kd_accel: %f", accel_gains.at(K_D));
    
    speed_controller_ = PIDController(speed_gains);
    accel_controller_ = PIDController(accel_gains);

    // parameters & global variables
    const double ctrl_period = declare_parameter<double>("ctrl_period", 0.03);
    const std::string role_name = declare_parameter<std::string>("role_name", "ego_vehicle");
    
    use_speed_pid_ = declare_parameter<bool>("use_lon_pid", false);
    last_ackermann_msg_received_sec_ = float(clock())/1000.0;
    current_accel_ = 0.0;
    current_velocity_ = 0.0;

    // initialize
    // // set initial maximum values
    onInfoAlter();

    // // control info (target)
    ego_info_.target.steering_angle = 0.0;
    ego_info_.target.speed = 0.0;
    ego_info_.target.speed_abs = 0.0;
    ego_info_.target.accel = 0.0;
    ego_info_.target.jerk = 0.0;

    // // control info (current)
    // ego_info_.current.time_sec = float(now().seconds());
    ego_info_.current.time_sec = float(clock())/1000.0;
    ego_info_.current.speed = 0.0;
    ego_info_.current.speed_abs = 0.0;
    ego_info_.current.accel = 0.0;

    // // control info (status)
    ego_info_.status.status = "n/a";
    ego_info_.status.speed_control_activation_count = 0;
    ego_info_.status.speed_control_accel_delta = 0.0;
    ego_info_.status.speed_control_accel_target = 0.0;
    ego_info_.status.accel_control_pedal_delta = 0.0;
    ego_info_.status.accel_control_pedal_target = 0.0;
    ego_info_.status.brake_upper_border = 0.0;
    ego_info_.status.throttle_lower_border = 0.0;

    // // control info (output)
    ego_info_.output.throttle = 0.0;
    ego_info_.output.brake = 1.0;
    ego_info_.output.steer = 0.0;
    ego_info_.output.reverse = false;
    ego_info_.output.hand_brake = true;


    // ROS
    // // subscribers
    sub_control_cmd_ = create_subscription<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS{1}, std::bind(&CarlaAckermannController::onCmd, this, _1));
    sub_vehicle_status_ = create_subscription<CarlaEgoVehicleStatus>("/carla/" + role_name + "/vehicle_status", rclcpp::QoS{1}, std::bind(&CarlaAckermannController::onStatus, this, _1));
    sub_accel_ = create_subscription<AccelWithCovarianceStamped>("/localization/acceleration", rclcpp::QoS{1}, std::bind(&CarlaAckermannController::onAccel, this, _1));
    sub_odom_ = create_subscription<Odometry>("/localization/kinematic_state", rclcpp::QoS{1}, std::bind(&CarlaAckermannController::onOdom, this, _1));
    // sub_vehicle_info_ = create_subscription<CarlaEgoVehicleInfo>("/carla/" + role_name + "/vehicle_info_", rclcpp::QoS{1}, std::bind(&CarlaAckermannController::onInfo, this, _1));
    // // publishers
    pub_control_cmd_ = create_publisher<CarlaEgoVehicleControl>("/carla/" + role_name + "/vehicle_control_cmd", rclcpp::QoS{1});
    pub_control_info_ = create_publisher<EgoVehicleControlInfo>("/carla/" + role_name + "/ackermann_control/control_info", rclcpp::QoS{1});
    
    // timer & subscription callbacks
    {
        const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ctrl_period));
        timer_control_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&CarlaAckermannController::callbackTimer, this));
    }   
}

// subscription callbacks
void CarlaAckermannController::onCmd(const AckermannControlCommand::SharedPtr msg)
{
    /*
    Stores the ackermann drive message for the next controller calculation

    :param msg: the current ackermann control input
    */
    
    last_ackermann_msg_received_sec_ = float(clock())/1000.0;
    // set target values
    setTargetSteeringAngle(msg->lateral.steering_tire_angle);
    setTargetSpeed(msg->longitudinal.speed);
    setTargetAccel(msg->longitudinal.acceleration);
    setTargetJerk(msg->longitudinal.jerk);
}
void CarlaAckermannController::onStatus(const CarlaEgoVehicleStatus::SharedPtr msg)
{
    /*
    Stores the ackermann drive message for the next controller calculation

    :param ros_ackermann_drive: the current ackermann control input
    :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
    */
    vehicle_status_ = *msg;
}
void CarlaAckermannController::onAccel(const AccelWithCovarianceStamped::SharedPtr msg)
{
    /*
    Stores the current accel for the next controller calculation
    */
    current_accel_ = msg->accel.accel.linear.x;
}
void CarlaAckermannController::onOdom(const Odometry::SharedPtr msg)
{
    /*
    Stores the current accel for the next controller calculation
    */
    current_velocity_ = msg->twist.twist.linear.x;
}
void CarlaAckermannController::onInfo(const CarlaEgoVehicleInfo::SharedPtr msg) // never occur..
{
    /*
    Stores the ackermann drive message for the next controller calculation

    :param ros_ackermann_drive: the current ackermann control input
    :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
    */   
    
    // set target values
    vehicle_info_ = *msg;
    
    // calculate restrictions
    ego_info_.restrictions.max_steering_angle = getVehicleMaxSteeringAngle(vehicle_info_);
    ego_info_.restrictions.max_speed = getVehicleMaxSpeed();
    ego_info_.restrictions.max_accel = getVehicleMaxAcceleration();
    ego_info_.restrictions.max_decel = getVehicleMaxDeceleration();
    const double min_accel = declare_parameter<double>("min_accel", 1.0);
    ego_info_.restrictions.min_accel = min_accel;
    // clipping the pedal in both directions to the same range using the usual lower
    // border: the max_accel to ensure the the pedal target is in symmetry to zero
    ego_info_.restrictions.max_pedal = std::min(ego_info_.restrictions.max_accel,  ego_info_.restrictions.max_decel);
}
void CarlaAckermannController::onInfoAlter() // occur once
{
    /*
    Alternative function of onInfo 
    (This is because Carla does not periodically publish info messages.)
    */   
    
    // set target values
    vehicle_info_ = initVehicleInfo();
    
    // calculate restrictions
    ego_info_.restrictions.max_steering_angle = getVehicleMaxSteeringAngle(vehicle_info_);
    ego_info_.restrictions.max_speed = getVehicleMaxSpeed();
    ego_info_.restrictions.max_accel = getVehicleMaxAcceleration();
    ego_info_.restrictions.max_decel = getVehicleMaxDeceleration();
    const double min_accel = declare_parameter<double>("min_accel", 1.0);
    ego_info_.restrictions.min_accel = min_accel;
    // clipping the pedal in both directions to the same range using the usual lower
    // border: the max_accel to ensure the the pedal target is in symmetry to zero
    ego_info_.restrictions.max_pedal = std::min(ego_info_.restrictions.max_accel,  ego_info_.restrictions.max_decel);
}
// main functions
void CarlaAckermannController::updateCurrentValues()
{
    /*
    Function to update vehicle control current values.

    we calculate the acceleration on ourselves, because we are interested only in
    the acceleration in respect to the driving direction
    In addition a small average filter is applied
    */
    const auto current_time_sec = float(clock())/1000.0; 

    // jw) calc current_speed & current_accel with << localization data >>
    const float current_speed = current_velocity_;
    ego_info_.current.accel = current_accel_;
    // const auto delta_time = current_time_sec - ego_info_.current.time_sec;
    // const float current_speed = vehicle_status_.velocity;
    // if(delta_time > 0){
    //     float delta_speed = current_speed - ego_info_.current.speed;
    //     float current_accel = delta_speed / delta_time;
    //     // average filter
        // ego_info_.current.accel = (ego_info_.current.accel * 4 + current_accel) / 5;
    // }

    ego_info_.current.time_sec = current_time_sec;
    ego_info_.current.speed = current_speed;
    ego_info_.current.speed_abs = std::abs(current_speed);

    // debug
    ego_info_.current.steer = vehicle_status_.control.steer;
    return;
}
void CarlaAckermannController::vehicleControlCycle()
{
    /* 
    Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message 
    */

    // perform actual control
    controlSteering(); // decide output.steering

    controlStopAndReverse(); //decide output.reverse & hand_brake
    runSpeedControlLoop();
    runAccelControlLoop();
    if(!ego_info_.output.hand_brake){
        updateDriveVehicleControlCommand(); // decide output.throttle & brake

        // only send out the Carla Control Command if AckermannDrive messages are
        // received in the last second (e.g. to allows manually controlling the vehicle)

        if((last_ackermann_msg_received_sec_ + 1.5) > (float(clock())/1000.0)){
            ego_info_.output.header.frame_id = "map";
            ego_info_.output.header.stamp = this->now();
            CarlaEgoVehicleControl op = ego_info_.output;
            pub_control_cmd_->publish(op);
        }
    }
    return;    
}
void CarlaAckermannController::sendEgoVehicleControlInfoMsg()
{
    /* 
    Function to send carla_ackermann_control.msg.EgoVehicleControlInfo message. 
    */
    ego_info_.header.frame_id = "map";
    ego_info_.header.stamp = this->now();
    pub_control_info_->publish(ego_info_);

    return;
}

// additional functions
// // (inner functions of vehicleControlCycle)
void CarlaAckermannController::controlSteering()
{
    /*
    Basic steering control
    */
    ego_info_.output.steer = ego_info_.target.steering_angle / ego_info_.restrictions.max_steering_angle; 
}
void CarlaAckermannController::controlStopAndReverse()
{
    /*
    Handle stop and switching to reverse gear
    */

    // from this velocity on it is allowed to switch to reverse gear
    double standing_still_epsilon = 0.1;
    // from this velocity on hand brake is turned on
    // double full_stop_epsilon = 0.00001;
    double full_stop_epsilon = 0.01; //jw

    // auto-control of hand-brake and reverse gear
    ego_info_.output.hand_brake = false;
    if (ego_info_.current.speed_abs < standing_still_epsilon){
        // standing still, change of driving direction allowed
        ego_info_.status.status = "standing";
        if (ego_info_.target.speed < 0){
            if (!ego_info_.output.reverse){
                RCLCPP_INFO_ONCE(get_logger(), "VehicleControl::Change of driving direction to reverse");
                ego_info_.output.reverse = true;
            }
        }
        else if (ego_info_.target.speed > 0){
            if (ego_info_.output.reverse){
                RCLCPP_INFO_ONCE(get_logger(), "VehicleControl::Change of driving direction to forward");
                ego_info_.output.reverse = false;
            }
        }
        if (ego_info_.target.speed_abs < full_stop_epsilon){
            ego_info_.status.status = "full stop";
            ego_info_.status.speed_control_accel_target = 0.0;
            ego_info_.status.accel_control_pedal_target = 0.0;
            setTargetSpeed(0.0);
            ego_info_.current.speed = 0.0;
            ego_info_.current.speed_abs = 0.0;
            ego_info_.current.accel = 0.0;
            ego_info_.output.hand_brake = true;
            ego_info_.output.brake = 1.0;
            ego_info_.output.throttle = 0.0;
        }
    }
    else{
        bool b_cur_speed = std::signbit(ego_info_.current.speed);
        bool b_tar_speed = std::signbit(ego_info_.target.speed);
        if ((b_cur_speed != b_tar_speed) && (b_cur_speed || b_tar_speed)){
            // requrest for change of driving direction
            // first we have to come to full stop before changing driving
            // direction
            RCLCPP_INFO_ONCE(get_logger(), "VehicleControl::Change of driving direction to forward");
            RCLCPP_INFO_ONCE(get_logger(), "VehicleControl: Request change of driving direction.\n v_current=%f, v_desired=%f\n VehicleControl: Request change of driving direction.\n", ego_info_.current.speed, ego_info_.target.speed);
            setTargetSpeed(0.0);
        }
    }
}
void CarlaAckermannController::runSpeedControlLoop()
{
    /*
    Run the PID control loop for the speed
    */
    float target_accel_abs = abs(ego_info_.target.accel);

    // target_accel_abs이 min_accel보다 작을 경우가 5번 이상 나오는 경우 speed_controller_를 통해 target_accel 새로 계산
    if (target_accel_abs < ego_info_.restrictions.min_accel){
        if (ego_info_.status.speed_control_activation_count < 5){
            ego_info_.status.speed_control_activation_count ++;
        }
    }
    else{
        if (ego_info_.status.speed_control_activation_count > 0){
            ego_info_.status.speed_control_activation_count --;
        }
    }
    // set the auto_mode of the controller accordingly
    bool auto_mode = false;
    if(use_speed_pid_){
        auto_mode = ego_info_.status.speed_control_activation_count >= 5;
    }

    if (auto_mode){
        ego_info_.status.speed_control_accel_delta = speed_controller_.run_step(ego_info_.target.speed, ego_info_.current.speed);
        // clamping
        float clamping_lower_border = -target_accel_abs;
        float clamping_upper_border = target_accel_abs;
        float epsilon = 0.00001;
        // per definition of ackermann drive: if zero, then use max value
        if (target_accel_abs < epsilon){
            clamping_lower_border = -ego_info_.restrictions.max_decel;
            clamping_upper_border = ego_info_.restrictions.max_accel;
        }
        ego_info_.status.speed_control_accel_target = std::clamp(ego_info_.status.speed_control_accel_target + ego_info_.status.speed_control_accel_delta, clamping_lower_border, clamping_upper_border); 
    }
    else{
        ego_info_.status.speed_control_accel_delta = 0.0;
        ego_info_.status.speed_control_accel_target = ego_info_.target.accel;
    }
}
void CarlaAckermannController::runAccelControlLoop()
{
    ego_info_.status.accel_control_pedal_delta = accel_controller_.run_step(ego_info_.status.speed_control_accel_target, ego_info_.current.accel);
    ego_info_.status.accel_control_pedal_target = std::clamp(ego_info_.status.accel_control_pedal_target + ego_info_.status.accel_control_pedal_delta,
                                                            -ego_info_.restrictions.max_pedal, ego_info_.restrictions.max_pedal);
}
void CarlaAckermannController::updateDriveVehicleControlCommand()
{
    /*
    Apply the current speed_control_target value to throttle/brake commands
    */

    // accel_control_pedal_target 과 throttle_lower_border 비교
    // ego_info_.output.brake, throttle 값 입력

    // the driving impedance moves the 'zero' acceleration border
    // Interpretation: To reach a zero acceleration the throttle has to pushed
    // down for a certain amount
    
    // throttle 디폴트 차감값(cuz 임티던스) (negative)
    ego_info_.status.throttle_lower_border = getVehicleDrivingImpedanceAcceleration(vehicle_info_, vehicle_status_, ego_info_.output.reverse);

    // the engine lay off acceleration defines the size of the coasting area
    // Interpretation: The engine already prforms braking on its own;
    // therefore pushing the brake is not required for small decelerations

    // 기본 braking 값(임피던스 and 엔진 layoff) (negative)
    ego_info_.status.brake_upper_border = ego_info_.status.throttle_lower_border + getVehicleLayOffEngineAcceleration(vehicle_info_);

    if (ego_info_.status.accel_control_pedal_target > ego_info_.status.throttle_lower_border){
        ego_info_.status.status = "accelerating";
        ego_info_.output.brake = 0.0;
        // the value has to be normed to max_pedal
        // be aware: is not required to take throttle_lower_border into the scaling factor,
        // because that border is in reality a shift of the coordinate system
        // the global maximum acceleration can practically not be reached anymore because of
        // driving impedance
        
        // (주행 임피던스를 기준으로 Carla 차량이 직면한 가속도 보정 제거)
        ego_info_.output.throttle = (ego_info_.status.accel_control_pedal_target - ego_info_.status.throttle_lower_border) / abs(ego_info_.restrictions.max_pedal);
    }
    else if (ego_info_.status.accel_control_pedal_target > ego_info_.status.brake_upper_border){
        ego_info_.status.status = "coasting";
        // no control required
        ego_info_.output.brake = 0.0;
        ego_info_.output.throttle = 0.0;
    }
    else {
        ego_info_.status.status = "braking";
        // braking required
        ego_info_.output.brake = (ego_info_.status.brake_upper_border - ego_info_.status.accel_control_pedal_target) / abs(ego_info_.restrictions.max_pedal);
        ego_info_.output.throttle = 0.0;
    }

    // finally clip the final control output (should actually never happen)
    ego_info_.output.brake = std::clamp(ego_info_.output.brake, float(0.0), float(1.0));
    ego_info_.output.throttle = std::clamp(ego_info_.output.throttle, float(0.0), float(1.0));    
}

// // (inner functions of onCmd)
void CarlaAckermannController::setTargetSteeringAngle(float t_steer)
{
    /*
    Set target steering angle
    */
    ego_info_.target.steering_angle = -t_steer; //carla <-> autoware (opposite steer)
    if (abs(ego_info_.target.steering_angle) > ego_info_.restrictions.max_steering_angle){
        RCLCPP_INFO_ONCE(get_logger(), "Max steering angle reached, clamping value");
        ego_info_.target.steering_angle = std::clamp(ego_info_.target.steering_angle, -ego_info_.restrictions.max_steering_angle, ego_info_.restrictions.max_steering_angle);
    }
    // debug
    ego_info_.target.steer = ego_info_.target.steering_angle / ego_info_.restrictions.max_steering_angle;
}
void CarlaAckermannController::setTargetSpeed(float t_speed)
{
    /*
    Set target speed
    */
    if (abs(t_speed) > ego_info_.restrictions.max_speed){
        RCLCPP_INFO_ONCE(get_logger(), "Max speed reached, clamping value");
        ego_info_.target.speed = std::clamp(ego_info_.target.speed, -ego_info_.restrictions.max_speed, ego_info_.restrictions.max_speed);
    }
    else{
        ego_info_.target.speed = t_speed;
    }
    ego_info_.target.speed_abs = abs(ego_info_.target.speed);
}
void CarlaAckermannController::setTargetAccel(float t_accel)
{
    /*
    set target accel
    */
    // double epsilon = 0.00001; 
    double epsilon = 0.01; // jw
    // if speed is set to zero, then use max decel value
    if (ego_info_.target.speed_abs < epsilon){
        ego_info_.target.accel = -ego_info_.restrictions.max_decel;
    }
    else{
        ego_info_.target.accel = std::clamp(t_accel, -ego_info_.restrictions.max_decel, ego_info_.restrictions.max_accel);
    }
}
void CarlaAckermannController::setTargetJerk(float t_jerk)
{
    /*
    set target jerk
    */
    ego_info_.target.jerk = t_jerk;
}

// timer callback
void CarlaAckermannController::callbackTimer()
{
    updateCurrentValues();
    vehicleControlCycle(); // publish final control_cmd
    sendEgoVehicleControlInfoMsg();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CarlaAckermannController)