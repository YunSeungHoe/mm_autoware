#include "carla_ackermann_controller.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <math.h>
#define DEG2RAD M_PI/180

CarlaEgoVehicleInfo CarlaAckermannController::initVehicleInfo()
{
    CarlaEgoVehicleInfo info;
    info.type = "vehicle.tesla.model3";
    info.rolename = "ego_vehicle";
    for (auto & wheel : info.wheels) {
        wheel.tire_friction = 3.5;
        wheel.damping_rate = 0.25;
        wheel.max_steer_angle = 1.221730351448059;
        wheel.radius = 37.0;
        wheel.max_brake_torque = 700.0;
    }
    info.max_rpm = 15000.0;
    info.moi = 1.0;
    info.damping_rate_full_throttle = 0.15000000596046448;
    info.damping_rate_zero_throttle_clutch_engaged = 2.0;
    info.damping_rate_zero_throttle_clutch_disengaged = 0.3499999940395355;
    info.use_gear_autobox = true;
    info.gear_switch_time = 0.0;
    info.clutch_strength = 10.0;
    info.mass = 1845.0;
    info.drag_coefficient = 0.15000000596046448;
    
    return info;
}

float CarlaAckermannController::getVehicleLayOffEngineAcceleration(CarlaEgoVehicleInfo vehicle_info)
{
    /*
    Calculate the acceleration a carla vehicle faces by the engine on lay off

    This respects the following forces:
    - engine brake force

    :param vehicle_info: the vehicle info
    :return: acceleration the vehicle [m/s^2 < 0]
    */
    return -getEngineBrakeForce() / getVehicleMass(vehicle_info);
}

float CarlaAckermannController::getEngineBrakeForce()
{
    /*
    Calculate the engine brake force of a carla vehicle if the gas pedal would be layed off

    As this heavily depends on the engine, the current gear and velocity, this is not
    trivial to calculate. Maybe one can get this from within Unreal to the outside,
    to enable better vehicle control.
    For the moment we just put a constant force.

    :param vehicle_info: the vehicle info
    :return: engine braking force [N]
    */
    return 500.0;
}

float CarlaAckermannController::getVehicleMass(CarlaEgoVehicleInfo vehicle_info)
{
    /* 
    Get the mass of a carla vehicle (defaults to 1500kg)

    :param vehicle_info: the vehicle info
    :return: mass of a carla vehicle [kg]
    */
    float mass = 1500.0;
    if(vehicle_info.mass) mass = vehicle_info.mass;

    return mass;
}

float CarlaAckermannController::getVehicleDrivingImpedanceAcceleration(CarlaEgoVehicleInfo vehicle_info, CarlaEgoVehicleStatus vehicle_status, bool reverse)
{
    /* 
    Calculate the acceleration a carla vehicle faces by the driving impedance

    This respects the following forces:
    - rolling resistance force
    - aerodynamic drag force
    - slope force

    :param vehicle_info: the vehicle info
    :param reverse: `True' if the vehicle is driving in reverse direction
    :return: acceleration the vehicle [m/s^2 <= 0 on flat surface]
    */

    // taking the following forumlar as basis
    // mass * acceleration = rolling_resitance_force + aerodynamic_drag_force
    // deceleration = -(rolling_resitance_force + aerodynamic_drag_force)/mass
    // future enhancements: incorporate also the expected motor braking force

    float rolling_resistance_force = getRollingResistanceForce(vehicle_info);
    float aerodynamic_drag_force = getAerodynamicDragForce(vehicle_status);
    float slope_force = getSlopeForce(vehicle_info, vehicle_status);
    if(reverse) slope_force *= -1;
    float deceleration = -(rolling_resistance_force + aerodynamic_drag_force + slope_force)/getVehicleMass(vehicle_info);
    
    return deceleration;
}

float CarlaAckermannController::getRollingResistanceForce(CarlaEgoVehicleInfo vehicle_info)
{
    /*
    Calculate the rolling resistance force of a carla vehicle

    :param vehicle_info: the vehicle info
    :return: rolling resistance force [N]
    */

    // usually somewhere between 0.007 to 0.014 for car tyres
    // and between 0.0025 to 0.005 for bycicle tyres
    // see also https://en.wikipedia.org/wiki/Rolling_resistance
    // @todo: currently not within vehicle_info

    float rolling_resistance_coefficient = 0.01;
    float normal_force = getWeightForce(vehicle_info);

    float rolling_resistance_force = rolling_resistance_coefficient * normal_force;

    return rolling_resistance_force;
}

float CarlaAckermannController::getWeightForce(CarlaEgoVehicleInfo vehicle_info)
{
    /*
    Get the weight of a carla vehicle

    :param vehicle_info: the vehicle info
    :return: weight of the vehicle [N]
    */
    float weight = getVehicleMass(vehicle_info) * getAccelerationOfGravity();
    
    return weight;
}

float CarlaAckermannController::getAccelerationOfGravity()
{
    /*
    Get the acceleration of gravity for a carla vehicle
    (for the moment constant at 9.81 m/s^2)

    :param vehicle_info: the vehicle info
    :return: acceleration of gravity [m/s^2]
    */
    return 9.81;
}

float CarlaAckermannController::getAerodynamicDragForce(CarlaEgoVehicleStatus vehicle_status)
{
    /*
    Calculate the aerodynamic drag force of a carla vehicle

    :param vehicle_status: the ego vehicle status
    :return: aerodynamic drag force [N]
    */

    // see also https://en.wikipedia.org/wiki/Automobile_drag_coefficient
    float default_aerodynamic_drag_coefficient = 0.3;
    float default_drag_reference_area = 2.37;
    // @todo currently not provided in vehicle_info
    float drag_area = default_aerodynamic_drag_coefficient * default_drag_reference_area;
    float rho_air_25 = 1.184;
    float speed_squared = vehicle_status.velocity * vehicle_status.velocity;

    float aerodynamic_drag_force = 0.5 * drag_area * rho_air_25 * speed_squared;
    return aerodynamic_drag_force;
}

float CarlaAckermannController::getSlopeForce(CarlaEgoVehicleInfo vehicle_info, CarlaEgoVehicleStatus vehicle_status)
{
    /*
    Calculate the force of a carla vehicle faces when driving on a slope.

    :param vehicle_info: the vehicle info
    :param vehicle_status: the ego vehicle status
    :return: slope force [N, >0 uphill, <0 downhill]
    */

    geometry_msgs::msg::Vector3 rpy;
    tf2::Quaternion q(vehicle_status.orientation.x, vehicle_status.orientation.y, vehicle_status.orientation.z, vehicle_status.orientation.w);
    tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);  // x: roll, y: pitch, z: yaw
    
    float slope_force = getAccelerationOfGravity() * getVehicleMass(vehicle_info) * sin(-rpy.y);
    return slope_force;
}

float CarlaAckermannController::getVehicleMaxSteeringAngle(CarlaEgoVehicleInfo vehicle_info)
{
    /*
    Get the maximum steering angle of a carla vehicle

    :param vehicle_info: the vehicle info
    :return: maximum steering angle [radians]
    */
   
    // 70 degrees is the default max steering angle of a car
    float max_steering_angle = 70 * DEG2RAD;
    // get max steering angle (use smallest non-zero value of all wheels)
    for (const auto & wheel : vehicle_info.wheels) {
        if(wheel.max_steer_angle){
            if(wheel.max_steer_angle && (wheel.max_steer_angle < max_steering_angle)){
                max_steering_angle = wheel.max_steer_angle;
            }
        }
    }
    // std::cout << "max_steering_angle: " << max_steering_angle << std::endl;
    return max_steering_angle;
}

float CarlaAckermannController::getVehicleMaxSpeed()
{
    /*
    Get the maximum speed of a carla vehicle

    :param vehicle_info: the vehicle info
    :return: maximum speed [m/s]
    */

    // 180 km/h is the default max speed of a car
    float max_speed = 180.0 / 3.6;

    return max_speed;
}

float CarlaAckermannController::getVehicleMaxAcceleration()
{
    /*
    Get the maximum acceleration of a carla vehicle

    default: 3.0 m/s^2: 0-100 km/h in 9.2 seconds

    :param vehicle_info: the vehicle info
    :return: maximum acceleration [m/s^2 > 0]
    */
    float max_acceleration = 3.0;

    return max_acceleration;
}

float CarlaAckermannController::getVehicleMaxDeceleration()
{
    /*
    Get the maximum deceleration of a carla vehicle

    default: 8 m/s^2

    :param vehicle_info: the vehicle info
    :return: maximum deceleration [m/s^2 > 0]
    */

    float max_deceleration = 8.0;

    return max_deceleration;
}