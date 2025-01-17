#include "controllers/pid.hpp"

PIDControllerNode::PIDControllerNode()
: Node("pid_controller")
{
    // PID Regler-Parameter initialisieren
    Kp_ = 1.0;
    Ki_ = 0.1;
    Kd_ = 0.05;
    dt_ = 0.01;

    integral_ = 0.0;
    previous_error_ = 0.0;

    // Initialize subscribers
    current_value_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "/currentValue", 1,
        std::bind(&PIDControllerNode::currentValueCallback, this,
                  std::placeholders::_1));

    setpoint_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "/setpoint", 1,
        std::bind(&PIDControllerNode::setpointCallback, this,
                  std::placeholders::_1));

    // Initialize publisher
    control_output_publisher_ =
        this->create_publisher<std_msgs::msg::Float64>("/controlOutput", 1);
}

void PIDControllerNode::currentValueCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    current_value_ = msg->data;
    auto control_output = computeControlCommand();
    std_msgs::msg::Float64 output_msg;

    // Publish the control output
    output_msg.data = control_output;
    control_output_publisher_->publish(output_msg);
}

void PIDControllerNode::setpointCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    setpoint_ = msg->data;
}

double PIDControllerNode::computeControlCommand()
{
    // Fehler berechnen
    double error = setpoint_ - current_value_;

    // Proportionalanteil
    double P = Kp_ * error;

    // Integralanteil
    integral_ += error * dt_;
    double I = Ki_ * integral_;

    // Differentialanteil
    double derivative = (error - previous_error_) / dt_;
    double D = Kd_ * derivative;

    // Stellgröße berechnen
    double output = P + I + D;

    // Vorherigen Fehler aktualisieren
    previous_error_ = error;

    // Stellgröße begrenzen (z. B. auf [-1, 1])
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;

    return output;
}