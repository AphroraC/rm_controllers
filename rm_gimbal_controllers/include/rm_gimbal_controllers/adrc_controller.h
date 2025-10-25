#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace rm_gimbal_controllers
{

class LinearESO
{
public:
  LinearESO(double omega_o, double b0, double h) : omega_o_(omega_o), b0_(b0), h_(h)
  {
    reset();
  }

  void reset()
  {
    z1_ = 0.0;
    z2_ = 0.0;
    z3_ = 0.0;
  }

  void update(double y, double u)
  {
    double beta1 = 3.0 * omega_o_;
    double beta2 = 3.0 * omega_o_ * omega_o_;
    double beta3 = omega_o_ * omega_o_ * omega_o_;

    double e = z1_ - y;

    z1_ += h_ * (z2_ - beta1 * e);
    z2_ += h_ * (z3_ - beta2 * e + b0_ * u);
    z3_ += h_ * (-beta3 * e);
  }

  double getPosition() const
  {
    return z1_;
  }
  double getVelocity() const
  {
    return z2_;
  }
  double getDisturbance() const
  {
    return z3_;
  }

  void setBandwidth(double omega_o)
  {
    omega_o_ = omega_o;
  }
  void setControlGain(double b0)
  {
    b0_ = b0;
  }

private:
  double omega_o_;
  double b0_;
  double h_;
  double z1_, z2_, z3_;
};

class ADRCController
{
public:
  // Constructor: b0 is fixed to 1.0 for velocity command output
  ADRCController(double omega_o, double omega_c, double h)
    : eso_(omega_o, 1.0, h), omega_c_(omega_c), h_(h)
  {
    vel_cmd_prev_ = 0.0;
  }

  void reset()
  {
    eso_.reset();
    vel_cmd_prev_ = 0.0;
  }

  // Compute velocity command (not torque)
  // y: actual position (rad)
  // r: desired position (rad)
  // r_dot: desired velocity (rad/s)
  // Returns: velocity command (rad/s)
  double computeVelocityCommand(double y, double r, double r_dot = 0.0)
  {
    // Update ESO with actual position and previous velocity command
    eso_.update(y, vel_cmd_prev_);

    double z1 = eso_.getPosition();       // Estimated position
    double z2 = eso_.getVelocity();       // Estimated velocity
    double z3 = eso_.getDisturbance();    // Estimated disturbance (in velocity domain)

    // PD control law
    double kp = omega_c_ * omega_c_;
    double kd = 2.0 * omega_c_;

    double e_pos = r - z1;
    double e_vel = r_dot - z2;

    // Velocity command = PD + feedforward - disturbance compensation
    double vel_cmd = kp * e_pos + kd * e_vel + r_dot - z3;

    vel_cmd_prev_ = vel_cmd;
    return vel_cmd;
  }

  double getDisturbance() const
  {
    return eso_.getDisturbance();
  }

  double getEstimatedVelocity() const
  {
    return eso_.getVelocity();
  }

  void setBandwidth(double omega_o, double omega_c)
  {
    omega_c_ = omega_c;
    eso_.setBandwidth(omega_o);
  }

private:
  LinearESO eso_;
  double omega_c_;
  double h_;
  double vel_cmd_prev_;  // Previous velocity command
};

}  // namespace rm_gimbal_controllers
