#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>

namespace arm_control {

/******************************************************************************
 * CubicSplineTrajectory - Cubic Spline Trajectory Interpolator
 * 
 * Fits a cubic spline to the discrete trajectory output by MPC, providing:
 * - Position q(t)
 * - Velocity q̇(t) = dq/dt
 * - Acceleration q̈(t) = d²q/dt²
 ******************************************************************************/
class CubicSplineTrajectory {
public:
  CubicSplineTrajectory() : numJoints_(0), valid_(false) {}
  
  /**
   * Fit cubic spline from MPC trajectory
   * @param times Time sequence [t0, t1, ..., tn]
   * @param states State sequence (each is a numJoints dimensional vector)
   * @param inputs Input sequence (velocity, used for endpoint conditions)
   */
  void fit(const std::vector<double>& times,
           const std::vector<Eigen::VectorXd>& states,
           const std::vector<Eigen::VectorXd>& inputs) {
    if (times.size() < 2 || states.size() < 2) {
      valid_ = false;
      return;
    }
    
    numJoints_ = states[0].size();
    size_t n = times.size();
    
    times_ = times;
    
    // Calculate cubic spline coefficients for each joint
    // Cubic spline: q(t) = a + b*(t-t_i) + c*(t-t_i)^2 + d*(t-t_i)^3
    // Store [a, b, c, d] coefficients for each interval
    
    coeffs_.resize(numJoints_);
    
    for (size_t j = 0; j < numJoints_; ++j) {
      // Extract position sequence for this joint
      std::vector<double> q(n);
      for (size_t i = 0; i < n; ++i) {
        q[i] = states[i](j);
      }
      
      // Use natural spline boundary conditions (second derivative is 0)
      // Could also use clamped conditions (first derivative given by inputs)
      coeffs_[j] = computeNaturalCubicSpline(times_, q);
    }
    
    tMin_ = times_.front();
    tMax_ = times_.back();
    valid_ = true;
  }
  
  /**
   * Evaluate spline at time t
   * @param t Time
   * @param q Output: Position
   * @param qdot Output: Velocity
   * @param qddot Output: Acceleration
   */
  void evaluate(double t, Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot) const {
    q.resize(numJoints_);
    qdot.resize(numJoints_);
    qddot.resize(numJoints_);
    
    if (!valid_ || times_.empty()) {
      q.setZero();
      qdot.setZero();
      qddot.setZero();
      return;
    }
    
    // Clamp t within valid range
    t = std::max(tMin_, std::min(t, tMax_));
    
    // Find the interval t belongs to
    size_t idx = findInterval(t);
    double dt = t - times_[idx];
    
    for (size_t j = 0; j < numJoints_; ++j) {
      const auto& c = coeffs_[j][idx];  // [a, b, c, d]
      
      // q(t) = a + b*dt + c*dt^2 + d*dt^3
      q(j) = c[0] + c[1]*dt + c[2]*dt*dt + c[3]*dt*dt*dt;
      
      // q̇(t) = b + 2*c*dt + 3*d*dt^2
      qdot(j) = c[1] + 2.0*c[2]*dt + 3.0*c[3]*dt*dt;
      
      // q̈(t) = 2*c + 6*d*dt
      qddot(j) = 2.0*c[2] + 6.0*c[3]*dt;
    }
  }
  
  bool isValid() const { return valid_; }
  double getMinTime() const { return tMin_; }
  double getMaxTime() const { return tMax_; }
  
private:
  /**
   * Compute natural cubic spline coefficients
   * Returns [a, b, c, d] coefficients for each interval
   */
  std::vector<std::array<double, 4>> computeNaturalCubicSpline(
      const std::vector<double>& t,
      const std::vector<double>& y) const {
    
    size_t n = t.size();
    std::vector<std::array<double, 4>> result(n - 1);
    
    if (n < 2) return result;
    if (n == 2) {
      // Linear interpolation
      double h = t[1] - t[0];
      result[0] = {y[0], (y[1] - y[0]) / h, 0.0, 0.0};
      return result;
    }
    
    // Calculate interval lengths
    std::vector<double> h(n - 1);
    for (size_t i = 0; i < n - 1; ++i) {
      h[i] = t[i + 1] - t[i];
      if (h[i] <= 0) h[i] = 1e-6;  // Avoid division by zero
    }
    
    // Build tridiagonal matrix equation to solve for c coefficients
    // Natural spline: c[0] = c[n-1] = 0
    std::vector<double> alpha(n - 1);
    for (size_t i = 1; i < n - 1; ++i) {
      alpha[i] = 3.0 / h[i] * (y[i + 1] - y[i]) - 3.0 / h[i - 1] * (y[i] - y[i - 1]);
    }
    
    // Solve tridiagonal system
    std::vector<double> c(n, 0.0);
    std::vector<double> l(n), mu(n), z(n);
    
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;
    
    for (size_t i = 1; i < n - 1; ++i) {
      l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
      if (std::abs(l[i]) < 1e-10) l[i] = 1e-10;
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    
    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c[n - 1] = 0.0;
    
    for (int i = n - 2; i >= 0; --i) {
      c[i] = z[i] - mu[i] * c[i + 1];
    }
    
    // Calculate a, b, d coefficients
    for (size_t i = 0; i < n - 1; ++i) {
      double a = y[i];
      double b = (y[i + 1] - y[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
      double d = (c[i + 1] - c[i]) / (3.0 * h[i]);
      result[i] = {a, b, c[i], d};
    }
    
    return result;
  }
  
  size_t findInterval(double t) const {
    if (t <= times_.front()) return 0;
    if (t >= times_.back()) return times_.size() - 2;
    
    // Binary search
    auto it = std::lower_bound(times_.begin(), times_.end(), t);
    size_t idx = std::distance(times_.begin(), it);
    if (idx > 0) --idx;
    return std::min(idx, times_.size() - 2);
  }
  
  size_t numJoints_;
  bool valid_;
  double tMin_, tMax_;
  std::vector<double> times_;
  std::vector<std::vector<std::array<double, 4>>> coeffs_;  // [joint][interval][a,b,c,d]
};

} // namespace arm_control
