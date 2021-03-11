#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace boost::numeric::odeint;

std::vector<std::vector<double>> path_integrate_kinematics;
std::vector<std::vector<double>> states_integrate_kinematics;

typedef boost::array<double, 3> state_type_unicycle;
typedef boost::array<double, 4> state_type_unicycle3D;

struct L1ControlParams {
  double L1;
  double speed;
  double yaw_rate_max;
  double speed_max;
  double z_gain;
};

L1ControlParams L1params;
// L1params.L1 = 1.5; // meters
// L1params.speed = 1.0; // m/s
// L1params.yaw_rate_max = 0.4; // rad/s
// L1param.speed_max = 1.5; // m/s

struct L1Command {
  double speed;
  double turn_rate;
  double altitude_rate;
};

float angle_diff(float a, float b)
{
    // Computes a-b, preserving the correct sign (counter-clockwise positive angles)
    // All angles are in degrees
    a = std::fmod(360000.0 + a, 360.0);
    b = std::fmod(360000.0 + b, 360.0);
    float d = a - b;
    d = std::fmod(d + 180.0, 360.0) - 180.0;
    return d;
}

pair<int, double> findLookahead3D(std::vector<double> x, std::vector<double> &x_L1, double R)
{
  // Find the lookahead point at this distance
  int i_L1;
  int i_closest = 0;
  double c_closest = 10000.0;
  int i_max = path_integrate_kinematics.size()-2;
  int i = i_max;
  bool intersection = true;
  float t_hat = -1.0;
  std::vector<double> d(3);
  std::vector<double> p1(3);
  std::vector<double> p2(3);
  std::vector<double> q(3);

  while ((t_hat < 0.0) || (t_hat > 1.0)) {
    // If i==0, we've checked the whole path and there is no intersection
    if (i == 0) {
      intersection = false;
      break;
    }

    // Calculate intersection vectors
    for (int j=0; j<3; j++) {
      p1[j] = path_integrate_kinematics[i][j];
      p2[j] = path_integrate_kinematics[i+1][j];
      d[j] = p2[j] - p1[j]; // vector from p2 to p1
      q[j] = p1[j] - x[j]; // vector from p1 to x
    }

    // Solve quadratic equation for the intersection between a circle of radius R
    // centered at x with the vector d.
    double a = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
    double b = 2*(d[0]*q[0] + d[1]*q[1] + d[2]*q[2]);
    double c = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] - R*R;
    double discriminant = b*b - 4*a*c;
    if (discriminant <=0) {
      // no intersection
      t_hat = -1.0;
    }
    else {
      // 1 or 2 solutions exist (numerically it has to be 2)
      double t_hat1 = (-b + std::sqrt(discriminant)) / (2*a);
      double t_hat2 = (-b - std::sqrt(discriminant)) / (2*a);
      t_hat = std::max(t_hat1, t_hat2);
    }
    
    // Keep track of the closest point on the path for default behavior
    // when there is no intersection.
    if (c < c_closest) {
      c_closest = c;
      i_closest = i;
    }
    
    // Next iteration proceeding from the end of the path to the start.
    i--;
  }

  if (intersection) {
    x_L1[0] = d[0]*t_hat + p1[0];
    x_L1[1] = d[1]*t_hat + p1[1];
    x_L1[2] = d[2]*t_hat + p1[2];
    i_L1 = i+1;
  }
  else {
    // Set lookahead point to the closest path node
    x_L1[0] = path_integrate_kinematics[i_closest][0];
    x_L1[1] = path_integrate_kinematics[i_closest][1];
    x_L1[2] = path_integrate_kinematics[i_closest][2];
    i_L1 = i_closest+1;
  }

  return make_pair(i_L1, t_hat);
}

pair<int, double> findLookahead2D(std::vector<double> x, std::vector<double> &x_L1, double R)
{
  // Find the lookahead point at this distance
  int i_L1;
  int i_closest = path_integrate_kinematics.size()-1;
  double c_closest = 1000;
  int i_max = path_integrate_kinematics.size()-1;
  int i = i_max;
  bool intersection = true;
  float t_hat = -1.0;
  std::vector<double> d(x.size());
  std::vector<double> p1(x.size());
  std::vector<double> p2(x.size());
  std::vector<double> q(x.size());
  double d_norm = 0;
  for (int j=0; j<x.size(); j++) {
    p2[j] = path_integrate_kinematics[i][j];
    d[j] = p2[j] - x[j];
    d_norm += d[j]*d[j];
  }
  if (d_norm < 0.1*R*R) {
    for (int j=0; j<x.size(); j++) x_L1[j] = p2[j];
    i_L1 = i_max-1;
    t_hat = 1.0;
  } else {
    while ((t_hat < 0.0) || (t_hat > 1.0)) {
      // If i==0, we've checked the whole path and there is no intersection
      if (i == 0) {
        intersection = false;
        break;
      }

      // Calculate intersection vectors
      for (int j=0; j<x.size(); j++) {
        p1[j] = path_integrate_kinematics[i-1][j];
        p2[j] = path_integrate_kinematics[i][j];
        d[j] = p2[j] - p1[j]; // vector from p2 to p1
        q[j] = p1[j] - x[j]; // vector from p1 to x
      }

      // Solve quadratic equation for the intersection between a circle of radius R
      // centered at x with the vector d.
      double a = d[0]*d[0] + d[1]*d[1];
      double b = 2*(d[0]*q[0] + d[1]*q[1]);
      double c = q[0]*q[0] + q[1]*q[1] - R*R;
      double discriminant = b*b - 4*a*c;
      if (discriminant <=0) {
        // no intersection
        t_hat = -1.0;
      } else {
        // 1 or 2 solutions exist (numerically it has to be 2)
        double t_hat1 = (-b + std::sqrt(discriminant)) / (2*a);
        double t_hat2 = (-b - std::sqrt(discriminant)) / (2*a);
        t_hat = std::max(t_hat1, t_hat2);
      }
      
      // Keep track of the closest point on the path for default behavior
      // when there is no intersection.
      if (c < c_closest) {
        c_closest = c;
        i_closest = i;
      }
      
      // Next iteration proceeding from the end of the path to the start.
      i--;
    }

    if (intersection) {
      x_L1[0] = d[0]*t_hat + p1[0];
      x_L1[1] = d[1]*t_hat + p1[1];
      x_L1[2] = d[2]*t_hat + p1[2];
      i_L1 = i+1;
    }
    else {
      // Set lookahead point to the closest path node
      x_L1[0] = path_integrate_kinematics[i_closest][0];
      x_L1[1] = path_integrate_kinematics[i_closest][1];
      x_L1[2] = path_integrate_kinematics[i_closest][2];
      t_hat = 1.0;
      i_L1 = i_closest;
    }
  }
  return make_pair(i_L1, t_hat);
}

L1Command L1_lookahead_control(std::vector<double> x, L1ControlParams p)
{
  // https://users.soe.ucsc.edu/~elkaim/Documents/L2PlusACC2013Complete.pdf
  // Lateral acceleration-based path following control

  // Get lookahead intersection point
  std::vector<double> position{x[0], x[1], x[2]};
  std::vector<double> x_L1{0.0, 0.0, 0.0};
  pair<int, double> lookahead_id_t_hat = findLookahead2D(position, x_L1, p.L1);
  std::vector<double> x_L1_3D{0.0, 0.0, 0.0};
  int L1_id = lookahead_id_t_hat.first;
  double t_hat = lookahead_id_t_hat.second;
  
  // Calculate lateral acceleration
  L1Command command;
  std::vector<double> L1_vec {x_L1[0] - x[0], x_L1[1] - x[1], 0.0};
  double L1_length = sqrt(L1_vec[0]*L1_vec[0] + L1_vec[1]*L1_vec[1]);
  if (L1_length <= 0.2*L1params.L1 && (L1_id == (path_integrate_kinematics.size()-2))) {
    // Reached the end of the path
    command.turn_rate = 0.0;
    command.speed = 0.0;
  } else {
    double sin_eta = (cos(x[3])*L1_vec[1] - sin(x[3])*L1_vec[0])/(L1_length); // using the cross product between Vg and L1 to get sin(eta)
    command.turn_rate = 2*p.speed*sin_eta/(p.L1);
    command.speed = L1params.speed;
  }
  command.altitude_rate = L1params.z_gain*(x_L1[2] - x[2]);

  return command;
}


void unicycle(const state_type_unicycle &x, state_type_unicycle &dxdt, double t)
{
  // x = [x, y, yaw]
  // xdot = [u[0]*cos(yaw), u[1]*sin(yaw), u[1]]
  // u = [min(speed, speed_max), min(turn_rate, turn_max)]
  
  // Get control command from L1_lookahead
  std::vector<double> state{x[0], x[1], 0.0, x[2]};
  L1Command command = L1_lookahead_control(state, L1params);
  double turn_rate = command.turn_rate;
  double zdot = command.altitude_rate;

  std::vector<double> u(2);
  u[1] = min(max(turn_rate, -L1params.yaw_rate_max), L1params.yaw_rate_max);
  u[0] = command.speed;

  dxdt[0] = u[0]*std::cos(x[2]);
  dxdt[1] = u[0]*std::sin(x[2]);
  dxdt[2] = u[1];
  return;
}

void unicycle3D(const state_type_unicycle3D &x, state_type_unicycle3D &dxdt, double t)
{
  // x = [x, y, z, yaw]
  // xdot = [u[0]*cos(yaw), u[0]*sin(yaw), u[1], u[2]]
  // u = [speed, zdot, turn_rate]
  
  // Get control command from L1_lookahead
  std::vector<double> state{x[0], x[1], x[2], x[3]};
  L1Command command = L1_lookahead_control(state, L1params);
  double turn_rate = command.turn_rate;
  double zdot = command.altitude_rate;

  std::vector<double> u(3);
  u[0] = command.speed;
  u[1] = zdot;
  u[2] = min(max(turn_rate, -L1params.yaw_rate_max), L1params.yaw_rate_max);

  dxdt[0] = u[0]*std::cos(x[3]);
  dxdt[1] = u[0]*std::sin(x[3]);
  dxdt[2] = u[1];
  dxdt[3] = u[2];
  return;
}

void unicycleObserver(const state_type_unicycle &x, const double t)
{
  std::vector<double> state{t, x[0], x[1], x[2]};
  states_integrate_kinematics.push_back(state);
  return;
}

void unicycleObserver3D(const state_type_unicycle3D &x, const double t)
{
  // Check if the path is finished
  double L1 = 0;
  for (int i=0; i<2; i++) L1 += (x[i] - path_integrate_kinematics[path_integrate_kinematics.size()-1][i])*(x[i] - path_integrate_kinematics[path_integrate_kinematics.size()-1][i]);
  if (L1 >= (0.3*L1params.L1)*(0.3*L1params.L1)) {
    std::vector<double> state{t, x[0], x[1], x[2], x[3]};
    states_integrate_kinematics.push_back(state);
  }
  return;
}

// int main(int argc, char **argv)
// {
//     state_type x = { 10.0 , 1.0 , 1.0 }; // initial conditions
//     // integrate( lorenz , x , 0.0 , 25.0 , 0.1 , write_lorenz );
// }

#endif