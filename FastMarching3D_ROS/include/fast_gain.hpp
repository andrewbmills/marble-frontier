#ifndef FAST_GAIN_HPP
#define FAST_GAIN_HPP

#include "raycast.hpp"
#include "gain.h"

struct SensorParams {
    float hfov, vfov, range, beam_angle;
};

class FastGainUpdater {
    private:
        Raycaster m_raycaster;
        SensorParams m_sensor_params;
        Eigen::MatrixXd m_beams;
        void initialize_beams();
    public:
        FastGainUpdater(MapGrid3D<float> &t_map, SensorParams t_sensor_params);
        void set_sensor_params(SensorParams sensor_params);
        std::vector<float> get_gains(std::vector<View> &goals);
};

FastGainUpdater::FastGainUpdater(MapGrid3D<float> &t_map, SensorParams t_sensor_params)
    : m_raycaster(t_map, -1.0),
    m_sensor_params{t_sensor_params}
{
    initialize_beams();
    return;
}

void FastGainUpdater::initialize_beams()
{
    unsigned m = floor(m_sensor_params.hfov/m_sensor_params.beam_angle) + 1;
    unsigned n = floor(m_sensor_params.vfov/m_sensor_params.beam_angle) + 1;
    m_beams = Eigen::MatrixXd(m*n, 3);
    for (int i=0; i<m; ++i) {
        float azimuth = -m_sensor_params.hfov/2.0 + i*m_sensor_params.beam_angle;
        for (int j=0; j<n; ++j) {
            float pitch = -m_sensor_params.vfov/2.0 + j*m_sensor_params.beam_angle;
            m_beams(i*n + j, 0) = std::cos(azimuth)*std::cos(pitch);
            m_beams(i*n + j, 1) = std::sin(azimuth)*std::cos(pitch);
            m_beams(i*n + j, 2) = std::sin(pitch);
        }
    }
    // std::cout << "The beams are:\n" << m_beams << std::endl;
    return;
}

void FastGainUpdater::set_sensor_params(SensorParams sensor_params)
{
    m_sensor_params = sensor_params;
    return;
}

std::vector<float> FastGainUpdater::get_gains(std::vector<View> &goals)
{
    unsigned beam_count = m_beams.rows();
    Eigen::MatrixXd rays(goals.size()*beam_count, 8);
    rays.setZero(goals.size()*beam_count, 8);
    for (int i=0; i<goals.size(); ++i) {
        Eigen::MatrixXd origin(1,3);
        Eigen::Matrix3d R = goals[i].pose.R.cast<double>();
        origin << goals[i].pose.position.x, goals[i].pose.position.y, goals[i].pose.position.z;
        rays.block(i*beam_count, 0, beam_count, 3) = origin.replicate(beam_count, 1);
        rays.block(i*beam_count, 3, beam_count, 3) = m_beams*R.transpose();
    }
    rays.block(0, 6, rays.rows(), 1) = m_sensor_params.range*Eigen::MatrixXd::Ones(rays.rows(), 1);

    std::cout << "First 3 pose rotation matrices:\n" << goals[0].pose.R << "\n\n" << goals[1].pose.R << "\n\n" << goals[2].pose.R << std::endl;
    std::cout << "Rays mat, first 10 rows:\n" << rays.block(0,0,10,8) << std::endl;

    // Perform raycasts
    m_raycaster.multicast(rays);

    std::cout << "Rays mat, first 10 rows:\n" << rays.block(0,0,10,8) << std::endl;

    // Compute small volumes
    float c = std::cos(m_sensor_params.beam_angle/2.0);
    float s = std::sin(m_sensor_params.beam_angle/2.0);
    float R = m_sensor_params.range;
    std::vector<float> gains(goals.size(), 0.0);
    for (int i=0; i<goals.size(); ++i) {
        for (int j=0; j<beam_count; ++j) {
            float r = rays(i*beam_count + j, 7);
            gains[i] += (R - r)*(r + R)*(r + R);
        }
        gains[i] = (1.0/3.0)*gains[i]*s*s*c;
    }
    return gains;
}

#endif