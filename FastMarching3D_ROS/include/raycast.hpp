#ifndef RAYCAST_HPP
#define RAYCAST_HPP

#include <math.h>
#include <Eigen/Dense>
#include <octomap/OcTree.h>
#include <omp.h>
#include "mapGrid3D.h"

// class Caster

float sign(float a)
{
    return (a < 0 ? -1.0 : 1.0);
}
class Raycaster {
    private:
        MapGrid3D<float> m_map;
        float m_default_map_value;
    public:
        Raycaster(MapGrid3D<float> &t_map, float t_default_map_value);
        unsigned check_occupied_or_unseen(Eigen::Vector3d p);
        float cast(Eigen::Vector3d origin, Eigen::Vector3d direction, float radius);
        void multicast(Eigen::MatrixXd &rays);
        void multicast_omp(Eigen::MatrixXd &rays);
};

Raycaster::Raycaster(MapGrid3D<float> &t_map, float t_default_map_value)
{
    m_map = t_map;
    m_default_map_value = t_default_map_value;
    return;
}

unsigned Raycaster::check_occupied_or_unseen(Eigen::Vector3d p)
{
    float edt_value = m_map.Query((double)p(0), (double)p(1), (double)p(2));
    if (edt_value > 0.01) return 0;
    else if (edt_value == m_default_map_value) return 2;
    else return 1;
}

float Raycaster::cast(Eigen::Vector3d origin, Eigen::Vector3d direction, float radius)
{
    // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
    // by John Amanatides and Andrew Woo, 1987
    // <http://www.cse.yorku.ca/~amana/research/grid.pdf>

    // direction is assumed to be a unit vector

    // Initialization
    // ROS_INFO("Casting ray of length %0.1f from [%0.2f, %0.2f, %0.2f] in direction [%0.2f, %0.2f, %0.2f]", radius,
    //          origin(0), origin(1), origin(2), direction(0), direction(1), direction(2));
    // ROS_INFO("Inside of map with minBounds of [%0.2f, %0.2f, %0.2f] and voxel size %0.1f.",
    //          m_map.minBounds.x, m_map.minBounds.y, m_map.minBounds.z, m_map.voxelSize);
    float t = 0;
    int step_x = sign(direction(0));
    int step_y = sign(direction(1))*m_map.size.x;
    int step_z = sign(direction(2))*m_map.size.x*m_map.size.y;
    float nearest_voxel_x = std::round((origin(0) - m_map.minBounds.x)/m_map.voxelSize)*m_map.voxelSize + m_map.minBounds.x;
    float t_max_x = m_map.voxelSize/2.0 - sign(direction(0))*(origin(0) - nearest_voxel_x);
    float nearest_voxel_y = std::round((origin(1) - m_map.minBounds.y)/m_map.voxelSize)*m_map.voxelSize + m_map.minBounds.y;
    float t_max_y = m_map.voxelSize/2.0 - sign(direction(1))*(origin(1) - nearest_voxel_y);
    float nearest_voxel_z = std::round((origin(2) - m_map.minBounds.z)/m_map.voxelSize)*m_map.voxelSize + m_map.minBounds.z;
    float t_max_z = m_map.voxelSize/2.0 - sign(direction(2))*(origin(2) - nearest_voxel_z);
    float t_dx = (std::abs(direction(0)) > 0.001) ? m_map.voxelSize/std::abs(direction(0)) : 1.2*radius;
    float t_dy = (std::abs(direction(1)) > 0.001) ? m_map.voxelSize/std::abs(direction(1)) : 1.2*radius;
    float t_dz = (std::abs(direction(2)) > 0.001) ? m_map.voxelSize/std::abs(direction(2)) : 1.2*radius;
    Eigen::Vector3d current = origin;
    unsigned occupied_or_unseen = 0;
    float edt_value = 0.1;
    float t_update_edt = 0.0;
    Point query{origin(0), origin(1), origin(2)};
    int current_id = m_map._ConvertPositionToIndex(query);

    // Do the cast
    while (t < radius && edt_value > 0.01) {
        if (t_max_x < t_max_y) {
            if (t_max_x < t_max_z) {
                current_id += step_x;
                t_max_x += t_dx;
            } else {
                current_id += step_z;
                t_max_z += t_dz;
            }
        } else {
            if (t_max_y < t_max_z) {
                current_id += step_y;
                t_max_y += t_dy;
            } else {
                current_id += step_z;
                t_max_z += t_dz;
            }
        }
        t = std::min(std::min(t_max_x, t_max_y), t_max_z);
        edt_value = m_map.Query(current_id);
    }
    if (edt_value > 0.01) return radius; // last voxel was free
    else if (edt_value == m_default_map_value) return t; // last voxel was unseen
    else return -t; // last voxel was occupied
}

void Raycaster::multicast(Eigen::MatrixXd &rays)
{
    // rays is an N x 8 matrix
    // cols 1-3 are the origin of the rays
    // cols 4-6 are the direction of the rays
    // col 7 is the range
    // col 8 stores the stopping distance of the ray once it's cast
    int N = rays.rows();
    for (int i=0; i<N; i++) {
        Eigen::Vector3d origin;
        origin << rays(i,0), rays(i,1), rays(i,2);
        Eigen::Vector3d direction;
        direction << rays(i,3), rays(i,4), rays(i,5);
        float range = rays(i,6);
        rays(i,7) = cast(origin, direction, range);
    }
    return;
}

void Raycaster::multicast_omp(Eigen::MatrixXd &rays)
{
    // rays is an N x 8 matrix
    // cols 1-3 are the origin of the rays
    // cols 4-6 are the direction of the rays
    // col 7 is the range
    // col 8 stores the stopping distance of the ray once it's cast
    int N = rays.rows();
    #pragma omp parallel num_threads(11)
    {
        #pragma omp for
        for (int i=0; i<N; i++) {
            Eigen::Vector3d origin;
            origin << rays(i,0), rays(i,1), rays(i,2);
            Eigen::Vector3d direction;
            direction << rays(i,3), rays(i,4), rays(i,5);
            float range = rays(i,6);
            rays(i,7) = cast(origin, direction, range);
        }
    }
    return;
}


#endif