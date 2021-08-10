#ifndef RAYCAST_HPP
#define RAYCAST_HPP

#include <math.h>
#include <Eigen/Dense>
#include <octomap/OcTree.h>
#include "mapGrid3D.h"

// class Caster

float sign(float a)
{
    return (a < 0 ? -1.0 : 1.0);
}

float raycast(Eigen::Vector3d origin, Eigen::Vector3d direction, float radius, unsigned (*check_occupied_or_unseen)(Eigen::Vector3d voxel))
{
    // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
    // by John Amanatides and Andrew Woo, 1987
    // <http://www.cse.yorku.ca/~amana/research/grid.pdf>

    // direction is assumed to be a unit vector
    // voxels are assumed to be unit cubes centered @ the nearest integer
    
    // Initialization
    float t = 0;
    float step_x = sign(direction(0));
    float step_y = sign(direction(1));
    float step_z = sign(direction(2));
    float t_dx = direction(0);
    float t_dy = direction(1);
    float t_dz = direction(2);
    float t_max_x = (std::abs(t_dx) > 0.001) ? 0.5/t_dx : 1.2*radius;
    float t_max_y = (std::abs(t_dy) > 0.001) ? 0.5/t_dy : 1.2*radius;
    float t_max_z = (std::abs(t_dz) > 0.001) ? 0.5/t_dz : 1.2*radius;
    Eigen::Vector3d current = origin;
    unsigned occupied_or_unseen = 0;

    // Do the cast
    while (t < radius && !occupied_or_unseen) {
        if (t_max_x < t_max_y) {
            if (t_max_x < t_max_z) {
                current(0) += step_x;
                t_max_x += t_dx;
            } else {
                current(2) += step_z;
                t_max_z += t_dz;
            }
        } else {
            if (t_max_y < t_max_z) {
                current(1) += step_y;
                t_max_y += t_dy;
            } else {
                current(2) += step_z;
                t_max_z += t_dz;
            }
        }
        t = std::min(std::min(t_max_x, t_max_y), t_max_z);
        occupied_or_unseen = check_occupied_or_unseen(current);
    }
    if (occupied_or_unseen == 0) return radius;
    else if (occupied_or_unseen == 1) return radius;
    else return t;
}

class CollisionChecker {
    public:
        CollisionChecker();
        virtual unsigned check_occupied_or_unseen(Eigen::Vector3d p);
};

class OctomapChecker: public CollisionChecker {
    private:
        octomap::OcTree* m_map;
    public:
        OctomapChecker(octomap::OcTree* t_map);
        ~OctomapChecker() {
            delete m_map;
            return;
        }
        unsigned check_occupied_or_unseen(Eigen::Vector3d p);
};

OctomapChecker::OctomapChecker(octomap::OcTree* t_map)
{
    m_map = t_map;
    return;
}


unsigned OctomapChecker::check_occupied_or_unseen(Eigen::Vector3d p)
{
    octomap::OcTreeNode* node = m_map->search((double)p(0), (double)p(1), (double)p(2));
    bool unseen = ((!(node)) || ((node->getOccupancy() >= 0.45) && (node->getOccupancy() <= 0.55)));
    delete node;
    if (unseen) return 2;
    else if (node->getOccupancy() > 0.55) return 1;
    else return 0;
}

class MapGridChecker: public CollisionChecker {
    private:
        MapGrid3D<float> m_map;
        float m_default_value;
    public:
        MapGridChecker(MapGrid3D<float> &t_map, float t_default_value);
        unsigned check_occupied_or_unseen(Eigen::Vector3d p);
};

MapGridChecker::MapGridChecker(MapGrid3D<float> &t_map, float t_default_value)
{
    m_map = t_map;
    m_default_value = t_default_value;
    return;
}

unsigned MapGridChecker::check_occupied_or_unseen(Eigen::Vector3d p)
{
    Point position = {p(0), p(1), p(2)};
    if (!m_map._CheckVoxelPositionInBounds(position)) return 2;

    float edt_value = m_map.Query((double)p(0), (double)p(1), (double)p(2));
    if (edt_value == m_default_value) return 2;
    else if (edt_value <= 0.0) return 1;
    else return 0;
}

#endif