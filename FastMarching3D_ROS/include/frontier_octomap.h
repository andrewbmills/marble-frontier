#ifndef FRONTIER_OCTOMAP_H
#define FRONTIER_OCTOMAP_H
#include <frontier.h>
#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>

bool IsUnseen(double x, double y, double z, octomap::OcTree* tree)
{
    octomap::OcTreeNode* node = tree->search(x, y, z);
    return ((!(node)) || ((node->getOccupancy() >= 0.45) && (node->getOccupancy() <= 0.55)));
}

bool IsAdjacentToUnseen(double x, double y, double z, octomap::OcTree* tree)
{
    double voxel_size = tree->getResolution();
    return IsUnseen(x + voxel_size,y,z,tree)+IsUnseen(x - voxel_size,y,z,tree)+
           IsUnseen(x,y + voxel_size,z,tree)+IsUnseen(x,y - voxel_size,z,tree)+
           IsUnseen(x,y,z + voxel_size,tree)+IsUnseen(x,y,z - voxel_size,tree);
}

std::vector<Point> CalculateFrontierFromOctomap(octomap::OcTree* tree)
{
    std::vector<Point> frontiers;
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!=end; ++it) {
        bool isFree = (it->getOccupancy() <= 0.5);
        if (isFree) {
            if (IsAdjacentToUnseen(it.getX(), it.getY(), it.getZ(), tree)) {
                Point frontier {(float)it.getX(), (float)it.getY(), (float)it.getZ()};
                frontiers.push_back(frontier);
            }
        }
    }
    return frontiers;
}

std::vector<Point> CalculateFrontierFromOctomap_OMP(octomap::OcTree* tree)
{
    std::vector<Point> frontiers;
    std::vector<Point> free_voxels;
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!=end; ++it) {
        bool isFree = (it->getOccupancy() <= 0.5);
        if (isFree) {
            Point free_voxel = {it.getX(), it.getY(), it.getZ()};
            free_voxels.push_back(free_voxel);
        }
    }

    std::vector<bool> isFrontier = std::vector<bool>(free_voxels.size());
    std::fill(isFrontier.begin(), isFrontier.end(), false);
    #pragma omp parallel num_threads(11)
    {
        #pragma omp for
        for(int i=0; i<free_voxels.size(); i++) {
            if (IsAdjacentToUnseen(free_voxels[i].x, free_voxels[i].y, free_voxels[i].z, tree)) {
                isFrontier[i] = true;
            }
        }
    }

    for (int i=0; i<isFrontier.size(); i++)
    {
        if (isFrontier[i]) frontiers.push_back(free_voxels[i]);
    }
    return frontiers;
}

#endif