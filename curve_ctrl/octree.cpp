#include "octree.h"

#include <cmath>
#include <boost/make_shared.hpp>

octree::point::point(pcl::PointXYZ c, int id) {
    coord = c;
    index = id;
}

octree::point::point(float x, float y, float z, int id) {
    coord.x = x;
    coord.y = y;
    coord.z = z;
    index = id;
}

octree::point::point(int id) {
    index = id;
}

bool octree::point::is_valid() {
    return index != -1
        && coord.x != HUGE_VALF
        && coord.y != HUGE_VALF
        && coord.z != HUGE_VALF;
}

octree::points::points(octree &tree) :
    tree(tree)
{
}

octree::points::iterator octree::points::begin() {
    if (tree.cloud->size() > 0) {
        return iterator(*this, point(tree.cloud->at(0), 0));
    }
    return end();
}

octree::points::iterator octree::points::end() {
    return iterator(*this, point((int)tree.cloud->size()));
}

octree::points::iterator::iterator(points &ps, point p) :
    point(p),
    ps(ps)
{
}

octree::points::iterator octree::points::iterator::operator++() {
    do {
        ++index;
        if (index < (int)ps.tree.cloud->size()) {
            coord = ps.tree.cloud->at(index);
        } else {
            break;
        }
    } while(!is_valid());
    return *this;
}

bool octree::points::iterator::operator!=(const iterator &other) {
    return index != other.index;
}

octree::octree(double resolution) :
    indicies(new std::vector<int>),
    cloud(new pcl::PointCloud<pcl::PointXYZ>),
    cloudSearch(resolution)
{
    cloudSearch.setInputCloud(cloud, indicies);
}

octree::iterator octree::begin() {
    return cloudSearch.begin();
}

octree::iterator octree::end() {
    return cloudSearch.end();
}

octree::points octree::get_points() {
    return points(*this);
}

octree::point octree::find_point(double radius, pcl::PointXYZ center) {
    if (cloudSearch.getLeafCount() <= 0) {
        return point();
    }

    std::vector<int> indicies;
    std::vector<float> k_sqr_distances;

    if (radius < cloudSearch.getResolution()) {
        radius = cloudSearch.getResolution();
    }

    cloudSearch.radiusSearch(center, radius, indicies, k_sqr_distances);

    if (indicies.size() == 0) {
        return point();
    }

    return point(cloud->at(indicies[0]), indicies[0]);
}

void octree::add_point(octree::point p) {
    cloudSearch.addPointToCloud(p.coord, cloud, indicies);
}

void octree::move_point(pcl::PointXYZ new_coord, point &p) {
    cloudSearch.deleteVoxelAtPoint(p.coord);
    p.coord = new_coord;
    cloud->at(p.index) = new_coord;
    cloudSearch.addPointFromCloud(p.index, /*pointIndicies*/ nullptr);
}

void octree::delete_point(octree::point p) {
    cloudSearch.deleteVoxelAtPoint(p.coord);
    cloud->at(p.index).x = HUGE_VALF;
    // VERY SLOW!!!!!
    remove_if(indicies->begin(), indicies->end(), [p](int index) -> bool {
        return p.index == index;
    });
}

void octree::get_voxel_bounds(iterator voxel, Eigen::Vector3f &bmin, Eigen::Vector3f &bmax) {
    cloudSearch.getVoxelBounds(voxel, bmin, bmax);
}

