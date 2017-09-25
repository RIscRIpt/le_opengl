#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

class octree {
public:
    typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Iterator iterator;

    friend class points;

    struct point {
        pcl::PointXYZ coord;
        int index;

        point(pcl::PointXYZ c, int id = -1);
        point(float x = 0.0f, float y = 0.0f, float z = 0.0f, int id = -1);
        point(int id);

        bool is_valid();
        inline void invalidate() { index = -1; };
        inline void validate() { index = -2; };
    };

    class points {
        friend class octree;

    public:
        struct iterator : public point {
            friend class points;

        public:
            iterator operator++();
            bool operator!=(const iterator &other);

        private:
            iterator(points &ps, point p);

            const points &ps;
        };
        iterator begin();
        iterator end();

    private:
        points(octree &tree);

        octree &tree;
    };

    octree(double resolution);

    iterator begin();
    iterator end();

    points get_points();

    void add_point(point p);
    point find_point(double radius, pcl::PointXYZ center);
    void move_point(pcl::PointXYZ new_coord, point &p);
    void delete_point(point p);

    void get_voxel_bounds(iterator voxel, Eigen::Vector3f &bmin, Eigen::Vector3f &bmax);

//private:
    boost::shared_ptr<std::vector<int>> indicies;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> cloudSearch;
};

