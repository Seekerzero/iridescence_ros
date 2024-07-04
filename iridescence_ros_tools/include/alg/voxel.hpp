#ifndef VOXEL_HPP
#define VOXEL_HPP

#include <Eigen/Core>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <functional>

#define HASH_P 114514

// #define Max_Voxel = 10000000000


namespace irtools{
    
    typedef struct VoxelConfig{
        /*point cloud pre-process*/
        double downsample_resolution_ = 0.1;
        int max_corner_num_ = 50;

        /* for key points*/
        double voxel_size_ = 0.1;
        int voxel_init_num_ = 10;
        double plane_merge_thre_ = 0.1;
        double plane_merge_dis_thre_ = 0.1;
        double plane_detection_thre_ = 0.1;

        double corner_thre_ = 10; 
    };

    struct VoxelHash{
        std::size_t operator()(const Eigen::Vector3d& vec) const {
            std::size_t hx = std::hash<double>()(vec(0));
            std::size_t hy = std::hash<double>()(vec(1));
            std::size_t hz = std::hash<double>()(vec(2));
            
            return hx ^ (hy << 1) ^ (hz << 2);
        }
    };

    class Voxel{
        public:
            Eigen::Vector3d position;
            Eigen::Vector3d size;
            Eigen::Vector3d normal;
            float intensity;


            Voxel(Eigen::Vector3d position, Eigen::Vector3d size, Eigen::Vector3d normal, float inensity = 0.0):
                position(position), size(size), normal(normal), intensity(intensity){};

            bool operator==(const Voxel& other) const{
                return position == other.position && size == other.size && normal == other.normal;
            }

            std::size_t operator()(const Voxel& voxel) const {
                VoxelHash hash_fn;
                return hash_fn(voxel.position) * HASH_P ^ hash_fn(voxel.size) * HASH_P;
            }
    };



    typedef struct Plane{
        pcl::PointXYZINormal p_center_;
        Eigen::Vector3d normal_;
        Eigen::Vector3d center_;
        Eigen::Matrix3d covariance_;

        float radius_ = 0.0;
        float min_eigen_value_ = 0.0;
        float intercept_ = 0.0;
        
        int id_ = 0;
        int sub_plane_num_ = 0;
        int points_num_ = 0;
        bool is_plane_ = true;

    }Plane;

    struct M_POINT{
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        float intensity;
    };

    class OctoTree{
        public:
        VoxelConfig config_;
        std::vector<Eigen::Vector3d> voxel_points_;
        Plane *plane_ptr_;

        int layer_;
        int octo_state_;
        int merge_num_ = 0;
        bool is_project_ = false;
        std::vector<Eigen::Vector3d> proj_normal_vec_;

        //check 6 directions of the voxel
        bool is_check_connect_[6];
        bool connect_state_[6];
        OctoTree *connect_voxel_[6];

        OctoTree *leaves_[8];

        Eigen::Vector3d center_;
        float quarter_length_;

        bool init_state_ = false;


        OctoTree(const VoxelConfig &config):config_(config){
                for(int i = 0; i < 6; i++){
                    is_check_connect_[i] = false;
                    connect_state_[i] = false;
                    connect_voxel_[i] = nullptr;
                }

                for(int i = 0; i < 8; i++){
                    leaves_[i] = nullptr;
                }
            }
        
        void init_plane();
        void init_octo_tree();

        
    };




    
}











#endif // VOXEL_HPP