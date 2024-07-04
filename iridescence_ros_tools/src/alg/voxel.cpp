#include <alg/voxel.hpp>


namespace irtools{

    /*This function takes the voxel points from the octotree to see if the tree points are laid on a plane*/
    void OctoTree::init_plane(){
        plane_ptr_->covariance_ = Eigen::Matrix3d::Zero();
        plane_ptr_->center_ = Eigen::Vector3d::Zero();
        plane_ptr_->normal_ = Eigen::Vector3d::Zero();
        plane_ptr_->points_num_ = voxel_points_.size();
        plane_ptr_->radius_ = 0.0;

        for (auto &point : voxel_points_){
            plane_ptr_->covariance_ += point * point.transpose();
            plane_ptr_->center_ += point;
        }

        plane_ptr_->center_ /= plane_ptr_->points_num_;
        plane_ptr_->covariance_ = plane_ptr_->covariance_ / plane_ptr_->points_num_ - plane_ptr_->center_ * plane_ptr_->center_.transpose();

        Eigen::EigenSolver<Eigen::Matrix3d> es(plane_ptr_->covariance_);
        Eigen::Matrix3cd eigen_vectors = es.eigenvectors();
        Eigen::Vector3cd eigen_values = es.eigenvalues();
        Eigen::Vector3d eigen_values_real = eigen_values.real();
        Eigen::Matrix3d::Index evals_min_index, evals_max_index;
        // eigen_values_real.minCoeff(&evals_min_index);
        // eigen_values_real.maxCoeff(&evals_max_index);
        eigen_values_real.rowwise().sum().minCoeff(&evals_min_index);
        eigen_values_real.rowwise().sum().maxCoeff(&evals_max_index);

        int evals_mid_index = 3 - evals_min_index - evals_max_index;


        if(eigen_values_real(evals_min_index) < config_.plane_detection_thre_){
            plane_ptr_->normal_ << eigen_vectors(0, evals_min_index).real(), eigen_vectors(1, evals_min_index).real(), eigen_vectors(2, evals_min_index).real();
            plane_ptr_->min_eigen_value_ = eigen_values_real(evals_min_index);
            plane_ptr_->radius_ = sqrt(eigen_values_real(evals_max_index));
            plane_ptr_->intercept_ = -plane_ptr_->normal_.dot(plane_ptr_->center_);
            plane_ptr_->radius_ = sqrt(eigen_values_real(evals_max_index));
            plane_ptr_->is_plane_ = true;

            plane_ptr_->p_center_.x = plane_ptr_->center_(0);
            plane_ptr_->p_center_.y = plane_ptr_->center_(1);
            plane_ptr_->p_center_.z = plane_ptr_->center_(2);

            plane_ptr_->p_center_.normal_x = plane_ptr_->normal_(0);
            plane_ptr_->p_center_.normal_y = plane_ptr_->normal_(1);
            plane_ptr_->p_center_.normal_z = plane_ptr_->normal_(2);
        }
        else{
            plane_ptr_->is_plane_ = false;
        }

    }

    void OctoTree::init_octo_tree(){
        if (voxel_points_.size() > config_.voxel_init_num_){
            init_plane();
        }
    }



}