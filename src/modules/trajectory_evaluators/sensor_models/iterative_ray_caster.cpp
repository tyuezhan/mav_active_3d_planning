#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/iterative_ray_caster.h"

#include <Eigen/Dense>

#include <vector>
#include <algorithm>
#include <cmath>

namespace mav_active_3d_planning {
    namespace sensor_models {

        ModuleFactory::Registration <IterativeRayCaster> IterativeRayCaster::registration("IterativeRayCaster");

        void IterativeRayCaster::setupFromParamMap(Module::ParamMap *param_map) {
            CameraModel::setupFromParamMap(param_map);
            setParam<double>(param_map, "ray_step", &p_ray_step_, (double) c_voxel_size_);
            setParam<double>(param_map, "downsampling_factor", &p_downsampling_factor_, 1.0);

            // Downsample to voxel size resolution at max range
            c_res_x_ = std::min(
                    (int) ceil(p_ray_length_ * c_field_of_view_x_ / ((double) c_voxel_size_ * p_downsampling_factor_)),
                    p_resolution_x_);
            c_res_y_ = std::min(
                    (int) ceil(p_ray_length_ * c_field_of_view_y_ / ((double) c_voxel_size_ * p_downsampling_factor_)),
                    p_resolution_y_);

            // Determine number of splits + split distances
            c_n_sections_ = (int) std::floor(std::log2(std::min((double) c_res_x_, (double) c_res_y_)));
            c_split_widths_.push_back(0);
            for (int i = 0; i < c_n_sections_; ++i) {
                c_split_widths_.push_back(std::pow(2, i));
                c_split_distances_.push_back(p_ray_length_ / std::pow(2.0, (double) i));
            }
            c_split_distances_.push_back(0.0);
            std::reverse(c_split_distances_.begin(), c_split_distances_.end());
            std::reverse(c_split_widths_.begin(), c_split_widths_.end());
        }

        bool
        IterativeRayCaster::getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                             const Eigen::Quaterniond &orientation) {
            // Setup ray table (contains at which segment to start, -1 if occluded
            ray_table_ = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

            // Ray-casting
            Eigen::Vector3d camera_direction;
            Eigen::Vector3d direction;
            Eigen::Vector3d current_position;
            double distance;
            bool cast_ray;
            double map_distance;
            for (int i = 0; i < c_res_x_; ++i) {
                for (int j = 0; j < c_res_y_; ++j) {
                    int current_segment = ray_table_(i, j); // get ray starting segment
                    if (current_segment < 0) {
                        continue;   // already occluded ray
                    }
                    CameraModel::getDirectionVector(&camera_direction, (double) i / ((double) c_res_x_ - 1.0),
                                                    (double) j / ((double) c_res_y_ - 1.0));
                    direction = orientation * camera_direction;
                    distance = c_split_distances_[current_segment];
                    cast_ray = true;
                    while (cast_ray) {
                        // iterate through all splits (segments)
                        while (distance < c_split_distances_[current_segment + 1]) {
                            current_position = position + distance * direction;
                            distance += p_ray_step_;

                            // Check voxel occupied
                            map_distance = 0.0;
                            if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(current_position, &map_distance)) {
                                if (map_distance < 0.0) {
                                    // Occlusion, mark neighboring rays as occluded
                                    markNeighboringRays(i, j, current_segment, -1);
                                    cast_ray = false;
                                    break;
                                }
                            }

                            // Add point (duplicates are handled in CameraModel::getVisibleVoxelsFromTrajectory)
                            getVoxelCenter(&current_position);
                            result->push_back(current_position);
                        }
                        if (cast_ray) {
                            current_segment++;
                            if (current_segment >= c_n_sections_) {
                                cast_ray = false; // done
                            } else {
                                // update ray starts of neighboring rays
                                markNeighboringRays(i, j, current_segment - 1, current_segment);
                            }
                        }
                    }
                }
            }
            return true;
        }

        void IterativeRayCaster::markNeighboringRays(int x, int y, int segment, int value) {
            // Set all nearby (towards bottom right) ray starts, depending on the segment depth, to a value.
            for (int i = x; i < std::min(c_res_x_, x + c_split_widths_[segment]); ++i) {
                for (int j = y; j < std::min(c_res_y_, y + c_split_widths_[segment]); ++j) {
                    ray_table_(i, j) = value;
                }
            }
        }

    } // namespace sensor_models
} // namepsace mav_active_3d_planning