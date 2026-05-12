/**
 * @file planning_env.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the world representation using point clouds
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <planning_env/planning_env.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace planning_env_ns
{
void PlanningEnvParameters::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("kSurfaceCloudDwzLeafSize", kSurfaceCloudDwzLeafSize);
  nh->get_parameter("kCollisionCloudDwzLeafSize", kCollisionCloudDwzLeafSize);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckRadius", kKeyposeGraphCollisionCheckRadius);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckPointNumThr", kKeyposeGraphCollisionCheckPointNumThr);
  nh->get_parameter("kKeyposeCloudStackNum", kKeyposeCloudStackNum);
  nh->get_parameter("kPointCloudRowNum", kPointCloudRowNum);
  nh->get_parameter("kPointCloudColNum", kPointCloudColNum);
  nh->get_parameter("kPointCloudLevelNum", kPointCloudLevelNum);
  nh->get_parameter("kMaxCellPointNum", kMaxCellPointNum);
  nh->get_parameter("kPointCloudCellSize", kPointCloudCellSize);
  nh->get_parameter("kPointCloudCellHeight", kPointCloudCellHeight);
  nh->get_parameter("kPointCloudManagerNeighborCellNum", kPointCloudManagerNeighborCellNum);
  nh->get_parameter("kCoverCloudZSqueezeRatio", kCoverCloudZSqueezeRatio);
  nh->get_parameter("kUseFrontier", kUseFrontier);
  nh->get_parameter("kFrontierClusterTolerance", kFrontierClusterTolerance);
  nh->get_parameter("kFrontierClusterMinSize", kFrontierClusterMinSize);
  nh->get_parameter("kUseCoverageBoundaryOnFrontier", kUseCoverageBoundaryOnFrontier);
  nh->get_parameter("kUseCoverageBoundaryOnObjectSurface", kUseCoverageBoundaryOnObjectSurface);
  int viewpoint_number = nh->get_parameter("viewpoint_manager/number_x").as_int();
  double viewpoint_resolution = nh->get_parameter("viewpoint_manager/resolution_x").as_double();
  double sensor_range = nh->get_parameter("kSensorRange").as_double();

  double local_planning_horizon_half_size = viewpoint_number * viewpoint_resolution / 2;
  kExtractFrontierRange.x() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.y() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.z() = 2;
}

PlanningEnv::PlanningEnv(rclcpp::Node::SharedPtr nh, std::string world_frame_id)
  : keypose_cloud_count_(0)
  , vertical_surface_extractor_()
  , vertical_frontier_extractor_()
  , robot_position_update_(false)
  , robot_yaw_(0.0)
  , robot_coverage_horizontal_fov_deg_(360.0)
{
  parameters_.ReadParameters(nh);

  keypose_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < keypose_cloud_stack_.size(); i++)
  {
    keypose_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }

  vertical_surface_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < vertical_surface_cloud_stack_.size(); i++)
  {
    vertical_surface_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }
  keypose_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", world_frame_id);
  stacked_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "stacked_cloud", world_frame_id);
  stacked_vertical_surface_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "stacked_vertical_surface_cloud", world_frame_id);

  stacked_vertical_surface_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());
  vertical_surface_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "coverage_cloud", world_frame_id);

  diff_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "diff_cloud", world_frame_id);

  collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  terrain_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud", world_frame_id);

  planner_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "planner_cloud", world_frame_id);

  pointcloud_manager_ = std::make_shared<pointcloud_manager_ns::PointCloudManager>(
      parameters_.kPointCloudRowNum, parameters_.kPointCloudColNum, parameters_.kPointCloudLevelNum,
      parameters_.kMaxCellPointNum, parameters_.kPointCloudCellSize, parameters_.kPointCloudCellHeight,
      parameters_.kPointCloudManagerNeighborCellNum);
  pointcloud_manager_->SetCloudDwzFilterLeafSize() = parameters_.kSurfaceCloudDwzLeafSize;

  rolling_occupancy_grid_ = std::make_shared<rolling_occupancy_grid_ns::RollingOccupancyGrid>(nh);

  squeezed_planner_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "squeezed_planner_cloud", world_frame_id);

  squeezed_planner_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());

  uncovered_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_cloud", world_frame_id);
  uncovered_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_frontier_cloud", world_frame_id);
  frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_cloud", world_frame_id);
  filtered_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "filtered_frontier_cloud", world_frame_id);
  occupied_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "occupied_cloud", world_frame_id);
  free_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "free_cloud", world_frame_id);
  unknown_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "unknown_cloud", world_frame_id);

  rolling_occupancy_grid_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_occupancy_grid_cloud", world_frame_id);

  rolling_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolling_frontier_cloud", world_frame_id);

  rolling_filtered_frontier_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_filtered_frontier_cloud", world_frame_id);

  rolled_in_occupancy_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_in_occupancy_cloud", world_frame_id);
  rolled_out_occupancy_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_out_occupancy_cloud", world_frame_id);

  pointcloud_manager_occupancy_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "pointcloud_manager_occupancy_cloud_", world_frame_id);

  kdtree_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
  kdtree_rolling_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);

  // Todo: parameterize
  vertical_surface_extractor_.SetRadiusThreshold(0.2);
  vertical_surface_extractor_.SetZDiffMax(2.0);
  vertical_surface_extractor_.SetZDiffMin(parameters_.kSurfaceCloudDwzLeafSize);
  vertical_frontier_extractor_.SetNeighborThreshold(2);

  Eigen::Vector3d rolling_occupancy_grid_resolution = rolling_occupancy_grid_->GetResolution();
  double vertical_frontier_neighbor_search_radius =
      std::max(rolling_occupancy_grid_resolution.x(), rolling_occupancy_grid_resolution.y());
  vertical_frontier_neighbor_search_radius =
      std::max(vertical_frontier_neighbor_search_radius, rolling_occupancy_grid_resolution.z());
  vertical_frontier_extractor_.SetRadiusThreshold(vertical_frontier_neighbor_search_radius);
  double z_diff_max = vertical_frontier_neighbor_search_radius * 5;
  double z_diff_min = vertical_frontier_neighbor_search_radius;
  vertical_frontier_extractor_.SetZDiffMax(z_diff_max);
  vertical_frontier_extractor_.SetZDiffMin(z_diff_min);
  vertical_frontier_extractor_.SetNeighborThreshold(2);
}

void PlanningEnv::UpdateCollisionCloud()
{
  collision_cloud_->clear();
  for (int i = 0; i < parameters_.kKeyposeCloudStackNum; i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud<PlannerCloudPointType, pcl::PointXYZI>(*vertical_surface_cloud_stack_[i], *cloud_tmp);
    *(collision_cloud_) += *cloud_tmp;
  }
  collision_cloud_downsizer_.Downsize(collision_cloud_, parameters_.kCollisionCloudDwzLeafSize,
                                      parameters_.kCollisionCloudDwzLeafSize, parameters_.kCollisionCloudDwzLeafSize);
}

void PlanningEnv::UpdateFrontiers()
{
  if (parameters_.kUseFrontier)
  {
    prev_robot_position_ = robot_position_;
    rolling_occupancy_grid_->GetFrontier(frontier_cloud_->cloud_, robot_position_, parameters_.kExtractFrontierRange);
    debug_stats_.raw_frontier_points = frontier_cloud_->cloud_->points.size();
    debug_stats_.vertical_filtered_frontier_points = 0;
    debug_stats_.clustered_frontier_points = 0;

    if (!frontier_cloud_->cloud_->points.empty())
    {
      if (parameters_.kUseCoverageBoundaryOnFrontier && !coverage_boundary_.points.empty())
      {
        GetCoverageCloudWithinBoundary<pcl::PointXYZI>(frontier_cloud_->cloud_);
      }
      vertical_frontier_extractor_.ExtractVerticalSurface<pcl::PointXYZI, pcl::PointXYZI>(
          frontier_cloud_->cloud_, filtered_frontier_cloud_->cloud_);
    }
    else
    {
      filtered_frontier_cloud_->cloud_->clear();
    }
    debug_stats_.vertical_filtered_frontier_points = filtered_frontier_cloud_->cloud_->points.size();

    // Cluster frontiers
    if (!filtered_frontier_cloud_->cloud_->points.empty())
    {
      kdtree_frontier_cloud_->setInputCloud(filtered_frontier_cloud_->cloud_);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(parameters_.kFrontierClusterTolerance);
      ec.setMinClusterSize(1);
      ec.setMaxClusterSize(10000);
      ec.setSearchMethod(kdtree_frontier_cloud_);
      ec.setInputCloud(filtered_frontier_cloud_->cloud_);
      ec.extract(cluster_indices);

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      int cluster_count = 0;
      for (int i = 0; i < cluster_indices.size(); i++)
      {
        if (cluster_indices[i].indices.size() < parameters_.kFrontierClusterMinSize)
        {
          continue;
        }
        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
        {
          int point_ind = cluster_indices[i].indices[j];
          filtered_frontier_cloud_->cloud_->points[point_ind].intensity = cluster_count;
          inliers->indices.push_back(point_ind);
        }
        cluster_count++;
      }
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(filtered_frontier_cloud_->cloud_);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*(filtered_frontier_cloud_->cloud_));
      debug_stats_.clustered_frontier_points = filtered_frontier_cloud_->cloud_->points.size();
      filtered_frontier_cloud_->Publish();
    }
    else
    {
      debug_stats_.clustered_frontier_points = 0;
    }
  }
}

void PlanningEnv::UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  if (cloud->points.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("standalone_logger"), "Terrain cloud empty");
  }
  else
  {
    terrain_cloud_->cloud_ = cloud;
  }
}

bool PlanningEnv::InCollision(double x, double y, double z) const
{
  if (stacked_cloud_->cloud_->points.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("standalone_logger"),
                "PlanningEnv::InCollision(): collision cloud empty, not checking collision");
    return false;
  }
  PlannerCloudPointType check_point;
  check_point.x = x;
  check_point.y = y;
  check_point.z = z;
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_sqdist;
  stacked_vertical_surface_cloud_kdtree_->radiusSearch(check_point, parameters_.kKeyposeGraphCollisionCheckRadius,
                                                       neighbor_indices, neighbor_sqdist);
  if (neighbor_indices.size() > parameters_.kKeyposeGraphCollisionCheckPointNumThr)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void PlanningEnv::UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                                    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  if (planner_cloud_->cloud_->points.empty())
  {
    std::cout << "Planning cloud empty, cannot update covered area" << std::endl;
    return;
  }

  geometry_msgs::msg::Point robot_position = robot_viewpoint.getPosition();
  double sensor_range = viewpoint_manager->GetSensorRange();
  double coverage_occlusion_thr = viewpoint_manager->GetCoverageOcclusionThr();
  double coverage_dilation_radius = viewpoint_manager->GetCoverageDilationRadius();
  std::vector<int> covered_point_indices;
  double vertical_fov_ratio = 0.3;  // bigger fov than viewpoints
  double diff_z_max = sensor_range * vertical_fov_ratio;
  double xy_dist_threshold = 3 * (parameters_.kSurfaceCloudDwzLeafSize / 2) / 0.3;
  double z_diff_threshold = 3 * parameters_.kSurfaceCloudDwzLeafSize;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      planner_cloud_->cloud_->points[i].g = 255;
      continue;
    }
    if (std::abs(point.z - robot_position.z) < diff_z_max)
    {
      if (!PointInRobotHorizontalFOV(robot_position, robot_yaw_, robot_coverage_horizontal_fov_deg_, point.x, point.y))
      {
        continue;
      }
      if (misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                     Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z),
                                     vertical_fov_ratio, sensor_range, xy_dist_threshold, z_diff_threshold))
      {
        if (robot_viewpoint.CheckVisibility<PlannerCloudPointType>(point, coverage_occlusion_thr))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          continue;
        }
      }
    }
    // mark covered by visited viewpoints
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          break;
        }
      }
    }
  }

  // Dilate the covered area
  squeezed_planner_cloud_->cloud_->clear();
  for (const auto& point : planner_cloud_->cloud_->points)
  {
    PlannerCloudPointType squeezed_point = point;
    squeezed_point.z = point.z / parameters_.kCoverCloudZSqueezeRatio;
    squeezed_planner_cloud_->cloud_->points.push_back(squeezed_point);
  }
  squeezed_planner_cloud_kdtree_->setInputCloud(squeezed_planner_cloud_->cloud_);

  for (const auto& ind : covered_point_indices)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[ind];
    std::vector<int> nearby_indices;
    std::vector<float> nearby_sqdist;
    squeezed_planner_cloud_kdtree_->radiusSearch(point, coverage_dilation_radius, nearby_indices, nearby_sqdist);
    if (!nearby_indices.empty())
    {
      for (const auto& idx : nearby_indices)
      {
        MY_ASSERT(idx >= 0 && idx < planner_cloud_->cloud_->points.size());
        planner_cloud_->cloud_->points[idx].g = 255;
      }
    }
  }

  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      int cloud_idx = 0;
      int cloud_point_idx = 0;
      pointcloud_manager_->GetCloudPointIndex(i, cloud_idx, cloud_point_idx);
      pointcloud_manager_->UpdateCoveredCloudPoints(cloud_idx, cloud_point_idx);
    }
  }
}

void PlanningEnv::GetUncoveredArea(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                   int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // Clear viewpoint covered point list
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    viewpoint_manager->ResetViewPointCoveredPointList(viewpoint_ind);
  }

  // Get uncovered points
  uncovered_cloud_->cloud_->clear();
  uncovered_frontier_cloud_->cloud_->clear();
  uncovered_point_num = 0;
  uncovered_frontier_point_num = 0;
  debug_stats_.planner_points = planner_cloud_->cloud_->points.size();
  debug_stats_.covered_planner_points = 0;
  debug_stats_.frontier_visible_to_candidates = 0;
  debug_stats_.frontier_visible_to_visited = 0;
  debug_stats_.frontier_visible_to_no_viewpoint = 0;
  debug_stats_.frontier_with_candidate_in_fov_range = 0;
  debug_stats_.frontier_with_unvisited_candidate_in_fov_range = 0;
  debug_stats_.frontier_with_visited_candidate_in_fov_range = 0;
  debug_stats_.frontier_no_viewpoint_with_unvisited_candidate_in_fov_range = 0;
  debug_stats_.frontier_nearest_candidate_dist_sum = 0.0;
  debug_stats_.frontier_nearest_unvisited_candidate_dist_sum = 0.0;
  debug_stats_.frontier_nearest_candidate_dist_max = 0.0;
  debug_stats_.frontier_nearest_unvisited_candidate_dist_max = 0.0;
  std::vector<Eigen::Vector3d> uncovered_points;
  std::vector<Eigen::Vector3d> uncovered_frontier_points;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      debug_stats_.covered_planner_points++;
      continue;
    }
    bool observed = false;
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          viewpoint_manager->AddUncoveredPoint(viewpoint_ind, uncovered_point_num);
          observed = true;
        }
      }
    }
    if (observed)
    {
      pcl::PointXYZI uncovered_point;
      uncovered_point.x = point.x;
      uncovered_point.y = point.y;
      uncovered_point.z = point.z;
      uncovered_point.intensity = i;
      uncovered_cloud_->cloud_->points.push_back(uncovered_point);
      uncovered_points.emplace_back(point.x, point.y, point.z);
      uncovered_point_num++;
    }
  }

  // Check uncovered frontiers
  if (parameters_.kUseFrontier)
  {
    for (int i = 0; i < filtered_frontier_cloud_->cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = filtered_frontier_cloud_->cloud_->points[i];
      bool observed = false;
      bool visible_to_visited = false;
      bool in_candidate_fov_range = false;
      bool in_unvisited_candidate_fov_range = false;
      bool in_visited_candidate_fov_range = false;
      double nearest_candidate_dist = DBL_MAX;
      double nearest_unvisited_candidate_dist = DBL_MAX;
      for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
      {
        geometry_msgs::msg::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
        double viewpoint_dist =
            misc_utils_ns::PointXYZDist<pcl::PointXYZI, geometry_msgs::msg::Point>(point, viewpoint_position);
        nearest_candidate_dist = std::min(nearest_candidate_dist, viewpoint_dist);
        if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
        {
          nearest_unvisited_candidate_dist = std::min(nearest_unvisited_candidate_dist, viewpoint_dist);
        }
        bool viewpoint_in_fov_range = viewpoint_manager->InFOVAndRange(
            Eigen::Vector3d(point.x, point.y, point.z),
            Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
        if (viewpoint_in_fov_range)
        {
          in_candidate_fov_range = true;
          if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
          {
            in_visited_candidate_fov_range = true;
          }
          else
          {
            in_unvisited_candidate_fov_range = true;
          }
        }
        if (viewpoint_manager->VisibleByViewPoint<pcl::PointXYZI>(point, viewpoint_ind))
        {
          if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
          {
            viewpoint_manager->AddUncoveredFrontierPoint(viewpoint_ind, uncovered_frontier_point_num);
            observed = true;
          }
          else
          {
            visible_to_visited = true;
          }
        }
      }
      if (nearest_candidate_dist < DBL_MAX)
      {
        debug_stats_.frontier_nearest_candidate_dist_sum += nearest_candidate_dist;
        debug_stats_.frontier_nearest_candidate_dist_max =
            std::max(debug_stats_.frontier_nearest_candidate_dist_max, nearest_candidate_dist);
      }
      if (nearest_unvisited_candidate_dist < DBL_MAX)
      {
        debug_stats_.frontier_nearest_unvisited_candidate_dist_sum += nearest_unvisited_candidate_dist;
        debug_stats_.frontier_nearest_unvisited_candidate_dist_max =
            std::max(debug_stats_.frontier_nearest_unvisited_candidate_dist_max,
                     nearest_unvisited_candidate_dist);
      }
      if (in_candidate_fov_range)
      {
        debug_stats_.frontier_with_candidate_in_fov_range++;
      }
      if (in_unvisited_candidate_fov_range)
      {
        debug_stats_.frontier_with_unvisited_candidate_in_fov_range++;
      }
      if (in_visited_candidate_fov_range)
      {
        debug_stats_.frontier_with_visited_candidate_in_fov_range++;
      }
      if (observed)
      {
        debug_stats_.frontier_visible_to_candidates++;
        pcl::PointXYZI uncovered_frontier_point;
        uncovered_frontier_point.x = point.x;
        uncovered_frontier_point.y = point.y;
        uncovered_frontier_point.z = point.z;
        uncovered_frontier_point.intensity = i;
        uncovered_frontier_cloud_->cloud_->points.push_back(uncovered_frontier_point);
        uncovered_frontier_points.emplace_back(point.x, point.y, point.z);
        uncovered_frontier_point_num++;
      }
      else if (visible_to_visited)
      {
        debug_stats_.frontier_visible_to_visited++;
      }
      else
      {
        debug_stats_.frontier_visible_to_no_viewpoint++;
        if (in_unvisited_candidate_fov_range)
        {
          debug_stats_.frontier_no_viewpoint_with_unvisited_candidate_in_fov_range++;
        }
      }
    }
  }

  viewpoint_manager->FilterViewPointCoveredPointListsByHorizontalFOV(uncovered_points, uncovered_frontier_points);
}

bool PlanningEnv::PointInRobotHorizontalFOV(const geometry_msgs::msg::Point& robot_position,
                                           double robot_yaw,
                                           double horizontal_fov_deg,
                                           double point_x,
                                           double point_y) const
{
  if (horizontal_fov_deg >= 360.0)
  {
    return true;
  }
  double dx = point_x - robot_position.x;
  double dy = point_y - robot_position.y;
  if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6)
  {
    return true;
  }
  double half_fov_rad = horizontal_fov_deg * M_PI / 360.0;
  double angle_diff = misc_utils_ns::mod(atan2(dy, dx) - robot_yaw + M_PI, 2 * M_PI) - M_PI;
  return std::abs(angle_diff) <= half_fov_rad;
}

void PlanningEnv::GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud)
{
  pointcloud_manager_->GetVisualizationPointCloud(vis_cloud);
}

void PlanningEnv::PublishStackedCloud()
{
  stacked_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredCloud()
{
  uncovered_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredFrontierCloud()
{
  uncovered_frontier_cloud_->Publish();
}

}  // namespace planning_env_ns
