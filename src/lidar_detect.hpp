/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_DETECT_HPP
#define LIDAR_DETECT_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/filters/voxel_grid.h>
#include "common_lib.h"

class LidarDetect
{
private:
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double circle_radius_;
    double edge_search_radius_, boundary_angle_rad_, cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_;
    bool use_template_fit_;
    double delta_width_circles_, delta_height_circles_;
    std::shared_ptr<rclcpp::Node> node_;

    // 存储中间结果的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_z0_cloud_;

public:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> filtered_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> plane_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> edge_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> center_z0_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> center_pub_;

    LidarDetect(std::shared_ptr<rclcpp::Node> node, Params &params)
        : node_(node),
          filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          plane_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          edge_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          center_z0_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        x_min_ = params.x_min;
        x_max_ = params.x_max;
        y_min_ = params.y_min;
        y_max_ = params.y_max;
        z_min_ = params.z_min;
        z_max_ = params.z_max;
        circle_radius_ = params.circle_radius;
        edge_search_radius_ = params.edge_search_radius;
        boundary_angle_rad_ = params.boundary_angle_deg * M_PI / 180.0;
        cluster_tolerance_  = params.cluster_tolerance;
        min_cluster_size_   = params.min_cluster_size;
        max_cluster_size_   = params.max_cluster_size;
        use_template_fit_   = params.use_template_fit;
        delta_width_circles_  = params.delta_width_circles;
        delta_height_circles_ = params.delta_height_circles;

        filtered_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 1);
        plane_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("plane_cloud", 1);
        aligned_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", 1);
        edge_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("edge_cloud", 1);
        center_z0_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("center_z0_cloud", 10);
        center_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("center_cloud", 10);
    }

    void detect_lidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud)
    {
        // 1. X、Y、Z方向滤波
        filtered_cloud_->clear();
        filtered_cloud_->reserve(cloud->size());

        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min_, x_max_);
        pass_x.filter(*filtered_cloud_);
    
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(filtered_cloud_);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min_, y_max_);
        pass_y.filter(*filtered_cloud_);
    
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(filtered_cloud_);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_min_, z_max_);
        pass_z.filter(*filtered_cloud_);
    
        RCLCPP_INFO(node_->get_logger(), "Filtered cloud size: %ld", filtered_cloud_->size());
        
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(filtered_cloud_);
        voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);
        voxel_filter.filter(*filtered_cloud_);
        RCLCPP_INFO(node_->get_logger(), "Filtered cloud size: %ld", filtered_cloud_->size());

        // 2. 平面分割
        plane_cloud_->clear();
        plane_cloud_->reserve(filtered_cloud_->size());

        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
        plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
        plane_segmentation.setMethodType(pcl::SAC_RANSAC);
        plane_segmentation.setDistanceThreshold(0.01);
        plane_segmentation.setInputCloud(filtered_cloud_);
        plane_segmentation.segment(*plane_inliers, *plane_coefficients);
    
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(filtered_cloud_);
        extract.setIndices(plane_inliers);
        extract.filter(*plane_cloud_);
        RCLCPP_INFO(node_->get_logger(), "Plane cloud size: %ld", plane_cloud_->size());
    
        // 3. 平面点云对齐
        aligned_cloud_->clear();
        aligned_cloud_->reserve(plane_cloud_->size());

        Eigen::Vector3d normal(plane_coefficients->values[0],
            plane_coefficients->values[1],
            plane_coefficients->values[2]);
        normal.normalize();
        Eigen::Vector3d z_axis(0, 0, 1);

        Eigen::Vector3d axis = normal.cross(z_axis);
        double angle = acos(normal.dot(z_axis));

        Eigen::AngleAxisd rotation(angle, axis);
        Eigen::Matrix3d R = rotation.toRotationMatrix();

        // 应用旋转矩阵，将平面对齐到 Z=0 平面
        float average_z = 0.0;
        int cnt = 0;
        for (const auto& pt : *plane_cloud_) {
            Eigen::Vector3d point(pt.x, pt.y, pt.z);
            Eigen::Vector3d aligned_point = R * point;
            aligned_cloud_->push_back(pcl::PointXYZ(aligned_point.x(), aligned_point.y(), 0.0));
            average_z += aligned_point.z();
            cnt++;
        }
        average_z /= cnt;

        // 4. Hole centres. Two strategies -- see Params::use_template_fit.
        if (use_template_fit_)
        {
            detectHolesTemplate(R, average_z, center_cloud);
            return;
        }

        // 4. 提取边缘点
        edge_cloud_->clear();
        edge_cloud_->reserve(aligned_cloud_->size());

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normal_estimator.setInputCloud(aligned_cloud_);
        normal_estimator.setRadiusSearch(edge_search_radius_);
        normal_estimator.compute(*normals);
    
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimator;
        boundary_estimator.setInputCloud(aligned_cloud_);
        boundary_estimator.setInputNormals(normals);
        boundary_estimator.setRadiusSearch(edge_search_radius_);
        boundary_estimator.setAngleThreshold(boundary_angle_rad_);
        boundary_estimator.compute(boundaries);
    
        for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
            if (boundaries.points[i].boundary_point > 0) {
                edge_cloud_->push_back(aligned_cloud_->points[i]);
            }
        }
        RCLCPP_INFO(node_->get_logger(), "Extracted %ld edge points.", edge_cloud_->size());

        // 5. 对边缘点进行聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(edge_cloud_);
    
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(edge_cloud_);
        ec.extract(cluster_indices);
    
        RCLCPP_INFO(node_->get_logger(), "Number of edge clusters: %ld", cluster_indices.size());
    
        // 6. 对每个聚类进行圆拟合
        center_z0_cloud_->clear();
        center_z0_cloud_->reserve(4);
        Eigen::Matrix3d R_inv = R.inverse();
    
        // 对每个聚类进行圆拟合
        for (size_t i = 0; i < cluster_indices.size(); ++i) 
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : cluster_indices[i].indices) {
                cluster->push_back(edge_cloud_->points[idx]);
            }
    
            // 圆拟合
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CIRCLE2D);
            seg.setMethodType(pcl::SAC_RANSAC);
            // Scale-relative, not absolute: upstream's 0.01 was tuned for the
            // 0.12 m holes of the full-size target (8.33% of radius). Hardcoding
            // an absolute value rejects every cluster on one target size or the
            // other -- verified: 0.007 accepted 0 of 4 clusters at r=0.12.
            seg.setDistanceThreshold(0.0833 * circle_radius_);
            seg.setMaxIterations(1000);
            seg.setInputCloud(cluster);
            seg.segment(*inliers, *coefficients);
    
            if (inliers->indices.size() > 0) 
            {
                // 计算拟合误差
                double error = 0.0;
                for (const auto& idx : inliers->indices) 
                {
                    double dx = cluster->points[idx].x - coefficients->values[0];
                    double dy = cluster->points[idx].y - coefficients->values[1];
                    double distance = sqrt(dx * dx + dy * dy) - circle_radius_;
                    error += abs(distance);
                }
                error /= inliers->indices.size();
    
                // 如果拟合误差较小，则认为是一个圆洞
                // Same reasoning: upstream's 0.02 is 16.7% of a 0.12 m radius, so
                // keep that ratio rather than the absolute value. At r=0.08 this is
                // 0.0133 m instead of 0.02 m, which is what stops board-corner arcs
                // being accepted as holes on the half-scale board.
                if (error < 0.1667 * circle_radius_)
                
                {
                    // 将恢复后的圆心坐标添加到点云中
                    pcl::PointXYZ center_point;
                    center_point.x = coefficients->values[0];
                    center_point.y = coefficients->values[1];
                    center_point.z = 0.0;
                    center_z0_cloud_->push_back(center_point);

                    // 将圆心坐标逆变换回原始坐标系
                    Eigen::Vector3d aligned_point(center_point.x, center_point.y, center_point.z + average_z);
                    Eigen::Vector3d original_point = R_inv * aligned_point;

                    pcl::PointXYZ center_point_origin;
                    center_point_origin.x = original_point.x();
                    center_point_origin.y = original_point.y();
                    center_point_origin.z = original_point.z();
                    center_cloud->points.push_back(center_point_origin);
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // Template-fit hole detection.
    //
    // WHY THIS EXISTS
    // Upstream finds holes bottom-up: classify rim points as boundary, cluster
    // each rim into a connected curve, RANSAC a circle. That needs the rim to BE
    // a dense curve. On a 16-beam Hesai XT16 the ring pitch is 35 mm at 1.0 m, so
    // a 162 mm hole is crossed by only ~4.6 rings and there is no cluster
    // tolerance that works: below the ring pitch each rim breaks into isolated
    // per-ring arcs (measured: 9-36 clusters, radii scattered 24-139 mm); above
    // it, every boundary point merges into one component through the board
    // outline (measured: exactly 1 cluster). Verified across 18 runs on 3 scenes.
    //
    // WHAT THIS DOES INSTEAD
    // The board geometry is known exactly, so the 4-hole pattern is a rigid
    // template with only 3 unknowns in the board plane: translation (tx, ty) and
    // rotation theta. Search that 3-DOF space for the pose where all four circles
    // land on emptiness while their surroundings stay populated. Every ring that
    // crosses a hole contributes, so 4 rings per hole is ample.
    //
    // Scoring uses a 5 mm occupancy grid: score = (points in the annuli around
    // the 4 circles) - 12 * (points inside them). An off-board pose scores 0
    // (nothing inside, nothing around), so it can never beat a real fit.
    // ------------------------------------------------------------------------
    void detectHolesTemplate(const Eigen::Matrix3d &R, double average_z,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud)
    {
        center_z0_cloud_->clear();
        edge_cloud_->clear();
        if (aligned_cloud_->empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[template] aligned cloud is empty");
            return;
        }

        const double cell = 0.005;
        double umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
        for (const auto &pt : *aligned_cloud_) {
            umin = std::min(umin, (double)pt.x); umax = std::max(umax, (double)pt.x);
            vmin = std::min(vmin, (double)pt.y); vmax = std::max(vmax, (double)pt.y);
        }
        const int W = (int)((umax - umin) / cell) + 2;
        const int H = (int)((vmax - vmin) / cell) + 2;
        std::vector<int> grid((size_t)W * H, 0);
        for (const auto &pt : *aligned_cloud_) {
            int i = (int)((pt.x - umin) / cell), j = (int)((pt.y - vmin) / cell);
            if (i >= 0 && i < W && j >= 0 && j < H) grid[(size_t)j * W + i]++;
        }

        // precomputed cell offsets: disk inside 0.90 r, annulus 1.10 r .. 1.45 r
        std::vector<std::pair<int,int>> disk, annulus;
        const int rc = (int)(1.45 * circle_radius_ / cell) + 1;
        for (int dj = -rc; dj <= rc; ++dj)
            for (int di = -rc; di <= rc; ++di) {
                double d = std::sqrt((double)di*di + (double)dj*dj) * cell;
                if (d < 0.90 * circle_radius_) disk.push_back({di, dj});
                else if (d > 1.10 * circle_radius_ && d < 1.45 * circle_radius_)
                    annulus.push_back({di, dj});
            }

        const double hw = delta_width_circles_ / 2.0, hh = delta_height_circles_ / 2.0;
        const int sx[4] = {-1, 1, 1, -1}, sy[4] = {1, 1, -1, -1};

        auto at = [&](double x, double y, const std::vector<std::pair<int,int>> &off) {
            int ci = (int)((x - umin) / cell), cj = (int)((y - vmin) / cell);
            int n = 0;
            for (const auto &o : off) {
                int i = ci + o.first, j = cj + o.second;
                if (i >= 0 && i < W && j >= 0 && j < H) n += grid[(size_t)j * W + i];
            }
            return n;
        };
        auto evaluate = [&](double tx, double ty, double th, int *n_in, int *n_ring,
                            int *per_in = nullptr, int *per_ring = nullptr) {
            double c = std::cos(th), s = std::sin(th);
            int in = 0, ring = 0;
            for (int k = 0; k < 4; ++k) {
                double px = sx[k] * hw, py = sy[k] * hh;
                double cx = tx + c * px - s * py, cy = ty + s * px + c * py;
                int i_k = at(cx, cy, disk), r_k = at(cx, cy, annulus);
                if (per_in) per_in[k] = i_k;
                if (per_ring) per_ring[k] = r_k;
                in += i_k; ring += r_k;
            }
            if (n_in) *n_in = in;
            if (n_ring) *n_ring = ring;
            return (double)ring - 12.0 * (double)in;
        };

        // Coarse search, then refine.
        //
        // The window MUST be centred on the board, not on the aligned frame's origin.
        // alignPlaneToZ0 applies a pure ROTATION about the LiDAR origin with no
        // translation, so the aligned frame's origin is where the sensor origin
        // projects onto the board plane -- which coincides with the board centre only
        // when the board is square-on. A board tilted by `a` at range `d` puts the
        // centre `d*tan(a)` away: 12 mm at scene1's 0.8 deg, but 156 mm at scene10's
        // 11.3 deg, outside the old fixed [-0.15, 0.15] box. The search then could not
        // reach the true optimum and settled on a shifted one with two holes off the
        // board (per-hole annulus support 320/293/14/2), which the aliasing guard
        // correctly rejected -- a good recording failing on a search-window bug.
        double cu = 0.0, cv = 0.0;
        for (const auto &pt : *aligned_cloud_) { cu += pt.x; cv += pt.y; }
        cu /= (double)aligned_cloud_->size();
        cv /= (double)aligned_cloud_->size();
        RCLCPP_INFO(node_->get_logger(),
                    "[template] aligned-cloud centroid (%+.0f, %+.0f) mm; searching +-150 mm around it",
                    1000.0 * cu, 1000.0 * cv);

        double bt = 0, bx = cu, by = cv, bs = -1e18;
        for (double th = 0; th < M_PI; th += M_PI / 72.0)
            for (double ty = cv - 0.15; ty <= cv + 0.15; ty += 0.010)
                for (double tx = cu - 0.15; tx <= cu + 0.15; tx += 0.010) {
                    double sc = evaluate(tx, ty, th, nullptr, nullptr);
                    if (sc > bs) { bs = sc; bx = tx; by = ty; bt = th; }
                }
        double stepT = 0.004, stepA = M_PI / 180.0 * 2.0;
        for (int iter = 0; iter < 12; ++iter) {
            bool improved = false;
            for (int dj = -1; dj <= 1; ++dj)
                for (int di = -1; di <= 1; ++di)
                    for (int da = -1; da <= 1; ++da) {
                        double sc = evaluate(bx + di*stepT, by + dj*stepT, bt + da*stepA,
                                             nullptr, nullptr);
                        if (sc > bs) { bs = sc; bx += di*stepT; by += dj*stepT;
                                       bt += da*stepA; improved = true; }
                    }
            if (!improved) { stepT *= 0.5; stepA *= 0.5; }
        }

        int n_in = 0, n_ring = 0, per_in[4] = {0,0,0,0}, per_ring[4] = {0,0,0,0};
        evaluate(bx, by, bt, &n_in, &n_ring, per_in, per_ring);

        // Is this a real lock? Compare points found inside the holes against what
        // the board's own density predicts if those areas were solid.
        double area = (umax - umin) * (vmax - vmin);
        double density = area > 0 ? aligned_cloud_->size() / area : 0.0;
        double expect_solid = density * 4.0 * M_PI * std::pow(0.90 * circle_radius_, 2);
        RCLCPP_INFO(node_->get_logger(),
            "[template] score %.0f  t=(%+.1f,%+.1f) mm  rot %.2f deg  inside %d "
            "(solid would be ~%.0f)  annuli %d",
            bs, 1000*bx, 1000*by, bt * 180.0 / M_PI, n_in, expect_solid, n_ring);

        // ALIASING GUARD -- this is the one that matters.
        //
        // The template can lock one hole-spacing off and still look self-consistent:
        // 2 circles land on real holes while the other 2 fall clean off the board,
        // where zero points reads as "void" just as convincingly. Measured on a
        // real scene: a fit shifted ~180 mm (vs the 213 mm vertical hole spacing)
        // reported only 46 points inside and passed every aggregate test, yet its
        // translation was 325 mm from the correct answer. RMSE cannot catch this --
        // both point sets are exact rectangles, so it reads ~1e-4 either way.
        //
        // The tell is per-hole: an off-board circle has almost no points in its
        // surrounding annulus, while a genuine hole is ringed by board. Aggregate
        // sums hide that; a per-hole minimum does not.
        double expect_ring = density * M_PI *
            (std::pow(1.45 * circle_radius_, 2) - std::pow(1.10 * circle_radius_, 2));
        RCLCPP_INFO(node_->get_logger(),
            "[template] per-hole annulus support: %d %d %d %d  (expect ~%.0f each)",
            per_ring[0], per_ring[1], per_ring[2], per_ring[3], expect_ring);
        if (expect_ring > 0) {
            for (int k = 0; k < 4; ++k) {
                if (per_ring[k] < 0.45 * expect_ring) {
                    RCLCPP_ERROR(node_->get_logger(),
                        "[template] NO LOCK (aliasing guard): hole %d has only %d "
                        "points around it, expected ~%.0f. The pattern is sitting "
                        "partly off the board -- most likely shifted by about one "
                        "hole spacing. Re-record with the whole hole pattern inside "
                        "the LiDAR's vertical FOV (move the board to ~1.15 m and "
                        "centre it on the LiDAR, not the camera).",
                        k, per_ring[k], expect_ring);
                    return;
                }
            }
        }
        if (expect_solid > 0 && n_in > 0.35 * expect_solid) {
            RCLCPP_ERROR(node_->get_logger(),
                "[template] NO LOCK: the four circles are not landing on voids "
                "(%d points inside vs %.0f if solid). Check the board geometry "
                "parameters and the distance filter box.", n_in, expect_solid);
            return;
        }

        Eigen::Matrix3d R_inv = R.inverse();
        double c = std::cos(bt), s = std::sin(bt);

        // The template pose is only determined modulo 180 deg -- a rectangle maps
        // onto itself under a half turn about its normal. Left unresolved, the SVD
        // still fits all 4 pairs perfectly (RMSE ~1e-4) while the correspondence is
        // rotated, producing a physically absurd extrinsic. So resolve it here from
        // gravity: the board is upright, hence LiDAR +z projected into the board
        // plane IS the board's "up", and the 4 centres can be labelled
        // TL/TR/BR/BL deterministically -- the same order the camera side emits
        // (boardCircleCenters, qr_detect.hpp:153-165).
        std::vector<Eigen::Vector2d> flat;
        std::vector<Eigen::Vector3d> world;
        for (int k = 0; k < 4; ++k) {
            double px = sx[k] * hw, py = sy[k] * hh;
            double cx = bx + c * px - s * py, cy = by + s * px + c * py;
            flat.push_back(Eigen::Vector2d(cx, cy));
            world.push_back(R_inv * Eigen::Vector3d(cx, cy, average_z));
        }

        Eigen::Vector3d centroid = (world[0] + world[1] + world[2] + world[3]) / 4.0;
        // plane normal in the LiDAR frame, oriented back toward the sensor at origin
        Eigen::Vector3d nrm = R_inv * Eigen::Vector3d(0, 0, 1);
        if (nrm.dot(centroid) > 0) nrm = -nrm;
        Eigen::Vector3d ez(0, 0, 1);
        Eigen::Vector3d up = ez - ez.dot(nrm) * nrm;
        if (up.norm() < 1e-6) {
            RCLCPP_ERROR(node_->get_logger(),
                "[template] board plane is horizontal; cannot resolve 'up'");
            return;
        }
        up.normalize();
        // right-as-seen-facing-the-board; with x fwd / y left / z up this is -y
        Eigen::Vector3d right = up.cross(nrm);

        int order[4] = {-1, -1, -1, -1};   // TL, TR, BR, BL
        for (int k = 0; k < 4; ++k) {
            Eigen::Vector3d d = world[k] - centroid;
            bool is_right = d.dot(right) > 0;
            bool is_top   = d.dot(up) > 0;
            int slot = is_top ? (is_right ? 1 : 0) : (is_right ? 2 : 3);
            if (order[slot] != -1) {
                RCLCPP_ERROR(node_->get_logger(),
                    "[template] two hole centres fell in the same quadrant; "
                    "ordering is ambiguous");
                return;
            }
            order[slot] = k;
        }

        for (int slot = 0; slot < 4; ++slot) {
            int k = order[slot];
            center_z0_cloud_->push_back(pcl::PointXYZ(flat[k].x(), flat[k].y(), 0.0));
            pcl::PointXYZ o;
            o.x = world[k].x(); o.y = world[k].y(); o.z = world[k].z();
            center_cloud->points.push_back(o);

            // populate edge_cloud_ with the annulus points so the /edge_cloud debug
            // topic still shows what the fit locked onto
            for (const auto &pt : *aligned_cloud_) {
                double d = std::hypot(pt.x - flat[k].x(), pt.y - flat[k].y());
                if (d > 1.00 * circle_radius_ && d < 1.35 * circle_radius_)
                    edge_cloud_->push_back(pt);
            }
        }
        RCLCPP_INFO(node_->get_logger(),
            "[template] emitted %ld hole centres, ordered TL,TR,BR,BL by gravity",
            center_cloud->points.size());
    }

    // 获取中间结果的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const { return filtered_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPlaneCloud() const { return plane_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedCloud() const { return aligned_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getEdgeCloud() const { return edge_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCenterZ0Cloud() const { return center_z0_cloud_; }
};

typedef std::shared_ptr<LidarDetect> LidarDetectPtr;

#endif