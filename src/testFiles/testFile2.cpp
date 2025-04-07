#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <filesystem>

using namespace std;
using namespace pcl;

class MinimalClientAsync : public rclcpp::Node {
public:
    using RequestListPotentialSolution3D = rcl_interfaces::srv::RequestListPotentialSolution3D;

    MinimalClientAsync(const std::string& node_name)
        : Node("client_" + node_name), cli(create_client<RequestListPotentialSolution3D>("fs3D/registration/all_solutions")) {
        while (!cli->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }
    }

    RequestListPotentialSolution3D::Response::SharedPtr send_request(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scan1,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scan2,
        int N, float VoxelSize, bool use_clahe, int r_min, int r_max, bool set_r_manual,
        float level_potential_rotation, float level_potential_translation, float normalization_factor) {
        
        auto request = std::make_shared<RequestListPotentialSolution3D::Request>();
        request->size_of_voxel = VoxelSize;
        request->dimension_size = N;
        request->sonar_scan_1 = scan1->toVector3fArray();
        request->sonar_scan_2 = scan2->toVector3fArray();
        request->use_clahe = use_clahe;
        request->r_min = r_min;
        request->r_max = r_max;
        request->level_potential_rotation = level_potential_rotation;
        request->level_potential_translation = level_potential_translation;
        request->set_normalization = normalization_factor;
        request->set_r_manual = set_r_manual;

        auto result = cli->async_send_request(request);
        return result.get();
    }

private:
    rclcpp::Client<RequestListPotentialSolution3D>::SharedPtr cli;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto minimal_client = std::make_shared<MinimalClientAsync>("myRandomName");
    RCLCPP_INFO(minimal_client->get_logger(), "Testing IO for point clouds ...");

    int whichScan1 = 0;
    int whichScan2 = 0;

    for (int i = 0; i < 20; ++i) {
        PointCloud<pcl::PointXYZRGB>::Ptr pcd1(new PointCloud<pcl::PointXYZRGB>);
        PointCloud<pcl::PointXYZRGB>::Ptr pcd2(new PointCloud<pcl::PointXYZRGB>);

        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(
                std::filesystem::path("/home/tim-external/dataFolder/pointclouds/PointcloudAlpha_") / 
                std::format("{:05d}.ply", whichScan1), *pcd1) == -1 ||
            pcl::io::loadPLYFile<pcl::PointXYZRGB>(
                std::filesystem::path("/home/tim-external/dataFolder/pointclouds/PointcloudAlpha_") /
                std::format("{:05d}.ply", whichScan2), *pcd2) == -1) {
            RCLCPP_ERROR(minimal_client->get_logger(), "Could not read one of the point clouds.");
            return -1;
        }

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T(0, 3) = 1.1;
        T.block<3, 3>(0, 0) = Eigen::AngleAxisf(i * 0.2, Eigen::Vector3f::UnitZ()).toRotationMatrix();

        pcd2->transform(T);
        RCLCPP_INFO(minimal_client->get_logger(), "Transformation Matrix GT: ");
        std::cout << T << std::endl;

        int N = 64;
        float maxDistance = 20.0f;
        float voxelSize = (2 * maxDistance * 1.5) / N;

        Eigen::Vector4f mean1, mean2;
        pcl::compute3DCentroid(*pcd1, mean1);
        pcl::compute3DCentroid(*pcd2, mean2);

        VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setLeafSize(voxelSize, voxelSize, voxelSize);
        PointCloud<pcl::PointXYZRGB>::Ptr pcd1_vox(new PointCloud<pcl::PointXYZRGB>);
        PointCloud<pcl::PointXYZRGB>::Ptr pcd2_vox(new PointCloud<pcl::PointXYZRGB>);
        voxel_grid.setInputCloud(pcd1);
        voxel_grid.filter(*pcd1_vox);
        voxel_grid.setInputCloud(pcd2);
        voxel_grid.filter(*pcd2_vox);

        bool use_clahe = true;
        int r_min = N / 8;
        int r_max = N / 2 - N / 8;
        bool set_r_manual = true;
        float level_potential_rotation = 0.01f;
        float level_potential_translation = 0.1f;
        float normalization_factor = 1.0f;

        auto response = minimal_client->send_request(pcd1_vox, pcd2_vox, N, voxelSize, use_clahe, r_min, r_max, set_r_manual,
                                                     level_potential_rotation, level_potential_translation, normalization_factor);

        float highestPeak = 0.0f;
        int indexHighestPeak = 0;

        for (size_t index = 0; index < response->list_potential_solutions.size(); ++index) {
            if (response->list_potential_solutions[index].transformation_peak_height > highestPeak) {
                highestPeak = response->list_potential_solutions[index].transformation_peak_height;
                indexHighestPeak = index;
            }
        }

        auto peak = response->list_potential_solutions[indexHighestPeak];
        Eigen::Quaternionf currentQuaternion(
            peak.resulting_transformation.orientation.w,
            peak.resulting_transformation.orientation.x,
            peak.resulting_transformation.orientation.y,
            peak.resulting_transformation.orientation.z
        );

        Eigen::Matrix4f resultingTransformation = Eigen::Matrix4f::Identity();
        resultingTransformation.block<3, 3>(0, 0) = currentQuaternion.toRotationMatrix();
        resultingTransformation(0, 3) = peak.resulting_transformation.position.x;
        resultingTransformation(1, 3) = peak.resulting_transformation.position.y;
        resultingTransformation(2, 3) = peak.resulting_transformation.position.z;

        RCLCPP_INFO(minimal_client->get_logger(), "ICP transformation: ");
        GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
        gicp.setMaximumIterations(50);
        gicp.setTransformationEpsilon(1e-8);
        gicp.setEuclideanFitnessEpsilon(1);

        PointCloud<pcl::PointXYZRGB>::Ptr final(new PointCloud<pcl::PointXYZRGB>);
        gicp.setInputSource(pcd1_vox);
        gicp.setInputTarget(pcd2_vox);
        gicp.align(*final, resultingTransformation);
        std::cout << "ICP transformation: \n" << gicp.getFinalTransformation() << std::endl;

        RCLCPP_INFO(minimal_client->get_logger(), "next");
    }

    rclcpp::shutdown();
    return 0;
}
