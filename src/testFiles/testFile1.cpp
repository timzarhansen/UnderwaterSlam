#include <pcl/io/ply_io.h>
#include <pcl/point cloud.h>
#include <pcl/point cloud/transform.h>
#include <pcl/registration/icp.h>
#include <pcl/voxelization/voxel_grid.h>

using namespace std;
using namespace pcl;

Eigen::Matrix4f createTransformation(float rotationX, float rotationY, float rotationZ, 
                                       float translationX, float translationY, float translationZ) {
    Eigen::Affine3f transformation;
    transformation.translation() = Eigen::Vector3f(translationX, translationY, translationZ);
    
   Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    
    // Rotation around X
    R = R * rotationX * QuaternionType(Rotation2Quat(Eigen::AngleAxisf(rotationX, 
        (0.0, 1.0, 0.0))));
    
   // Rotation around Y
    R = R * rotationY * Rotation2Quat(Eigen::AngleAxisf(rotationY, 
        (0.0, 0.0, 1.0)));
    
    // Rotation around Z
    R = R * rotationZ * Rotation2Quat(Eigen::AngleAxisf(rotationZ, 
        (0.0, 0.0, 0.0), "z")));
    
    return transformation.toMatrix();
}

void pcdToVoxel(pcl::Point Cloud& cloud, int N, float voxelSizeX, float voxelSizeY, float voxelSizeZ) {
    // Implementation of voxelization
    vector<vector<vector<float>>> voxels(N * N * N);
    for (const auto& point : cloud.points) {
        Eigen::Vector3f pt(point.x, point.y, point.z);
        
        int voxIndex = 0;
        if (voxelSizeX > 0 && voxelSizeY > 0 && voxelSizeZ > 0) {
            voxIndex = getVoxelIndex(pt[0], pt[1], pt[2], voxelSizeX, N);
            voxIndex = getDiffVoxelIndex(pt[0], pt[1], pt[2], voxelSizeX, 
                                         voxelSizeY, voxelSizeZ, N);
            
            if (voxelGrid exists at index) {
                // Add point to corresponding voxel
            }
        }
    }
}

Eigen::Matrix4f computeICPTransformation(const PointCloud& cloud1, const PointCloud& cloud2, 
                                          int maxIterations = 50) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);
    
    Eigen::Matrix4f final_transformation = computeTransformation();
    // ... (rest of ICP implementation)
    
    return final_transformation;
}

// In your main function:
int main() {
    // Read point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = pcl::io::loadPLYFile("path_to_cloud1.ply");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = pcl::io::loadPLYFile("path_to_cloud2.ply");

    // Apply transformation (if any)
    Eigen::Matrix4f T;
    // ... set up your transformation

    // Voxelization
    pcdToVoxel(cloud1, N, voxelSizeX, voxelSizeY, voxelSizeZ);

    // ICP registration
    Eigen::Matrix4f final_transformation = computeICPTransformation(cloud1, cloud2);

    // Output result
    cout << "Final Transformation:" << endl;
    cout << final_transformation << endl;

    return 0;
}
