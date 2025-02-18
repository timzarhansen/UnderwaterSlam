//
// Created by jurobotics on 13.09.21.
//
#define PCL_NO_PRECOMPILE
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// #include "ping360_sonar_msgs/msg/sonar_echo.hpp"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "commonbluerovmsg/srv/save_graph.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <filesystem>
// #include "pcl/conversions.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/registration.h>
#include <Eigen/Dense>


// #include "commonbluerovmsg/msg/state_robot_for_evaluation.hpp"


// #define NUMBER_OF_POINTS_DIMENSION 64
// #define MAX_DISTANCE 20
// #define VOXEL_SIZE 0.95
// #define DIMENSION_OF_VOXEL_DATA_FOR_MATCHING 40 // was 50 //tuhh tank 6
// #define NUMBER_OF_POINTS_MAP 512//was 512
// // 80 simulation ;300 valentin; 45.0 for Keller; 10.0 TUHH TANK ;15.0 Ocean ;35.0 DFKI
// #define DIMENSION_OF_MAP 35.0
// #define HOW_MANY_SKIPS_OF_MESSAGES 10
//
// #define IGNORE_DISTANCE_TO_ROBOT 1.0 // was 1.0 // TUHH 0.2
// #define DEBUG_REGISTRATION false
//
// #define ROTATION_SONAR M_PI // sonar on robot M_PI // simulation 0
// #define SHOULD_USE_ROSBAG false
// #define FACTOR_OF_MATCHING 1.0 //1.5
// #define THRESHOLD_FOR_TRANSLATION_MATCHING 0.1 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben
//
#define INTEGRATED_NOISE_XYZ 0.03 // was 0.03  // TUHH 0.005
#define INTEGRATED_NOISE_RPY 0.03 // was 0.03 // TUHH 0.005
//
// #define USE_INITIAL_TRANSLATION_LOOP_CLOSURE true
// #define MAXIMUM_LOOP_CLOSURE_DISTANCE 1.0 // 0.2 TUHH 2.0 valentin Keller 4.0 Valentin Oben // 2.0 simulation
//
// #define SONAR_LOOKING_DOWN false
// #define USES_GROUND_TRUTH false

class rosClassSlam : public rclcpp::Node
{
public:
    rosClassSlam() : Node("odometrypublisher"), graphSaved(6, POINT_CLOUD_SAVED)
    {
        //Parameter Definitions
        this->declare_parameter<int>("number_of_skips", 2);
        this->declare_parameter<std::string>("pcl_topic_name", "/Alpha/velodyne_points");
        this->declare_parameter<std::string>("pose_topic_name", "/Alpha/poseArray");
        this->declare_parameter<std::string>("gt_topic_name", "/Alpha/gt_xyz");
        this->declare_parameter<int>("time_until_save", 1);
        this->declare_parameter<std::string>("which_registration", "ICP");
        this->declare_parameter<double>("scan_radius_max", 25.0);

        this->which_registration = this->get_parameter("which_registration").as_string();
        std::cout << "which_registration: " << which_registration << std::endl;
        this->number_of_skips = this->get_parameter("number_of_skips").as_int();
         std::cout << "number_of_skips: " << this->number_of_skips << std::endl;
        this->pcl_topic_name= this->get_parameter("pcl_topic_name").as_string();
         std::cout << "pcl_topic_name: " << this->pcl_topic_name << std::endl;
        this->pose_topic_name= this->get_parameter("pose_topic_name").as_string();
         std::cout << "pose_topic_name: " << this->pose_topic_name << std::endl;
        this->gt_topic_name= this->get_parameter("gt_topic_name").as_string();
        std::cout << "gt_topic_name: " << this->gt_topic_name << std::endl;
        this->time_until_save= this->get_parameter("time_until_save").as_int();
        std::cout << "time_until_save: " << this->time_until_save << std::endl;
        this->scan_radius_max= this->get_parameter("scan_radius_max").as_double();
        std::cout << "scan_radius_max: " << this->scan_radius_max << std::endl;
//        this->time_until_save = 1;


        if (this->which_registration=="fs3d32"||this->which_registration=="ICP"||this->which_registration=="GICP"||this->which_registration=="fs3d32ICP"||this->which_registration=="fs3d32GICP") {
            this->dimension_of_registration = 32;
            this->voxel_size = 2*this->scan_radius_max/32;
            this->scanRegistrationObject = new scanRegistrationClass(32);
        }

        if (this->which_registration=="fs3d64"||this->which_registration=="fs3d64ICP"||this->which_registration=="fs3d64GICP") {
            this->dimension_of_registration = 64;
            this->voxel_size = 2*this->scan_radius_max/64;
            this->scanRegistrationObject= new scanRegistrationClass(64);
        }

        if (this->which_registration=="fs3d128"||this->which_registration=="fs3d128ICP"||this->which_registration=="fs3d128GICP") {
            this->dimension_of_registration = 128;
            this->voxel_size = 2*this->scan_radius_max/128;
            this->scanRegistrationObject= new scanRegistrationClass(128);
        }





        //we have to make sure, to get ALL the data. Therefor we have to change that in the future.
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepAll(), rmw_qos_profile_system_default);
        qos.history(rmw_qos_history_policy_e::RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        qos.reliability(rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
        qos.liveliness(rmw_qos_liveliness_policy_e::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
        qos.deadline(rmw_time_t(RMW_DURATION_INFINITE));
        qos.lifespan(rmw_time_t(RMW_DURATION_INFINITE));
        qos.liveliness_lease_duration(rmw_time_t(RMW_DURATION_INFINITE));
        qos.avoid_ros_namespace_conventions(false);

        this->callback_group_subscriber1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        this->callback_group_subscriber2_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;


        this->subscriberVelodyne = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->pcl_topic_name, qos,
            std::bind(&rosClassSlam::valodyneCallback,
                      this, std::placeholders::_1), sub1_opt);
        this->subscriberGroundTruth = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->gt_topic_name, qos,
            std::bind(&rosClassSlam::groundTruthGPSEvaluationCallback,
                      this, std::placeholders::_1), sub2_opt);
        // this->serviceSaveGraph = this->create_service<commonbluerovmsg::srv::SaveGraph>("saveGraphOfSLAM",
        //                                                                                 std::bind(
        //                                                                                         &rosClassSlam::saveGraph,
        //                                                                                         this,
        //                                                                                         std::placeholders::_1,
        //                                                                                         std::placeholders::_2));

        this->publisherPoseOdometry = this->create_publisher<geometry_msgs::msg::PoseArray>(
            this->pose_topic_name, qos);

        std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(100.0);
        this->timer_ = this->create_wall_timer(
            my_timer_duration, std::bind(&rosClassSlam::timerFunction, this));

        // std::chrono::duration<double> my_timer_duration_odometry = std::chrono::duration<double>(0.01);
        // this->odometryTimer = this->create_wall_timer(
        //     my_timer_duration_odometry, std::bind(&rosClassSlam::updatingPCLCallback, this));

        this->sigmaScaling = 1.0;
        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;
        this->saveGraphStructure = false;
        this->numberOfTimesFirstScan = 0;

        this->maxTimeOptimization = 1.0;
        this->numberOfScans = 0;
        this->time_last_pointcloud = std::chrono::steady_clock::now();
        std::string whichRobot;
        if (this->pcl_topic_name=="/Alpha/velodyne_points") {
            whichRobot = "Alpha";
        }
        if (this->pcl_topic_name=="/Bob/velodyne_points") {
            whichRobot = "Bob";
        }
        if (this->pcl_topic_name=="/Carol/velodyne_points") {
            whichRobot = "Carol";
        }
        this->folderForSaving = std::string(this->which_registration+"_"+std::to_string(this->number_of_skips)+"_"+std::to_string(this->scan_radius_max)+"_"+whichRobot);
        std::cout << "endet initilization" << std::endl;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriberVelodyne;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriberGroundTruth;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisherPoseOdometry;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherPointcloudMap;
    // rclcpp::Service<commonbluerovmsg::srv::SaveGraph>::SharedPtr serviceSaveGraph;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::TimerBase::SharedPtr odometryTimer;

    std::mutex groundTruthMutex;
    std::mutex odometryMutex;
    std::mutex pclMutex;
    //Matrices:
    Eigen::Matrix4d currentEstimatedTransformation;
    Eigen::Matrix4d initialGuessTransformation;

    // GT savings
    std::deque<transformationStamped> currentPositionGTDeque;
    std::deque<pclMeasurement> pclMeasurementDeque;

    Eigen::Matrix4d currentGTPosition;

    int indexLastFullScan;
    // double fitnessScore;
    double sigmaScaling;

    graphSlamSaveStructure graphSaved;
    scanRegistrationClass* scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan, saveGraphStructure;
    std::string saveStringGraph;
    double maxTimeOptimization;
    int numberOfScans;
    int numberOfTimesFirstScan;
    // parameters
    int dimension_of_registration;
    int number_of_skips;
    std::string pcl_topic_name;
    std::string pose_topic_name;
    std::string gt_topic_name;
    double time_until_save;
    std::string which_registration;
    double scan_radius_max;
    double voxel_size;
    std::chrono::steady_clock::time_point time_last_pointcloud;
    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::string folderForSaving;
    //PCL for memory Saving
    pclMeasurement lastPCL;
    pclMeasurement currentPCl;
    void valodyneCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->pclMutex);
        pclMeasurement currentPCL;

        pcl::PointCloud<pcl::PointXYZ> cloudIncoming;
        pcl::fromROSMsg(*msg, cloudIncoming);
        currentPCL.pointcloud = cloudIncoming;
        currentPCL.time = rclcpp::Time(msg->header.stamp).seconds();
        this->pclMeasurementDeque.push_back(currentPCL);
        std::cout <<  std::setprecision(19);
        std::cout << "new PCL coming in: " << currentPCL.time << std::endl;

    }

    bool returnNextPCL(pclMeasurement &currentMeasurementComingIn) {
        std::lock_guard<std::mutex> lock(this->pclMutex);
        if (this->pclMeasurementDeque.empty()) {
            return 0;
        }

        auto compareTimeStampsPCL = [](const pclMeasurement& a, const pclMeasurement& b) {
            return a.time < b.time;
        };
        std::sort(this->pclMeasurementDeque.begin(), this->pclMeasurementDeque.end(), compareTimeStampsPCL);
        std::cout << "size of PCL Deque: " << this->pclMeasurementDeque.size() << std::endl;
        currentMeasurementComingIn = this->pclMeasurementDeque.front();
        this->pclMeasurementDeque.pop_front();
        return 1;
    }

    void updatingPCLCallback()
    {


        // this->time_last_pointcloud = std::chrono::steady_clock::now();

        pclMeasurement PCLTMP;

        bool returnValue = returnNextPCL(this->currentPCl);
        if (!returnValue) {
            return;
        }
        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(this->currentPCl.pointcloud, this->currentPCl.pointcloud, indices);

        pcl::PointCloud<pcl::PointXYZ> cloudIncoming = this->currentPCl.pointcloud;



        if (this->firstSonarInput)
        {

            this->graphSaved.addVertexPCL(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                                          Eigen::Matrix3d::Zero(),PCLTMP, this->currentPCl.time,
                                          FIRST_ENTRY);
            this->graphSaved.print();
            this->firstSonarInput = false;
            sleep(1);
            this->saveCurrentGTPosition();
            std::cout << "Saving GT position first time Done" << std::endl;
            this->lastPCL = this->currentPCl;
            return;
        }

        this->numberOfScans++;
        if (this->numberOfScans % this->number_of_skips != 0) {
            return;
        }
        // std::cout << "this->numberOfScans: " << this->numberOfScans << std::endl;
        // double* voxelData1;
        // double* voxelData2;
        // voxelData1 = (double*)malloc(
        //     sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
        // voxelData2 = (double*)malloc(
        //     sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);


        Eigen::Matrix4d currentRegistrationEstimation;
        pcl::PointCloud<pcl::PointXYZ> pclLastScan;
        // pclLastScan = this->graphSaved.getVertexList()->back().getPCLMeasurement().pointcloud;
        pclLastScan = this->lastPCL.pointcloud;

        registrationOfTwoPointclouds(pclLastScan,cloudIncoming,currentRegistrationEstimation,this->which_registration);
        // edge differenceOfEdge = ;

        Eigen::Matrix4d tmpTransformation = this->graphSaved.getVertexList()->back().getTransformation();
        tmpTransformation = tmpTransformation * currentRegistrationEstimation;
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);


        this->graphSaved.addVertexPCL(this->graphSaved.getVertexList()->back().getKey() + 1, pos, rot,
                                      this->graphSaved.getVertexList()->back().getCovarianceMatrix(),
                                      PCLTMP,
                                       this->currentPCl.time,
                                      POINT_CLOUD_SAVED);
        // std::cout << "added vertex to Graph" << std::endl;
        Eigen::Vector3d currentRegistrationEstimationTranslation;
        Eigen::Quaterniond currentRegistrationEstimationRotation;
        generalHelpfulTools::splitTransformationMatrixToQuadAndTrans(currentRegistrationEstimationTranslation,
                                                                     currentRegistrationEstimationRotation,
                                                                     currentRegistrationEstimation);
        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        //overwrite the covariance matrix. at some point not necessary
        covarianceMatrix(0, 0) = INTEGRATED_NOISE_XYZ;
        covarianceMatrix(1, 1) = INTEGRATED_NOISE_XYZ;
        covarianceMatrix(2, 2) = INTEGRATED_NOISE_RPY;
        // std::cout << "Saving Registration in Graph:" << std::endl;
        // std::cout << currentRegistrationEstimation << std::endl;
        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 currentRegistrationEstimationTranslation, currentRegistrationEstimationRotation,
                                 covarianceMatrix, INTEGRATED_POSE);
        // std::cout << "save GT Pose" << std::endl;
        this->saveCurrentGTPosition();
        this->lastPCL = this->currentPCl;
        // std::cout << "added edge to Graph" << std::endl;
        ////////////// look for loop closure  //////////////
        // slamToolsRos::loopDetectionByClosestPath(this->graphSaved, this->scanRegistrationObject,
        //                                          NUMBER_OF_POINTS_DIMENSION, IGNORE_DISTANCE_TO_ROBOT,
        //                                          DIMENSION_OF_VOXEL_DATA_FOR_MATCHING, DEBUG_REGISTRATION,
        //                                          USE_INITIAL_TRANSLATION_LOOP_CLOSURE, 250, 500,
        //                                          THRESHOLD_FOR_TRANSLATION_MATCHING, MAXIMUM_LOOP_CLOSURE_DISTANCE);


        this->graphSaved.isam2OptimizeGraph(true, 1);
        //        this->graphSaved.isam2OptimizeGraph(true,1);
        // std::cout << "optimize Stuff" << std::endl;
        // visualize graph
        geometry_msgs::msg::PoseArray poseArrayToPublish = getFullPoseArrayOfGraph();
        this->publisherPoseOdometry->publish(poseArrayToPublish);

        // slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherSonarEcho,
        //                                         this->publisherMarkerArray, this->sigmaScaling,
        //                                         this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures,this->publisherEKF);



        // std::cout << "published everything " << std::endl;
    }

    // bool saveGraph(const std::shared_ptr<commonbluerovmsg::srv::SaveGraph::Request> req,
    //                std::shared_ptr<commonbluerovmsg::srv::SaveGraph::Response> res) {
    //     this->createMapAndSaveToFile();
    //     //create image without motion compensation
    //     //create image with motion compensation(saved images)
    //     //create image with SLAM compensation
    //     std::cout << "test for saving1" << std::endl;
    //     std::lock_guard<std::mutex> lock(this->odometryMutex);
    //     std::cout << "test for saving2" << std::endl;
    //
    //     std::ofstream myFile1, myFile2;
    //     myFile1.open(
    //             "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/positionEstimationOverTime" +
    //             std::string(NAME_OF_CURRENT_METHOD) + ".csv");
    //     myFile2.open(
    //             "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/groundTruthOverTime" +
    //             std::string(NAME_OF_CURRENT_METHOD) + ".csv");
    //
    //
    //     for (int k = 0; k < this->graphSaved.getVertexList()->size(); k++) {
    //
    //         Eigen::Matrix4d tmpMatrix1 = this->graphSaved.getVertexList()->at(k).getTransformation();
    //         Eigen::Matrix4d tmpMatrix2 = this->graphSaved.getVertexList()->at(k).getGroundTruthTransformation();
    //         for (int i = 0; i < 4; i++) {
    //             for (int j = 0; j < 4; j++) {
    //                 myFile1 << tmpMatrix1(i, j) << " ";//number of possible rotations
    //             }
    //             myFile1 << "\n";
    //         }
    //
    //
    //         for (int i = 0; i < 4; i++) {
    //             for (int j = 0; j < 4; j++) {
    //                 myFile2 << tmpMatrix2(i, j) << " ";//number of possible rotations
    //             }
    //             myFile2 << "\n";
    //         }
    //
    //     }
    //
    //
    //     myFile1.close();
    //     myFile2.close();
    //
    //     res->saved = true;
    //     return true;
    // }


    void groundTruthGPSEvaluationCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // std::cout << "Ground Truth getting Callback before Mutex" << std::endl;
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        // std::cout << "Ground Truth getting Callback after Mutex" << std::endl;
        // auto currentGTPose = msg->poses.back();
        Eigen::Quaterniond currentRotation(msg->pose.orientation.w, msg->pose.orientation.x,
                                           msg->pose.orientation.y, msg->pose.orientation.z);
        Eigen::Vector3d currentTranslation(msg->pose.position.x, msg->pose.position.y,
                                           msg->pose.position.z);
        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrix(currentTranslation, currentRotation);
        //first time? calc current Position
        // Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(msg->roll, msg->pitch,msg->yaw);

        // tmpMatrix(0, 3) = msg->x_position;
        // tmpMatrix(1, 3) = msg->y_position;
        // tmpMatrix(2, 3) = msg->z_position;

        transformationStamped tmpValue;
        tmpValue.transformation = tmpMatrix;
        // tmpValue.timeStamp = std::chrono::microseconds(msg->header.stamp.nanosec) * 1000;
        // auto currentTimeOfMessage = std::chrono::microseconds(msg->header.stamp.nanosec) * 1000;
        tmpValue.timeStamp = rclcpp::Time(msg->header.stamp).seconds();
        // std::cout << "adding stuff to deque: " << std::endl;

        this->currentPositionGTDeque.push_back(tmpValue);
        auto compareTimeStamps = [](const transformationStamped& a, const transformationStamped& b) {
            return a.timeStamp < b.timeStamp;
        };
        std::sort(currentPositionGTDeque.begin(), currentPositionGTDeque.end(), compareTimeStamps);
        // std::cout << tmpValue.transformation << std::endl;
        std::cout <<  std::setprecision(19);
        std::cout << "added to GT: " << tmpValue.timeStamp << std::endl;
        // std::cout << rclcpp::Time(msg->header.stamp).seconds() << std::endl;
    }

    // Eigen::Matrix4d getCurrentGTPosition()
    // {
    //     std::lock_guard<std::mutex> lock(this->groundTruthMutex);
    //     return this->currentGTPosition;
    // }


    void saveCurrentGTPosition() {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);

        std::cout <<  std::setprecision(19);
        if (this->currentPositionGTDeque.empty()) {
            std::cout << "GT array empty" << std::endl;
            return;
        }

        auto vertexList = this->graphSaved.getVertexList();
        double currentTimeStampOfInterest = vertexList->back().getTimeStamp();
        // auto it = std::upper_bound(this->currentPositionGTDeque.begin(), this->currentPositionGTDeque.end(), currentTimeStampOfInterest,
        //                          [](double ts, const transformationStamped& v) { return ts < v.timeStamp; });

        int currentEntry = this->currentPositionGTDeque.size()-1;
        // std::cout << "currentEntry" << currentEntry<< std::endl;
        while (currentTimeStampOfInterest<this->currentPositionGTDeque[currentEntry].timeStamp) {
            currentEntry --;
            if (currentEntry<=0) {
                break;
            }
        }
        currentEntry++;
        if (currentEntry == this->currentPositionGTDeque.size() ) {
            currentEntry = this->currentPositionGTDeque.size()-1;
        }
        // std::cout << "currentEntry" << currentEntry<< std::endl;
        // if 0 or max then just take that  this->currentPositionGTDeque.begin()
        // if (i == 0 || i == this->currentPositionGTDeque.size()) break;

        vertexList->back().setGroundTruthTransformation(this->currentPositionGTDeque[currentEntry].transformation);
        std::cout << "Size GT Array Before: " << this->currentPositionGTDeque.size()<< std::endl;
        std::cout << "timestep of interest: " << currentTimeStampOfInterest<< std::endl;
        // std::cout << this->currentPositionGTDeque[currentEntry].transformation << std::endl;
        std::cout << "timestep i: " << this->currentPositionGTDeque[currentEntry].timeStamp<< std::endl;
        std::cout << "timestep back: " << this->currentPositionGTDeque.back().timeStamp<< std::endl;
        std::cout << "timestep front: " << this->currentPositionGTDeque.front().timeStamp<< std::endl;
        if (currentEntry > 0) {
            std::cout << "timestep i-1: " << this->currentPositionGTDeque[currentEntry-1].timeStamp<< std::endl;
        }
        if (currentEntry < this->currentPositionGTDeque.size()-1) {
            std::cout << "timestep i+1: " << this->currentPositionGTDeque[currentEntry+1].timeStamp<< std::endl;
        }
        // Remove processed entries
        // for (int k = 0; k < j; ++k) {
        //     this->currentPositionGTDeque.pop_front();
        // }
        // std::cout << "Size GT Array: " << this->currentPositionGTDeque.size()<< std::endl;
        // std::cout << "Deleting entries: "<< std::endl;
        // Check if there are any entries left and they are within the 10-second window
        while (!this->currentPositionGTDeque.empty() && (currentTimeStampOfInterest - this->currentPositionGTDeque.front().timeStamp > 100.0)) {
            // std::cout << "one deleted: "<< std::endl;
            this->currentPositionGTDeque.pop_front();
        }

        std::cout << "Size GT Array afterwards: " << this->currentPositionGTDeque.size()<< std::endl;
        // std::cout << "done" << std::endl;
    }









    // void saveCurrentGTPosition()
    // {
    //     std::cout <<  std::setprecision(19);
    //     // std::cout << "adding GT to Graph" << std::endl;
    //     std::lock_guard<std::mutex> lock(this->groundTruthMutex);
    //     std::cout << "adding GT to Graph after Mutex" << std::endl;
    //     std::cout << this->currentPositionGTDeque.empty() << std::endl;
    //     if (!this->currentPositionGTDeque.empty()) {
    //         std::cout << this->currentPositionGTDeque.size() << std::endl;
    //         std::cout << "vertex back Timestamp: " << this->graphSaved.getVertexList()->back().getTimeStamp() << std::endl;
    //         std::cout << "lastPos GT: " << this->currentPositionGTDeque.back().timeStamp << std::endl;
    //         std::cout << "firstPos GT: " << this->currentPositionGTDeque[0].timeStamp << std::endl;
    //     }
    //
    //     while (!this->currentPositionGTDeque.empty())
    //     {
    //         double currentTimeStampOfInterest = this->currentPositionGTDeque[0].timeStamp;
    //         std::cout << "currentTimeStampOfInterest: " << currentTimeStampOfInterest<<std::endl;
    //         int i = this->graphSaved.getVertexList()->size() - 1;
    //         while (this->graphSaved.getVertexList()->at(i).getTimeStamp() >= currentTimeStampOfInterest)
    //         {
    //             i--;
    //             if (i == -1)
    //             {
    //                 break;
    //             }
    //         }
    //         i++;
    //         if (i == this->graphSaved.getVertexList()->size())
    //         {
    //             break;
    //         }
    //         //            if (i == 0) {
    //         //                break;
    //         //            }
    //
    //         std::cout << this->graphSaved.getVertexList()->at(i).getTimeStamp() << std::endl;
    //         std::cout << currentTimeStampOfInterest << std::endl;
    //
    //
    //         //sort in
    //         int j = 0;
    //         while (this->graphSaved.getVertexList()->at(i).getTimeStamp() >=
    //             this->currentPositionGTDeque[j].timeStamp)
    //         {
    //             j++;
    //             if (j == this->currentPositionGTDeque.size())
    //             {
    //                 break;
    //             }
    //         }
    //         if (j == this->currentPositionGTDeque.size())
    //         {
    //             break;
    //         }
    //         std::cout << "setting gt transformation:" << std::endl;
    //         std::cout << this->graphSaved.getVertexList()->at(i).getTimeStamp() << std::endl;
    //         std::cout << this->currentPositionGTDeque[j].timeStamp << std::endl;
    //         this->graphSaved.getVertexList()->at(i).setGroundTruthTransformation(
    //             this->currentPositionGTDeque[j].transformation);
    //
    //         int runningParameter = j-1;
    //         if (runningParameter>0) {
    //             for (int k = 0; k < runningParameter; k++)
    //             {
    //                 this->currentPositionGTDeque.pop_front();
    //             }
    //         }
    //         //            this->currentPositionGTDeque.pop_front();
    //     }
    //     std::cout << "done" << std::endl;
    // }

    // void createMapAndSaveToFile() {
    //     std::vector<pclValues> dataSet;
    //     double maximumIntensity = slamToolsRos::getDatasetFromGraphForPointcloudMap(dataSet, this->graphSaved,
    //                                                                       this->odometryMutex);
    //
    //     for (int currentPosition = 0;
    //          currentPosition < dataSet.size(); currentPosition++) {
    //
    //         // Take Dataset and compute one big Pointcloud that gets published(slow but should work for now)
    //
    //
    //          }
    // }

    void timerFunction() {

        // this->time_last_pointcloud = std::chrono::steady_clock::now();
        auto currentTime = std::chrono::steady_clock::now();
        double timeToCalculate = std::chrono::duration_cast<std::chrono::seconds>(currentTime - this->time_last_pointcloud).count();
        std::cout << "Timing function called: " << timeToCalculate << " ___ " << this->time_until_save*60 << std::endl;
        if (timeToCalculate>this->time_until_save*60) {
            saveFullPoseArrayOfGraph();
        }
    }

    void registrationOfTwoPointclouds(pcl::PointCloud<pcl::PointXYZ> firstPCL,pcl::PointCloud<pcl::PointXYZ> secondPCL,Eigen::Matrix4d &finalTransformation,std::string registrationMethod){

        //ICP stuff
        if (registrationMethod=="ICP") {
            double fitnessScore;
            Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
            // Eigen::Matrix4d resultingICPRegistration = this->scanRegistrationObject->generalizedIcpRegistrationSimple(firstPCL,secondPCL,fitnessScore,initialGuess);
            Eigen::Matrix4d resultingICPRegistration = this->scanRegistrationObject->icpRegistration(firstPCL,secondPCL,fitnessScore,initialGuess);
            finalTransformation = resultingICPRegistration;
            // std::cout << "our match after ICP" << std::endl;
            // std::cout << finalTransformation << std::endl;
        }

        //GICP stuff
        if (registrationMethod=="GICP") {
            double fitnessScore;
            Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
            // Eigen::Matrix4d resultingICPRegistration = this->scanRegistrationObject->generalizedIcpRegistrationSimple(firstPCL,secondPCL,fitnessScore,initialGuess);
            Eigen::Matrix4d resultingICPRegistration = this->scanRegistrationObject->generalizedIcpRegistrationSimple(firstPCL,secondPCL,fitnessScore,initialGuess);
            finalTransformation = resultingICPRegistration;
            // std::cout << "our match after GICP" << std::endl;
            // std::cout << finalTransformation << std::endl;
        }


        // FS3D stuff
        if (registrationMethod=="fs3d32" || registrationMethod=="fs3d64"||registrationMethod=="fs3d128"||
            registrationMethod=="fs3d32ICP"||registrationMethod=="fs3d64ICP"|| registrationMethod=="fs3d128ICP"||
            registrationMethod=="fs3d32GICP"||registrationMethod=="fs3d64GICP"|| registrationMethod=="fs3d128GICP") {
            double* voxelData1;
            double* voxelData2;
            voxelData1 = (double*)calloc(
                this->dimension_of_registration * this->dimension_of_registration * this->dimension_of_registration,
                sizeof(double));
            voxelData2 = (double*)calloc(
                this->dimension_of_registration * this->dimension_of_registration * this->dimension_of_registration,
                sizeof(double));
            pcl::PointXYZ shift = pcl::PointXYZ(0, 0, 0);
            // double voxelSize = (double)DIMENSION_OF_MAP / NUMBER_OF_POINTS_DIMENSION;


            // pcl::io::savePLYFile("/home/tim-external/dataFolder/pointclouds/testPCLs/test_1_ply.ply", pclLastScan);
            // std::cout << "sizeGraph: "<< this->graphSaved.getVertexList()->size() << std::endl;
            // std::cout << "size First PCL: "<< pclLastScan.size() << std::endl;
            slamToolsRos::convertPointToVoxel(firstPCL, voxelData1, this->dimension_of_registration,
                                              this->voxel_size, this->voxel_size, this->voxel_size, shift);


            // pcl::io::savePLYFile("/home/tim-external/dataFolder/pointclouds/testPCLs/test_2_ply.ply", cloudIncoming);
            // std::cout << "size Second PCL: "<< cloudIncoming.size() << std::endl;
            slamToolsRos::convertPointToVoxel(secondPCL, voxelData2, this->dimension_of_registration,
                                              this->voxel_size, this->voxel_size, this->voxel_size, shift);

            // std::ofstream voxel1,voxel2;
            // voxel1.open("/home/tim-external/dataFolder/pointclouds/testPCLs/voxel1Before.csv");
            // voxel2.open("/home/tim-external/dataFolder/pointclouds/testPCLs/voxel2Before.csv");
            // //save errors angle
            // for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION*NUMBER_OF_POINTS_DIMENSION*NUMBER_OF_POINTS_DIMENSION; i++) {
            //     voxel1 << voxelData1[i];//time
            //     voxel1 << "\n";//error
            //     voxel2 << voxelData2[i];//time
            //     voxel2 << "\n";//error
            // }
            // voxel1.close();
            // voxel2.close();
            // std::cout << "after pcl Conversion " << std::endl;
            double maximumVoxelData = 1;
            //Compute difference based on Registration
            Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
            double timeToCalculate = 0;
            // std::cout << "starting Registration: " << std::endl;
            std::vector<fsregistration::msg::PotentialSolution3D> potentialSolutionsList = this->scanRegistrationObject->
                registrationOfTwoVoxels3DSOFFTAllSoluations(voxelData1, maximumVoxelData,voxelData2, maximumVoxelData,
                                                            initialGuess,
                                                            covarianceMatrix, this->voxel_size, timeToCalculate);
            // std::cout << "finished Registration: " << std::endl;
            //fine from list the right Solution and the do ICP afterwards.

            double highestPeak = 0;
            Eigen::Matrix4d currentRegistrationEstimation = Eigen::Matrix4d::Identity();
            for (auto& estimatedTransformation : potentialSolutionsList)
            {
                //rotation

                if (estimatedTransformation.transformation_peak_height > highestPeak)
                {
                    Eigen::Quaterniond rotation(estimatedTransformation.resulting_transformation.orientation.w,
                                                estimatedTransformation.resulting_transformation.orientation.x,
                                                estimatedTransformation.resulting_transformation.orientation.y,
                                                estimatedTransformation.resulting_transformation.orientation.z);

                    currentRegistrationEstimation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
                    currentRegistrationEstimation.block<3, 1>(0, 3) = Eigen::Vector3d(
                        estimatedTransformation.resulting_transformation.position.x,
                        estimatedTransformation.resulting_transformation.position.y,
                        estimatedTransformation.resulting_transformation.position.z);
                    // std::cout << estimatedTransformation.potentialRotation.angle << std::endl;
                    highestPeak = estimatedTransformation.transformation_peak_height;
                }
                //translation
            }
            // std::cout << "our match after FS3D" << std::endl;
            // std::cout << currentRegistrationEstimation << std::endl;

            free(voxelData1);
            free(voxelData2);


            //fine alignment if ICP should be used
            if (registrationMethod=="fs3d32ICP"||registrationMethod=="fs3d64ICP"|| registrationMethod=="fs3d128ICP") {
                double fitnessScore;
                Eigen::Matrix4d initialGuess = currentRegistrationEstimation;
                // std::cout << "starting ICP" << std::endl;
                // Eigen::Matrix4d resultingICPRegistration = this->scanRegistrationObject->generalizedIcpRegistrationSimple(firstPCL,secondPCL,fitnessScore,initialGuess);


                Eigen::Matrix4d resultingICPRegistration = this->scanRegistrationObject->icpRegistration(firstPCL,secondPCL,fitnessScore,initialGuess);

                currentRegistrationEstimation = resultingICPRegistration;
                // std::cout << "our match before ICP" << std::endl;
                // std::cout << initialGuess << std::endl;
                // std::cout << "our match after ICP" << std::endl;
                // std::cout << currentRegistrationEstimation << std::endl;
            }
            if (registrationMethod=="fs3d32GICP"||registrationMethod=="fs3d64GICP"|| registrationMethod=="fs3d128GICP") {
                double fitnessScore;
                Eigen::Matrix4d initialGuess = currentRegistrationEstimation;
                // std::cout << "starting ICP" << std::endl;
                // std::cout << initialGuess << std::endl;
                // Eigen::Matrix4d resultingICPRegistration = this->scanRegistrationObject->generalizedIcpRegistrationSimple(firstPCL,secondPCL,fitnessScore,initialGuess);


                Eigen::Matrix4d resultingICPRegistration = this->scanRegistrationObject->generalizedIcpRegistrationSimple(firstPCL,secondPCL,fitnessScore,initialGuess);

                currentRegistrationEstimation = resultingICPRegistration;
                // std::cout << "our match before GICP" << std::endl;
                // std::cout << initialGuess << std::endl;
                // std::cout << "our match after GICP" << std::endl;
                // std::cout << currentRegistrationEstimation << std::endl;
            }
            finalTransformation = currentRegistrationEstimation;
        }

    }
public:

    void run()
    {
        std::cout << "running now in while Loop" << std::endl;
        while (rclcpp::ok()) // Check if the ROS 2 node is still running
        {
            this->updatingPCLCallback();
            sleep(0.01); // Sleep for 0.01 second to avoid high CPU usage
        }
    }



    geometry_msgs::msg::PoseArray getFullPoseArrayOfGraph()
    {
        // std::cout << " starting getting dataset "<<std::endl;
        std::vector<Eigen::Matrix4d> dataSet;

        slamToolsRos::getDatasetFromGraphforPoseArray(dataSet, this->graphSaved,
            this->odometryMutex);
        // std::cout << " got dataset "<<std::endl;
        geometry_msgs::msg::PoseArray resultPoseArray;
        resultPoseArray.header.stamp = this->get_clock()->now();
        resultPoseArray.header.frame_id = "world";

        for (int currentPosition = 0;
             currentPosition < dataSet.size(); currentPosition++)
        {
            // Take Dataset and compute one big Pointcloud that gets published(slow but should work for now)
            geometry_msgs::msg::Pose currentPose;
            Eigen::Vector3d pos;
            Eigen::Quaterniond rot;
            generalHelpfulTools::splitTransformationMatrixToQuadAndTrans(pos,rot,dataSet[currentPosition]);
            currentPose.position.x = pos(0);
            currentPose.position.y = pos(1);
            currentPose.position.z = pos(2);
            currentPose.orientation.w = rot.w();
            currentPose.orientation.x = rot.x();
            currentPose.orientation.y = rot.y();
            currentPose.orientation.z = rot.z();

            resultPoseArray.poses.push_back(currentPose);
        }
        return resultPoseArray;
    }

    void saveFullPoseArrayOfGraph() {
        // std::cout << " starting getting dataset "<<std::endl;
        std::vector<Eigen::Matrix4d> poseDataSet;

        slamToolsRos::getDatasetFromGraphforPoseArray(poseDataSet, this->graphSaved,
            this->odometryMutex);


        std::vector<Eigen::Matrix4d> gtDataSet;

        slamToolsRos::getDatasetFromGraphforGroundTruthArray(gtDataSet, this->graphSaved,
            this->odometryMutex);
        std::cout << "this->folderForSaving: "<< this->folderForSaving << std::endl;
        std::filesystem::create_directory("/home/tim-external/dataFolder/odometryResults/"+this->folderForSaving);
        std::ofstream myFile1;
        myFile1.open("/home/tim-external/dataFolder/odometryResults/"+this->folderForSaving+"/gt.csv");
        for (int i = 0; i<gtDataSet.size(); i++) {

            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    myFile1 << gtDataSet[i](j, k) << ",";//number of possible rotations
                }
                myFile1 << "\n";
            }
        }
        myFile1.close();

        std::ofstream myFile2;
        myFile2.open("/home/tim-external/dataFolder/odometryResults/"+this->folderForSaving+"/poses.csv");
        for (int i = 0; i<poseDataSet.size(); i++) {

            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    myFile2 << poseDataSet[i](j, k) << ",";//number of possible rotations
                }
                myFile2 << "\n";
            }
        }
        myFile2.close();



        exit(1);
    }
};


int main(int argc, char** argv)
{


    rclcpp::init(argc, argv);
    auto node = std::make_shared<rosClassSlam>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // executor.spin();


    // Start the execution in a separate thread
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // Call the run method in the main thread
    node->run();

    // Wait for the executor thread to finish
    executor_thread.join();



    rclcpp::shutdown();

    return (0);
}
