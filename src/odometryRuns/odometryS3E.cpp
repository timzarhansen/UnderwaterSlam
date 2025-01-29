//
// Created by jurobotics on 13.09.21.
//

#include "geometry_msgs/msg/pose_stamped.hpp"

// #include "ping360_sonar_msgs/msg/sonar_echo.hpp"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "commonbluerovmsg/srv/save_graph.hpp"
#include <pcl_conversions/pcl_conversions.h>
// #include "pcl/conversions.h"
#include <pcl/PCLPointCloud2.h>
// #include "commonbluerovmsg/msg/state_robot_for_evaluation.hpp"




#define NUMBER_OF_POINTS_DIMENSION 64
// #define DIMENSION_OF_VOXEL_DATA_FOR_MATCHING 40 // was 50 //tuhh tank 6
// #define NUMBER_OF_POINTS_MAP 512//was 512
// // 80 simulation ;300 valentin; 45.0 for Keller; 10.0 TUHH TANK ;15.0 Ocean ;35.0 DFKI
// #define DIMENSION_OF_MAP 35.0
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

class rosClassSlam : public rclcpp::Node {
public:
    rosClassSlam() : Node("ourgraphslam"), graphSaved(6, POINT_CLOUD_SAVED),
                     scanRegistrationObject(NUMBER_OF_POINTS_DIMENSION) {
        //we have to make sure, to get ALLL the data. Therefor we have to change that in the future.
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_system_default);
        qos.history(rmw_qos_history_policy_e::RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        qos.reliability(rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability( rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
        qos.liveliness( rmw_qos_liveliness_policy_e::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
        qos.deadline(rmw_time_t(RMW_DURATION_INFINITE));
        qos.lifespan( rmw_time_t(RMW_DURATION_INFINITE));
        qos.liveliness_lease_duration( rmw_time_t(RMW_DURATION_INFINITE));
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
                "sonar/intensity", qos,
                std::bind(&rosClassSlam::valodyneCallback,
                          this, std::placeholders::_1), sub2_opt);

        this->serviceSaveGraph = this->create_service<commonbluerovmsg::srv::SaveGraph>("saveGraphOfSLAM",
                                                                                        std::bind(
                                                                                                &rosClassSlam::saveGraph,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));

        this->publisherPoseSLAM = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "slamEndPose", qos);

        std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(5.0);
        this->timer_ = this->create_wall_timer(
                my_timer_duration, std::bind(&rosClassSlam::createMapOfAllScans, this));


        this->sigmaScaling = 1.0;
        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;
        this->saveGraphStructure = false;
        this->numberOfTimesFirstScan = 0;

        this->maxTimeOptimization = 1.0;
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriberVelodyne;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherPoseSLAM;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherPointcloudMap;
    rclcpp::Service<commonbluerovmsg::srv::SaveGraph>::SharedPtr serviceSaveGraph;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex groundTruthMutex;
    std::mutex odometryMutex;

    //Matrices:
    Eigen::Matrix4d currentEstimatedTransformation;
    Eigen::Matrix4d initialGuessTransformation;

    // GT savings
    std::deque<transformationStamped> currentPositionGTDeque;
    Eigen::Matrix4d currentGTPosition;

    int indexLastFullScan;
    double fitnessScore;
    double sigmaScaling;

    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan, saveGraphStructure;
    std::string saveStringGraph;
    double maxTimeOptimization;
    int numberOfTimesFirstScan;


    void valodyneCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->odometryMutex);
        pclMeasurement PCLTMP;

        PCLTMP.time = rclcpp::Time(msg->header.stamp).seconds();

        pcl::PointCloud<pcl::PointXYZ> cloudPtr;

        pcl::fromROSMsg(*msg, cloudPtr);

        // pcl_conversions::toPCL(*msgPtr, cloudPtr);
        PCLTMP.pointcloud = cloudPtr;


        if (this->firstSonarInput) {

            this->graphSaved.addVertexPCL(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                                       Eigen::Matrix3d::Zero(), PCLTMP, rclcpp::Time(msg->header.stamp).seconds(),
                                       FIRST_ENTRY);
            this->firstSonarInput = false;
            return;
        }



        //Compute difference based on Registration

        edge differenceOfEdge =  ;

        Eigen::Matrix4d tmpTransformation = this->graphSaved.getVertexList()->back().getTransformation();
        tmpTransformation = tmpTransformation * differenceOfEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);


        this->graphSaved.addVertexPCL(this->graphSaved.getVertexList()->back().getKey() + 1, pos, rot,
                                   this->graphSaved.getVertexList()->back().getCovarianceMatrix(),
                                   PCLTMP,
                                   rclcpp::Time(msg->header.stamp).seconds(),
                                   POINT_CLOUD_SAVED);


        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        covarianceMatrix(0, 0) = INTEGRATED_NOISE_XYZ;
        covarianceMatrix(1, 1) = INTEGRATED_NOISE_XYZ;
        covarianceMatrix(2, 2) = INTEGRATED_NOISE_RPY;
        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 covarianceMatrix, INTEGRATED_POSE);

            ////////////// look for loop closure  //////////////
            // slamToolsRos::loopDetectionByClosestPath(this->graphSaved, this->scanRegistrationObject,
            //                                          NUMBER_OF_POINTS_DIMENSION, IGNORE_DISTANCE_TO_ROBOT,
            //                                          DIMENSION_OF_VOXEL_DATA_FOR_MATCHING, DEBUG_REGISTRATION,
            //                                          USE_INITIAL_TRANSLATION_LOOP_CLOSURE, 250, 500,
            //                                          THRESHOLD_FOR_TRANSLATION_MATCHING, MAXIMUM_LOOP_CLOSURE_DISTANCE);



            this->graphSaved.isam2OptimizeGraph(true, 2);
//        this->graphSaved.isam2OptimizeGraph(true,1);
        // visualize graph



        // slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherSonarEcho,
        //                                         this->publisherMarkerArray, this->sigmaScaling,
        //                                         this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures,this->publisherEKF);

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

    void groundTruthGPSEvaluationCallback(const commonbluerovmsg::msg::StateRobotForEvaluation::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        //first time? calc current Position
        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(msg->roll, msg->pitch,
                                                                                        msg->yaw);
        tmpMatrix(0, 3) = msg->x_position;
        tmpMatrix(1, 3) = msg->y_position;
        tmpMatrix(2, 3) = msg->z_position;
        transformationStamped tmpValue;
        tmpValue.transformation = tmpMatrix;
        tmpValue.timeStamp = msg->timestamp;
        this->currentPositionGTDeque.push_back(tmpValue);
    }

    Eigen::Matrix4d getCurrentGTPosition() {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        return this->currentGTPosition;
    }

    void saveCurrentGTPosition() {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        while (!this->currentPositionGTDeque.empty()) {
            double currentTimeStampOfInterest = this->currentPositionGTDeque[0].timeStamp;
//            std::cout << currentTimeStampOfInterest << std::endl;
            int i = this->graphSaved.getVertexList()->size() - 1;
            while (this->graphSaved.getVertexList()->at(i).getTimeStamp() >= currentTimeStampOfInterest) {
                i--;
                if (i == -1) {
                    break;
                }
            }
            i++;
            if (i == this->graphSaved.getVertexList()->size()) {
                break;
            }
//            if (i == 0) {
//                break;
//            }

//            std::cout << this->graphSaved.getVertexList()->at(i).getTimeStamp() << std::endl;
//            std::cout << currentTimeStampOfInterest << std::endl;



            //sort in
            int j = 0;
            while (this->graphSaved.getVertexList()->at(i).getTimeStamp() >=
                   this->currentPositionGTDeque[j].timeStamp) {
                j++;
                if (j == this->currentPositionGTDeque.size()) {
                    break;
                }
            }
            if (j == this->currentPositionGTDeque.size()) {
                break;
            }
//            std::cout << this->graphSaved.getVertexList()->at(i).getTimeStamp() << std::endl;
//            std::cout << this->currentPositionGTDeque[j].timeStamp << std::endl;
            this->graphSaved.getVertexList()->at(i).setGroundTruthTransformation(
                    this->currentPositionGTDeque[j].transformation);


            for (int k = 0; k < j + 1; k++) {
                this->currentPositionGTDeque.pop_front();
            }




//            this->currentPositionGTDeque.pop_front();
        }

    }

    void createMapAndSaveToFile() {
        std::vector<pclValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForPointcloudMap(dataSet, this->graphSaved,
                                                                          this->odometryMutex);

        for (int currentPosition = 0;
             currentPosition < dataSet.size(); currentPosition++) {

            // Take Dataset and compute one big Pointcloud that gets published(slow but should work for now)


             }
    }


public:

    void createMapOfAllScans() {
        std::vector<pclValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForPointcloudMap(dataSet, this->graphSaved,
                                                                          this->odometryMutex);

        for (int currentPosition = 0;
             currentPosition < dataSet.size(); currentPosition++) {

            // Take Dataset and compute one big Pointcloud that gets published(slow but should work for now)


             }
    }


};


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rosClassSlam>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return (0);
}
