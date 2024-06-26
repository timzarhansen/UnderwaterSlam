//
// Created by tim-external on 13.11.23.
//
#include "mocap_msgs/msg/rigid_bodies.hpp"
#include "mocap_msgs/msg/markers.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "sensor_msgs/Imu.h"
//#include "mavros_msgs/Altitude.h"
//#include "geometry_msgs/TwistStamped.h"
//#include "ping360_sonar/SonarEcho.h"
#include "ping360_sonar_msgs/msg/sonar_echo.hpp"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
//#include "commonbluerovmsg/saveGraph.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
//#include "scanRegistrationClass.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "commonbluerovmsg/srv/save_graph.hpp"
#include "commonbluerovmsg/msg/state_robot_for_evaluation.hpp"
//#include "std_srvs/msg/set_bool_request.hpp"
//#include <std_srvs/msg/set_bool.hpp>
#include <iostream>


#define NUMBER_OF_POINTS_DIMENSION 256
#define DIMENSION_OF_VOXEL_DATA_FOR_MATCHING 40 // was 50 //tuhh tank 6
#define NUMBER_OF_POINTS_MAP 512//was 512
// 80 simulation ;300 valentin; 45.0 for Keller; 10.0 TUHH TANK ;15.0 Ocean ;35.0 DFKI
#define DIMENSION_OF_MAP 35.0

#define IGNORE_DISTANCE_TO_ROBOT 1.0 // was 1.0 // TUHH 0.2
#define DEBUG_REGISTRATION false

#define ROTATION_SONAR M_PI // sonar on robot M_PI // simulation 0
#define SHOULD_USE_ROSBAG false
#define FACTOR_OF_MATCHING 1.0 //1.5
#define THRESHOLD_FOR_TRANSLATION_MATCHING 0.1 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben

#define INTEGRATED_NOISE_XY 0.03 // was 0.03  // TUHH 0.005
#define INTEGRATED_NOISE_YAW 0.03 // was 0.03 // TUHH 0.005

#define USE_INITIAL_TRANSLATION_LOOP_CLOSURE true
#define MAXIMUM_LOOP_CLOSURE_DISTANCE 1.0 // 0.2 TUHH 2.0 valentin Keller 4.0 Valentin Oben // 2.0 simulation

#define SONAR_LOOKING_DOWN false
#define USES_GROUND_TRUTH false
#define PATH_TO_MATLAB_FOLDER "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/ICRA2024Scripts/dataFolder/"

#define NAME_OF_CURRENT_METHOD "testFolder"


class rosClassSlam : public rclcpp::Node {
public:
    rosClassSlam() : Node("ourgraphslam"), graphSaved(3, INTENSITY_BASED_GRAPH),
                     scanRegistrationObject(NUMBER_OF_POINTS_DIMENSION) {
        //we have to make sure, to get ALLL the data. Therefor we have to change that in the future.
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10000), rmw_qos_profile_system_default);
        qos.history(rmw_qos_history_policy_e::RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        qos.reliability(rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
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
        this->callback_group_subscriber3_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        this->callback_group_subscriber4_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        this->callback_group_subscriber5_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = this->callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = this->callback_group_subscriber2_;
        auto sub3_opt = rclcpp::SubscriptionOptions();
        sub3_opt.callback_group = this->callback_group_subscriber3_;
        auto sub5_opt = rclcpp::SubscriptionOptions();
        sub5_opt.callback_group = this->callback_group_subscriber5_;

        this->subscriberEKF = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos,
                std::bind(&rosClassSlam::stateEstimationCallback,
                          this, std::placeholders::_1), sub1_opt);


//        rclcpp::sleep_for(std::chrono::nanoseconds(std::chrono::seconds(1)));

        this->subscriberPing360Sonar = this->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
                "sonar/intensity", qos,
                std::bind(&rosClassSlam::ping360SonarCallback,
                          this, std::placeholders::_1), sub2_opt);

        this->subscriberGroundTruth = this->create_subscription<mocap_msgs::msg::RigidBodies>(
                "rigid_bodies", qos,
                std::bind(&rosClassSlam::gtCallback,
                          this, std::placeholders::_1), sub3_opt);
        this->subscriberMarkersOfGroundTruth = this->create_subscription<mocap_msgs::msg::Markers>(
                "markers", qos,
                std::bind(&rosClassSlam::markersCallback,
                          this, std::placeholders::_1), sub5_opt);

//        this->subscriberPing360Sonar = n_.subscribe("sonar/intensity", 10000, &rosClassSlam::ping360SonarCallback, this);
        this->serviceSaveGraph = this->create_service<commonbluerovmsg::srv::SaveGraph>("saveGraphOfSLAM",
                                                                                        std::bind(
                                                                                                &rosClassSlam::saveGraph,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2),
                                                                                        rmw_qos_profile_services_default,
                                                                                        this->callback_group_subscriber4_);


//        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassSlam::saveGraph, this);

//        publisherSonarEcho = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
        this->publisherSonarEcho = this->create_publisher<nav_msgs::msg::Path>(
                "positionOverTime", qos);


//        publisherEKF = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
        this->publisherEKF = this->create_publisher<nav_msgs::msg::Path>(
                "positionOverTimeGT", qos);


//        publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
        this->publisherMarkerArray = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "covariance", qos);
//        publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
        this->publisherMarkerArrayLoopClosures = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "loopClosures", qos);
//        publisherOccupancyMap = n_.advertise<nav_msgs::OccupancyGrid>("occupancyHilbertMap", 10);
        this->publisherOccupancyMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "occupancyHilbertMap", qos);
//        publisherPoseSLAM = n_.advertise<geometry_msgs::PoseStamped>("slamEndPose", 10);
        this->publisherPoseSLAM = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "slamEndPose", qos);

        std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(5.0);
        this->timer_ = this->create_wall_timer(
                my_timer_duration, std::bind(&rosClassSlam::createImageOfAllScans, this));

//        if (SHOULD_USE_ROSBAG) {
//            pauseRosbag = n_.serviceClient<std_srvs::SetBoolRequest>(nameOfTheServiceCall);
//        }



//        graphSaved.addVertex(1, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
//                             Eigen::Vector3d(0, 0, 0), 0, rclcpp::Time::now().toSec(),
//                             FIRST_ENTRY);

//        std::deque<double> subgraphs{0.3, 2};
//        graphSaved.initiallizeSubGraphs(subgraphs, 10);
        this->sigmaScaling = 1.0;

//        this->maxTimeOptimization = 1.0;

        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;
        this->saveGraphStructure = false;
        this->numberOfTimesFirstScan = 0;

//        this->occupancyMap.createRandomMap();
        this->maxTimeOptimization = 1.0;


        map.info.height = NUMBER_OF_POINTS_MAP;
        map.info.width = NUMBER_OF_POINTS_MAP;
        map.info.resolution = DIMENSION_OF_MAP / NUMBER_OF_POINTS_MAP;
        map.info.origin.position.x = -DIMENSION_OF_MAP / 2;
        map.info.origin.position.y = -DIMENSION_OF_MAP / 2;
        map.info.origin.position.z = +0.5;
        for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
            for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
                //determine color:
                map.data.push_back(50);
            }
        }
        this->numberOfMarkersSeen = 0;
        this->numberOfScan = 0;
    }


private:
    nav_msgs::msg::OccupancyGrid map;

    int numberOfScan;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriberEKF;
    rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr subscriberPing360Sonar;
    rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr subscriberGroundTruth;
    rclcpp::Subscription<mocap_msgs::msg::Markers>::SharedPtr subscriberMarkersOfGroundTruth;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherPoseSLAM;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisherOccupancyMap;
    rclcpp::Service<commonbluerovmsg::srv::SaveGraph>::SharedPtr serviceSaveGraph;
//    rclcpp::Service<commonbluerovmsg::srv::ResetEkf>::SharedPtr pauseRosbag;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber3_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber4_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber5_;
    rclcpp::TimerBase::SharedPtr timer_;


    std::mutex stateEstimationMutex;
    std::mutex groundTruthMutex;
    std::mutex graphSlamMutex;
    //GraphSlam things
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherSonarEcho;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherEKF;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisherMarkerArray;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisherMarkerArrayLoopClosures;


    //PCL
    //std::vector<ping360_sonar::SonarEcho> sonarIntensityList;
    //Matrices:
    Eigen::Matrix4d currentEstimatedTransformation;
    Eigen::Matrix4d initialGuessTransformation;


    //EKF savings
    std::deque<edge> posDiffOverTimeEdges;
    std::deque<transformationStamped> ekfTransformationList;
//    std::deque<double>  timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector; xPositionVector, yPositionVector, zPositionVector,
//    std::deque<Eigen::Quaterniond> rotationVector;

    // GT savings
    std::deque<transformationStamped> currentPositionGTDeque;
    Eigen::Matrix4d currentGTPosition;
    std::atomic<int> numberOfMarkersSeen;

    //int numberOfEdgesBetweenScans;
    int indexLastFullScan;
    //double timeCurrentFullScan;
    double fitnessScore;
    double sigmaScaling;

    //double beginningAngleOfRotation;
    //double lastAngle;
    //double startTimeOfCorrection;
    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan, saveGraphStructure;
    std::string saveStringGraph;
    double maxTimeOptimization;
    //hilbertMap occupancyMap;
    int numberOfTimesFirstScan;


    void ping360SonarCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
//        std::cout <<  std::setprecision(19);
//        std::cout << "timeToAdd" << std::endl;
//        std::cout << msg << std::endl;

//        std::cout << "huhu1" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
//        std::cout << "huhu2" << std::endl;
        intensityMeasurement intensityTMP;
        if (SONAR_LOOKING_DOWN) {
            intensityTMP.angle = std::fmod(-msg->angle / 400.0 * M_PI * 2.0 + ROTATION_SONAR,
                                           M_PI * 2);// TEST TRYING OUT -
        } else {
            intensityTMP.angle = std::fmod(msg->angle / 400.0 * M_PI * 2.0 + ROTATION_SONAR,
                                           M_PI * 2);// TEST TRYING OUT -
        }

        intensityTMP.time = rclcpp::Time(msg->header.stamp).seconds(); // rclcpp::Time(msg->header.stamp).seconds();
        intensityTMP.range = msg->range;
        intensityTMP.increment = msg->step_size;
        std::vector<double> intensitiesVector;
        for (int i = 0; i < msg->intensities.size(); i++) {
            intensitiesVector.push_back(msg->intensities[i]);
        }
        intensityTMP.intensities = intensitiesVector;

        if (firstSonarInput) {

            this->graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                                       Eigen::Matrix3d::Zero(), intensityTMP, rclcpp::Time(msg->header.stamp).seconds(),
                                       FIRST_ENTRY);
//            if(SONAR_LOOKING_DOWN) {
//                //SAVE TUHH GT POSE
//                this->graphSaved.getVertexList()->at(0).setGroundTruthTransformation(getCurrentGTPosition());
//            }
            firstSonarInput = false;
            return;
        }


        bool waitingForMessages = waitForEKFMessagesToArrive(rclcpp::Time(msg->header.stamp).seconds());
        if (!waitingForMessages) {
            std::cout << "return no message found: " << rclcpp::Time(msg->header.stamp).seconds() << "    "
                      << rclcpp::Clock(RCL_ROS_TIME).now().seconds() << std::endl;
            return;
        }
        edge differenceOfEdge = slamToolsRos::calculatePoseDiffByTimeDepOnEKF(
                this->graphSaved.getVertexList()->back().getTimeStamp(), rclcpp::Time(msg->header.stamp).seconds(),
                this->ekfTransformationList, this->stateEstimationMutex);


        slamToolsRos::clearSavingsOfPoses(this->graphSaved.getVertexList()->back().getTimeStamp() - 5.0,
                                          this->ekfTransformationList, this->currentPositionGTDeque,
                                          this->stateEstimationMutex);

        Eigen::Matrix4d tmpTransformation = this->graphSaved.getVertexList()->back().getTransformation();
        tmpTransformation = tmpTransformation * differenceOfEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);


        this->graphSaved.addVertex(this->graphSaved.getVertexList()->back().getKey() + 1, pos, rot,
                                   this->graphSaved.getVertexList()->back().getCovarianceMatrix(),
                                   intensityTMP,
                                   rclcpp::Time(msg->header.stamp).seconds(),
                                   PING360_MEASUREMENT);

//        if (USES_GROUND_TRUTH) {
//            //SAVE qualisys GT POSE
//            this->graphSaved.getVertexList()->back().setGroundTruthTransformation(gtPosition);
//            this->graphSaved.getVertexList()->back().setNumberOfMarkersSeen((double)(this->numberOfMarkersSeen));
//        }

        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        covarianceMatrix(0, 0) = INTEGRATED_NOISE_XY;
        covarianceMatrix(1, 1) = INTEGRATED_NOISE_XY;
        covarianceMatrix(2, 2) = INTEGRATED_NOISE_YAW;
        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 covarianceMatrix, INTEGRATED_POSE);


        slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherSonarEcho,
                                                this->publisherMarkerArray, this->sigmaScaling,
                                                this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures,
                                                this->publisherEKF);

        int indexOfLastKeyframe;
        double angleDiff = slamToolsRos::angleBetweenLastKeyframeAndNow(this->graphSaved);// i think this is always true

        std::cout << angleDiff << std::endl;
        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
        if (abs(angleDiff) > 2 * M_PI / FACTOR_OF_MATCHING) {
            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);
            if (firstCompleteSonarScan) {
                numberOfTimesFirstScan++;
                if (numberOfTimesFirstScan > 2 * FACTOR_OF_MATCHING - 1) {
                    firstCompleteSonarScan = false;
                }
                return;
            }

//            if (SHOULD_USE_ROSBAG) {
//                std_srvs::SetBool srv;
//                srv.request.data = true;
//                pauseRosbag.call(srv);
//            }
            //sort in GT

//
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

            //angleDiff = angleBetweenLastKeyframeAndNow(false);
//            indexOfLastKeyframe = slamToolsRos::getLastIntensityKeyframe(this->graphSaved);
            int indexStart1, indexEnd1, indexStart2, indexEnd2;
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved.getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
            indexStart2 = indexEnd1;
            slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart2, indexEnd2, this->graphSaved);


            std::cout << "scanAcusitionTime: " << this->graphSaved.getVertexList()->at(indexStart2).getTimeStamp() -
                                                  this->graphSaved.getVertexList()->at(indexEnd2).getTimeStamp()
                      << std::endl;

            //we inverse the initial guess, because the registration creates a T from scan 1 to scan 2.
            // But the graph creates a transformation from 1 -> 2 by the robot, therefore inverse.
            this->initialGuessTransformation =
                    (this->graphSaved.getVertexList()->at(indexStart2).getTransformation().inverse() *
                     this->graphSaved.getVertexList()->at(indexStart1).getTransformation());

//            std::cout << "this->initialGuessTransformation.inverse()" << std::endl;
//            std::cout << this->initialGuessTransformation.inverse() << std::endl;

            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                                  this->initialGuessTransformation(0, 0));

            double *voxelData1;
            double *voxelData2Uncorrected;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2Uncorrected = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);

            double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1, indexEnd1,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());//get voxel

            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPointUncorrected(voxelData2Uncorrected, indexStart1, indexEnd1,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());//get voxel

            this->numberOfScan++;


            string ourString1 =
                    std::string(PATH_TO_MATLAB_FOLDER) + std::string(NAME_OF_CURRENT_METHOD) + "/correctedScan" +
                    std::to_string(numberOfScan) + ".csv";
            string ourString2 =
                    std::string(PATH_TO_MATLAB_FOLDER) + std::string(NAME_OF_CURRENT_METHOD) + "/uncorrectedScan" +
                    std::to_string(numberOfScan) + ".csv";
            std::ofstream outMatrix;
            outMatrix.open(ourString1);
            for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
                for (int k = 0; k < NUMBER_OF_POINTS_DIMENSION; k++)
                    outMatrix << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * k] << ',';
                outMatrix << '\n';
            }
            outMatrix.close();

            outMatrix.open(ourString2);
            for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
                for (int k = 0; k < NUMBER_OF_POINTS_DIMENSION; k++)
                    outMatrix << voxelData2Uncorrected[j + NUMBER_OF_POINTS_DIMENSION * k] << ',';
                outMatrix << '\n';
            }
            outMatrix.close();





            Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
            std::cout << "direct matching consecutive: " << std::endl;
            // result is matrix to transform scan 1 to scan 2 therefore later inversed + initial guess inversed


        }







//        std::cout << "huhu3" << std::endl;
    }

    void stateEstimationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
//        std::cout << "starting State Estimation" << std::endl;
        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
//        std::cout << "afterMutex" << std::endl;

        double currentTimeStamp = rclcpp::Time(msg->header.stamp).seconds();

        // calculate where to put the current new message
        int i = 0;
        if (!this->ekfTransformationList.empty()) {
            i = this->ekfTransformationList.size();
            while (this->ekfTransformationList[i - 1].timeStamp > currentTimeStamp) {
                i--;
            }
        }

        if (i == this->ekfTransformationList.size() || i == 0) {
            Eigen::Quaterniond tmpQuad(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
            Eigen::Vector3d tmpVec(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            Eigen::Matrix4d transformationMatrix = generalHelpfulTools::getTransformationMatrix(tmpVec, tmpQuad);

            transformationStamped tmpTransformationStamped;
            tmpTransformationStamped.timeStamp = rclcpp::Time(msg->header.stamp).seconds();
            tmpTransformationStamped.transformation = transformationMatrix;
//            this->rotationVector.push_back(tmpQuad);

            this->ekfTransformationList.push_back(tmpTransformationStamped);
//            this->xPositionVector.push_back(msg->pose.pose.position.x);
//            this->yPositionVector.push_back(msg->pose.pose.position.y);
//            this->zPositionVector.push_back(msg->pose.pose.position.z);
        } else {
            std::cout << "should mean an EKF message came in different order" << std::endl;
            exit(0);
        }
    }


    bool saveGraph(const std::shared_ptr<commonbluerovmsg::srv::SaveGraph::Request> req,
                   std::shared_ptr<commonbluerovmsg::srv::SaveGraph::Response> res) {
        std::cout << "test for saving1" << std::endl;

        this->createMapAndSaveToFile();
        //create image without motion compensation
        //create image with motion compensation(saved images)
        //create image with SLAM compensation
        std::cout << "test for saving2" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        std::cout << "test for saving3" << std::endl;

        std::ofstream myFile1, myFile2, myFile3, myFile4;
        myFile1.open(
                std::string(PATH_TO_MATLAB_FOLDER) + "positionEstimationOverTime" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        myFile2.open(
                std::string(PATH_TO_MATLAB_FOLDER) + "groundTruthOverTime" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        myFile3.open(
                std::string(PATH_TO_MATLAB_FOLDER) + "angleAndIntensities" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        myFile4.open(
                std::string(PATH_TO_MATLAB_FOLDER) + "numberOfMarkers" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        for (int k = 0; k < this->graphSaved.getVertexList()->size(); k++) {

            Eigen::Matrix4d tmpMatrix1 = this->graphSaved.getVertexList()->at(k).getTransformation();
            Eigen::Matrix4d tmpMatrix2 = this->graphSaved.getVertexList()->at(k).getGroundTruthTransformation();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    myFile1 << tmpMatrix1(i, j) << " ";//number of possible rotations
                }
                myFile1 << "\n";
            }


            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    myFile2 << tmpMatrix2(i, j) << " ";//number of possible rotations
                }
                myFile2 << "\n";
            }

            intensityMeasurement tmpIntensityMeasurements = this->graphSaved.getVertexList()->at(k).getIntensities();

            myFile3 << tmpIntensityMeasurements.range << "," << tmpIntensityMeasurements.angle << ",";
            for (auto intensity: tmpIntensityMeasurements.intensities) {
                myFile3 << intensity << ",";
            }
            myFile3 << "\n";

            myFile4 << this->graphSaved.getVertexList()->at(k).getNumberOfMarkersSeen();

            myFile4 << "\n";
        }


        myFile1.close();
        myFile2.close();
        myFile3.close();
        res->saved = true;
        return true;
    }

    bool waitForEKFMessagesToArrive(double timeUntilWait) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << "tmp1" << std::endl;
        while (this->ekfTransformationList.empty() && timeToCalculate < 2000) {
            rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        }
//        std::cout << "tmp2" << std::endl;
        while (timeUntilWait > ekfTransformationList.back().timeStamp) {
//            std::cout <<  std::setprecision(19);
//            std::cout << "test1" << std::endl;
//            std::cout <<  timeUntilWait << std::endl;
//            std::cout << timeVector.back() << std::endl;
            // @TODO different solution for waiting
//            rclcpp::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf",rclcpp::Duration(10));
//            rclcpp::Duration(0.001).sleep();
            rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//            std::cout << timeToCalculate << std::endl;
            double timeToWait = 4000;
//            if (USES_GROUND_TRUTH) {
//                timeToWait = 20;
//            }
            if (timeToCalculate > timeToWait) {
                std::cout << "we break" << std::endl;
                break;
            }
        }
//        std::cout << "tmp3" << std::endl;
//        rclcpp::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
        rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
//        std::cout << "tmp4" << std::endl;
        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << timeToCalculate << std::endl;
        double timeToWait = 40;
        if (USES_GROUND_TRUTH) {
            timeToWait = 4000;
        }
        if (timeToCalculate > timeToWait) {
            return false;
        } else {
            return true;
        }
    }

    bool waitForGTMessagesToArrive(double timeUntilWait) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << "tmp1" << std::endl;
        while (this->currentPositionGTDeque.empty() && timeToCalculate < 2000) {
            rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        }
//        std::cout << "tmp2" << std::endl;
        while (timeUntilWait > this->currentPositionGTDeque.back().timeStamp) {

            rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            double timeToWait = 25000;
            if (timeToCalculate > timeToWait) {
                std::cout << "we break" << std::endl;
                return false;
            }
        }

        return true;
    }

    void groundTruthEvaluationCallback(const commonbluerovmsg::msg::StateRobotForEvaluation::SharedPtr msg) {
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


    void markersCallback(const mocap_msgs::msg::Markers::SharedPtr msg) {
//        rclcpp::Time(msg->header.stamp).seconds();
        this->numberOfMarkersSeen = msg->markers.size();

    }

    void gtCallback(const mocap_msgs::msg::RigidBodies::SharedPtr msg) {

        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        if (msg->rigidbodies.size() < 1) {
            return;
        }
        for (auto bodies: msg->rigidbodies) {
            if (bodies.rigid_body_name == "0") {
                Eigen::Quaterniond rotation;
                Eigen::Vector3d translation;
                translation.x() = bodies.pose.position.x;
                translation.y() = bodies.pose.position.y;
                translation.z() = bodies.pose.position.z;


                rotation.x() = bodies.pose.orientation.x;
                rotation.y() = bodies.pose.orientation.y;
                rotation.z() = bodies.pose.orientation.z;
                rotation.w() = bodies.pose.orientation.w;


                Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrix(translation, rotation);
//                std:: cout << tmpMatrix << std::endl;
                transformationStamped tmpValue;
                tmpValue.transformation = tmpMatrix;
                tmpValue.timeStamp = rclcpp::Time(msg->header.stamp).seconds();
                this->currentGTPosition = tmpMatrix;

                this->currentPositionGTDeque.push_back(tmpValue);

            } else {
                std::cout << "no not 0 there" << std::endl;
            }
        }
    }

    void groundTruthEvaluationTUHHCallback(const geometry_msgs::msg::Point::SharedPtr &msg) {


//        std::cout << "test" << std::endl;
        //first time? calc current Position
        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0);
        tmpMatrix(0, 3) = msg->x;
        tmpMatrix(1, 3) = msg->y;
        tmpMatrix(2, 3) = msg->z;
//        std:: cout << tmpMatrix << std::endl;
        transformationStamped tmpValue;
        tmpValue.transformation = tmpMatrix;
        tmpValue.timeStamp = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();

        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        this->currentGTPosition = tmpMatrix;

//        this->currentPositionGTDeque.push_back(tmpValue);
    }

    Eigen::Matrix4d getCurrentGTPosition() {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        return this->currentGTPosition;
    }


    void createMapAndSaveToFile() {

        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex);
        //homePosition is 0 0
        //size of mapData is defined in NUMBER_OF_POINTS_MAP

        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        double *mapData;
        mapData = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        //set zero voxel and index
        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }

        for (int currentPosition = 0;
             currentPosition < dataSet.size(); currentPosition++) {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.
            //was 90 yaw and 180 roll

            Eigen::Matrix4d transformationOfIntensityRay =
                    generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0.0 / 180.0 * M_PI) *
                    generalHelpfulTools::getTransformationMatrixFromRPY(0.0 / 180.0 * M_PI, 0, 0) *
                    dataSet[currentPosition].transformation;
            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             dataSet[currentPosition].intensity.angle);

            int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT / (dataSet[currentPosition].intensity.range /
                                                                    ((double) dataSet[currentPosition].intensity.intensities.size())));


            for (int j = ignoreDistance;
                 j <
                 dataSet[currentPosition].intensity.intensities.size(); j++) {
                double distanceOfIntensity =
                        j / ((double) dataSet[currentPosition].intensity.intensities.size()) *
                        ((double) dataSet[currentPosition].intensity.range);

                int incrementOfScan = dataSet[currentPosition].intensity.increment;
                for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                    Eigen::Vector4d positionOfIntensity(
                            distanceOfIntensity,
                            0,
                            0,
                            1);
                    double rotationOfPoint = l / 400.0;
                    Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0,
                                                                                                                0,
                                                                                                                rotationOfPoint);
                    positionOfIntensity = rotationForBetterView * positionOfIntensity;

                    positionOfIntensity =
                            transformationOfIntensityRay * rotationOfSonarAngleMatrix * positionOfIntensity;
                    //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                    int indexX =
                            (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;


                    if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] +
                                dataSet[currentPosition].intensity.intensities[j];
                        //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                        //                    std::cout << "random: " << std::endl;
                    }
                }
            }

        }


        //make sure next iteration the correct registrationis calculated
        //TO THE END
        //NOW: TO THE BEGINNING


        double maximumOfVoxelData = 0;
        double minimumOfVoxelData = INFINITY;

        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            if (voxelDataIndex[i] > 0) {
                mapData[i] = mapData[i] / voxelDataIndex[i];
                if (maximumOfVoxelData < mapData[i]) {
                    maximumOfVoxelData = mapData[i];
                }
                if (minimumOfVoxelData > mapData[i]) {
                    minimumOfVoxelData = mapData[i];
                }
                //std::cout << voxelData[i] << std::endl;
            }
        }


        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {

            mapData[i] = (mapData[i] - minimumOfVoxelData) / (maximumOfVoxelData - minimumOfVoxelData) * 250;
        }


        std::ofstream myFile1;
        myFile1.open(
                std::string(PATH_TO_MATLAB_FOLDER) + "currentMap" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
            for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
                myFile1 << mapData[j + NUMBER_OF_POINTS_MAP * i] << std::endl;//number of possible rotations
            }
        }

        myFile1.close();


    }


public:

    void createImageOfAllScans() {
//        std::cout << "starting map Creation" << std::endl;
        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex);
        //homePosition is 0 0
        //size of mapData is defined in NUMBER_OF_POINTS_MAP

        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        double *mapData;
        mapData = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        //set zero voxel and index
        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }

        for (int currentPosition = 0;
             currentPosition < dataSet.size(); currentPosition++) {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.
            //was 90 yaw and 180 roll

            Eigen::Matrix4d transformationOfIntensityRay =
                    generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0.0 / 180.0 * M_PI) *
                    generalHelpfulTools::getTransformationMatrixFromRPY(0.0 / 180.0 * M_PI, 0, 0) *
                    dataSet[currentPosition].transformation;
            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             dataSet[currentPosition].intensity.angle);

            int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT / (dataSet[currentPosition].intensity.range /
                                                                    ((double) dataSet[currentPosition].intensity.intensities.size())));


            for (int j = ignoreDistance;
                 j <
                 dataSet[currentPosition].intensity.intensities.size(); j++) {
                double distanceOfIntensity =
                        j / ((double) dataSet[currentPosition].intensity.intensities.size()) *
                        ((double) dataSet[currentPosition].intensity.range);

                int incrementOfScan = dataSet[currentPosition].intensity.increment;
                for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                    Eigen::Vector4d positionOfIntensity(
                            distanceOfIntensity,
                            0,
                            0,
                            1);
                    double rotationOfPoint = l / 400.0;
                    Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0,
                                                                                                                0,
                                                                                                                rotationOfPoint);
                    positionOfIntensity = rotationForBetterView * positionOfIntensity;

                    positionOfIntensity =
                            transformationOfIntensityRay * rotationOfSonarAngleMatrix * positionOfIntensity;
                    //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                    int indexX =
                            (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;


                    if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] +
                                dataSet[currentPosition].intensity.intensities[j];
                        //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                        //                    std::cout << "random: " << std::endl;
                    }
                }
            }

        }


        //make sure next iteration the correct registrationis calculated
        //TO THE END
        //NOW: TO THE BEGINNING


        double maximumOfVoxelData = 0;
        double minimumOfVoxelData = INFINITY;

        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            if (voxelDataIndex[i] > 0) {
                mapData[i] = mapData[i] / voxelDataIndex[i];
                if (maximumOfVoxelData < mapData[i]) {
                    maximumOfVoxelData = mapData[i];
                }
                if (minimumOfVoxelData > mapData[i]) {
                    minimumOfVoxelData = mapData[i];
                }
                //std::cout << voxelData[i] << std::endl;
            }
        }


        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {

            mapData[i] = (mapData[i] - minimumOfVoxelData) / (maximumOfVoxelData - minimumOfVoxelData) * 250;
        }


        nav_msgs::msg::OccupancyGrid occupanyMap;
        occupanyMap.header.frame_id = "map_ned";
        occupanyMap.info.height = NUMBER_OF_POINTS_MAP;
        occupanyMap.info.width = NUMBER_OF_POINTS_MAP;
        occupanyMap.info.resolution = DIMENSION_OF_MAP / NUMBER_OF_POINTS_MAP;
        occupanyMap.info.origin.position.x = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.y = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.z = +0.5;
        for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
            for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
                //determine color:
                occupanyMap.data.push_back((int) (mapData[j + NUMBER_OF_POINTS_MAP * i]));
            }
        }
        this->publisherOccupancyMap->publish(occupanyMap);
        free(voxelDataIndex);
        free(mapData);
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
