//
// Created by jurobotics on 13.09.21.
//

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ping360_sonar_msgs/msg/sonar_echo.hpp"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "commonbluerovmsg/srv/save_graph.hpp"
#include "commonbluerovmsg/msg/state_robot_for_evaluation.hpp"
#include "micron_driver_ros/msg/scan_line.hpp"


struct sonarMeasurement {
    ping360_sonar_msgs::msg::SonarEcho pingMSG;
    micron_driver_ros::msg::ScanLine micronMSG;
    int typeMeasurement;
    double timeStamp;
};

#define TEST_MM_INSTEAD_OF_METER false


#define NUMBER_OF_POINTS_DIMENSION 256
#define DIMENSION_OF_VOXEL_DATA_FOR_MATCHING 45 // was 50 //tuhh tank 6
#define NUMBER_OF_POINTS_MAP 512//was 512
// 80 simulation ;300 valentin; 45.0 for Keller; 10.0 TUHH TANK ;15.0 Ocean ;35.0 DFKI
#define DIMENSION_OF_MAP 40.0

#define IGNORE_DISTANCE_TO_ROBOT 0.5 // was 1.0 // TUHH 0.2


#define ROTATION_SONAR M_PI // sonar on robot M_PI // simulation 0
#define SHOULD_USE_ROSBAG false
#define FACTOR_OF_MATCHING 1.5 //1.5
#define THRESHOLD_FOR_TRANSLATION_MATCHING 0.01 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben

#define INTEGRATED_NOISE_XY 0.03 // was 0.03  // TUHH 0.005
#define INTEGRATED_NOISE_YAW 0.03 // was 0.03 // TUHH 0.005

#define USE_INITIAL_TRANSLATION_LOOP_CLOSURE true
#define MAXIMUM_LOOP_CLOSURE_DISTANCE 4.0 // 0.2 TUHH 2.0 valentin Keller 4.0 Valentin Oben // 2.0 simulation

#define SONAR_LOOKING_DOWN false
#define USES_GROUND_TRUTH false

#define DEBUG_REGISTRATION false
#define USES_REGISTRATIONS true
#define NAME_OF_CURRENT_METHOD "randomTest"

//#define NAME_OF_CURRENT_METHOD "_circle_dead_reckoning_"
//#define NAME_OF_CURRENT_METHOD "_circle_dead_reckoning_wsm_"
//#define NAME_OF_CURRENT_METHOD "_circle_dynamic_slam_1_0_"
//#define NAME_OF_CURRENT_METHOD "_circle_dynamic_slam_4_0_"
//#define NAME_OF_CURRENT_METHOD "_video_4_0_"

//#define NAME_OF_CURRENT_METHOD "_TEST2_classical_slam_"

//occupancyMap(256, NUMBER_OF_POINTS_DIMENSION, 70, hilbertMap::HINGED_FEATURES)
class rosClassSlam : public rclcpp::Node {
public:
    rosClassSlam() : Node("ourgraphslam"), graphSaved(3, INTENSITY_BASED_GRAPH),
                     scanRegistrationObject(NUMBER_OF_POINTS_DIMENSION) {
        //we have to make sure, to get ALLL the data. Therefor we have to change that in the future.
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_system_default);
        qos.history(rmw_qos_history_policy_e::RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        qos.reliability(rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
        qos.liveliness(rmw_qos_liveliness_policy_e::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
        qos.deadline(rmw_time_t(RMW_DURATION_INFINITE));
        qos.lifespan(rmw_time_t(RMW_DURATION_INFINITE));
        qos.liveliness_lease_duration(rmw_time_t(RMW_DURATION_INFINITE));
        qos.avoid_ros_namespace_conventions(false);










//        auto qos = rclcpp::QoSInitialization::from_rmw(ourQOSSLAM);

        this->callback_group_subscriber1_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        this->callback_group_subscriber2_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;

        this->callback_group_subscriber3_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub3_opt = rclcpp::SubscriptionOptions();
        sub3_opt.callback_group = callback_group_subscriber3_;


        this->subscriberEKF = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos,
                std::bind(&rosClassSlam::stateEstimationCallback,
                          this, std::placeholders::_1), sub1_opt);


//        rclcpp::sleep_for(std::chrono::nanoseconds(std::chrono::seconds(1)));

        this->subscriberPing360Sonar = this->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
                "sonar/intensity", qos,
                std::bind(&rosClassSlam::ping360HelperCallback,
                          this, std::placeholders::_1), sub2_opt);


        this->subscriberMicronSonar = this->create_subscription<micron_driver_ros::msg::ScanLine>(
                "tritech_sonar/scan_lines", qos,
                std::bind(&rosClassSlam::micronHelperCallback,
                          this, std::placeholders::_1), sub2_opt);

        this->sonarTimer_ = this->create_wall_timer(
                10ms, std::bind(&rosClassSlam::startSonarFunction, this), this->callback_group_subscriber3_);

//        this->sonarTimer_ = this->create_wall_timer(
//                10ms, std::bind(&rosClassSlam::testSonarFunction, this), this->callback_group_subscriber3_);





//        this->subscriberPing360Sonar = n_.subscribe("sonar/intensity", 10000, &rosClassSlam::ping360SonarCallback, this);
        this->serviceSaveGraph = this->create_service<commonbluerovmsg::srv::SaveGraph>("saveGraphOfSLAM",
                                                                                        std::bind(
                                                                                                &rosClassSlam::saveGraph,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));


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

        std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(10.0);
        this->timer_ = this->create_wall_timer(
                my_timer_duration, std::bind(&rosClassSlam::createImageOfAllScans, this));

//        std::chrono::duration<double>  my_timer_duration2 = std::chrono::duration<double>(1/100.0);




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
    }


private:
    nav_msgs::msg::OccupancyGrid map;


    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriberEKF;
    rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr subscriberPing360Sonar;
    rclcpp::Subscription<micron_driver_ros::msg::ScanLine>::SharedPtr subscriberMicronSonar;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherPoseSLAM;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisherOccupancyMap;
    rclcpp::Service<commonbluerovmsg::srv::SaveGraph>::SharedPtr serviceSaveGraph;
//    rclcpp::Service<commonbluerovmsg::srv::ResetEkf>::SharedPtr pauseRosbag;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber3_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr sonarTimer_;

    std::mutex stateEstimationMutex;
    std::mutex groundTruthMutex;
    std::mutex graphSlamMutex;
    std::mutex sonarInputMutex;
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
//    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector;
    std::deque<transformationStamped> ekfTransformationList;
//    std::deque<Eigen::Quaterniond> rotationVector;

    // GT savings
    std::deque<transformationStamped> currentPositionGTDeque;
    Eigen::Matrix4d currentGTPosition;


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

    std::deque<sonarMeasurement> ourListOfSonarMeasurements;


    void ping360HelperCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->sonarInputMutex);


        sonarMeasurement tmpSonarMeas;

        tmpSonarMeas.pingMSG = *msg;
        tmpSonarMeas.timeStamp = rclcpp::Time(msg->header.stamp).seconds();
        tmpSonarMeas.typeMeasurement = 1;
//        std::cout << "sonarMessage is coming in" << std::endl;
        this->ourListOfSonarMeasurements.push_back(tmpSonarMeas);
    }

    void micronHelperCallback(const micron_driver_ros::msg::ScanLine::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->sonarInputMutex);


        sonarMeasurement tmpSonarMeas;

        tmpSonarMeas.micronMSG = *msg;
        tmpSonarMeas.timeStamp = rclcpp::Time(msg->header.stamp).seconds();
        tmpSonarMeas.typeMeasurement = 2;
        this->ourListOfSonarMeasurements.push_back(tmpSonarMeas);
    }

    void testSonarFunction() {
        std::cout << "before" << std::endl;
        std::lock_guard<std::mutex> lock(this->sonarInputMutex);
        std::cout << "after" << std::endl;

        if (this->ourListOfSonarMeasurements.size() > 50) {
            int indexFirstPing = 0;
            int indexSecondPing = 0;
//            std::vector<int> micronIndexList;
            double correctionTime = this->ourListOfSonarMeasurements[0].timeStamp;

            //find 3 ping
            int numberOfPingMessagesToSkip = 4;
            int i = 0;
            int counter = 0;
            while (counter < numberOfPingMessagesToSkip) {
                if (this->ourListOfSonarMeasurements[i].typeMeasurement == 1) {
                    counter++;
                }
                i++;
            }

            indexFirstPing = i - 1;

            // find 4 ping
            i = 0;
            counter = 0;
            while (counter < numberOfPingMessagesToSkip + 1) {
                if (this->ourListOfSonarMeasurements[i].typeMeasurement == 1) {
                    counter++;
                }
                i++;
            }
            indexSecondPing = i - 1;


            //calculate EKF estimation 3->4(ping)

            edge differenceOfEdgeEKF = slamToolsRos::calculatePoseDiffByTimeDepOnEKF(
                    this->ourListOfSonarMeasurements[indexFirstPing].timeStamp,
                    this->ourListOfSonarMeasurements[indexSecondPing].timeStamp,
                    this->ekfTransformationList, this->stateEstimationMutex);
            Eigen::Matrix4d transofrmationDiffEKF = differenceOfEdgeEKF.getTransformation();

//            std::cout << std::setprecision(15) << "EKF Time: "
//                      << this->ourListOfSonarMeasurements[indexFirstPing].timeStamp << " "
//                      << this->ourListOfSonarMeasurements[indexSecondPing].timeStamp << std::endl;
            std::cout << "------------------------EKF----------------------" << std::endl;

            //calculate every T between 3  and 4 with Microns.
            std::vector<edge> listOfEdges;
            Eigen::Matrix4d transofrmationDiffMicron = Eigen::Matrix4d::Identity();
            for (i = indexFirstPing; i < indexSecondPing; i++) {
                edge tmpEdge = slamToolsRos::calculatePoseDiffByTimeDepOnEKF(
                        this->ourListOfSonarMeasurements[i].timeStamp,
                        this->ourListOfSonarMeasurements[i + 1].timeStamp,
                        this->ekfTransformationList, this->stateEstimationMutex);

//                std::cout << std::setprecision(15) << "micron Time: " << this->ourListOfSonarMeasurements[i].timeStamp
//                          << " "
//                          << this->ourListOfSonarMeasurements[i + 1].timeStamp << std::endl;
                std::cout << "----------------------------------------------" << std::endl;
                transofrmationDiffMicron = generalHelpfulTools::addTwoTransformationMatrixInBaseFrame(
                        transofrmationDiffMicron, tmpEdge.getTransformation());
                std::cout << tmpEdge.getTransformation() << std::endl;
                std::cout << transofrmationDiffMicron << std::endl;
                std::cout << transofrmationDiffEKF << std::endl;
                listOfEdges.push_back(tmpEdge);

            }





            //show difference

            std::cout << transofrmationDiffEKF << std::endl;
            std::cout << transofrmationDiffMicron << std::endl;
            Eigen::Vector3d positionEKF, positionMicron;
            Eigen::Quaterniond rotationEKF, rotationMicron;
            generalHelpfulTools::splitTransformationMatrixToQuadAndTrans(positionEKF, rotationEKF,
                                                                         transofrmationDiffEKF);
            generalHelpfulTools::splitTransformationMatrixToQuadAndTrans(positionMicron, rotationMicron,
                                                                         transofrmationDiffMicron);

            std::cout << generalHelpfulTools::getRollPitchYaw(rotationEKF)[2] << std::endl;
            std::cout << generalHelpfulTools::getRollPitchYaw(rotationMicron)[2] << std::endl;
            std::cout << "diff factor Pos: " << positionEKF[0] / positionMicron[0] << std::endl;
            std::cout << "diff factor Rot: " << generalHelpfulTools::getRollPitchYaw(rotationEKF)[2] /
                                                generalHelpfulTools::getRollPitchYaw(rotationMicron)[2] << std::endl;
            std::cout << "transofrmationDiffEKF" << std::endl;


        }
    }

    void startSonarFunction() {
//        std::cout << "beforeSonarMutex" << std::endl;

        this->sonarInputMutex.lock();
//        std::cout << "afterSonarMutex" << std::endl;
//        std::cout << this->ourListOfSonarMeasurements.size() << std::endl;
        if (this->ourListOfSonarMeasurements.empty()) {
            this->sonarInputMutex.unlock();
//            std::cout << "empty" << std::endl;
            return;
        }
        if (this->ourListOfSonarMeasurements.size() < 5) {
            this->sonarInputMutex.unlock();
//            std::cout << "less than 5" << std::endl;
            return;
        }
        double timeToBeat = this->ourListOfSonarMeasurements[0].timeStamp;
        int indexToBeat = 0;
        for (int i = 0; i < this->ourListOfSonarMeasurements.size(); i++) {
            if (this->ourListOfSonarMeasurements[i].timeStamp < timeToBeat) {
                indexToBeat = i;
                timeToBeat = this->ourListOfSonarMeasurements[i].timeStamp;
            }


        }
        ping360_sonar_msgs::msg::SonarEcho::SharedPtr pingMSG;
        micron_driver_ros::msg::ScanLine::SharedPtr micronMSG;
        int messageTypeUsed = this->ourListOfSonarMeasurements[indexToBeat].typeMeasurement;


        if (messageTypeUsed == 1) {
            //ping360 msg
            pingMSG = std::make_shared<ping360_sonar_msgs::msg::SonarEcho>(
                    this->ourListOfSonarMeasurements[indexToBeat].pingMSG);
        } else {
            //micron msg
            micronMSG = std::make_shared<micron_driver_ros::msg::ScanLine>(
                    this->ourListOfSonarMeasurements[indexToBeat].micronMSG);
        }


        this->ourListOfSonarMeasurements.erase(this->ourListOfSonarMeasurements.begin() + indexToBeat);
        this->sonarInputMutex.unlock();
        if (messageTypeUsed == 1) {
            //ping360 msg
//            std::cout << "starting ping stuff" << std::endl;
            ping360SonarCallback(pingMSG);
        } else {
            //micron msg
            micronSonarCallback(micronMSG);
        }
//        std::cout << "doneWithPingStuff" << std::endl;

        return;
    }


    void ping360SonarCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
//        std::cout << "ping360 before mutex" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
//        std::cout << "ping360 callback" << std::endl;
        intensityMeasurement intensityTMP;
        if (SONAR_LOOKING_DOWN) {
            intensityTMP.angle = std::fmod(-msg->angle / 400.0 * M_PI * 2.0 + ROTATION_SONAR,
                                           M_PI * 2);// TEST TRYING OUT -
        } else {
            intensityTMP.angle = std::fmod(msg->angle / 400.0 * M_PI * 2.0 + ROTATION_SONAR,
                                           M_PI * 2);// TEST TRYING OUT -
        }

        intensityTMP.time = rclcpp::Time(msg->header.stamp).seconds();
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
        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement

        bool waitingForMessages = waitForEKFMessagesToArrive(rclcpp::Time(msg->header.stamp).seconds());
        if (!waitingForMessages) {
            std::cout << "return no message found: " << rclcpp::Time(msg->header.stamp).seconds() << "    "
                      << rclcpp::Clock(RCL_ROS_TIME).now().seconds() << std::endl;
            return;
        }
        edge differenceOfEdge = slamToolsRos::calculatePoseDiffByTimeDepOnEKF(
                this->graphSaved.getVertexList()->back().getTimeStamp(), rclcpp::Time(msg->header.stamp).seconds(),
                this->ekfTransformationList, this->stateEstimationMutex);
        slamToolsRos::clearSavingsOfPoses(this->graphSaved.getVertexList()->back().getTimeStamp() - 2.0,
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
//            //SAVE TUHH GT POSE
//            this->graphSaved.getVertexList()->back().setGroundTruthTransformation(getCurrentGTPosition());
//        }
        //sort in GT
        if (USES_GROUND_TRUTH) {
            this->saveCurrentGTPosition();
        }

        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        covarianceMatrix(0, 0) = INTEGRATED_NOISE_XY;
        covarianceMatrix(1, 1) = INTEGRATED_NOISE_XY;
        covarianceMatrix(2, 2) = INTEGRATED_NOISE_YAW;
        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 covarianceMatrix, INTEGRATED_POSE);


        int indexOfLastKeyframe;
        double angleDiff = slamToolsRos::angleBetweenLastKeyframeAndNow(this->graphSaved);// i think this is always true
//        std::cout << angleDiff << std::endl;
        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
        if (abs(angleDiff) > 2 * M_PI / FACTOR_OF_MATCHING) {
            std::cout << this->ourListOfSonarMeasurements.size() << std::endl;

            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);
            if (firstCompleteSonarScan) {
                numberOfTimesFirstScan++;
                if (numberOfTimesFirstScan > 2 * FACTOR_OF_MATCHING - 1) {
                    firstCompleteSonarScan = false;
                }
                return;
            }



//            angleDiff = angleBetweenLastKeyframeAndNow(false);
//            indexOfLastKeyframe = slamToolsRos::getLastIntensityKeyframe(this->graphSaved);
            int indexStart1, indexEnd1, indexStart2, indexEnd2;
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved.getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
            indexStart2 = indexEnd1;
            slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart2, indexEnd2, this->graphSaved);


            std::cout << "scanAcusitionTime: " << this->graphSaved.getVertexList()->at(indexStart2).getTimeStamp() -
                                                  this->graphSaved.getVertexList()->at(indexEnd2).getTimeStamp()
                      << std::endl;
            if (this->graphSaved.getVertexList()->at(indexStart2).getTimeStamp() -
                this->graphSaved.getVertexList()->at(indexEnd2).getTimeStamp() < 20.0) {
                std::cout << "scanAcusitionTime2: "
                          << this->graphSaved.getVertexList()->at(indexStart2).getTimeStamp() -
                             this->graphSaved.getVertexList()->at(indexEnd2).getTimeStamp()
                          << std::endl;
                std::cout << "scanAcusitionTime1: "
                          << this->graphSaved.getVertexList()->at(indexStart1).getTimeStamp() -
                             this->graphSaved.getVertexList()->at(indexEnd1).getTimeStamp()
                          << std::endl;
            }
            //we inverse the initial guess, because the registration creates a T from scan 1 to scan 2.
            // But the graph creates a transformation from 1 -> 2 by the robot, therefore inverse.
//            this->initialGuessTransformation =
//                            this->graphSaved.getVertexList()->at(indexStart1).getTransformation().inverse()*
//                            this->graphSaved.getVertexList()->at(indexStart2).getTransformation();
            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(indexStart2).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->at(indexStart1).getTransformation();

//            std::cout << "this->initialGuessTransformation.inverse()" << std::endl;
//            std::cout << this->initialGuessTransformation.inverse() << std::endl;

            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                                  this->initialGuessTransformation(0, 0));

            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);

            double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1, indexEnd1,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());//get voxel


            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2, indexStart2, indexEnd2,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());//get voxel

            Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
            std::cout << "direct matching consecutive: " << std::endl;
            // result is matrix to transform scan 1 to scan 2 therefore later inversed + initial guess inversed
            double timeToCalculate;
            if (USES_REGISTRATIONS) {
                this->currentEstimatedTransformation = this->scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(
                        voxelData1, maximumVoxel1, voxelData2, maximumVoxel2,
                        this->initialGuessTransformation,
                        covarianceEstimation, (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                              (double) NUMBER_OF_POINTS_DIMENSION, timeToCalculate);
            }


            slamToolsRos::saveResultingRegistrationTMPCOPY(indexStart1, indexEnd1, indexStart2, indexEnd2,
                                                           this->graphSaved, NUMBER_OF_POINTS_DIMENSION,
                                                           IGNORE_DISTANCE_TO_ROBOT,
                                                           DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                           DEBUG_REGISTRATION, this->currentEstimatedTransformation,
                                                           initialGuessTransformation);

//            slamToolsRos::saveResultingRegistration(indexStart1, indexStart2,
//                                                    this->graphSaved, NUMBER_OF_POINTS_DIMENSION,
//                                                    IGNORE_DISTANCE_TO_ROBOT, DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
//                                                    DEBUG_REGISTRATION, this->currentTransformation);

            std::cout << this->initialGuessTransformation << std::endl;
            std::cout << this->currentEstimatedTransformation << std::endl;

            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)),
                    initialGuessAngle);



            //only if angle diff is smaller than 40 degreece its ok
            if (abs(differenceAngleBeforeAfter) < 40.0 / 180.0 * M_PI && USES_REGISTRATIONS) {
                //inverse the transformation because we want the robot transformation, not the scan transformation
                Eigen::Matrix4d transformationEstimationRobot1_2 = this->currentEstimatedTransformation;
                Eigen::Quaterniond qTMP(transformationEstimationRobot1_2.block<3, 3>(0, 0));

                graphSaved.addEdge(indexStart2,
                                   indexStart1,
                                   transformationEstimationRobot1_2.block<3, 1>(0, 3), qTMP,
                                   covarianceEstimation,
                                   LOOP_CLOSURE);//@TODO still not sure about size

            } else {
                std::cout << "we just skipped that registration" << std::endl;
            }
            std::cout << "loopClosure: " << std::endl;

            ////////////// look for loop closure  //////////////
            if (USES_REGISTRATIONS) {


                slamToolsRos::loopDetectionByClosestPath(this->graphSaved, this->scanRegistrationObject,
                                                         NUMBER_OF_POINTS_DIMENSION, IGNORE_DISTANCE_TO_ROBOT,
                                                         DIMENSION_OF_VOXEL_DATA_FOR_MATCHING, DEBUG_REGISTRATION,
                                                         USE_INITIAL_TRANSLATION_LOOP_CLOSURE, 2500, 2500,
                                                         THRESHOLD_FOR_TRANSLATION_MATCHING,
                                                         MAXIMUM_LOOP_CLOSURE_DISTANCE);
                this->graphSaved.isam2OptimizeGraph(true, 2);

            }
//            this->graphSaved.isam2OptimizeGraph(true, 2);

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved.getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
//            double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//            std::cout << "timeToCalculate: " << timeToCalculate << std::endl;


            slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherSonarEcho,
                                                    this->publisherMarkerArray, this->sigmaScaling,
                                                    this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures,
                                                    this->publisherEKF);
            //            this->graphSaved.classicalOptimizeGraph(true);
            std::cout << "next: " << std::endl;

//            if (SHOULD_USE_ROSBAG) {
//                std_srvs::SetBool srv;
//                srv.request.data = false;
//                pauseRosbag.call(srv);
//            }

        }
//        this->graphSaved.isam2OptimizeGraph(true,1);
        slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherSonarEcho,
                                                this->publisherMarkerArray, this->sigmaScaling,
                                                this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures,
                                                this->publisherEKF);
    }

    void micronSonarCallback(const micron_driver_ros::msg::ScanLine::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        if (firstSonarInput) {
//            std::cout << "micron callback skipped" << std::endl;
            return;
        }
//        std::cout << "micron callback" << std::endl;


        intensityMeasurement intensityTMP;
        intensityTMP.angle = std::fmod(msg->angle / 360.0 * M_PI * 2.0 + M_PI * 2, M_PI * 2);// TEST TRYING OUT -

        intensityTMP.time = rclcpp::Time(msg->header.stamp).seconds();
        intensityTMP.range = msg->bins.back().distance;//msg->bin_distance_step * msg->bins.size();
        intensityTMP.increment = msg->bin_distance_step;

        std::vector<double> intensitiesVector;
        for (int i = 0; i < msg->bins.size(); i++) {
            intensitiesVector.push_back((double) (msg->bins[i].intensity));
        }
        intensityTMP.intensities = intensitiesVector;


        bool waitingForMessages = waitForEKFMessagesToArrive(rclcpp::Time(msg->header.stamp).seconds());
        if (!waitingForMessages) {
            std::cout << "return no message found: " << rclcpp::Time(msg->header.stamp).seconds() << "    "
                      << rclcpp::Clock(RCL_ROS_TIME).now().seconds() << std::endl;
            return;
        }
        edge differenceOfEdge = slamToolsRos::calculatePoseDiffByTimeDepOnEKF(
                this->graphSaved.getVertexList()->back().getTimeStamp(), rclcpp::Time(msg->header.stamp).seconds(),
                this->ekfTransformationList, this->stateEstimationMutex);
//        slamToolsRos::clearSavingsOfPoses(this->graphSaved.getVertexList()->back().getTimeStamp() - 2.0,
//                                          this->ekfTransformationList, this->currentPositionGTDeque,
//                                          this->stateEstimationMutex);

        Eigen::Matrix4d tmpTransformation = this->graphSaved.getVertexList()->back().getTransformation();
        tmpTransformation = tmpTransformation * differenceOfEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);


        this->graphSaved.addVertex(this->graphSaved.getVertexList()->back().getKey() + 1, pos, rot,
                                   this->graphSaved.getVertexList()->back().getCovarianceMatrix(),
                                   intensityTMP,
                                   rclcpp::Time(msg->header.stamp).seconds(),
                                   MICRON_MEASUREMENT);

        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        covarianceMatrix(0, 0) = INTEGRATED_NOISE_XY;
        covarianceMatrix(1, 1) = INTEGRATED_NOISE_XY;
        covarianceMatrix(2, 2) = INTEGRATED_NOISE_YAW;
        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 covarianceMatrix, INTEGRATED_POSE);
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
            Eigen::Vector3d tmpVec;
            if (TEST_MM_INSTEAD_OF_METER) {
                tmpVec = Eigen::Vector3d(msg->pose.pose.position.x * 100000.0, msg->pose.pose.position.y * 100000.0,
                                         msg->pose.pose.position.z * 100000.0);
            } else {
                tmpVec = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                         msg->pose.pose.position.z);
            }
//            Eigen::Vector3d tmpVec(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
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
        this->createMapAndSaveToFile();
        //create image without motion compensation
        //create image with motion compensation(saved images)
        //create image with SLAM compensation
        std::cout << "test for saving1" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        std::cout << "test for saving2" << std::endl;

        std::ofstream myFile1, myFile2;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/positionEstimationOverTime" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        myFile2.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/groundTruthOverTime" +
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

        }


        myFile1.close();
        myFile2.close();

        res->saved = true;
        return true;
    }

    bool waitForEKFMessagesToArrive(double timeUntilWait) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        while (this->ekfTransformationList.empty() && timeToCalculate < 2000) {
            rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        }

        while (timeUntilWait > ekfTransformationList.back().timeStamp) {

            rclcpp::sleep_for(std::chrono::nanoseconds(2000000));
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

            double timeToWait = 4000;

            if (timeToCalculate > timeToWait) {
                std::cout << "we break" << std::endl;
                break;
            }
        }
        rclcpp::sleep_for(std::chrono::nanoseconds(2000000));

        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

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

        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex, false);
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
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/currentMap" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
            for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
                myFile1 << mapData[j + NUMBER_OF_POINTS_MAP * i] << std::endl;//number of possible rotations
            }
        }

        myFile1.close();


        // here save the graph in a file



        std::vector<intensityValues> fullDataSetWithMicron;
        double maximumIntensityFullSet = slamToolsRos::getDatasetFromGraphForMap(fullDataSetWithMicron,
                                                                                 this->graphSaved,
                                                                                 this->graphSlamMutex, true);


//        compute3DEnvironment(fullDataSetWithMicron);


        Json::Value keyFrames;
        int currentKeyFrameNumber = 0;
        for (int j = 0; j < fullDataSetWithMicron.size(); j++) {
//            if(fullDataSetWithMicron[j].type == MICRON_MEASUREMENT){
//                myFile2 <<
//            }
//            if(fullDataSetWithMicron[j].type == PING360_MEASUREMENT){
//
//
//            }
            // if point cloud is used then save this keyframe

            Eigen::Vector3d positionIntensity;
            Eigen::Vector3d rpy;
            Eigen::Quaterniond rotationTMP;

            generalHelpfulTools::splitTransformationMatrixToQuadAndTrans(positionIntensity, rotationTMP,
                                                                         fullDataSetWithMicron[j].transformation);
            rpy = generalHelpfulTools::getRollPitchYaw(rotationTMP);


            Json::Value positionOfKeyframe;
            positionOfKeyframe["x"] = positionIntensity.x();
            positionOfKeyframe["y"] = positionIntensity.y();
            positionOfKeyframe["z"] = positionIntensity.z();
            positionOfKeyframe["roll"] = 0;
            positionOfKeyframe["pitch"] = 0;
            positionOfKeyframe["yaw"] = rpy.z();
            Json::Value intensities;
            for (int i = 0; i < fullDataSetWithMicron[j].intensity.intensities.size(); i++) {
                intensities["intensity"][i] = fullDataSetWithMicron[j].intensity.intensities[i];
            }
            intensities["range"] = fullDataSetWithMicron[j].intensity.range;
            intensities["angle"] = fullDataSetWithMicron[j].intensity.angle;
            intensities["increment"] = fullDataSetWithMicron[j].intensity.increment;
            intensities["time"] = fullDataSetWithMicron[j].intensity.time;
            intensities["type"] = fullDataSetWithMicron[j].type;

            keyFrames[j]["position"] = positionOfKeyframe;
            keyFrames[j]["intensityValues"] = intensities;

        }








        // create the main object
        Json::Value outputText;
        outputText["keyFrames"] = keyFrames;

        std::ofstream ifs;
        ifs.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/fullGraphDataset.csv");
        ifs << outputText << '\n';
        ifs.close();


    }

    void compute3DEnvironment(std::vector<intensityValues> fullDataSetWithMicron) {
        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(
                sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        double *mapData;
        mapData = (double *) malloc(
                sizeof(double) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        //set zero voxel and index
        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }

        for (int currentPosition = 0;
             currentPosition < fullDataSetWithMicron.size(); currentPosition++) {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.
            //was 90 yaw and 180 roll

            if (fullDataSetWithMicron[currentPosition].type == MICRON_MEASUREMENT) {



                //this is Micron Sonar
                Eigen::Matrix4d transformationOfSonarPosition = fullDataSetWithMicron[currentPosition].transformation;
                //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
                Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(
                        fullDataSetWithMicron[currentPosition].intensity.angle + M_PI, 0,
                        0);

                int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT /
                                            (fullDataSetWithMicron[currentPosition].intensity.range /
                                             ((double) fullDataSetWithMicron[currentPosition].intensity.intensities.size())));


                for (int j = ignoreDistance;
                     j < fullDataSetWithMicron[currentPosition].intensity.intensities.size(); j++) {

                    double distanceOfIntensity =
                            j / ((double) fullDataSetWithMicron[currentPosition].intensity.intensities.size()) *
                            ((double) fullDataSetWithMicron[currentPosition].intensity.range);

//                    int incrementOfScan = fullDataSetWithMicron[currentPosition].intensity.increment;
                    for (int l = -5; l <= +5; l++) {
                        for (int openingAngle = -35; l <= +35; l++) {
                            Eigen::Vector4d positionOfIntensity(
                                    0,
                                    0,
                                    distanceOfIntensity,
                                    1);
                            double incrementAngle = l / 400.0 * 2 * M_PI;
                            double openingAngleFor3D = openingAngle / 180 * M_PI;
                            Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(
                                    incrementAngle,
                                    0,
                                    0);
                            Eigen::Matrix4d openingAngleRotation = generalHelpfulTools::getTransformationMatrixFromRPY(
                                    0,
                                    openingAngleFor3D,
                                    0);
                            positionOfIntensity =
                                    rotationOfSonarAngleMatrix * openingAngleRotation * rotationForBetterView *
                                    positionOfIntensity;

//                            positionOfIntensity =
//                                    transformationOfIntensityRay * rotationOfSonarAngleMatrix * positionOfIntensity;
                            //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                            int indexX =
                                    (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                           2) +
                                    NUMBER_OF_POINTS_MAP / 2;
                            int indexY =
                                    (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                           2) +
                                    NUMBER_OF_POINTS_MAP / 2;
                            int indexZ =
                                    (int) (positionOfIntensity.z() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                           2) +
                                    NUMBER_OF_POINTS_MAP / 2;


                            if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP &&
                                indexZ < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
                                indexX >= 0 && indexZ >= 0) {
                                //                    std::cout << indexX << " " << indexY << std::endl;
                                //if index fits inside of our data, add that data. Else Ignore
                                voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY + NUMBER_OF_POINTS_MAP*NUMBER_OF_POINTS_MAP*indexZ] =
                                        voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY+ NUMBER_OF_POINTS_MAP*NUMBER_OF_POINTS_MAP*indexZ] + 1;
                                //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                                mapData[indexX + NUMBER_OF_POINTS_MAP * indexY+ NUMBER_OF_POINTS_MAP*NUMBER_OF_POINTS_MAP*indexZ] =
                                        mapData[indexX + NUMBER_OF_POINTS_MAP * indexY+ NUMBER_OF_POINTS_MAP*NUMBER_OF_POINTS_MAP*indexZ] +
                                                fullDataSetWithMicron[currentPosition].intensity.intensities[j];
                                //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                                //                    std::cout << "random: " << std::endl;
                            }
                        }
                    }
                }
            }
        }



        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            if(voxelDataIndex[i]>0){
                mapData[i] = mapData[i]/voxelDataIndex[i];
            }
        }

        std::ofstream myFile1;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/3D/csvFiles/current3DMap" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        for (int k = 0; k < NUMBER_OF_POINTS_MAP; k++) {
            for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
                for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {

                    myFile1 << mapData[k + NUMBER_OF_POINTS_MAP * j +
                                       NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * i]
                            << std::endl;//number of possible rotations
                }
            }
        }


        myFile1.close();




    }

public:

    void createImageOfAllScans() {
//        std::cout << "starting map Creation" << std::endl;
        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex, false);
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
//        std::cout << "publishing occupancy map" << std::endl;

        this->publisherOccupancyMap->publish(occupanyMap);
//        std::cout << "published occupancy map" << std::endl;
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

//    Eigen::Matrix4d firstTransformation = Eigen::Matrix4d::Identity();
//    Eigen::Matrix4d secondTransformation = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 1);
//    Eigen::Matrix4d thirdTransformation = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 2);
//
//    secondTransformation(0, 3) = 2.5;
//    secondTransformation(1, 3) = 1.5;
//
//    thirdTransformation(0, 3) = 3.5;
//    thirdTransformation(1, 3) = 2.5;
//
//    double t = 0.00199600798;
//
//    Eigen::Matrix4d completeTransformation1 = Eigen::Matrix4d::Identity();
//    Eigen::Matrix4d completeTransformation2 = Eigen::Matrix4d::Identity();
//    Eigen::Matrix4d completeTransformation3 = Eigen::Matrix4d::Identity();
//    Eigen::Matrix4d completeTransformation4 = Eigen::Matrix4d::Identity();
//    for (int i = 0; i < 1002; i++) {
//        double currentTimeOfInterestStart = t*i;
//        double currentTimeOfInterestEnd = t*(i+1);
//
//        if(currentTimeOfInterestStart<1 && currentTimeOfInterestEnd<1){
//
//            double interpolationFactor1 = currentTimeOfInterestStart;
//            double interpolationFactor2 = currentTimeOfInterestEnd;
//
//
//            Eigen::Matrix4d tmp1 = generalHelpfulTools::interpolationTwo4DTransformations(
//                    firstTransformation, secondTransformation, interpolationFactor1);
//            Eigen::Matrix4d tmp2 = generalHelpfulTools::interpolationTwo4DTransformations(
//                    firstTransformation, secondTransformation, interpolationFactor2);
//
//
//            completeTransformation1 = completeTransformation1*(tmp1.inverse()*tmp2);
//
//        }
//
//        if(currentTimeOfInterestStart<1 && currentTimeOfInterestEnd>1){
//
//
//            double interpolationFactor1 = currentTimeOfInterestStart;
//            double interpolationFactor2 = currentTimeOfInterestEnd-1;
//
//
//            Eigen::Matrix4d tmp1 = generalHelpfulTools::interpolationTwo4DTransformations(
//                    firstTransformation, secondTransformation, interpolationFactor1);
//
//            Eigen::Matrix4d tmp2 = generalHelpfulTools::interpolationTwo4DTransformations(
//                    secondTransformation, thirdTransformation, interpolationFactor2);
//
//            completeTransformation1 = completeTransformation1*((tmp1.inverse()*secondTransformation)*(secondTransformation.inverse()*tmp2));
//
//
//        }
//        if(currentTimeOfInterestStart>1 && currentTimeOfInterestEnd>1){
//
//
//            double interpolationFactor1 = currentTimeOfInterestStart-1;
//            double interpolationFactor2 = currentTimeOfInterestEnd-1;
//
//
//            Eigen::Matrix4d tmp1 = generalHelpfulTools::interpolationTwo4DTransformations(
//                    secondTransformation, thirdTransformation, interpolationFactor1);
//            Eigen::Matrix4d tmp2 = generalHelpfulTools::interpolationTwo4DTransformations(
//                    secondTransformation, thirdTransformation, interpolationFactor2);
//
//
//            completeTransformation1 = completeTransformation1*(tmp1.inverse()*tmp2);
//
//        }
//
//
//
//
//
//
//
//
//    }
//    std::cout << completeTransformation1 << std::endl;
//
//    std::cout << thirdTransformation << std::endl;



    return (0);
}
