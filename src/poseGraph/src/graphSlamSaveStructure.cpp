//
// Created by tim on 23.02.21.
//

#include "graphSlamSaveStructure.h"
#include "random"

void
graphSlamSaveStructure::addEdge(int fromKey, int toKey, Eigen::Vector3d positionDifference,
                                Eigen::Quaterniond rotationDifference, Eigen::Matrix3d covarianceMatrix,
                                int typeOfEdge) {
    edge edgeToAdd(fromKey, toKey, positionDifference, rotationDifference, covarianceMatrix, this->degreeOfFreedom,
                   typeOfEdge, this->graph.size());

    switch (this->degreeOfFreedom) {
        case 3: {
            // std::cout << "this shouldnt happen atm." << std::endl;
            auto model3D = gtsam::noiseModel::Gaussian::Covariance(covarianceMatrix);
            // add a factor between the keys
            if (abs(toKey - fromKey) > 2) {
                this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(
                        edgeToAdd.getPositionDifference().x(), edgeToAdd.getPositionDifference().y(),
                        generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())[2]),
                    model3D); //@TODO different Noise Model
            } else {
                this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(
                    fromKey, toKey, gtsam::Pose2(edgeToAdd.getPositionDifference().x(),
                                                 edgeToAdd.getPositionDifference().y(),
                                                 generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())
                                                 [2]),
                    model3D);
            }
            this->edgeList.push_back(edgeToAdd);
            break;
        }
        case 6: {
            // gtsam::Matrix covariance6DMatrix(6, 6);
            // covariance6DMatrix(0, 0) = covarianceMatrix(0, 0);
            // covariance6DMatrix(1, 1) = covarianceMatrix(0, 0);
            // covariance6DMatrix(2, 2) = covarianceMatrix(0, 0);
            // covariance6DMatrix(3, 3) = covarianceMatrix(2, 2);
            // covariance6DMatrix(4, 4) = covarianceMatrix(2, 2);
            // covariance6DMatrix(5, 5) = covarianceMatrix(2, 2);
            gtsam::Vector tmpVector = gtsam::Vector(6);
            tmpVector << 0.03, 0.03, 0.03,0.01,0.01,0.01;
            // auto model6D = gtsam::noiseModel::Gaussian::Covariance(tmpVector);
            auto model6D = gtsam::noiseModel::Diagonal::Sigmas(tmpVector);

            // add a factor between the keys
            if (abs(toKey - fromKey) > 2) {
                this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(
                    fromKey, toKey, gtsam::Pose3(gtsam::Rot3(edgeToAdd.getRotationDifference()),
                                                 gtsam::Point3(edgeToAdd.getPositionDifference())),
                    model6D); //@TODO different Noise Model
            } else {
                this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(
                    fromKey, toKey, gtsam::Pose3(gtsam::Rot3(edgeToAdd.getRotationDifference()),
                                                 gtsam::Point3(edgeToAdd.getPositionDifference())),
                    model6D);
            }
            this->edgeList.push_back(edgeToAdd);
            break;
        }
        default:
            std::cout << "not yet implemented DOF XXXX" << std::endl;
            std::exit(-1);
    }

    // add edge to edge list
}

void graphSlamSaveStructure::addVertex(int key, const Eigen::Vector3d &positionVertex,
                                       const Eigen::Quaterniond &rotationVertex,
                                       const Eigen::Matrix3d &covarianceMatrix,
                                       intensityMeasurement intensityInput, double timeStamp,
                                       int typeOfVertex) {
    vertex vertexToAdd(key, positionVertex, rotationVertex, this->degreeOfFreedom,
                       intensityInput, covarianceMatrix, timeStamp, typeOfVertex);
    this->vertexList.push_back(vertexToAdd);
    //ADD BETTER INITIAL STATE
    double tmpYaw = generalHelpfulTools::getRollPitchYaw(rotationVertex)[2];
    this->currentEstimate.insert(key, gtsam::Pose2(positionVertex[0], positionVertex[1], tmpYaw));
}

void graphSlamSaveStructure::addVertexPCL(int key, const Eigen::Vector3d &positionVertex,
                                          const Eigen::Quaterniond &rotationVertex,
                                          const Eigen::Matrix3d &covarianceMatrix,
                                          pclMeasurement pclInput, double timeStamp,
                                          int typeOfVertex) {
    std::cout << "addVertexPCL: " << timeStamp<<std::endl;
    vertex vertexToAdd(key, positionVertex, rotationVertex, this->degreeOfFreedom,
                       pclInput, covarianceMatrix, timeStamp, typeOfVertex);
    this->vertexList.push_back(vertexToAdd);
    //ADD BETTER INITIAL STATE
    double tmpYaw = generalHelpfulTools::getRollPitchYaw(rotationVertex)[2];
    if (this->degreeOfFreedom == 3) {
        this->currentEstimate.insert(key, gtsam::Pose2(positionVertex[0], positionVertex[1], tmpYaw));
    } else {
        if (this->degreeOfFreedom == 6) {
            this->currentEstimate.insert(key, gtsam::Pose3(gtsam::Rot3(rotationVertex), gtsam::Point3(positionVertex)));
        }
    }
}


void graphSlamSaveStructure::isam2OptimizeGraph(bool verbose, int numberOfUpdates) {
    //    this->isam->printStats();
    //    this->graph.print();
    //    std::cout <<this->graph.size() << std::endl;
    //    std::cout <<this->isam->size() << std::endl;
    // std::cout << "isam2OptimizeGraph : " << this->degreeOfFreedom << std::endl;
    // this->currentEstimate.print();
    // std::cout << "graph : " << std::endl;
    // this->graph.print();
    // std::cout << "isam : " << std::endl;
    // this->isam->print();
    this->isam->update(this->graph, this->currentEstimate);
    // std::cout << "after Update : " << std::endl;

    for (int i = 0; i < numberOfUpdates; i++) {
        this->isam->update();
    }

    //    this->isam->printStats();
    //    std::cout << "######" << std::endl;
    //    this->currentEstimate.print();
    //    std::cout << "#" << std::endl;
    this->currentEstimate = this->isam->calculateEstimate();
    //    this->currentEstimate.print();
    //    std::cout << "######" << std::endl;


    //    this->isam->update();
    //    this->currentEstimate = this->isam->calculateEstimate();
    //    this->currentEstimate.print();
    //    std::cout << "######"<< std::endl;


    //    this->currentEstimate.print("Final Result:\n");
    //    gtsam::Marginals marginals(this->graph, this->currentEstimate);

    switch (this->degreeOfFreedom) {
        case 3:
            for (int i = 1; i < this->vertexList.size(); i++) {
                gtsam::Pose2 iterativePose = this->currentEstimate.at(this->vertexList[i].getKey()).cast<
                    gtsam::Pose2>();
                this->vertexList.at(i).setPositionVertex(Eigen::Vector3d(iterativePose.x(), iterativePose.y(), 0));
                this->vertexList.at(i).setRotationVertex(
                    generalHelpfulTools::getQuaternionFromRPY(0, 0, iterativePose.theta()));
                //        std::cout << "covariance:\n" << marginals.marginalCovariance(this->vertexList.at(i).getKey()) << std::endl;
                //        std::cout << "Pose :\n" << iterativePose << std::endl;
                this->vertexList.at(i).setCovarianceMatrix2D(
                    this->isam->marginalCovariance(this->vertexList.at(i).getKey()));
            }
            break;

        case 6:
            // std::cout << "isam2 case 6 started : " << std::endl;
            for (int i = 1; i < this->vertexList.size(); i++) {
                gtsam::Pose3 iterativePose = this->currentEstimate.at(this->vertexList[i].getKey()).cast<
                    gtsam::Pose3>();
                // std::cout << "got interactive Pose : " << std::endl;
                this->vertexList.at(i).setPositionVertex(
                    Eigen::Vector3d(iterativePose.x(), iterativePose.y(), iterativePose.z()));
                Eigen::Matrix<double, 3, 3> matrixTMP = iterativePose.rotation().matrix();
                this->vertexList.at(i).setRotationVertex(Eigen::Quaterniond(matrixTMP));
                // std::cout << "Got translation and Rotation : " << std::endl;
                // std::cout << "covariance:\n" << this->isam->marginalCovariance(this->vertexList.at(i).getKey()) << std::endl;
                // std::cout << "Pose :\n" << iterativePose << std::endl;
                Eigen::MatrixXd matrix6D = this->isam->marginalCovariance(this->vertexList.at(i).getKey());
                this->vertexList.at(i).setCovarianceMatrix3D(matrix6D);

                // std::cout << "Got covariance: " << std::endl;
            }
            break;
        default:
            std::cout << "not yet implemented DOF XXXX" << std::endl;
            std::exit(-1);
    }

    this->graph.resize(0);
    this->currentEstimate.clear();
}


void graphSlamSaveStructure::classicalOptimizeGraph(bool verbose) {
    //    gtsam::GaussNewtonParams parameters;
    //    parameters.relativeErrorTol = 1e-5;
    //    parameters.maxIterations = 100;
    //    parameters.setVerbosity("ERROR");
    //    gtsam::GaussNewtonOptimizer optimizer(this->graph, this->currentEstimate, parameters);

    gtsam::LevenbergMarquardtParams params;
    //    params.setVerbosity("ERROR");
    params.setOrderingType("METIS");
    params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");


    gtsam::LevenbergMarquardtOptimizer optimizer(this->graph, this->currentEstimate, params);

    // ... and optimize
    this->currentEstimate = optimizer.optimize();


    //    this->isam->printStats();
    //    std::cout << "######" << std::endl;
    //    this->currentEstimate.print();
    //    std::cout << "#" << std::endl;
    //    this->currentEstimate = this->isam->calculateEstimate();
    //    this->currentEstimate.print();
    //    std::cout << "######" << std::endl;


    //    this->isam->update();
    //    this->currentEstimate = this->isam->calculateEstimate();
    //    this->currentEstimate.print();
    //    std::cout << "######"<< std::endl;


    //    this->currentEstimate.print("Final Result:\n");
    gtsam::Marginals marginals(this->graph, this->currentEstimate);


    for (int i = 1; i < this->vertexList.size(); i++) {
        gtsam::Pose2 iterativePose = this->currentEstimate.at(this->vertexList[i].getKey()).cast<gtsam::Pose2>();
        this->vertexList.at(i).setPositionVertex(Eigen::Vector3d(iterativePose.x(), iterativePose.y(), 0));
        this->vertexList.at(i).
                setRotationVertex(generalHelpfulTools::getQuaternionFromRPY(0, 0, iterativePose.theta()));
        //        std::cout << "covariance:\n" << marginals.marginalCovariance(this->vertexList.at(i).getKey()) << std::endl;
        //        std::cout << "Pose :\n" << iterativePose << std::endl;
        this->vertexList.at(i).setCovarianceMatrix2D(this->isam->marginalCovariance(this->vertexList.at(i).getKey()));
    }

    //    this->graph.resize(0);
    //    this->currentEstimate.clear();
}

void graphSlamSaveStructure::printCurrentState() {
    std::cout << "current State:" << std::endl;
    for (int i = 0; i < this->vertexList.size(); i++) {
        std::cout << "State:" << i << std::endl;
        std::cout << "Pose:" << std::endl;
        this->currentEstimate.at(this->vertexList[i].getKey()).print();
    }
}

void graphSlamSaveStructure::printCurrentStateGeneralInformation() {
    std::cout << "Number of Vertex:" << this->vertexList.size() << std::endl;
    for (int i = 0; i < this->vertexList.size(); i++) {
        std::cout << "Vertex Nr. " << i << " Pos:\n"
                << this->vertexList[i].getPositionVertex() << std::endl;
    }
}

std::vector<vertex> *graphSlamSaveStructure::getVertexList() {
    return &this->vertexList;
}

std::vector<edge> *graphSlamSaveStructure::getEdgeList() {
    return &this->edgeList;
}


void graphSlamSaveStructure::saveGraphJson(std::string nameSavingFile) {
    //    Json::Value keyFrames;
    //
    //    int currentKeyFrameNumber = 0;
    //    for (int i = 0; i < this->vertexList.size(); i++) {
    //        if (this->vertexList[i].getTypeOfVertex() == POINT_CLOUD_SAVED) {
    //            // if point cloud is used then save this keyframe
    //            Json::Value positionOfKeyframe;
    //            positionOfKeyframe["x"] = this->vertexList[i].getPositionVertex()[0];
    //            positionOfKeyframe["y"] = this->vertexList[i].getPositionVertex()[1];
    //            positionOfKeyframe["z"] = this->vertexList[i].getPositionVertex()[2];
    //            positionOfKeyframe["roll"] = 0;
    //            positionOfKeyframe["pitch"] = 0;
    //            positionOfKeyframe["yaw"] = this->getYawAngle(this->vertexList[i].getRotationVertex());
    //            Json::Value pointCloud;
    //
    //            for (int j = 0; j < this->vertexList[i].getPointCloudCorrected()->points.size(); j++) {
    //                Json::Value tmpPoint;
    //                tmpPoint["x"] = this->vertexList[i].getPointCloudCorrected()->points[j].x;
    //                tmpPoint["y"] = this->vertexList[i].getPointCloudCorrected()->points[j].y;
    //                tmpPoint["z"] = this->vertexList[i].getPointCloudCorrected()->points[j].z;
    //                pointCloud[j]["point"] = tmpPoint;
    //            }
    //            keyFrames[currentKeyFrameNumber]["position"] = positionOfKeyframe;
    //            keyFrames[currentKeyFrameNumber]["pointCloud"] = pointCloud;
    //            currentKeyFrameNumber++;
    //        }
    //    }
    //
    //    // create the main object
    //    Json::Value outputText;
    //    outputText["keyFrames"] = keyFrames;
    //
    //    std::ofstream ifs;
    //    ifs.open(nameSavingFile);
    //    ifs << outputText << '\n';
    //    ifs.close();
}


void graphSlamSaveStructure::addRandomNoiseToGraph(double stdDiviationGauss, double percentageOfRandomNoise) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    std::normal_distribution<> disNormal(0.0, stdDiviationGauss);


    double maximumIntensity = 0;
    for (int i = 0; i < this->vertexList.size(); i++) {
        intensityMeasurement tmpIntensity = this->vertexList[i].getIntensities();
        //find max intensity;
        for (int j = 0; j < tmpIntensity.intensities.size(); j++) {
            if (tmpIntensity.intensities[j] > maximumIntensity) {
                maximumIntensity = tmpIntensity.intensities[j];
            }
        }
    }

    for (int i = 0; i < this->vertexList.size(); i++) {
        intensityMeasurement tmpIntensity = this->vertexList[i].getIntensities();
        //find max intensity;
        for (int j = 0; j < tmpIntensity.intensities.size(); j++) {
            if (dis(gen) < percentageOfRandomNoise) {
                tmpIntensity.intensities[j] = dis(gen) * maximumIntensity;
                //                std::cout << tmpIntensity.intensities[j] << std::endl;
            }


            tmpIntensity.intensities[j] = tmpIntensity.intensities[j] + disNormal(gen) * maximumIntensity;
            if (tmpIntensity.intensities[j] < 0) {
                tmpIntensity.intensities[j] = 0;
            }
        }
        this->vertexList[i].setIntensities(tmpIntensity);
    }


    //    this->vertexList[i].setIntensities(tmpIntensity);
}

void graphSlamSaveStructure::setPoseDifferenceEdge(int numberOfEdge, Eigen::Matrix4d poseDiff) {
    this->edgeList.at(numberOfEdge).setPositionDifference(poseDiff.block<3, 1>(0, 3));
    this->edgeList.at(numberOfEdge).setRotationDifference(Eigen::Quaterniond(poseDiff.block<3, 3>(0, 0)));
    int fromKey = this->edgeList.at(numberOfEdge).getFromKey();
    int toKey = this->edgeList.at(numberOfEdge).getToKey();
    int keyFactorInGraph = this->edgeList.at(numberOfEdge).getKeyOfEdgeInGraph();


    this->graph.remove(this->edgeList.at(numberOfEdge).getKeyOfEdgeInGraph());
    int tmpInt = this->graph.size();
    this->edgeList.at(numberOfEdge).setKeyOfEdgeInGraph(tmpInt);

    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(
                                                                        this->edgeList.at(numberOfEdge).
                                                                        getPositionDifference().x(),
                                                                        this->edgeList.at(numberOfEdge).
                                                                        getPositionDifference().y(),
                                                                        generalHelpfulTools::getRollPitchYaw(
                                                                            this->edgeList.at(numberOfEdge).
                                                                            getRotationDifference())[2]),
                                                                    this->loopClosureNoiseModel);
    //@TODO different Noise Model
}

void graphSlamSaveStructure::print() {
    this->graph.print();
}
