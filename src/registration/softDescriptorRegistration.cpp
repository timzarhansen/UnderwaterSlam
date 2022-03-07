//
// Created by tim-linux on 01.03.22.
//

#include "softDescriptorRegistration.h"

bool compareTwoAngleCorrelation(angleAndCorrelation i1, angleAndCorrelation i2) {
    return (i1.angle < i2.angle);
}

std::vector<double> linspace(double start_in, double end_in, int num_in) {
    if (num_in < 0) {
        std::cout << "number of linspace negative" << std::endl;
        exit(-1);
    }
    std::vector<double> linspaced;

    double start = start_in;
    double end = end_in;
    auto num = (double) num_in;

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);//stepSize

    for (int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

bool compareTwoPeaks(indexPeak i1, indexPeak i2) {
    return (i1.height > i2.height);
}

double thetaIncrement(double index, int bandwidth) {
    return M_PI * (2 * index + 1) / (4.0 * bandwidth);
}

double phiIncrement(double index, int bandwidth) {
    return M_PI * index / bandwidth;
}

double angleDifference(double angle1, double angle2) {//gives angle 1 - angle 2
    double correctedAngle1 = std::fmod(angle1 + 4 * M_PI, 2 * M_PI);
    double correctedAngle2 = std::fmod(angle2 + 4 * M_PI, 2 * M_PI);

    return correctedAngle1 - correctedAngle2;
}

double
softDescriptorRegistration::getSpectrumFromPCL3D(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData,
                                                 double voxelData[],
                                                 double magnitude[], double phase[], double fromTo, int N) {


    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                voxelData[k + N * (j + N * i)] = 0;
            }
        }
    }

    for (int i = 0; i < pointCloudInputData->points.size(); i++) {
        double positionPointX = pointCloudInputData->points[i].x;
        double positionPointY = pointCloudInputData->points[i].y;
        double positionPointZ = pointCloudInputData->points[i].z;
        int indexX = (int) std::round((positionPointX + fromTo) / (fromTo * 2) * N) - 1;
        int indexY = (int) std::round((positionPointY + fromTo) / (fromTo * 2) * N) - 1;
        int indexZ = (int) std::round((positionPointZ + fromTo) / (fromTo * 2) * N) - 1;//set to zero
        voxelData[indexZ + N * (indexX + N * indexY)] = 1;
    }



    //from voxel data to row major
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                inputSpacialData[k + N * (j + N * i)][0] = voxelData[k + N * (j + N * i)]; // real part
                inputSpacialData[k + N * (j + N * i)][1] = 0; // imaginary part
            }
        }
    }



//    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    fftw_execute(planVoxelToFourier3D);
//    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
//              << "[ms]" << std::endl;


    //calc magnitude and phase
    double maximumMagnitude = 0;

    //get magnitude and find maximum
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                magnitude[k + N * (j + N * i)] = sqrt(
                        spectrumOut[k + N * (j + N * i)][0] *
                        spectrumOut[k + N * (j + N * i)][0] +
                        spectrumOut[k + N * (j + N * i)][1] *
                        spectrumOut[k + N *
                                        (j + N * i)][1]); // real part;
                if (maximumMagnitude < magnitude[k + N * (j + N * i)]) {
                    maximumMagnitude = magnitude[k + N * (j + N * i)];
                }

                phase[k + N * (j + N * i)] = atan2(spectrumOut[k + N * (j + N * i)][1],
                                                   spectrumOut[k + N * (j + N * i)][0]);
            }
        }
    }

    //free(inputSpacialData);
    return maximumMagnitude;
}

double
softDescriptorRegistration::getSpectrumFromPCL2D(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData,
                                                 double voxelData[],
                                                 double magnitude[], double phase[], double fromTo, int N) {


    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            voxelData[j + N * i] = 0.0;
        }
    }
    for (int i = 0; i < pointCloudInputData->points.size(); i++) {
        int numberOfPoints = 40;
        std::vector<double> vectorForSettingZeroX= linspace(0,pointCloudInputData->points[i].x,numberOfPoints);
        std::vector<double> vectorForSettingZeroY= linspace(0,pointCloudInputData->points[i].y,numberOfPoints);


        for(int j = 0 ; j < numberOfPoints - 1; j++){
            double positionPointX = vectorForSettingZeroX[j];
            double positionPointY = vectorForSettingZeroY[j];
            double positionPointZ = 0;
            int indexX = (int) std::round((positionPointX + fromTo) / (fromTo * 2) * N) - 1;
            int indexY = (int) std::round((positionPointY + fromTo) / (fromTo * 2) * N) - 1;
            int indexZ = (int) std::round((positionPointZ + fromTo) / (fromTo * 2) * N) - 1;//set to zero
            voxelData[indexX + N * indexY] = 0.5;

        }

    }
    for (int i = 0; i < pointCloudInputData->points.size(); i++) {

            double positionPointX = pointCloudInputData->points[i].x;
            double positionPointY = pointCloudInputData->points[i].y;
            double positionPointZ = 0;
            int indexX = (int) std::round((positionPointX + fromTo) / (fromTo * 2) * N) - 1;
            int indexY = (int) std::round((positionPointY + fromTo) / (fromTo * 2) * N) - 1;
            int indexZ = (int) std::round((positionPointZ + fromTo) / (fromTo * 2) * N) - 1;//set to zero
            voxelData[indexX + N * indexY] = 1.0;


    }
//    for (int i = 0; i < pointCloudInputData->points.size(); i++) {
//        double positionPointX = pointCloudInputData->points[i].x;
//        double positionPointY = pointCloudInputData->points[i].y;
//        double positionPointZ = pointCloudInputData->points[i].z;
//        int indexX = (int) std::round((positionPointX + fromTo) / (fromTo * 2) * N) - 1;
//        int indexY = (int) std::round((positionPointY + fromTo) / (fromTo * 2) * N) - 1;
//        int indexZ = (int) std::round((positionPointZ + fromTo) / (fromTo * 2) * N) - 1;//set to zero
//        voxelData[indexX + N * indexY] = 1;
//    }

//    fftw_complex *inputSpacialData;
//    inputSpacialData = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N);

    //from voxel data to row major
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            inputSpacialData[j + N * i][0] = voxelData[j + N * i]; // real part
            inputSpacialData[j + N * i][1] = 0; // imaginary part
        }
    }



//    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    fftw_execute(planVoxelToFourier2D);
//    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
//              << "[ms]" << std::endl;


    //calc magnitude and phase
    double maximumMagnitude = 0;

    //get magnitude and find maximum
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            magnitude[j + N * i] = sqrt(
                    spectrumOut[j + N * i][0] *
                    spectrumOut[j + N * i][0] +
                    spectrumOut[j + N * i][1] *
                    spectrumOut[j + N * i][1]); // real part;
            if (maximumMagnitude < magnitude[j + N * i]) {
                maximumMagnitude = magnitude[j + N * i];
            }

            phase[j + N * i] = atan2(spectrumOut[j + N * i][1], spectrumOut[j + N * i][0]);

        }
    }

//    free(inputSpacialData);
    return maximumMagnitude;
}


Eigen::Matrix4d
softDescriptorRegistration::registrationOfTwoPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2,
                                                 const double cellSize, bool debug, double goodGuessAlpha) {
    std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();
    double maximumScan1 = this->getSpectrumFromPCL2D(pointCloudInputData1, this->voxelData1, this->magnitude1,
                                                     this->phase1, cellSize * this->N,
                                                     this->N);
    double maximumScan2 = this->getSpectrumFromPCL2D(pointCloudInputData2, this->voxelData2, this->magnitude2,
                                                     this->phase2, cellSize * this->N,
                                                     this->N);

    if (debug){
        std::ofstream myFile1, myFile2, myFile3, myFile4, myFile5, myFile6;
        myFile1.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/magnitudeFFTW1.csv");
        myFile2.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/phaseFFTW1.csv");
        myFile3.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW1.csv");
        myFile4.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/magnitudeFFTW2.csv");
        myFile5.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/phaseFFTW2.csv");
        myFile6.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW2.csv");
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                    myFile1 << magnitude1[j + N * i]; // real part
                    myFile1 << "\n";
                    myFile2 << phase1[j + N * i]; // imaginary part
                    myFile2 << "\n";
                    myFile3 << voxelData1[j + N * i]; // imaginary part
                    myFile3 << "\n";
                    myFile4 << magnitude2[j + N * i]; // real part
                    myFile4 << "\n";
                    myFile5 << phase2[j + N * i]; // imaginary part
                    myFile5 << "\n";
                    myFile6 << voxelData2[j + N * i]; // imaginary part
                    myFile6 << "\n";
            }
        }

        myFile1.close();
        myFile2.close();
        myFile3.close();
        myFile4.close();
        myFile5.close();
        myFile6.close();
    }
    double globalMaximumMagnitude;
    if (maximumScan2 < maximumScan1) {
        globalMaximumMagnitude = maximumScan1;
    } else {
        globalMaximumMagnitude = maximumScan2;
    }

    //normalize and fftshift
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                int indexX = (N / 2 + i) % N;
                int indexY = (N / 2 + j) % N;
//                int indexZ = (N / 2 + k) % N;

                magnitude1Shifted[indexY + N * indexX] =
                        magnitude1[j + N * i] / globalMaximumMagnitude;
                magnitude2Shifted[indexY + N * indexX] =
                        magnitude2[j + N * i] / globalMaximumMagnitude;
            }
        }
    }


    //re-initialize to zero
    for (int i = 0; i < N * N; i++) {
        resampledMagnitudeSO3_1[i] = 0;
        resampledMagnitudeSO3_2[i] = 0;
        resampledMagnitudeSO3_1TMP[i] = 0;
        resampledMagnitudeSO3_2TMP[i] = 0;
    }

    int minRNumber = 4;
    int maxRNumber = N / 2 - 2;
    int bandwidth = N / 2;

    for (int r = minRNumber; r < maxRNumber; r++) {
        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
                int xIndex = std::round((double) r * std::sin(thetaIncrement((double) j + 1, bandwidth)) *
                                        std::cos(phiIncrement((double) k + 1, bandwidth)) + bandwidth) - 1;
                int yIndex = std::round((double) r * std::sin(thetaIncrement((double) j + 1, bandwidth)) *
                                        std::sin(phiIncrement((double) k + 1, bandwidth)) + bandwidth) - 1;
//                int zIndex =
//                        std::round((double) r * std::cos(thetaIncrement((double) j + 1, bandwidth)) + bandwidth) - 1;
                resampledMagnitudeSO3_1TMP[k + j * bandwidth * 2] =
                        255 * magnitude1Shifted[yIndex + N * xIndex];
                resampledMagnitudeSO3_2TMP[k + j * bandwidth * 2] =
                        255 * magnitude2Shifted[yIndex + N * xIndex];
            }
        }
        cv::Mat magTMP1(N, N, CV_64FC1, resampledMagnitudeSO3_1TMP);
        cv::Mat magTMP2(N, N, CV_64FC1, resampledMagnitudeSO3_2TMP);
        magTMP1.convertTo(magTMP1, CV_8UC1);
        magTMP2.convertTo(magTMP2, CV_8UC1);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        clahe->apply(magTMP1, magTMP1);
        clahe->apply(magTMP2, magTMP2);
        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
                resampledMagnitudeSO3_1[k + j * bandwidth * 2] = resampledMagnitudeSO3_1[k + j * bandwidth * 2] +
                                                                 ((double) magTMP1.data[k + j * bandwidth * 2]) / 255.0;
                resampledMagnitudeSO3_2[k + j * bandwidth * 2] = resampledMagnitudeSO3_2[k + j * bandwidth * 2] +
                                                                 ((double) magTMP2.data[k + j * bandwidth * 2]) / 255.0;
            }
        }

    }
    if(debug){
        std::ofstream myFile7, myFile8;
        myFile7.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resampledVoxel1.csv");
        myFile8.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resampledVoxel2.csv");

        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                myFile7 << resampledMagnitudeSO3_1[k + j * bandwidth * 2]; // real part
                myFile7 << "\n";
                myFile8 << resampledMagnitudeSO3_2[k + j * bandwidth * 2]; // real part
                myFile8 << "\n";
            }
        }
        myFile7.close();
        myFile8.close();
    }
    std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
    std::cout << "First Part: " << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - begin1).count()
              << "[ms]" << std::endl;

    std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();




    //use sofft descriptor to calculate the correlation
    this->sofftCorrelationObject.correlationOfTwoSignalsInSO3(resampledMagnitudeSO3_1, resampledMagnitudeSO3_2,
                                                              resultingCorrelationComplex);
    if(debug){
        FILE *fp;
        fp = fopen("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resultCorrelation3D.csv", "w");
        for (int i = 0; i < 8 * bwOut * bwOut * bwOut; i++)
            fprintf(fp, "%.16f\n", resultingCorrelationComplex[i][0]);
        fclose(fp);
    }

    std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
    std::cout << "Second Part: " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count()
              << "[ms]" << std::endl;
    std::chrono::steady_clock::time_point begin3 = std::chrono::steady_clock::now();
    double currentThetaAngle;
    double currentPsiAngle;
    double maxCorrelation = 0;
    std::vector<angleAndCorrelation> correlationOfAngle;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            currentThetaAngle = i * 2.0 * M_PI / N;
            currentPsiAngle = j * 2.0 * M_PI / N;
            //[i + N * j]
            angleAndCorrelation tmpHolding;
            tmpHolding.correlation = resultingCorrelationComplex[i + N * (j + N * 0)][0]; // real part
            if (tmpHolding.correlation > maxCorrelation) {
                maxCorrelation = tmpHolding.correlation;
            }
            tmpHolding.angle = std::fmod(currentThetaAngle + currentPsiAngle + 4 * M_PI, 2 * M_PI);
            correlationOfAngle.push_back(tmpHolding);
        }
    }


    std::sort(correlationOfAngle.begin(), correlationOfAngle.end(), compareTwoAngleCorrelation);

    //std::cout << correlationOfAngle[1].angle << std::endl;
    std::vector<float> correlationAveraged, angleList;
    double currentAverageAngle = correlationOfAngle[0].angle;
    //angleList.push_back(currentAverageAngle);
    int numberOfAngles = 1;
    double averageCorrelation = correlationOfAngle[0].correlation;
    for (int i = 1; i < correlationOfAngle.size(); i++) {

        if (std::abs(currentAverageAngle - correlationOfAngle[i].angle) < 1.0 / N / 4.0) {
            numberOfAngles = numberOfAngles + 1;
            averageCorrelation = averageCorrelation + correlationOfAngle[i].correlation;
        } else {

            correlationAveraged.push_back((float) (averageCorrelation / numberOfAngles / maxCorrelation));
            angleList.push_back((float) currentAverageAngle);
            numberOfAngles = 1;
            averageCorrelation = correlationOfAngle[i].correlation;
            currentAverageAngle = correlationOfAngle[i].angle;

        }
    }
    correlationAveraged.push_back((float) (averageCorrelation / numberOfAngles / maxCorrelation));
    angleList.push_back((float) currentAverageAngle);
    if(debug){
        std::ofstream myFile9;
        myFile9.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resultingCorrelation1D.csv");

        for (int i = 0; i < correlationAveraged.size(); i++) {
            myFile9 << correlationAveraged[i]; // real part
            myFile9 << "\n";

        }
        myFile9.close();
    }

    auto minmax = std::min_element(correlationAveraged.begin(), correlationAveraged.end());
    long distanceToMinElement = std::distance(correlationAveraged.begin(), minmax);
    std::rotate(correlationAveraged.begin(), correlationAveraged.begin() + distanceToMinElement,
                correlationAveraged.end());

    std::vector<int> out;

    PeakFinder::findPeaks(correlationAveraged, out, true);
    std::rotate(correlationAveraged.begin(),
                correlationAveraged.begin() + correlationAveraged.size() - distanceToMinElement,
                correlationAveraged.end());
    for (int i = 0; i < out.size(); ++i) {
        out[i] = out[i] + (int) distanceToMinElement;
        if (out[i] >= correlationAveraged.size()) {
            out[i] = out[i] - correlationAveraged.size();
        }
    }

    std::chrono::steady_clock::time_point end3 = std::chrono::steady_clock::now();
    std::cout << "Third Part: " << std::chrono::duration_cast<std::chrono::milliseconds>(end3 - begin3).count()
              << "[ms]" << std::endl;
    std::chrono::steady_clock::time_point begin4 = std::chrono::steady_clock::now();

    std::vector<double> xShiftList, yShiftList, heightPeakList, estimatedAngleList;


    // for each angle calculate the shift correlation of that angle
    //for( int angleIndex=0; angleIndex<out.size(); ++angleIndex){

    int startIndex, endIndex;
    if (abs(goodGuessAlpha + 100) < 0.0001) {
        //this means that
        startIndex = 0;
        endIndex = out.size();
    } else {
        //guess known therefore take the angle which is closest to the initial guess
        int indexCorrectAngle = 0;
        for (int i = 1; i < out.size(); i++) {
            if (std::abs(angleDifference(angleList[out[indexCorrectAngle]], goodGuessAlpha)) >
                std::abs(angleDifference(angleList[out[i]], goodGuessAlpha))) {
                indexCorrectAngle = i;
            }
        }
        startIndex = indexCorrectAngle;
        endIndex = indexCorrectAngle + 1;
    }
    ///////////////////////////////////// THIS IS THE OLD SHIFT CALCULATION /////////////////////////////////////
//    for (int angleIndex = startIndex; angleIndex < endIndex; ++angleIndex) {
//
//        double currentAngle = -angleList[out[angleIndex]];//describes angle from A to B, therefore we have to reverse the angle
//        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputDataTMP2(new pcl::PointCloud<pcl::PointXYZ>);
//        Eigen::Matrix4d rotationMatrixTMP;
//        //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
//        Eigen::AngleAxisd tmpRotVec(currentAngle, Eigen::Vector3d(0, 0, 1));
//        Eigen::Matrix3d tmpMatrix3d = tmpRotVec.toRotationMatrix();
//        rotationMatrixTMP.block<3, 3>(0, 0) = tmpMatrix3d;
//        rotationMatrixTMP(0, 3) = 0;//x
//        rotationMatrixTMP(1, 3) = 0;//y
//        rotationMatrixTMP(2, 3) = 0;//z
//        rotationMatrixTMP(3, 3) = 1;//1
//        //copy the rotated PCL from PCL1 to PCL2
//        pcl::transformPointCloud(*pointCloudInputData2, *pointCloudInputDataTMP2, rotationMatrixTMP);
//
//
//        maximumScan1 = getSpectrumFromPCL2D(pointCloudInputData1, voxelData1, magnitude1, phase1,
//                                            cellSize * this->N,
//                                            N);
//        maximumScan2 = getSpectrumFromPCL2D(pointCloudInputDataTMP2, voxelData2, magnitude2, phase2,
//                                            cellSize * this->N,
//                                            N);
//
//        //calc phase diff and fftshift + reduce to 2D
//        for (int i = 0; i < N; i++) {
//            for (int j = 0; j < N; j++) {
//
//                int indexX = (N / 2 + i) % N;
//                int indexY = (N / 2 + j) % N;
//                std::complex<double> tmpComplex;
//                tmpComplex.real(0);
//                tmpComplex.imag(
//                        phase1[indexY + N * indexX] - phase2[indexY + N * indexX]);
//                std::complex<double> resultCompexNumber = std::exp(tmpComplex);
//                resultingPhaseDiff2D[j + N * i][0] = resultCompexNumber.real();
//                resultingPhaseDiff2D[j + N * i][1] = resultCompexNumber.imag();
//
//
//            }
//        }
//
//
//        fftw_execute(planFourierToVoxel2D);
//
//        std::chrono::steady_clock::time_point begin5 = std::chrono::steady_clock::now();
//        double meanCorrelation = 0;
//        double maximumCorrelation = 0;
//        for (int i = 0; i < N; i++) {
//            for (int j = 0; j < N; j++) {
//                int indexX = (N / 2 + i) % N;
//                int indexY = (N / 2 + j) % N;
//                //calc magnitude and fftshift
//                resultingCorrelationDouble[indexY + N * indexX] = sqrt(
//                        resultingShiftPeaks2D[j + N * i][0] *
//                        resultingShiftPeaks2D[j + N * i][0] +
//                        resultingShiftPeaks2D[j + N * i][1] *
//                        resultingShiftPeaks2D[j + N * i][1]); // real part;
//                meanCorrelation = meanCorrelation + resultingCorrelationDouble[indexY + N * indexX];
//                if (maximumCorrelation < resultingCorrelationDouble[indexY + N * indexX]) {
//                    maximumCorrelation = resultingCorrelationDouble[indexY + N * indexX];
//                }
//
//            }
//        }
//        meanCorrelation=(meanCorrelation/N)/N;
//
//        double variance = 0;
//        for( int i=0; i < N*N; i++ )
//        {
//            variance += (resultingCorrelationDouble[i] - meanCorrelation) * (resultingCorrelationDouble[i] - meanCorrelation);
//        }
//        variance =variance/ (N*N);
//
////        for (int i = 0; i < N; i++) {
////            for (int j = 0; j < N; j++) {
////                correlation2DResult[j + N * i] = resultingCorrelationDouble[N / 2 + N * (j + N * i)];
////            }
////        }
//        std::vector<indexPeak> localMaximaVector;
//        PeakFinder::findPeaks2D(resultingCorrelationDouble, localMaximaVector, N);
//
//        std::chrono::steady_clock::time_point end5 = std::chrono::steady_clock::now();
//        std::cout << "Time of Part translation: "
//                  << std::chrono::duration_cast<std::chrono::milliseconds>(end5 - begin5).count()
//                  << "[ms]" << std::endl;
//
//        std::sort(localMaximaVector.begin(), localMaximaVector.end(), compareTwoPeaks);
//
//        std::vector<double> differencePeaks;
//        int maximumDiffIterator = 0;
//        double maximumDiffValue = 0;
//        for (int i = 1; i < localMaximaVector.size(); i++) {
//            differencePeaks.push_back(localMaximaVector[i - 1].height - localMaximaVector[i].height);
//            if (localMaximaVector[i - 1].height - localMaximaVector[i].height > maximumDiffValue) {
//                maximumDiffValue = localMaximaVector[i - 1].height - localMaximaVector[i].height;
//                maximumDiffIterator = i - 1;
//            }
//        }
//        maximumDiffIterator = 0;//set it anyway to zero; can be used to know if the result seems valid
//
//        heightPeakList.push_back(localMaximaVector[maximumDiffIterator].height);
//        xShiftList.push_back((localMaximaVector[maximumDiffIterator].y - N / 2.0) * cellSize * this->N * 2.0 / N);
//        yShiftList.push_back((localMaximaVector[maximumDiffIterator].x - N / 2.0) * cellSize * this->N * 2.0 / N);
//        estimatedAngleList.push_back(currentAngle);
//
//        //currently these metrics are not used.
//        std::cout << "current angle: " << currentAngle << std::endl;
//        std::cout << "SNR peak/mean: " << heightPeakList[heightPeakList.size()-1]/meanCorrelation << std::endl;
//        std::cout << "SNR var/mean: " << variance/meanCorrelation << std::endl;
//        std::cout << "height of Peak: "<< heightPeakList[heightPeakList.size()-1] << std::endl;
//
//        if(debug){
//            std::ofstream myFile10;
//            myFile10.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resultingCorrelationShift.csv");
//
//            for (int i = 0; i < N; i++) {
//                for (int j = 0; j < N; j++) {
//                    myFile10 << resultingCorrelationDouble[j + N * i];
//                    myFile10 << "\n";
//                }
//            }
//            myFile10.close();
//        }
//    }




// THIS IS THE NEW CALCULATION OF SHIFT
    for (int angleIndex = startIndex; angleIndex < endIndex; ++angleIndex) {

        std::chrono::steady_clock::time_point begin5 = std::chrono::steady_clock::now();

        double currentAngle = -angleList[out[angleIndex]];//describes angle from A to B, therefore we have to reverse the angle
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputDataTMP2(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4d rotationMatrixTMP;
        //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd tmpRotVec(currentAngle, Eigen::Vector3d(0, 0, 1));
        Eigen::Matrix3d tmpMatrix3d = tmpRotVec.toRotationMatrix();
        rotationMatrixTMP.block<3, 3>(0, 0) = tmpMatrix3d;
        rotationMatrixTMP(0, 3) = 0;//x
        rotationMatrixTMP(1, 3) = 0;//y
        rotationMatrixTMP(2, 3) = 0;//z
        rotationMatrixTMP(3, 3) = 1;//1
        //copy the rotated PCL from PCL1 to PCL2
        pcl::transformPointCloud(*pointCloudInputData2, *pointCloudInputDataTMP2, rotationMatrixTMP);


        maximumScan1 = getSpectrumFromPCL2D(pointCloudInputData1, voxelData1, magnitude1, phase1,
                                            cellSize * this->N,
                                            N);
        maximumScan2 = getSpectrumFromPCL2D(pointCloudInputDataTMP2, voxelData2, magnitude2, phase2,
                                            cellSize * this->N,
                                            N);

        //fftshift and calculate multiplication of spectrums
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {

                int indexX = i;//(N / 2 + i) % N;
                int indexY = j;//(N / 2 + j) % N;
                //calculate the spectrum back
                std::complex<double> tmpComplex1 = magnitude1[indexY + N * indexX]*std::exp(std::complex<double>(0,phase1[indexY + N * indexX]));
                std::complex<double> tmpComplex2 = magnitude2[indexY + N * indexX]*std::exp(std::complex<double>(0,phase2[indexY + N * indexX]));

                resultingPhaseDiff2D[j + N * i][0] = ((tmpComplex1)*conj(tmpComplex2)).real();
                resultingPhaseDiff2D[j + N * i][1] = ((tmpComplex1)*conj(tmpComplex2)).imag();

            }
        }


        fftw_execute(planFourierToVoxel2D);



        // fftshift and magnitude
        //double meanCorrelation = 0;
        int indexMaximumCorrelationI;
        int indexMaximumCorrelationJ;
        double maximumCorrelation = 0;
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                int indexX = (N / 2 + i) % N;
                int indexY = (N / 2 + j) % N;

                resultingCorrelationDouble[indexY + N * indexX] = sqrt(
                        resultingShiftPeaks2D[j + N * i][0] *
                        resultingShiftPeaks2D[j + N * i][0] +
                        resultingShiftPeaks2D[j + N * i][1] *
                        resultingShiftPeaks2D[j + N * i][1]); // real part;
                //meanCorrelation = meanCorrelation + resultingCorrelationDouble[indexY + N * indexX];
                if (maximumCorrelation < resultingCorrelationDouble[indexY + N * indexX]) {
                    maximumCorrelation = resultingCorrelationDouble[indexY + N * indexX];
                    indexMaximumCorrelationI=indexX;
                    indexMaximumCorrelationJ=indexY;
                }

            }
        }

        //meanCorrelation=(meanCorrelation/N)/N;
//        std::vector<indexPeak> localMaximaVector;
//        PeakFinder::findPeaks2D(resultingCorrelationDouble, localMaximaVector, N);
//
//
//
//        std::sort(localMaximaVector.begin(), localMaximaVector.end(), compareTwoPeaks);
//
//        std::vector<double> differencePeaks;
//        int maximumDiffIterator = 0;
//        double maximumDiffValue = 0;
//        for (int i = 1; i < localMaximaVector.size(); i++) {
//            differencePeaks.push_back(localMaximaVector[i - 1].height - localMaximaVector[i].height);
//            if (localMaximaVector[i - 1].height - localMaximaVector[i].height > maximumDiffValue) {
//                maximumDiffValue = localMaximaVector[i - 1].height - localMaximaVector[i].height;
//                maximumDiffIterator = i - 1;
//            }
//        }
//        maximumDiffIterator = 0;//set it anyway to zero; can be used to know if the result seems valid
        std::chrono::steady_clock::time_point end5 = std::chrono::steady_clock::now();
        std::cout << "Time of Part translation: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end5 - begin5).count()
                  << "[ms]" << std::endl;
        heightPeakList.push_back(maximumCorrelation);
        xShiftList.push_back((indexMaximumCorrelationJ - N / 2.0) * cellSize );
        yShiftList.push_back((indexMaximumCorrelationI - N / 2.0) * cellSize );
        estimatedAngleList.push_back(currentAngle);

        //currently these metrics are not used.
        std::cout << "current angle: " << currentAngle << std::endl;
//        std::cout << "SNR peak/mean: " << heightPeakList[heightPeakList.size()-1]/meanCorrelation << std::endl;
//        std::cout << "SNR var/mean: " << variance/meanCorrelation << std::endl;
        std::cout << "height of Peak: "<< heightPeakList[heightPeakList.size()-1] << std::endl;
        std::cout << "index I: " <<  indexMaximumCorrelationI << std::endl;
        std::cout << "index J: " <<  indexMaximumCorrelationJ << std::endl;
        if(debug){
            std::ofstream myFile10;
            myFile10.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resultingCorrelationShift.csv");

            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    myFile10 << resultingCorrelationDouble[j + N * i];
                    myFile10 << "\n";
                }
            }
            myFile10.close();
        }
    }



    auto maxElementIter = std::max_element(heightPeakList.begin(), heightPeakList.end());
    int distanceToMaxElement = (int) std::distance(heightPeakList.begin(), maxElementIter);



    std::chrono::steady_clock::time_point end4 = std::chrono::steady_clock::now();
    std::cout << "Fourth Part: " << std::chrono::duration_cast<std::chrono::milliseconds>(end4 - begin4).count()
              << "[ms]" << std::endl;

    std::cout << "#######################################################" << std::endl;
    std::cout << "Estimation PCL 2 to PCL 1 " << std::endl;
    std::cout << "Estimated  Angle: " << estimatedAngleList[distanceToMaxElement] << std::endl;
    std::cout << "Estimated xShift: " << xShiftList[distanceToMaxElement] << std::endl;
    std::cout << "Estimated yShift: " << yShiftList[distanceToMaxElement] << std::endl;

    Eigen::Matrix4d estimatedRotationScans;//from second scan to first
    //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd rotation_vectorTMP(estimatedAngleList[distanceToMaxElement], Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpRotMatrix3d = rotation_vectorTMP.toRotationMatrix();
    estimatedRotationScans.block<3, 3>(0, 0) = tmpRotMatrix3d;
    estimatedRotationScans(0, 3) = xShiftList[distanceToMaxElement];
    estimatedRotationScans(1, 3) = yShiftList[distanceToMaxElement];
    estimatedRotationScans(2, 3) = 0;
    estimatedRotationScans(3, 3) = 1;

    //std::cout << estimatedRotationScans << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2RotatedTo1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pointCloudInputData2, *pointCloudInputData2RotatedTo1, estimatedRotationScans);


    return estimatedRotationScans.inverse();//makes out of transform from 2 to 1, transform 1 to 2
}