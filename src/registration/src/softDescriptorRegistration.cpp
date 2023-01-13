//
// Created by tim-external on 01.03.22.
//

#include "softDescriptorRegistration.h"

bool compareTwoAngleCorrelation(rotationPeak i1, rotationPeak i2) {
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
    return M_PI * (1 * index + 0) / (2.0 * bandwidth);
}

double phiIncrement(double index, int bandwidth) {
    return M_PI * index / bandwidth;
}

double angleDifference(double angle1, double angle2) {//gives angle 1 - angle 2
    return atan2(sin(angle1 - angle2), cos(angle1 - angle2));
}

double
softDescriptorRegistration::getSpectrumFromVoxelData2D(double voxelData[], double magnitude[], double phase[],
                                                       bool gaussianBlur) {

    double *voxelDataTMP;
    voxelDataTMP = (double *) malloc(sizeof(double) * N * N);
    for (int i = 0; i < this->N * this->N; i++) {
        voxelDataTMP[i] = voxelData[i];
    }
    if (gaussianBlur) {
        cv::Mat magTMP1(this->N, this->N, CV_64F, voxelDataTMP);
        //add gaussian blur
        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
    }



    //from voxel data to row and input for fftw
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            inputSpacialData[j + N * i][0] = voxelDataTMP[j + N * i]; // real part
            inputSpacialData[j + N * i][1] = 0; // imaginary part
        }
    }

    fftw_execute(planVoxelToFourier2D);

    //calc magnitude and phase
    double maximumMagnitude = 0;

    //get magnitude and find maximum
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
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

    free(voxelDataTMP);
    return maximumMagnitude;
}

double
softDescriptorRegistration::getSpectrumFromVoxelData2DCorrelation(double voxelData[], double magnitude[],
                                                                  double phase[],
                                                                  bool gaussianBlur, double normalizationFactor) {

    double *voxelDataTMP;
    voxelDataTMP = (double *) malloc(sizeof(double) * correlationN * correlationN);
    for (int i = 0; i < this->correlationN * this->correlationN; i++) {
        voxelDataTMP[i] = voxelData[i];
    }
    if (gaussianBlur) {
        cv::Mat magTMP1(this->correlationN, this->correlationN, CV_64F, voxelData);
        //add gaussian blur
        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
    }

    for (int i = 0; i < this->correlationN; i++) {
        inputSpacialDataCorrelation[i][0] = 0;
        inputSpacialDataCorrelation[i][1] = 0;
    }


    //from voxel data to row and input for fftw
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            inputSpacialDataCorrelation[(j + (int) (this->correlationN / 4)) +
                                        this->correlationN * (i + (int) (this->correlationN / 4))][0] =
                    normalizationFactor * voxelData[j +
                                                    N *
                                                    i]; // real part
//            inputSpacialDataCorrelation[j + N * i][1] = 0; // imaginary part
        }
    }
//    for (int j = 0; j < this->correlationN*this->correlationN; j++) {
//        std::cout << inputSpacialDataCorrelation[j][0] << std::endl;
//    }
    fftw_execute(planVoxelToFourier2DCorrelation);

    //calc magnitude and phase
    double maximumMagnitude = 0;

    //get magnitude and find maximum
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            magnitude[j + this->correlationN * i] = sqrt(
                    spectrumOutCorrelation[j + this->correlationN * i][0] *
                    spectrumOutCorrelation[j + this->correlationN * i][0] +
                    spectrumOutCorrelation[j + this->correlationN * i][1] *
                    spectrumOutCorrelation[j + this->correlationN * i][1]); // real part;
            if (maximumMagnitude < magnitude[j + this->correlationN * i]) {
                maximumMagnitude = magnitude[j + this->correlationN * i];
            }

            phase[j + this->correlationN * i] = atan2(spectrumOutCorrelation[j + this->correlationN * i][1],
                                                      spectrumOutCorrelation[j + this->correlationN * i][0]);

        }
    }

    free(voxelDataTMP);
    return maximumMagnitude;
}


double
softDescriptorRegistration::sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[],
                                                                 double goodGuessAlpha, bool debug) {

    std::vector<rotationPeak> allAnglesList = this->sofftRegistrationVoxel2DListOfPossibleRotations(voxelData1Input,
                                                                                                    voxelData2Input,
                                                                                                    debug);

    int indexCorrectAngle = 0;
    for (int i = 1; i < allAnglesList.size(); i++) {
        if (std::abs(angleDifference(allAnglesList[indexCorrectAngle].angle, goodGuessAlpha)) >
            std::abs(angleDifference(allAnglesList[i].angle, goodGuessAlpha))) {
            indexCorrectAngle = i;
        }
    }
    return allAnglesList[indexCorrectAngle].angle;//this angle is from Pos1 to Pos 2
}

std::vector<rotationPeak>
softDescriptorRegistration::sofftRegistrationVoxel2DListOfPossibleRotations(double voxelData1Input[],
                                                                            double voxelData2Input[], bool debug) {

    double maximumScan1Magnitude = this->getSpectrumFromVoxelData2D(voxelData1Input, this->magnitude1,
                                                                    this->phase1, false);
    double maximumScan2Magnitude = this->getSpectrumFromVoxelData2D(voxelData2Input, this->magnitude2,
                                                                    this->phase2, false);


    if (debug) {
        std::ofstream myFile1, myFile2, myFile3, myFile4, myFile5, myFile6;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/magnitudeFFTW1.csv");
        myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/phaseFFTW1.csv");
        myFile3.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/voxelDataFFTW1.csv");
        myFile4.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/magnitudeFFTW2.csv");
        myFile5.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/phaseFFTW2.csv");
        myFile6.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/voxelDataFFTW2.csv");
        for (int j = 0; j < N; j++) {
            for (int i = 0; i < N; i++) {
                myFile1 << magnitude1[j + N * i]; // real part
                myFile1 << "\n";
                myFile2 << phase1[j + N * i]; // imaginary part
                myFile2 << "\n";
                myFile3 << voxelData1Input[j + N * i]; // imaginary part
                myFile3 << "\n";
                myFile4 << magnitude2[j + N * i]; // real part
                myFile4 << "\n";
                myFile5 << phase2[j + N * i]; // imaginary part
                myFile5 << "\n";
                myFile6 << voxelData2Input[j + N * i]; // imaginary part
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
    if (maximumScan2Magnitude < maximumScan1Magnitude) {
        globalMaximumMagnitude = maximumScan1Magnitude;
    } else {
        globalMaximumMagnitude = maximumScan2Magnitude;
    }

    //normalize and fftshift
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            int indexX = (N / 2 + i) % N;
            int indexY = (N / 2 + j) % N;

            magnitude1Shifted[indexY + N * indexX] =
                    magnitude1[j + N * i] / globalMaximumMagnitude;
            magnitude2Shifted[indexY + N * indexX] =
                    magnitude2[j + N * i] / globalMaximumMagnitude;
        }
    }


    //re-initialize to zero
    for (int i = 0; i < N * N; i++) {
        resampledMagnitudeSO3_1[i] = 0;
        resampledMagnitudeSO3_2[i] = 0;
        resampledMagnitudeSO3_1TMP[i] = 0;
        resampledMagnitudeSO3_2TMP[i] = 0;
    }

    int minRNumber = 10;//was 4
    int maxRNumber = N / 2 - 2;
    int bandwidth = N / 2;
    //CHANGE HERE HAPPEND TESTS
    for (int r = maxRNumber - 1; r < maxRNumber; r++) {
        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
                int xIndex = std::round((double) r * std::sin(thetaIncrement((double) j, bandwidth)) *
                                        std::cos(phiIncrement((double) k, bandwidth)) + bandwidth) - 1;
                int yIndex = std::round((double) r * std::sin(thetaIncrement((double) j, bandwidth)) *
                                        std::sin(phiIncrement((double) k, bandwidth)) + bandwidth) - 1;
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
                resampledMagnitudeSO3_1[j + k * bandwidth * 2] = resampledMagnitudeSO3_1[j + k * bandwidth * 2] +
                                                                 ((double) magTMP1.data[j + k * bandwidth * 2]) / 255.0;
                resampledMagnitudeSO3_2[j + k * bandwidth * 2] = resampledMagnitudeSO3_2[j + k * bandwidth * 2] +
                                                                 ((double) magTMP2.data[j + k * bandwidth * 2]) / 255.0;
            }
        }
//        std::cout << resampledMagnitudeSO3_1[100 + 100 * bandwidth * 2] << std::endl;
//        std::cout << resampledMagnitudeSO3_1[100 + 100 * bandwidth * 2] << std::endl;
    }


    if (debug) {
        std::ofstream myFile7, myFile8;
        myFile7.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resampledVoxel1.csv");
        myFile8.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resampledVoxel2.csv");

        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                myFile7 << resampledMagnitudeSO3_1[j + k * bandwidth * 2]; // real part
                myFile7 << "\n";
                myFile8 << resampledMagnitudeSO3_2[j + k * bandwidth * 2]; // real part
                myFile8 << "\n";
            }
        }
        myFile7.close();
        myFile8.close();
    }

    //use soft descriptor to calculate the correlation
    this->sofftCorrelationObject.correlationOfTwoSignalsInSO3(resampledMagnitudeSO3_1, resampledMagnitudeSO3_2,
                                                              resultingCorrelationComplex);
    if (debug) {
        FILE *fp;
        fp = fopen(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultCorrelation3D.csv",
                "w");
        for (int i = 0; i < 8 * bwOut * bwOut * bwOut; i++)
            fprintf(fp, "%.16f\n", resultingCorrelationComplex[i][0]);
        fclose(fp);
    }

    //calcs the rotation angle around z axis for 2D scans
    double currentThetaAngle;
    double currentPhiAngle;
    double maxCorrelation = 0;
    std::vector<rotationPeak> correlationOfAngle;
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            currentThetaAngle = j * 2.0 * M_PI / N;
            currentPhiAngle = i * 2.0 * M_PI / N;
            //[i + N * j]
            rotationPeak tmpHolding;
            tmpHolding.peakCorrelation = resultingCorrelationComplex[j + N * (i + N * 0)][0]; // real part
            if (tmpHolding.peakCorrelation > maxCorrelation) {
                maxCorrelation = tmpHolding.peakCorrelation;
            }
            // test on dataset with N and N/2 and 0   first test + n/2
            tmpHolding.angle = std::fmod(-(currentThetaAngle + currentPhiAngle) + 6 * M_PI + 0.0 * M_PI / (N),
                                         2 * M_PI);
            correlationOfAngle.push_back(tmpHolding);
        }
    }

    std::sort(correlationOfAngle.begin(), correlationOfAngle.end(), compareTwoAngleCorrelation);

    std::vector<float> correlationAveraged, angleList;
    double currentAverageAngle = correlationOfAngle[0].angle;
    //angleList.push_back(currentAverageAngle);
    int numberOfAngles = 1;
    double averageCorrelation = correlationOfAngle[0].peakCorrelation;
    for (int i = 1; i < correlationOfAngle.size(); i++) {

        if (std::abs(currentAverageAngle - correlationOfAngle[i].angle) < 1.0 / N / 4.0) {
            numberOfAngles = numberOfAngles + 1;
            averageCorrelation = averageCorrelation + correlationOfAngle[i].peakCorrelation;
        } else {

            correlationAveraged.push_back((float) (averageCorrelation / numberOfAngles));
            angleList.push_back((float) currentAverageAngle);
            numberOfAngles = 1;
            averageCorrelation = correlationOfAngle[i].peakCorrelation;
            currentAverageAngle = correlationOfAngle[i].angle;

        }
    }
    correlationAveraged.push_back((float) (averageCorrelation / numberOfAngles));

    angleList.push_back((float) currentAverageAngle);
    if (debug) {
        std::ofstream myFile9;
        myFile9.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultingCorrelation1D.csv");

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

    PeakFinder::findPeaks(correlationAveraged, out, true, 4.0);

    std::rotate(correlationAveraged.begin(),
                correlationAveraged.begin() + correlationAveraged.size() - distanceToMinElement,
                correlationAveraged.end());
    for (int i = 0; i < out.size(); ++i) {
        out[i] = out[i] + (int) distanceToMinElement;
        if (out[i] >= correlationAveraged.size()) {
            out[i] = out[i] - correlationAveraged.size();
        }
    }

    std::vector<rotationPeak> returnVectorWithPotentialAngles;

    for (int i = 0; i < out.size(); i++) {
        rotationPeak tmpPeak{};
        tmpPeak.angle = angleList[out[i]];
        tmpPeak.peakCorrelation = correlationAveraged[out[i]];
        returnVectorWithPotentialAngles.push_back(tmpPeak);
    }

    return returnVectorWithPotentialAngles;
}


//Eigen::Vector2d softDescriptorRegistration::sofftRegistrationVoxel2DTranslation(double voxelData1Input[],
//                                                                                double voxelData2Input[],
//                                                                                double &fitnessX, double &fitnessY,
//                                                                                double cellSize,
//                                                                                Eigen::Vector3d initialGuess,
//                                                                                bool useInitialGuess,
//                                                                                double &heightMaximumPeak, bool debug) {
//
//    //std::vector<double> xShiftList, yShiftList, heightPeakList, estimatedAngleList, heightPeakAngleList;
//
//    //i have to inverse initialGuess ( i think it is because the cross correlation gives translation from 2 -> 1 not from 1 ->2)
//    initialGuess = -initialGuess;
//    // create padding in translation voxelData
//
//
//
//
//    double maximumScan1 = this->getSpectrumFromVoxelData2DCorrelation(voxelData1Input, this->magnitude1Correlation,
//                                                                      this->phase1Correlation, false, 1);
//
////    if (debug) {
////        std::ofstream myFile1, myFile2, myFile3;
////        myFile1.open(
////                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/magnitudeFFTW1.csv");
////        myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/phaseFFTW1.csv");
////        myFile3.open(
////                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/voxelDataFFTW1.csv");
////
////        for (int j = 0; j < this->correlationN; j++) {
////            for (int i = 0; i < this->correlationN; i++) {
////                myFile1 << magnitude1Correlation[j + this->correlationN * i]; // real part
////                myFile1 << "\n";
////                myFile2 << phase1Correlation[j + this->correlationN * i]; // imaginary part
////                myFile2 << "\n";
////                myFile3 << inputSpacialDataCorrelation[j + this->correlationN * i][0]; // imaginary part
////                myFile3 << "\n";
////            }
////        }
////
////        myFile1.close();
////        myFile2.close();
////        myFile3.close();
////    }
//
//    double maximumScan2 = this->getSpectrumFromVoxelData2DCorrelation(voxelData2Input, this->magnitude2Correlation,
//                                                                      this->phase2Correlation, false, 1);
//
////    if (debug) {
////        std::ofstream myFile4, myFile5, myFile6;
////        myFile4.open(
////                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/magnitudeFFTW2.csv");
////        myFile5.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/phaseFFTW2.csv");
////        myFile6.open(
////                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/voxelDataFFTW2.csv");
////        for (int j = 0; j < this->correlationN; j++) {
////            for (int i = 0; i < this->correlationN; i++) {
////                myFile4 << magnitude2Correlation[j + this->correlationN * i]; // real part
////                myFile4 << "\n";
////                myFile5 << phase1Correlation[j + this->correlationN * i]; // imaginary part
////                myFile5 << "\n";
////                myFile6 << inputSpacialDataCorrelation[j + this->correlationN * i][0]; // real part
////                myFile6 << "\n";
////            }
////        }
////
////
////        myFile4.close();
////        myFile5.close();
////        myFile6.close();
////    }
//
//
//
//    //fftshift and calculate correlation of spectrums
//    for (int j = 0; j < this->correlationN; j++) {
//        for (int i = 0; i < this->correlationN; i++) {
//
////            int indexX = (this->correlationN / 2 + i) % this->correlationN;
////            int indexY = (this->correlationN / 2 + j) % this->correlationN;
//            int indexX = i;
//            int indexY = j;
//            //calculate the spectrum back
//            std::complex<double> tmpComplex1 =
//                    magnitude1Correlation[indexY + this->correlationN * indexX] *
//                    std::exp(std::complex<double>(0, phase1Correlation[indexY + this->correlationN * indexX]));
//            std::complex<double> tmpComplex2 =
//                    magnitude2Correlation[indexY + this->correlationN * indexX] *
//                    std::exp(std::complex<double>(0, phase2Correlation[indexY + this->correlationN * indexX]));
////                std::complex<double> tmpComplex1 = std::exp(std::complex<double>(0, phase1[indexY + N * indexX]));
////                std::complex<double> tmpComplex2 = std::exp(std::complex<double>(0, phase2[indexY + N * indexX]));
////                std::complex<double> tmpComplex;
////                tmpComplex.real(0);
////                tmpComplex.imag(phase1[indexY + N * indexX] - phase2[indexY + N * indexX]);
////                std::complex<double> resultCompexNumber = std::exp(tmpComplex);
////                resultingPhaseDiff2D[j + N * i][0] = resultCompexNumber.real();
////                resultingPhaseDiff2D[j + N * i][1] = resultCompexNumber.imag();
////            std::cout << tmpComplex1 << std::endl;
//            std::complex<double> resultComplex = ((tmpComplex1) * conj(tmpComplex2));
//            resultingPhaseDiff2DCorrelation[j + this->correlationN * i][0] = resultComplex.real();
//            resultingPhaseDiff2DCorrelation[j + this->correlationN * i][1] = resultComplex.imag();
//
//        }
//    }
//
//
//    fftw_execute(planFourierToVoxel2DCorrelation);
//
//
//
//    // fftshift and calc magnitude
//    //double meanCorrelation = 0;
//    int indexMaximumCorrelationI;
//    int indexMaximumCorrelationJ;
//    double maximumCorrelation = 0;
//    for (int j = 0; j < this->correlationN; j++) {
//        for (int i = 0; i < this->correlationN; i++) {
//            int indexX = (this->correlationN / 2 + i + this->correlationN) % this->correlationN;// changed j and i here
//            int indexY = (this->correlationN / 2 + j + this->correlationN) % this->correlationN;
////            int indexX = i;// changed j and i here
////            int indexY = j;
//            resultingCorrelationDouble[indexY + this->correlationN * indexX] = sqrt(
//                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] *
//                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] +
//                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1] *
//                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1]); // magnitude;
//            if (resultingCorrelationDouble[indexY + this->correlationN * indexX] < 10000) {
//                resultingCorrelationDouble[indexY + this->correlationN * indexX] = 0;
//            }
//            //meanCorrelation = meanCorrelation + resultingCorrelationDouble[indexY + N * indexX];
//            if (maximumCorrelation < resultingCorrelationDouble[indexY + this->correlationN * indexX]) {
//                maximumCorrelation = resultingCorrelationDouble[indexY + this->correlationN * indexX];
//                indexMaximumCorrelationI = indexX;
//                indexMaximumCorrelationJ = indexY;
//            }
//
//        }
//    }
//    ////////////////////////////////// HERE COMES THE NEW STUFFFF //////////////////////////////////
//    float impactOfNoise = 4.0;
//    std::vector<std::vector<int>> xPeaks, yPeaks;
//
//    for (int j = 0; j < this->correlationN; j++) {
//        std::vector<float> inputYLine;
//        for (int i = 0; i < this->correlationN; i++) {
//            inputYLine.push_back((float) resultingCorrelationDouble[j + this->correlationN * i]);
//        }
//        std::vector<int> out;
//        PeakFinder::findPeaks(inputYLine, out, false, impactOfNoise);
////        for(int i = 0 ; i < out.size();i++){
////            std::cout << out[i] << std::endl;
////        }
////        std::cout <<"next"<< std::endl;
//        yPeaks.push_back(out);
//    }
//
//    for (int i = 0; i < this->correlationN; i++) {
//        std::vector<float> inputXLine;
//        for (int j = 0; j < this->correlationN; j++) {
//            inputXLine.push_back((float) resultingCorrelationDouble[j + this->correlationN * i]);
//        }
//        std::vector<int> out;
//        PeakFinder::findPeaks(inputXLine, out, false, impactOfNoise);
////        for(int j = 0 ; j < out.size();j++){
////            std::cout << out[j] << std::endl;
////        }
////        std::cout <<"next"<< std::endl;
//        xPeaks.push_back(out);
//
//    }
////    for (int j = 0; j < this->correlationN; j++) {
////        for (int i = 0; i < this->correlationN; i++) {
////            auto iteratorX = std::find(xPeaks[i].begin(), xPeaks[i].end(), j);
////            if( iteratorX != xPeaks[i].end()){
////                auto iteratorY = std::find(yPeaks[j].begin(), yPeaks[j].end(), i);
////                if(iteratorY!= yPeaks[j].end()){
////                    std::cout << "found Peak:" << std::endl;
//////                    std::cout << *iteratorX << std::endl;
//////                    std::cout << *iteratorY  << std::endl;
////                    std::cout << i+1 << std::endl;
////                    std::cout << j+1  << std::endl;
////                }
////            }
////        }
////    }
//
//
////    for (int j = 0; j < this->correlationN*this->correlationN; j++) {
////        resultingCorrelationDouble[j]=sqrt(
////                resultingShiftPeaks2DCorrelation[j][0] *
////                resultingShiftPeaks2DCorrelation[j][0] +
////                resultingShiftPeaks2DCorrelation[j][1] *
////                resultingShiftPeaks2DCorrelation[j][1]);
////    }
//    if (debug) {
//        std::ofstream myFile10;
//        myFile10.open(
//                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultingCorrelationShift.csv");
//
//        for (int j = 0; j < this->correlationN; j++) {
//            for (int i = 0; i < this->correlationN; i++) {
//                myFile10 << resultingCorrelationDouble[j + this->correlationN * i];
//                myFile10 << "\n";
//            }
//        }
//        myFile10.close();
//    }
//
//    //89 131
//    if (useInitialGuess) {
//        //find local maximum in 2d array
//        int initialIndexX = (int) (initialGuess[0] / cellSize + this->correlationN / 2);
//        int initialIndexY = (int) (initialGuess[1] / cellSize + this->correlationN / 2);
//        int localMaxDiffX = 0;
//        int localMaxDiffY = 0;
//        do {
//            localMaxDiffX = 0;
//            localMaxDiffY = 0;
//
//            for (int i = -1; i < 2; i++) {
//                for (int j = -1; j < 2; j++) {
//                    if (resultingCorrelationDouble[(initialIndexY + localMaxDiffY) +
//                                                   this->correlationN * (initialIndexX + localMaxDiffX)] <
//                        resultingCorrelationDouble[(initialIndexY + j) + this->correlationN * (initialIndexX + i)]) {
//                        localMaxDiffX = i;
//                        localMaxDiffY = j;
//                    }
//                }
//            }
//            initialIndexY += localMaxDiffY;
//            initialIndexX += localMaxDiffX;
//        } while (localMaxDiffX != 0 || localMaxDiffY != 0);
//        indexMaximumCorrelationI = initialIndexX;
//        indexMaximumCorrelationJ = initialIndexY;
//    }
//    heightMaximumPeak = resultingCorrelationDouble[indexMaximumCorrelationJ +
//                                                   this->correlationN * indexMaximumCorrelationI];//Hope that is correct
//    // @TODO find SubPixel accuracy
//
//
//
////    std::cout << "estimated indexToStart:" << std::endl;
////    std::cout << indexMaximumCorrelationI<< std::endl;
////    std::cout << indexMaximumCorrelationJ << std::endl;
//    Eigen::Vector3d translationCalculated((indexMaximumCorrelationI - (int) (this->correlationN / 2.0)) * cellSize,
//                                          (indexMaximumCorrelationJ - (int) (this->correlationN / 2.0)) * cellSize, 0);
////    std::cout << "translationCalculated: "<< std::endl;
////    std::cout << translationCalculated << std::endl;
//
//
//    //currently these metrics are not used.
////        std::cout << "current angle: " << currentAngle << std::endl;
////        std::cout << "SNR peak/mean: " << heightPeakList[heightPeakList.size()-1]/meanCorrelation << std::endl;
////        std::cout << "SNR var/mean: " << variance/meanCorrelation << std::endl;
////        std::cout << "height of Peak: " << heightPeakList[heightPeakList.size() - 1] << std::endl;
////        std::cout << "index I: " << indexMaximumCorrelationI << std::endl;
////        std::cout << "index J: " << indexMaximumCorrelationJ << std::endl;
//
//
//
//
//    // calculate
//
//    // x calculation of C
//    double aParam = resultingCorrelationDouble[indexMaximumCorrelationJ +
//                                               this->correlationN * indexMaximumCorrelationI];
//    double bParam = indexMaximumCorrelationI;
//    double cParam = 0;
//    for (int i = 0; i < this->correlationN; i++) {
//        double xTMP = i;
//        double yTMP = resultingCorrelationDouble[indexMaximumCorrelationJ + this->correlationN * i];
//        double cTMP = abs((xTMP - bParam) / (sqrt(-2 * log(yTMP / aParam))));
//        if (xTMP != indexMaximumCorrelationI) {
//            cParam = cParam + cTMP;
//        }
//
//    }
//    fitnessX = cParam / (this->correlationN - 1) * cellSize;
//    //std::cout << "cParam X: " << fitnessX<<std::endl;
//
//
//    //aParam=resultingCorrelationDouble[indexMaximumCorrelationJ + N * indexMaximumCorrelationI];
//    bParam = indexMaximumCorrelationJ;
//    cParam = 0;
//    for (int i = 0; i < this->correlationN; i++) {
//        double xTMP = i;
//        double yTMP = resultingCorrelationDouble[i + this->correlationN * indexMaximumCorrelationI];
//        double cTMP = abs((xTMP - bParam) / (sqrt(-2 * log(yTMP / aParam))));
//        if (xTMP != indexMaximumCorrelationJ) {
//            cParam = cParam + cTMP;
//        }
//
//    }
//    fitnessY = cParam / (this->correlationN - 1) * cellSize;
//
//
//    //translationCalculated = generalHelpfulTools::getQuaternionFromRPY(0,0,M_PI).toRotationMatrix()*translationCalculated;
//    if (!isfinite(fitnessX)) {
//        fitnessX = 10;
//    }
//    if (!isfinite(fitnessY)) {
//        fitnessY = 10;
//    }
//    Eigen::Vector2d returnVector;
//    returnVector[0] = translationCalculated[0];
//    returnVector[1] = translationCalculated[1];
//    return -returnVector;//returning - because we want from 1 to 2 and not the other way around
//
//}

std::vector<translationPeak>
softDescriptorRegistration::sofftRegistrationVoxel2DTranslationAllPossibleSolutions(double voxelData1Input[],
                                                                                    double voxelData2Input[],
                                                                                    double cellSize,
                                                                                    double normalizationFactor,
                                                                                    bool debug,
                                                                                    int numberOfRotationForDebug,
                                                                                    int registrationNoiseImpactFactor,
                                                                                    double ignorePercentageFactor) {
    //copy and normalize voxelDataInput



    // create padding in translation voxelData
    double maximumScan1 = this->getSpectrumFromVoxelData2DCorrelation(voxelData1Input, this->magnitude1Correlation,
                                                                      this->phase1Correlation, false,
                                                                      normalizationFactor);


    double maximumScan2 = this->getSpectrumFromVoxelData2DCorrelation(voxelData2Input, this->magnitude2Correlation,
                                                                      this->phase2Correlation, false,
                                                                      normalizationFactor);


    //calculate correlation of spectrums
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {

            int indexX = i;
            int indexY = j;
            //calculate the spectrum back
            std::complex<double> tmpComplex1 =
                    magnitude1Correlation[indexY + this->correlationN * indexX] *
                    std::exp(std::complex<double>(0, phase1Correlation[indexY + this->correlationN * indexX]));
            std::complex<double> tmpComplex2 =
                    magnitude2Correlation[indexY + this->correlationN * indexX] *
                    std::exp(std::complex<double>(0, phase2Correlation[indexY + this->correlationN * indexX]));
            std::complex<double> resultComplex = ((tmpComplex1) * conj(tmpComplex2));
            resultingPhaseDiff2DCorrelation[j + this->correlationN * i][0] = resultComplex.real();
            resultingPhaseDiff2DCorrelation[j + this->correlationN * i][1] = resultComplex.imag();

        }
    }

    // back fft
    fftw_execute(planFourierToVoxel2DCorrelation);


    // normalization Test:

//    std::cout << this->normalizationFactorCalculation(254,254)<< std::endl;
//    std::cout << this->normalizationFactorCalculation(255,255)<< std::endl;
//    std::cout << this->normalizationFactorCalculation(256,256)<< std::endl;
//    std::cout << this->normalizationFactorCalculation(255,255)/this->normalizationFactorCalculation(30,30)<< std::endl;

    // fftshift and calc magnitude
    //double meanCorrelation = 0;
    int indexMaximumCorrelationI;
    int indexMaximumCorrelationJ;
    double maximumCorrelation = 0;
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            int indexX = (this->correlationN / 2 + i + this->correlationN) % this->correlationN;// changed j and i here
            int indexY = (this->correlationN / 2 + j + this->correlationN) % this->correlationN;
            double normalizationFactorForCorrelation = sqrt(
                    this->correlationN * this->correlationN / this->normalizationFactorCalculation(indexX, indexY));
//            double normalizationFactorForCorrelation = 1;



            //            int indexX = i;// changed j and i here
//            int indexY = j;
            resultingCorrelationDouble[indexY + this->correlationN * indexX] = normalizationFactorForCorrelation * sqrt(
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] *
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] +
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1] *
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1]); // magnitude;
//            if(resultingCorrelationDouble[indexY + this->correlationN * indexX] < 10000){
//                resultingCorrelationDouble[indexY + this->correlationN * indexX] = 0;
//            }
            //meanCorrelation = meanCorrelation + resultingCorrelationDouble[indexY + N * indexX];
            if (maximumCorrelation < resultingCorrelationDouble[indexY + this->correlationN * indexX]) {
                maximumCorrelation = resultingCorrelationDouble[indexY + this->correlationN * indexX];
            }
        }
    }
    ////////////////////////////////// HERE COMES THE NEW STUFFFF //////////////////////////////////

    //add gaussian blur
//    if (true) {
//        cv::Mat magTMP1(this->correlationN, this->correlationN, CV_64F, resultingCorrelationDouble);
//        for (int i = 0; i < 5; i++) {
//            cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        }
//    }

    //function of 2D peak detection

    std::vector<translationPeak> potentialTranslations = this->peakDetectionOf2DCorrelation(maximumCorrelation,
                                                                                            cellSize,
                                                                                            registrationNoiseImpactFactor,
                                                                                            ignorePercentageFactor);


    if (debug) {
        std::ofstream myFile10;
        myFile10.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultingCorrelationShift_" +
                std::to_string(numberOfRotationForDebug) + "_.csv");

        for (int j = 0; j < this->correlationN; j++) {
            for (int i = 0; i < this->correlationN; i++) {
                myFile10 << resultingCorrelationDouble[j + this->correlationN * i];
                myFile10 << "\n";
            }
        }
        myFile10.close();
    }



    // calculate for each maxima a covariance(my algorithm)
    int definedRadiusVoxel = ceil(this->correlationN / 30);
    double definedRadiusSI = cellSize * this->correlationN / 30.0;
    for (auto &potentialTranslation: potentialTranslations) {
        double resultingIntegral = 0;
        double maximumIntegral = M_PI * definedRadiusSI * definedRadiusSI * potentialTranslation.peakHeight;
        for (int i = -definedRadiusVoxel; i < definedRadiusVoxel + 1; i++) {
            for (int j = -definedRadiusVoxel; j < definedRadiusVoxel + 1; j++) {
                if (sqrt((double) (i * i + j * j)) *
                    cellSize < definedRadiusSI) {
                    resultingIntegral += resultingCorrelationDouble[(potentialTranslation.translationVoxel.y() + j) +
                                                                    this->correlationN *
                                                                    (potentialTranslation.translationVoxel.x() + i)];
                }
            }
        }
        potentialTranslation.covarianceX = resultingIntegral / maximumIntegral;
        potentialTranslation.covarianceY = resultingIntegral / maximumIntegral;

    }
//    double maximumPeak = 0;
//    // find max peak
//    for (auto &potentialTranslation: potentialTranslations) {
//
//        if(maximumPeak<potentialTranslation.peakHeight){
//            maximumPeak = potentialTranslation.peakHeight;
//        }
//    }

    // remove all possible peaks under 1% of maximum peak.
//    std::vector<translationPeak> returnResultsPeaks;
//    for (auto &potentialTranslation: potentialTranslations) {
//        if(maximumPeak*0.01<potentialTranslation.peakHeight){
//            returnResultsPeaks.push_back(potentialTranslation);
//        }
//    }




    return potentialTranslations;

}

Eigen::Matrix4d softDescriptorRegistration::registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],
                                                                             double voxelData2Input[],
                                                                             Eigen::Matrix4d &initialGuess,
                                                                             bool useInitialAngle,
                                                                             bool useInitialTranslation,
                                                                             double cellSize,
                                                                             bool useGauss,
                                                                             bool debug) {


    double goodGuessAlpha = std::atan2(initialGuess(1, 0),
                                       initialGuess(0, 0));


    std::vector<translationPeak> listOfTranslations;
    std::vector<Eigen::Matrix4d> listOfTransformations;
//    std::vector<double> maximumHeightPeakList;
    std::vector<rotationPeak> estimatedAngles;
    if (useInitialAngle) {
        double angleTMP = this->sofftRegistrationVoxel2DRotationOnly(voxelData1Input, voxelData2Input, goodGuessAlpha,
                                                                     debug);
        rotationPeak rotationPeakTMP;
        rotationPeakTMP.angle = angleTMP;
        estimatedAngles.push_back(rotationPeakTMP);

    } else {
        estimatedAngles = this->sofftRegistrationVoxel2DListOfPossibleRotations(voxelData1Input, voxelData2Input,
                                                                                debug);
    }

//    std::cout << "number of possible solutions: " << estimatedAngles.size() << std::endl;

    int angleIndex = 0;
    for (auto &estimatedAngle: estimatedAngles) {

        //copy data
        for (int i = 0; i < N * N; i++) {
            this->voxelData1[i] = voxelData1Input[i];
            this->voxelData2[i] = voxelData2Input[i];
        }

        cv::Mat magTMP1(this->N, this->N, CV_64F, voxelData1);
        cv::Mat magTMP2(this->N, this->N, CV_64F, voxelData2);
        //add gaussian blur
        if (useGauss) {
            for (int i = 0; i < 2; i++) {
                cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
                cv::GaussianBlur(magTMP2, magTMP2, cv::Size(9, 9), 0);
            }
        }

        cv::Point2f pc(magTMP1.cols / 2., magTMP1.rows / 2.);
        //positive values mean COUNTER CLOCK WISE (open cv description) threfore negative rotation
        cv::Mat r = cv::getRotationMatrix2D(pc, estimatedAngle.angle * 180.0 / M_PI, 1.0);
        cv::warpAffine(magTMP1, magTMP1, r, magTMP1.size()); // what size I should use?

        std::vector<translationPeak> potentialTranslations = this->sofftRegistrationVoxel2DTranslationAllPossibleSolutions(
                voxelData1, voxelData2,
                cellSize,
                1.0,
                debug, angleIndex);
        Eigen::Matrix4d estimatedRotationScans = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd rotation_vectorTMP(estimatedAngle.angle, Eigen::Vector3d(0, 0, 1));
        Eigen::Matrix3d tmpRotMatrix3d = rotation_vectorTMP.toRotationMatrix();
        estimatedRotationScans.block<3, 3>(0, 0) = tmpRotMatrix3d;
        translationPeak bestFitTranslation;
        if (useInitialTranslation) {
            double distance = 100000;
            for (auto &potentialTranslation: potentialTranslations) {
                double diffX = potentialTranslation.translationSI.x() - initialGuess(0, 3);
                double diffY = potentialTranslation.translationSI.y() - initialGuess(1, 3);
                if (distance > sqrt(diffX * diffX + diffY * diffY)) {

                    bestFitTranslation = potentialTranslation;
                }
            }
        } else {
            double highestPeak = 0;
            int indexHighestPeak = 0;
            for (int i = 0; i < potentialTranslations.size(); i++) {
                if (potentialTranslations[i].peakHeight > highestPeak) {
                    indexHighestPeak = i;
                    highestPeak = potentialTranslations[i].peakHeight;
                }
            }
            bestFitTranslation = potentialTranslations[indexHighestPeak];
        }

        estimatedRotationScans(0, 3) = bestFitTranslation.translationSI.x();
        estimatedRotationScans(1, 3) = bestFitTranslation.translationSI.y();
        estimatedRotationScans(2, 3) = 0;

        listOfTransformations.push_back(estimatedRotationScans);
        listOfTranslations.push_back(bestFitTranslation);

        angleIndex++;
    }

    //find maximum peak:
    double highestPeak = 0;
    int indexHighestPeak = 0;
    for (int i = 0; i < listOfTransformations.size(); i++) {
        if (highestPeak < listOfTranslations[i].peakHeight) {
            highestPeak = listOfTranslations[i].peakHeight;
            indexHighestPeak = i;
        }
    }


    return listOfTransformations[indexHighestPeak];//should be the transformation matrix from 1 to 2
}

std::vector<transformationPeak>
softDescriptorRegistration::registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],
                                                                      double voxelData2Input[],
                                                                      double cellSize,
                                                                      bool useGauss,
                                                                      bool debug, int registrationNoiseImpactFactor,
                                                                      double ignorePercentageFactor) {


    std::vector<transformationPeak> listOfTransformations;
    std::vector<double> maximumHeightPeakList;
    std::vector<rotationPeak> estimatedAnglePeak;

    estimatedAnglePeak = this->sofftRegistrationVoxel2DListOfPossibleRotations(voxelData1Input, voxelData2Input,
                                                                               debug);


    std::cout << "number of possible solutions: " << estimatedAnglePeak.size() << std::endl;
    for (auto &estimatedAngle: estimatedAnglePeak) {
        std::cout << estimatedAngle.angle << std::endl;
    }

    int angleIndex = 0;
    for (auto &estimatedAngle: estimatedAnglePeak) {

        //copy data
        for (int i = 0; i < N * N; i++) {
            this->voxelData1[i] = voxelData1Input[i];
            this->voxelData2[i] = voxelData2Input[i];
        }

        cv::Mat magTMP1(this->N, this->N, CV_64F, voxelData1);
        cv::Mat magTMP2(this->N, this->N, CV_64F, voxelData2);
        //add gaussian blur
        if (useGauss) {
            for (int i = 0; i < 2; i++) {
                cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
                cv::GaussianBlur(magTMP2, magTMP2, cv::Size(9, 9), 0);
            }
        }

        cv::Point2f pc(magTMP1.cols / 2., magTMP1.rows / 2.);
        //positive values mean COUNTER CLOCK WISE (open cv description) threfore negative rotation
        cv::Mat r = cv::getRotationMatrix2D(pc, estimatedAngle.angle * 180.0 / M_PI, 1.0);
        cv::warpAffine(magTMP1, magTMP1, r, magTMP1.size()); // what size I should use?


        std::vector<translationPeak> potentialTranslations = this->sofftRegistrationVoxel2DTranslationAllPossibleSolutions(
                voxelData1, voxelData2,
                cellSize,
                1.0,
                debug, angleIndex, registrationNoiseImpactFactor,
                ignorePercentageFactor);

        transformationPeak transformationPeakTMP;
        transformationPeakTMP.potentialRotation = estimatedAngle;
        transformationPeakTMP.potentialTranslations = potentialTranslations;
        listOfTransformations.push_back(transformationPeakTMP);
        angleIndex++;
    }

    // save list of transformations
    if (debug) {

        std::ofstream myFile12;
        myFile12.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/dataForReadIn.csv");

        myFile12 << listOfTransformations.size();//number of possible rotations
        myFile12 << "\n";
        for (auto &listOfTransformation: listOfTransformations) {
            myFile12 << listOfTransformation.potentialTranslations.size();//numberOfPossibleTranslations
            myFile12 << "\n";
        }
        myFile12 << cellSize;//numberOfPossibleTranslations
        myFile12 << "\n";
        myFile12.close();

    }
    //save every transformation in files.
    if (debug) {

        int numberOfTransformations = 0;
        for (auto &listOfTransformation: listOfTransformations) {
            Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
            //rotation
            currentMatrix.block<3, 3>(0, 0) = generalHelpfulTools::getQuaternionFromRPY(0, 0,
                                                                                        listOfTransformation.potentialRotation.angle).toRotationMatrix();
            for (auto &potentialTranslation: listOfTransformation.potentialTranslations) {
                //translation
                currentMatrix.block<3, 1>(0, 3) = Eigen::Vector3d(potentialTranslation.translationSI.x(),
                                                                  potentialTranslation.translationSI.y(), 0);

                std::ofstream myFile12;
                myFile12.open(
                        "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/potentialTransformation" +
                        std::to_string(numberOfTransformations) + ".csv");
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        myFile12 << currentMatrix(i, j) << " ";//number of possible rotations
                    }
                    myFile12 << "\n";
                }
                myFile12 << "\n";
                myFile12 << potentialTranslation.peakHeight;
                myFile12 << "\n";
                myFile12 << potentialTranslation.covarianceX;
                myFile12 << "\n";
                myFile12 << potentialTranslation.covarianceY;
                myFile12.close();
                numberOfTransformations++;
            }

        }


    }
    return listOfTransformations;
}

std::vector<translationPeak>
softDescriptorRegistration::peakDetectionOf2DCorrelation(double maximumCorrelation, double cellSize,
                                                         int impactOfNoiseFactor,
                                                         double percentageOfMaxCorrelationIgnored) {

    float impactOfNoise = std::pow(2, impactOfNoiseFactor);

    std::vector<std::vector<int>> xPeaks, yPeaks;

    for (int j = 0; j < this->correlationN; j++) {
        std::vector<float> inputYLine;
        for (int i = 0; i < this->correlationN; i++) {
            inputYLine.push_back((float) resultingCorrelationDouble[j + this->correlationN * i]);
        }
        std::vector<int> out;
        PeakFinder::findPeaks(inputYLine, out, false, impactOfNoise);
//        for(int i = 0 ; i < out.size();i++){
//            std::cout << out[i] << std::endl;
//        }
//        std::cout <<"next"<< std::endl;
        yPeaks.push_back(out);
    }

    for (int i = 0; i < this->correlationN; i++) {
        std::vector<float> inputXLine;
        for (int j = 0; j < this->correlationN; j++) {
            inputXLine.push_back((float) resultingCorrelationDouble[j + this->correlationN * i]);
        }
        std::vector<int> out;
        PeakFinder::findPeaks(inputXLine, out, false, impactOfNoise);
//        for(int j = 0 ; j < out.size();j++){
//            std::cout << out[j] << std::endl;
//        }
//        std::cout <<"next"<< std::endl;
        xPeaks.push_back(out);

    }
    std::vector<translationPeak> potentialTranslations;
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            auto iteratorX = std::find(xPeaks[i].begin(), xPeaks[i].end(), j);
            if (iteratorX != xPeaks[i].end()) {
                auto iteratorY = std::find(yPeaks[j].begin(), yPeaks[j].end(), i);
                if (iteratorY != yPeaks[j].end()) {
//                    std::cout << "found Peak:" << std::endl;
//                    std::cout << *iteratorX << std::endl;
//                    std::cout << *iteratorY  << std::endl;

//                    std::cout << i << std::endl;
//                    std::cout << j  << std::endl;
                    if (maximumCorrelation * percentageOfMaxCorrelationIgnored <
                        resultingCorrelationDouble[j + this->correlationN * i]) {
                        translationPeak tmpTranslationPeak;
                        tmpTranslationPeak.translationSI.x() = -((i - (int) (this->correlationN / 2.0)) * cellSize);
                        tmpTranslationPeak.translationSI.y() = -((j - (int) (this->correlationN / 2.0)) * cellSize);
                        tmpTranslationPeak.translationVoxel.x() = i;
                        tmpTranslationPeak.translationVoxel.y() = j;
                        tmpTranslationPeak.peakHeight = resultingCorrelationDouble[j + this->correlationN * i];
                        potentialTranslations.push_back(tmpTranslationPeak);
                    }

                }
            }
        }
    }
    return potentialTranslations;
}


double softDescriptorRegistration::normalizationFactorCalculation(int x, int y) {

    double tmpCalculation = 0;
    if (x < ceil(this->correlationN / 2)) {
        tmpCalculation = this->correlationN * this->correlationN * (x + 1);
    } else {
        tmpCalculation = this->correlationN * this->correlationN * (this->correlationN - (x + 1) + 1);

    }

    if (y < ceil(this->correlationN / 2)) {
        tmpCalculation = tmpCalculation * (y + 1);
    } else {
        tmpCalculation = tmpCalculation * (this->correlationN - (y + 1) + 1);
    }

    return (tmpCalculation);
}