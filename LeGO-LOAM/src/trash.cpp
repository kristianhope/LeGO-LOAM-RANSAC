/*
bool calculateTransformationCorner(int iterCount){

        int pointSelNum = laserCloudOri->points.size(); // Number of found features for odom
        int sampleNum = 5;    // Number of random samples in RANSAC

        largestInlierSet.reset(new pcl::PointCloud<PointType>()); 
        largestInlierSetCoeffs.reset(new pcl::PointCloud<PointType>());
        smallestOutlierSet.reset(new pcl::PointCloud<PointType>());

        int largestInlierCount = 0;
        float inlierLimit = 0.1;

        // Do RANSAC

        for (int i = 0; i < 2000; i++){

            float sampleTransform[6];
            // Reset sample transform
            for (int p = 0; p < 6; p++) {
                sampleTransform[p] = transformCur[p];
            }

            // Sample random indices
            std::vector<int> selectedIndices;
            while (selectedIndices.size() < sampleNum) {
                int index = std::rand() % pointSelNum;
                if (std::find(selectedIndices.begin(), selectedIndices.end(), index) == selectedIndices.end()) { // find return last element if not found
                    selectedIndices.push_back(index);
                }
            }

            // Create point clouds with random points
            pcl::PointCloud<PointType> ransacSamples;
            ransacSamples.clear();
            pcl::PointCloud<PointType> ransacCoeffs;
            ransacCoeffs.clear();

            // Populate point clouds
            for (int index : selectedIndices) {
                ransacSamples.push_back(laserCloudOri->points[index]);
                ransacCoeffs.push_back(coeffSel->points[index]);
            }
            
            // Solve least squares problem with random points
            cv::Mat matA(sampleNum, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matAt(6, sampleNum, CV_32F, cv::Scalar::all(0));
            cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matB(sampleNum, 1, CV_32F, cv::Scalar::all(0));
            cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
            cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

            float srx = sin(transformCur[0]);
            float crx = cos(transformCur[0]);
            float sry = sin(transformCur[1]);
            float cry = cos(transformCur[1]);
            float srz = sin(transformCur[2]);
            float crz = cos(transformCur[2]);
            float tx = transformCur[3];
            float ty = transformCur[4];
            float tz = transformCur[5];

            float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; 
            float a4 = tx*a1 - ty*a2 - tz*a3; float a5 = srx*srz; float a6 = crz*srx; 
            float a7 = ty*a6 - tz*crx - tx*a5; float a8 = crx*cry*srz; float a9 = crx*cry*crz; 
            float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;
            float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
            float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;
            float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
            float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

            // Calculate A and B matrices for LS
            for (int j = 0; j < sampleNum; j++) {
                pointOri = ransacSamples.points[j];
                coeff = ransacCoeffs.points[j];

                float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                            +(b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;
                float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;
                float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;
                float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
                            + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
                            + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;
                float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
                            + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
                            + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;
                float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

                float d2 = coeff.intensity;

                matA.at<float>(j, 0) = arx;
                matA.at<float>(j, 1) = ary;
                matA.at<float>(j, 2) = arz;
                matA.at<float>(j, 3) = atx;
                matA.at<float>(j, 4) = aty;
                matA.at<float>(j, 5) = atz;
                
                matB.at<float>(j, 0) = -0.05 * d2;
            }

            cv::transpose(matA, matAt);
            matAtA = matAt * matA;
            matAtB = matAt * matB;
            // Solve LS problem
            cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
            
            // Degeneracy check
            if (iterCount == 0) {
                cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
                cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
                cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

                cv::eigen(matAtA, matE, matV);
                matV.copyTo(matV2);

                isDegenerate = false;
                float eignThre[6] = {10, 10, 10, 10, 10, 10};
                for (int i = 5; i >= 0; i--) {
                    if (matE.at<float>(0, i) < eignThre[i]) {
                        for (int j = 0; j < 6; j++) {
                            matV2.at<float>(i, j) = 0;
                        }
                        isDegenerate = true;
                    } else {
                        break;
                    }
                }
                matP = matV.inv() * matV2;
            }
            if (isDegenerate) {
                cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
                matX.copyTo(matX2);
                matX = matP * matX2;
            }

            sampleTransform[0] += matX.at<float>(0, 0);
            sampleTransform[1] += matX.at<float>(1, 0);
            sampleTransform[2] += matX.at<float>(2, 0);
            sampleTransform[3] += matX.at<float>(3, 0);
            sampleTransform[4] += matX.at<float>(4, 0);
            sampleTransform[5] += matX.at<float>(5, 0);
            
            // Inlier check
            int inlierCount = 0;
            pcl::PointCloud<PointType> inlierCloud;
            inlierCloud.clear();
            pcl::PointCloud<PointType> inlierCoeff;
            inlierCoeff.clear();
            pcl::PointCloud<PointType> outlierCloud;
            outlierCloud.clear();

            // Find which points are inliers with this transform
            for (int k = 0; k < pointSelNum; k++) {
                PointType point = laserCloudOri->points[k];
                PointType coeff = coeffSel->points[k];
                PointType transformedPoint;

                float s = 10 * (point.intensity - int(point.intensity));
                float rx = s * sampleTransform[0];
                float ry = s * sampleTransform[1];
                float rz = s * sampleTransform[2];
                float tx = s * sampleTransform[3];
                float ty = s * sampleTransform[4];
                float tz = s * sampleTransform[5];

                // z-x-y rotation 
                // Rotate around z-axis and translate
                float x1 = cos(rz) * (point.x - tx) + sin(rz) * (point.y - ty);
                float y1 = -sin(rz) * (point.x - tx) + cos(rz) * (point.y - ty);
                float z1 = (point.z - tz);
                // Rotate around x-axis
                float x2 = x1;
                float y2 = cos(rx) * y1 + sin(rx) * z1;
                float z2 = -sin(rx) * y1 + cos(rx) * z1;
                // Rotate around y-axis
                transformedPoint.x = cos(ry) * x2 - sin(ry) * z2;
                transformedPoint.y = y2;
                transformedPoint.z = sin(ry) * x2 + cos(ry) * z2;
                transformedPoint.intensity = point.intensity;

                // calculate new point to line distance
                tripod1 = tripod1Cloud->points[k];
                tripod2 = tripod2Cloud->points[k];
                float ld2 = pointToLineDist(transformedPoint, tripod1, tripod2);

                if (ld2 < inlierLimit) {
                    inlierCount += 1;
                    inlierCloud.push_back(point);
                    inlierCoeff.push_back(coeff);
                }
                else {
                    outlierCloud.push_back(point);
                }
            }
            // If new best inlier count
            if (inlierCount > largestInlierCount) {
                largestInlierSet.reset(new pcl::PointCloud<PointType>(inlierCloud)); 
                largestInlierSetCoeffs.reset(new pcl::PointCloud<PointType>(inlierCoeff));
                smallestOutlierSet.reset(new pcl::PointCloud<PointType>(outlierCloud));
                largestInlierCount = inlierCount;
            }
        }  // RANSAC complete

        // Do optimization with the largest inlier set
        int inlierSetSize = largestInlierSet->points.size();
        printf("Inliers : %d/%d \n", inlierSetSize,pointSelNum);

        sensor_msgs::PointCloud2 laserCloudOutMsg;
        pcl::toROSMsg(*largestInlierSet, laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
        laserCloudOutMsg.header.frame_id = "camera";
        pubInlierCloud.publish(laserCloudOutMsg);

        pcl::toROSMsg(*smallestOutlierSet, laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
        laserCloudOutMsg.header.frame_id = "camera";
        pubOutlierCloud.publish(laserCloudOutMsg);

       if (inlierSetSize >= 0.20*pointSelNum) {
            // Solve least squares problem with inliers
            cv::Mat matA(inlierSetSize, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matAt(6, inlierSetSize, CV_32F, cv::Scalar::all(0));
            cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matB(inlierSetSize, 1, CV_32F, cv::Scalar::all(0));
            cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
            cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

            float srx = sin(transformCur[0]);
            float crx = cos(transformCur[0]);
            float sry = sin(transformCur[1]);
            float cry = cos(transformCur[1]);
            float srz = sin(transformCur[2]);
            float crz = cos(transformCur[2]);
            float tx = transformCur[3];
            float ty = transformCur[4];
            float tz = transformCur[5];

            float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; 
            float a4 = tx*a1 - ty*a2 - tz*a3; float a5 = srx*srz; float a6 = crz*srx; 
            float a7 = ty*a6 - tz*crx - tx*a5; float a8 = crx*cry*srz; float a9 = crx*cry*crz; 
            float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;
            float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
            float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;
            float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
            float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

            // Calculate A and B matrices
            for (int i = 0; i < sampleNum; i++) {
                pointOri = largestInlierSet->points[i];
                coeff = largestInlierSetCoeffs->points[i];

                float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                            +(b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;
                float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;
                float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;
                float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
                            + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
                            + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;
                float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
                            + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
                            + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;
                float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

                float d2 = coeff.intensity;

                matA.at<float>(i, 0) = arx;
                matA.at<float>(i, 1) = ary;
                matA.at<float>(i, 2) = arz;
                matA.at<float>(i, 3) = atx;
                matA.at<float>(i, 4) = aty;
                matA.at<float>(i, 5) = atz;
                
                matB.at<float>(i, 0) = -0.05 * d2;
            }

            cv::transpose(matA, matAt);
            matAtA = matAt * matA;
            matAtB = matAt * matB;
            cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
            
            // Degeneracy check
            if (iterCount == 0) {
                cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
                cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
                cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

                cv::eigen(matAtA, matE, matV);
                matV.copyTo(matV2);

                isDegenerate = false;
                float eignThre[6] = {10, 10, 10, 10, 10, 10};
                for (int i = 5; i >= 0; i--) {
                    if (matE.at<float>(0, i) < eignThre[i]) {
                        for (int j = 0; j < 6; j++) {
                            matV2.at<float>(i, j) = 0;
                        }
                        isDegenerate = true;
                    } else {
                        break;
                    }
                }
                matP = matV.inv() * matV2;
            }
            if (isDegenerate) {
                cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
                matX.copyTo(matX2);
                matX = matP * matX2;
            }

            transformCur[0] += matX.at<float>(0, 0);
            transformCur[1] += matX.at<float>(1, 0);
            transformCur[2] += matX.at<float>(2, 0);
            transformCur[3] += matX.at<float>(3, 0);
            transformCur[4] += matX.at<float>(4, 0);
            transformCur[5] += matX.at<float>(5, 0);
            for(int i=0; i<6; i++){
                if(isnan(transformCur[i]))
                    transformCur[i]=0;
            }

            float deltaR = sqrt(
                                pow(rad2deg(matX.at<float>(0, 0)), 2)+
                                pow(rad2deg(matX.at<float>(0, 0)), 2)+
                                pow(rad2deg(matX.at<float>(1, 0)), 2));
            float deltaT = sqrt(
                                pow(matX.at<float>(1, 0) * 100, 2) +
                                pow(matX.at<float>(2, 0) * 100, 2)+
                                pow(matX.at<float>(2, 0) * 100, 2));

            if (deltaR < 0.1 && deltaT < 0.1) {
                return false;
            }
            return true;
        }
        return true;
    }
*/