#include "StereoCalib.h"
#include "ImageProcess.h"


StereoCalib::StereoCalib(void)
{
}


StereoCalib::~StereoCalib(void)
{
}

/*----------------------------
* 功能 : 执行单目摄像机标定
*		 注意此函数不能在 calibrateStereoCamera 中调用，只适合外部调用
*		 棋盘角点的图像坐标默认存放在 cornerDatas.imagePoints1 中
*----------------------------
* 函数 : StereoCalib::calibrateSingleCamera
* 访问 : public
* 返回 : 0 - 操作失败，1 - 操作成功
*
* 参数 : cornerDatas			[in]	棋盘角点数据
* 参数 : cameraParams			[out]	双目标定数据
*/
int StereoCalib::calibrateSingleCamera(CornerDatas& cornerDatas, CameraParams& cameraParams)
{
	cameraParams.imageSize = cornerDatas.imageSize;

	/***
	*	执行单目定标
	*/
	cv::calibrateCamera(
		cornerDatas.objectPoints,
		cornerDatas.imagePoints1,
		cornerDatas.imageSize,
		cameraParams.cameraMatrix,
		cameraParams.distortionCoefficients,
		cameraParams.rotations,
		cameraParams.translations,
		cameraParams.flags
		);

	return 1;
}

/*----------------------------
* 功能 : 执行双目摄像机标定
*		 若每个摄像机尚未标定，则首先进行单目标定，再进行双目标定
*----------------------------
* 函数 : StereoCalib::calibrateStereoCamera
* 访问 : public
* 返回 : 0 - 操作失败，1 - 操作成功
*
* 参数 : cornerDatas			[in]	棋盘角点数据
* 参数 : stereoParams			[in]	双目标定数据
* 参数 : cameraUncalibrated		[in]	每个摄像机是否已经单独标定
*/
int StereoCalib::calibrateStereoCamera(CornerDatas& cornerDatas, StereoParams& stereoParams, bool cameraUncalibrated /* = false */)
{
	if (cameraUncalibrated)
	{
		/***
		*	执行单目定标
		*/
		cv::calibrateCamera(
			cornerDatas.objectPoints,
			cornerDatas.imagePoints1,
			cornerDatas.imageSize,
			stereoParams.cameraParams1.cameraMatrix,
			stereoParams.cameraParams1.distortionCoefficients,
			stereoParams.cameraParams1.rotations,
			stereoParams.cameraParams1.translations,
			stereoParams.cameraParams1.flags
			);

		cv::calibrateCamera(
			cornerDatas.objectPoints,
			cornerDatas.imagePoints2,
			cornerDatas.imageSize,
			stereoParams.cameraParams2.cameraMatrix,
			stereoParams.cameraParams2.distortionCoefficients,
			stereoParams.cameraParams2.rotations,
			stereoParams.cameraParams2.translations,
			stereoParams.cameraParams2.flags
			);
		cout << "1号模组矩阵" << endl;
		cout << stereoParams.cameraParams1.cameraMatrix << endl;
		cout << stereoParams.cameraParams1.distortionCoefficients << endl;
		cout << "2号模组矩阵" << endl;
		cout << stereoParams.cameraParams2.cameraMatrix << endl;
		cout << stereoParams.cameraParams2.distortionCoefficients << endl;
		/***
		*	保存单目定标结果至本地
		*/
// 		saveCameraParams(stereoParams.cameraParams1, (m_workDir + "cameraParams_left.yml").c_str());
// 		saveCameraParams(stereoParams.cameraParams2, (m_workDir + "cameraParams_right.yml").c_str());
	}

	

	stereoParams.imageSize = cornerDatas.imageSize;

	double ret = cv::stereoCalibrate(
		cornerDatas.objectPoints,
		cornerDatas.imagePoints1,
		cornerDatas.imagePoints2,
		stereoParams.cameraParams1.cameraMatrix,
		stereoParams.cameraParams1.distortionCoefficients,
		stereoParams.cameraParams2.cameraMatrix,
		stereoParams.cameraParams2.distortionCoefficients,
		cornerDatas.imageSize,
		stereoParams.rotation,
		stereoParams.translation,
		stereoParams.essential,
		stereoParams.foundational,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6),
		CV_CALIB_USE_INTRINSIC_GUESS
		//CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_FOCAL_LENGTH + CV_CALIB_FIX_K3
//		stereoParams.flags +
//		cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5
		);

	cout << "矫正后1号模组矩阵" << endl;
	cout << stereoParams.cameraParams1.cameraMatrix << endl;
	cout << "矫正后2号模组矩阵" << endl;
	cout << stereoParams.cameraParams2.cameraMatrix << endl;

	return 1;
}

/*----------------------------
* 功能 : 计算单目标定误差
*----------------------------
* 函数 : StereoCalib::getCameraCalibrateError
* 访问 : public
* 返回 : 0 - 操作失败，1 - 操作成功
*
* 参数 : _objectPoints	[in]	棋盘角点的世界坐标
* 参数 : _imagePoints	[in]	棋盘角点的图像坐标
* 参数 : cameraParams	[in]	标定的摄像机参数
* 参数 : err			[out]	单目标定误差
*/
int StereoCalib::getCameraCalibrateError(vector<vector<cv::Point3f> >& _objectPoints, vector<vector<cv::Point2f> >& _imagePoints, CameraParams& cameraParams, double& err)
{
	cv::Mat imagePoints2;
	int totalPoints = 0;
	double totalErr = 0;

	size_t nImages = _objectPoints.size();

	for (int i = 0; i < nImages; i++)
	{
		// 提取当前棋盘对应的角点坐标子矩阵
		vector<cv::Point3f>& objectPoints = _objectPoints[i];
		vector<cv::Point2f>& imagePoints = _imagePoints[i];
		totalPoints += objectPoints.size();

		// 计算重投影点的坐标
		projectPoints(
			objectPoints,
			cameraParams.rotations[i],
			cameraParams.translations[i],
			cameraParams.cameraMatrix,
			cameraParams.distortionCoefficients,
			imagePoints2);

		// 计算重投影误差
		cv::Mat imagePoints1 = cv::Mat(imagePoints);
		double erri = norm(imagePoints1, imagePoints2, cv::NORM_L2);
		totalErr += erri * erri;
	}

	// 平均的重投影误差
	err = std::sqrt(totalErr / totalPoints);

	return 1;
}

/*----------------------------------
* 功能 : 生成单个摄像头的校正矩阵
*----------------------------------
* 函数 : StereoCalib::rectifySingleCamera
* 访问 : public
* 返回 : 0 - 操作失败，1 - 操作成功
*
* 参数 : cameraParams	[in]	标定的摄像机参数
* 参数 : remapMatrixs	[out]	单目校正结果
*/
int StereoCalib::rectifySingleCamera(CameraParams& cameraParams, RemapMatrixs& remapMatrixs)
{
	cv::initUndistortRectifyMap(
		cameraParams.cameraMatrix,
		cameraParams.distortionCoefficients,
		cv::Mat(),
		getOptimalNewCameraMatrix(
		cameraParams.cameraMatrix,
		cameraParams.distortionCoefficients,
		cameraParams.imageSize, 1, cameraParams.imageSize, 0),
		cameraParams.imageSize,
		CV_16SC2,
		remapMatrixs.mX1,
		remapMatrixs.mY1);

	return 1;
}


/*----------------------------
* 功能 : 计算双目标定误差
*----------------------------
* 函数 : StereoCalib::getStereoCalibrateError
* 访问 : public
* 返回 : 0 - 操作失败，1 - 操作成功
*
* 参数 : cornerDatas	[in]	棋盘角点数据
* 参数 : stereoParams	[in]	双目标定数据
* 参数 : err			[out]	双目标定误差
*/

int StereoCalib::getStereoCalibrateError(CornerDatas& cornerDatas, StereoParams& sterepParams, double& err)
{
	vector<cv::Vec3f> epilines[2];
	vector<vector<cv::Point2f> > imagePoints[2];
	cv::Mat cameraMatrix[2], distCoeffs[2];
	int npoints = 0;
	int i, j, k;

	imagePoints[0] = cornerDatas.imagePoints1;
	imagePoints[1] = cornerDatas.imagePoints2;
	cameraMatrix[0] = sterepParams.cameraParams1.cameraMatrix;
	cameraMatrix[1] = sterepParams.cameraParams2.cameraMatrix;
	distCoeffs[0] = sterepParams.cameraParams1.distortionCoefficients;
	distCoeffs[1] = sterepParams.cameraParams2.distortionCoefficients;

	for (i = 0; i < cornerDatas.nImageNum; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		cv::Mat imgpt[2];

		for (k = 0; k < 2; k++)
		{
			imgpt[k] = cv::Mat(imagePoints[k][i]);
			// 计算校正后的棋盘角点坐标
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
			// 计算对极线
			computeCorrespondEpilines(imgpt[k], k + 1, sterepParams.foundational, epilines[k]);
		}

		// 计算对极线误差
		for (j = 0; j < npt; j++)
		{
			double errij =
				fabs(imagePoints[0][i][j].x * epilines[1][j][0] +
				imagePoints[0][i][j].y * epilines[1][j][1] + epilines[1][j][2]) +
				fabs(imagePoints[1][i][j].x * epilines[0][j][0] +
				imagePoints[1][i][j].y * epilines[0][j][1] + epilines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	err /= npoints;

	return 1;
}

int StereoCalib::CalCorrespondEpilinesErr(CornerDatas& cornerDatas,StereoParams stereoParams, vector<vector<Mat>> image_seq, int successImageNum, double& err_avr)
{
	int points_total = 0;// (# of points in the individual iamge) * (# of images)
	vector<Point3f> lines[2]; //for the epipolar lines

	Mat intrinsic_matrix_left = stereoParams.cameraParams1.cameraMatrix;
	Mat distortion_coeffs_left = stereoParams.cameraParams1.distortionCoefficients;

	Mat intrinsic_matrix_right = stereoParams.cameraParams2.cameraMatrix;
	Mat distortion_coeffs_right = stereoParams.cameraParams2.distortionCoefficients;

	cv::Mat F = stereoParams.foundational;
	cv::Mat R = stereoParams.rotation;
	cv::Mat T = stereoParams.translation;
	Mat left;
	Mat right;
	for (int i = 0; i < successImageNum; i++)
	{
		string imageFileName;
		std::stringstream StrStm;
		StrStm << i + 1;
		StrStm >> imageFileName;
		imageFileName += ".bmp";

		int npt = (int)cornerDatas.imagePoints1[i].size(); //# of points in the individual image

		vector<Point2f> & point_left_temp = cornerDatas.imagePoints1[i];
		undistortPoints(point_left_temp, point_left_temp,
			intrinsic_matrix_left, distortion_coeffs_left,
			Mat(), intrinsic_matrix_left);
		computeCorrespondEpilines(point_left_temp, 0 + 1, F, lines[0]);
		
		vector<Point2f> & point_right_temp = cornerDatas.imagePoints2[i];
		undistortPoints(point_right_temp, point_right_temp,
			intrinsic_matrix_right, distortion_coeffs_right,
			Mat(), intrinsic_matrix_right);
		computeCorrespondEpilines(point_right_temp, 1 + 1, F, lines[1]);

		/***********************************验证矫正系数*******************************************/
		/*
		imwrite("DistortRectify\\UnDistortRectify\\left\\" + imageFileName, image_seq[0][i]);
		undistort(image_seq[0][i], left,
			intrinsic_matrix_left, distortion_coeffs_left,
			Mat());
		imwrite("DistortRectify\\DistortRectify\\left\\" + imageFileName, left);

		imwrite("DistortRectify\\UnDistortRectify\\right\\" + imageFileName, image_seq[1][i]);	
		undistort(image_seq[1][i], right,
			intrinsic_matrix_right, distortion_coeffs_right,
			Mat());
		imwrite("DistortRectify\\DistortRectify\\right\\" + imageFileName, right);
		*/

		for (int j = 0; j < npt; j++)
		{
			double err = fabs(point_left_temp[j].x * lines[1][j].x +
				point_left_temp[j].y * lines[1][j].y + lines[1][j].z) +
				fabs(point_right_temp[j].x * lines[0][j].x +
				point_right_temp[j].y * lines[0][j].y + lines[0][j].z);
			err_avr += err;
		}
		points_total += npt;
	}

	err_avr = err_avr / points_total;

	std::cout << "average epipolar line error = " << err_avr << endl;

	std::cout << "开始保存定标结果………………" << endl;

	ofstream fout("F:\\caliberation_result.txt", ios::app);
	fout << "Left Intrinsic Matrix:" << endl;
	fout << intrinsic_matrix_left << endl;
	fout << "Left DIstortion Coefficient" << endl;
	fout << distortion_coeffs_left << endl;

	fout << "Right Intrinsic Matrix:" << endl;
	fout << intrinsic_matrix_right << endl;
	fout << "Right DIstortion Coefficient" << endl;
	fout << distortion_coeffs_right << endl;

	fout << "Rotation Vector:" << endl;
	fout << stereoParams.rotation << endl;
	fout << "Translation Vector:" << endl;
	fout << stereoParams.translation << endl;
	// 	fout << "RMS Error:" << endl;
	// 	fout << rms << endl;
	fout << "Epipolar error:" << endl;
	fout << err_avr << endl;
	std::cout << "完成保存" << endl;
	fout << endl;
	fout.close();

	return 0;
}

int StereoCalib::detectCorners(cv::Size boardSize, vector<vector<Mat>>& image_seq, int& successImageNum, StereoCalib::CornerDatas& cornerDatas)
{
	
	int nImageCount = cornerDatas.nImageNum;

	for (int i = 0; i != nImageCount; i++)
	{
		cout << "Left Frame #" << i + 1 << "..." << endl;
		cout << "Right Frame #" << i + 1 << "..." << endl;
		/******Load the Image******/
		string imageFileName;
		std::stringstream StrStm;
		StrStm << i + 1;
		StrStm >> imageFileName;
		imageFileName += ".bmp";
		string strLeftImagePath = "SourceImage\\left\\" + imageFileName;
		string strRightImagePath = "SourceImage\\right\\" + imageFileName;
		Mat image_left = imread(strLeftImagePath);
		if (image_left.empty())
		{
			cout << "Cannot open the left images" << endl;
			continue;
		}
		Mat image_right = imread(strRightImagePath);
		if (image_left.empty())
		{
			cout << "Cannot open the right images" << endl;
			continue;
		}
		Mat imageGray_left, imageGray_right;
		cvtColor(image_left, imageGray_left, COLOR_BGR2GRAY);
		cvtColor(image_right, imageGray_right, COLOR_BGR2GRAY);

		/*******Find the Corner*******/
		bool found_l = false, found_r = false;
		vector<Point2f>corners_l, corners_r; //a tempary container for the individual frame
		//Left Camera
		found_l = findChessboardCorners(imageGray_left, boardSize, corners_l,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		if (!found_l)
		{
			cout << "can not find chessboard corners!\n";
			continue;
			exit(1);
		}
		else
		{
			cout << "find the left chessboard corners!\n";
			cornerSubPix(imageGray_left, corners_l, Size(11, 11),
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		}

		//Right Camera
		found_r = findChessboardCorners(imageGray_right, boardSize, corners_r,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		if (!found_r)
		{
			cout << "can not find chessboard corners!\n";
			continue;
			exit(1);
		}
		else
		{
			cout << "find the right chessboard corners!\n";
			cornerSubPix(imageGray_right, corners_r, Size(11, 11),
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		}

		//put the calculated images in the sequence
		bool bgetImageSeq = true;
		if (bgetImageSeq)
		{
			image_seq[0].push_back(imageGray_left);
			image_seq[1].push_back(imageGray_right);
		}

		//Draw and label the corners
		if (found_l && found_r)
		{
			drawChessboardCorners(image_left, boardSize, corners_l, found_l);
			drawChessboardCorners(image_right, boardSize, corners_r, found_r);

			imwrite("SaveImage\\left" + imageFileName, image_left);
			imwrite("SaveImage\\right" + imageFileName, image_right);
			cout << "Find the #" << i + 1 << " pairs" << endl;
			//Group the corners
			cornerDatas.imagePoints1.push_back(corners_l);
			cornerDatas.imagePoints2.push_back(corners_r);
			successImageNum = successImageNum + 1;
		}
	}

	/*******INFORMATION OF THE CORNERS ON THE CHESSBOAR******/
	Size2f squareSize = Size2f(60.f, 60.f);  // Set this to the actual square size
//	vector<vector<Point3f> > object_Points;  /****  保存定标板上角点的三维坐标   ****/

	for (int t = 0; t < successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < boardSize.height; i++)
		{
			for (int j = 0; j < boardSize.width; j++)
			{
				/* 假设定标板放在世界坐标系中z=0的平面上 */
				Point3d tempPoint;
				tempPoint.x = i * squareSize.width;
				tempPoint.y = j * squareSize.height;
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);//For the individual image
			}
		}
		cornerDatas.objectPoints.push_back(tempPointSet);//For all images
	}

	return 0;
}
/*----------------------------
* 功能 : 执行双目摄像机校正，生成双目校正数据
*----------------------------
* 函数 : StereoCalib::rectifyStereoCamera
* 访问 : public
* 返回 : 0 - 操作失败，1 - 操作成功
*
* 参数 : cornerDatas	[in]	棋盘角点数据
* 参数 : stereoParams	[in]	双目标定数据
* 参数 : remapMatrixs	[out]	双目校正结果数据
* 参数 : method			[in]	双目校正方法
*/
int StereoCalib::rectifyStereoCamera(CornerDatas& cornerDatas, StereoParams& stereoParams, RemapMatrixs& remapMatrixs, RECTIFYMETHOD method)
{
	//初始化
	remapMatrixs.mX1 = cv::Mat(stereoParams.imageSize, CV_32FC1);
	remapMatrixs.mY1 = cv::Mat(stereoParams.imageSize, CV_32FC1);
	remapMatrixs.mX2 = cv::Mat(stereoParams.imageSize, CV_32FC1);
	remapMatrixs.mY2 = cv::Mat(stereoParams.imageSize, CV_32FC1);

	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect roi1, roi2;
	if (stereoParams.alpha < 0 || stereoParams.alpha > 1)
		stereoParams.alpha = -1;

	//执行双目校正
	stereoRectify(
		stereoParams.cameraParams1.cameraMatrix,
		stereoParams.cameraParams1.distortionCoefficients,
		stereoParams.cameraParams2.cameraMatrix,
		stereoParams.cameraParams2.distortionCoefficients,
		stereoParams.imageSize,
		stereoParams.rotation,
		stereoParams.translation,
		R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY,
		stereoParams.alpha,
		stereoParams.imageSize,
		&roi1, &roi2);

	//使用HARTLEY方法的额外处理
	if (method == RECTIFY_HARTLEY)
	{
		cv::Mat F, H1, H2;
		F = findFundamentalMat(
			cornerDatas.imagePoints1,
			cornerDatas.imagePoints2,
			cv::FM_8POINT, 0, 0);
		stereoRectifyUncalibrated(
			cornerDatas.imagePoints1,
			cornerDatas.imagePoints2,
			F, stereoParams.imageSize, H1, H2, 3);

		R1 = stereoParams.cameraParams1.cameraMatrix.inv() * H1 * stereoParams.cameraParams1.cameraMatrix;
		R2 = stereoParams.cameraParams2.cameraMatrix.inv() * H2 * stereoParams.cameraParams2.cameraMatrix;
		P1 = stereoParams.cameraParams1.cameraMatrix;
		P2 = stereoParams.cameraParams2.cameraMatrix;
	}

	//生成图像校正所需的像素映射矩阵
	initUndistortRectifyMap(
		stereoParams.cameraParams1.cameraMatrix,
		stereoParams.cameraParams1.distortionCoefficients,
		R1, P1,
		stereoParams.imageSize,
		CV_16SC2,
		remapMatrixs.mX1, remapMatrixs.mY1);

	initUndistortRectifyMap(
		stereoParams.cameraParams2.cameraMatrix,
		stereoParams.cameraParams2.distortionCoefficients,
		R2, P2,
		stereoParams.imageSize,
		CV_16SC2,
		remapMatrixs.mX2, remapMatrixs.mY2);

	//输出数据
	Q.copyTo(remapMatrixs.Q);
	remapMatrixs.roi1 = roi1;
	remapMatrixs.roi2 = roi2;

	return 1;
}

int StereoCalib::initCornerData(int nImageNum, int nPointsPerImage, float* pAllCornerPoint1, float* pAllCornerPoint2, cv::Size imageSize,CornerDatas& cornerDatas)
{
	
	cornerDatas.nImageNum = nImageNum;
	cornerDatas.nPointsPerImage = nPointsPerImage;
	cornerDatas.imageSize = imageSize;
	cornerDatas.objectPoints.resize(nImageNum, vector<cv::Point3f>(cornerDatas.nPointsPerImage, cv::Point3f(0, 0, 0)));
	cornerDatas.imagePoints1.resize(nImageNum, vector<cv::Point2f>(cornerDatas.nPointsPerImage, cv::Point2f(0, 0)));
	cornerDatas.imagePoints2.resize(nImageNum, vector<cv::Point2f>(cornerDatas.nPointsPerImage, cv::Point2f(0, 0)));

	//计算棋盘角点的世界坐标值
	int i, j, k, n;
	for (i = 0; i < nImageNum; i++)
	{
		if (i == 0)
		{
			for (n = 0; n < 52; n++)
			{
				cornerDatas.objectPoints[i][n] = cv::Point3f(pAllCornerPoint1[8 * n], pAllCornerPoint1[8 * n + 1], 0);
				cornerDatas.imagePoints1[i][n] = cv::Point2f(pAllCornerPoint1[8 * n + 3], pAllCornerPoint1[8 * n + 4]);
				cornerDatas.imagePoints2[i][n] = cv::Point2f(pAllCornerPoint2[8 * n + 3], pAllCornerPoint2[8 * n + 4]);
			}
		}
		else
		{
			for (n = 52; n < 39+52; n++)
			{
				cornerDatas.objectPoints[i][n - 52] = cv::Point3f(pAllCornerPoint1[8 * n], pAllCornerPoint1[8 * n + 1],110);
				cornerDatas.imagePoints1[i][n - 52] = cv::Point2f(pAllCornerPoint1[8 * n + 3], pAllCornerPoint1[8 * n + 4]);
				cornerDatas.imagePoints2[i][n - 52] = cv::Point2f(pAllCornerPoint2[8 * n + 3], pAllCornerPoint2[8 * n + 4]);
			}
		}		
	}
	
	return 1;
}

void StereoCalib::GetCornerDatas(float* fAllCornerPoint, MyArea* strAreaSmallChoose, MyArea*strAreaBigChoose)
{

	//大圆坐标
	for (int i = 0; i < 52; i++)
	{
		if ((i >= 0) && (i <= 8))
		{
			fAllCornerPoint[8 * i] = i * 55;
			fAllCornerPoint[8 * i + 1] = 0;
			fAllCornerPoint[8 * i + 2] = 0;
			fAllCornerPoint[8 * i + 3] = strAreaBigChoose[i].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaBigChoose[i].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;

		}
		else if ((i >= 9) && (i <= 15))
		{
			fAllCornerPoint[8 * i] = 0;
			fAllCornerPoint[8 * i + 1] = (i - 8) * 55;
			fAllCornerPoint[8 * i + 2] = 0;
			fAllCornerPoint[8 * i + 3] = strAreaBigChoose[i].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaBigChoose[i].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
		else if ((i >= 16) && (i <= 20))
		{
			fAllCornerPoint[8 * i] = 110;
			fAllCornerPoint[8 * i + 1] = (i - 14) * 55;
			fAllCornerPoint[8 * i + 2] = 0;
			fAllCornerPoint[8 * i + 3] = strAreaBigChoose[i].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaBigChoose[i].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
		else if ((i >= 21) && (i <= 25))
		{
			fAllCornerPoint[8 * i] = 165;
			fAllCornerPoint[8 * i + 1] = (i - 19) * 55;
			fAllCornerPoint[8 * i + 2] = 0;
			fAllCornerPoint[8 * i + 3] = strAreaBigChoose[i].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaBigChoose[i].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
		else if ((i >= 26) && (i <= 30))
		{
			fAllCornerPoint[8 * i] = 275;
			fAllCornerPoint[8 * i + 1] = (i - 24) * 55;
			fAllCornerPoint[8 * i + 2] = 0;
			fAllCornerPoint[8 * i + 3] = strAreaBigChoose[i].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaBigChoose[i].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
		else if ((i >= 31) && (i <= 35))
		{
			fAllCornerPoint[8 * i] = 330;
			fAllCornerPoint[8 * i + 1] = (i - 29) * 55;
			fAllCornerPoint[8 * i + 2] = 0;
			fAllCornerPoint[8 * i + 3] = strAreaBigChoose[i].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaBigChoose[i].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
		else if ((i >= 36) && (i <= 42))
		{
			fAllCornerPoint[8 * i] = 440;
			fAllCornerPoint[8 * i + 1] = (i - 35) * 55;
			fAllCornerPoint[8 * i + 2] = 0;
			fAllCornerPoint[8 * i + 3] = strAreaBigChoose[i].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaBigChoose[i].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
		else
		{
			fAllCornerPoint[8 * i] = (i - 43) * 55;
			fAllCornerPoint[8 * i + 1] = 440;
			fAllCornerPoint[8 * i + 2] = 0;
			fAllCornerPoint[8 * i + 3] = strAreaBigChoose[i].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaBigChoose[i].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
	}
	//增加小圆 坐标

	for (int i = 52; i < 91; i++)
	{

		if ((i >= 52) && (i <= 60))
		{
			fAllCornerPoint[8 * i] = (i - 52) * 55;
			fAllCornerPoint[8 * i + 1] = 0;
			fAllCornerPoint[8 * i + 2] = 110;
			fAllCornerPoint[8 * i + 3] = strAreaSmallChoose[i - 52].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaSmallChoose[i - 52].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;

		}
		else if ((i >= 61) && (i <= 67))
		{
			fAllCornerPoint[8 * i] = 0;
			fAllCornerPoint[8 * i + 1] = (i - 60) * 55;
			fAllCornerPoint[8 * i + 2] = 110;
			fAllCornerPoint[8 * i + 3] = strAreaSmallChoose[i - 52].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaSmallChoose[i - 52].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
		else if ((i >= 68) && (i <= 74))
		{
			fAllCornerPoint[8 * i] = 220;
			fAllCornerPoint[8 * i + 1] = (i - 67) * 55;
			fAllCornerPoint[8 * i + 2] = 110;
			fAllCornerPoint[8 * i + 3] = strAreaSmallChoose[i - 52].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaSmallChoose[i - 52].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}
		else if ((i >= 75) && (i <= 81))
		{
			fAllCornerPoint[8 * i] = 440;
			fAllCornerPoint[8 * i + 1] = (i - 74) * 55;
			fAllCornerPoint[8 * i + 2] = 110;
			fAllCornerPoint[8 * i + 3] = strAreaSmallChoose[i - 52].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaSmallChoose[i - 52].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;

		}
		else
		{
			fAllCornerPoint[8 * i] = (i - 82) * 55;
			fAllCornerPoint[8 * i + 1] = 440;
			fAllCornerPoint[8 * i + 2] = 110;
			fAllCornerPoint[8 * i + 3] = strAreaSmallChoose[i - 52].nCx;
			fAllCornerPoint[8 * i + 4] = strAreaSmallChoose[i - 52].nCy;
			fAllCornerPoint[8 * i + 5] = 0;
			fAllCornerPoint[8 * i + 6] = 0;
			fAllCornerPoint[8 * i + 7] = 1;
		}

	}
}
