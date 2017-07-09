#include "StereoCalib.h"
#include "ImageProcess.h"


StereoCalib::StereoCalib(void)
{
}


StereoCalib::~StereoCalib(void)
{
}

/*----------------------------
* ���� : ִ�е�Ŀ������궨
*		 ע��˺��������� calibrateStereoCamera �е��ã�ֻ�ʺ��ⲿ����
*		 ���̽ǵ��ͼ������Ĭ�ϴ���� cornerDatas.imagePoints1 ��
*----------------------------
* ���� : StereoCalib::calibrateSingleCamera
* ���� : public
* ���� : 0 - ����ʧ�ܣ�1 - �����ɹ�
*
* ���� : cornerDatas			[in]	���̽ǵ�����
* ���� : cameraParams			[out]	˫Ŀ�궨����
*/
int StereoCalib::calibrateSingleCamera(CornerDatas& cornerDatas, CameraParams& cameraParams)
{
	cameraParams.imageSize = cornerDatas.imageSize;

	/***
	*	ִ�е�Ŀ����
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
* ���� : ִ��˫Ŀ������궨
*		 ��ÿ���������δ�궨�������Ƚ��е�Ŀ�궨���ٽ���˫Ŀ�궨
*----------------------------
* ���� : StereoCalib::calibrateStereoCamera
* ���� : public
* ���� : 0 - ����ʧ�ܣ�1 - �����ɹ�
*
* ���� : cornerDatas			[in]	���̽ǵ�����
* ���� : stereoParams			[in]	˫Ŀ�궨����
* ���� : cameraUncalibrated		[in]	ÿ��������Ƿ��Ѿ������궨
*/
int StereoCalib::calibrateStereoCamera(CornerDatas& cornerDatas, StereoParams& stereoParams, bool cameraUncalibrated /* = false */)
{
	if (cameraUncalibrated)
	{
		/***
		*	ִ�е�Ŀ����
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
		cout << "1��ģ�����" << endl;
		cout << stereoParams.cameraParams1.cameraMatrix << endl;
		cout << stereoParams.cameraParams1.distortionCoefficients << endl;
		cout << "2��ģ�����" << endl;
		cout << stereoParams.cameraParams2.cameraMatrix << endl;
		cout << stereoParams.cameraParams2.distortionCoefficients << endl;
		/***
		*	���浥Ŀ������������
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

	cout << "������1��ģ�����" << endl;
	cout << stereoParams.cameraParams1.cameraMatrix << endl;
	cout << "������2��ģ�����" << endl;
	cout << stereoParams.cameraParams2.cameraMatrix << endl;

	return 1;
}

/*----------------------------
* ���� : ���㵥Ŀ�궨���
*----------------------------
* ���� : StereoCalib::getCameraCalibrateError
* ���� : public
* ���� : 0 - ����ʧ�ܣ�1 - �����ɹ�
*
* ���� : _objectPoints	[in]	���̽ǵ����������
* ���� : _imagePoints	[in]	���̽ǵ��ͼ������
* ���� : cameraParams	[in]	�궨�����������
* ���� : err			[out]	��Ŀ�궨���
*/
int StereoCalib::getCameraCalibrateError(vector<vector<cv::Point3f> >& _objectPoints, vector<vector<cv::Point2f> >& _imagePoints, CameraParams& cameraParams, double& err)
{
	cv::Mat imagePoints2;
	int totalPoints = 0;
	double totalErr = 0;

	size_t nImages = _objectPoints.size();

	for (int i = 0; i < nImages; i++)
	{
		// ��ȡ��ǰ���̶�Ӧ�Ľǵ������Ӿ���
		vector<cv::Point3f>& objectPoints = _objectPoints[i];
		vector<cv::Point2f>& imagePoints = _imagePoints[i];
		totalPoints += objectPoints.size();

		// ������ͶӰ�������
		projectPoints(
			objectPoints,
			cameraParams.rotations[i],
			cameraParams.translations[i],
			cameraParams.cameraMatrix,
			cameraParams.distortionCoefficients,
			imagePoints2);

		// ������ͶӰ���
		cv::Mat imagePoints1 = cv::Mat(imagePoints);
		double erri = norm(imagePoints1, imagePoints2, cv::NORM_L2);
		totalErr += erri * erri;
	}

	// ƽ������ͶӰ���
	err = std::sqrt(totalErr / totalPoints);

	return 1;
}

/*----------------------------------
* ���� : ���ɵ�������ͷ��У������
*----------------------------------
* ���� : StereoCalib::rectifySingleCamera
* ���� : public
* ���� : 0 - ����ʧ�ܣ�1 - �����ɹ�
*
* ���� : cameraParams	[in]	�궨�����������
* ���� : remapMatrixs	[out]	��ĿУ�����
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
* ���� : ����˫Ŀ�궨���
*----------------------------
* ���� : StereoCalib::getStereoCalibrateError
* ���� : public
* ���� : 0 - ����ʧ�ܣ�1 - �����ɹ�
*
* ���� : cornerDatas	[in]	���̽ǵ�����
* ���� : stereoParams	[in]	˫Ŀ�궨����
* ���� : err			[out]	˫Ŀ�궨���
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
			// ����У��������̽ǵ�����
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
			// ����Լ���
			computeCorrespondEpilines(imgpt[k], k + 1, sterepParams.foundational, epilines[k]);
		}

		// ����Լ������
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

		/***********************************��֤����ϵ��*******************************************/
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

	std::cout << "��ʼ���涨����������������" << endl;

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
	std::cout << "��ɱ���" << endl;
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
//	vector<vector<Point3f> > object_Points;  /****  ���涨����Ͻǵ����ά����   ****/

	for (int t = 0; t < successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < boardSize.height; i++)
		{
			for (int j = 0; j < boardSize.width; j++)
			{
				/* ���趨��������������ϵ��z=0��ƽ���� */
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
* ���� : ִ��˫Ŀ�����У��������˫ĿУ������
*----------------------------
* ���� : StereoCalib::rectifyStereoCamera
* ���� : public
* ���� : 0 - ����ʧ�ܣ�1 - �����ɹ�
*
* ���� : cornerDatas	[in]	���̽ǵ�����
* ���� : stereoParams	[in]	˫Ŀ�궨����
* ���� : remapMatrixs	[out]	˫ĿУ���������
* ���� : method			[in]	˫ĿУ������
*/
int StereoCalib::rectifyStereoCamera(CornerDatas& cornerDatas, StereoParams& stereoParams, RemapMatrixs& remapMatrixs, RECTIFYMETHOD method)
{
	//��ʼ��
	remapMatrixs.mX1 = cv::Mat(stereoParams.imageSize, CV_32FC1);
	remapMatrixs.mY1 = cv::Mat(stereoParams.imageSize, CV_32FC1);
	remapMatrixs.mX2 = cv::Mat(stereoParams.imageSize, CV_32FC1);
	remapMatrixs.mY2 = cv::Mat(stereoParams.imageSize, CV_32FC1);

	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect roi1, roi2;
	if (stereoParams.alpha < 0 || stereoParams.alpha > 1)
		stereoParams.alpha = -1;

	//ִ��˫ĿУ��
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

	//ʹ��HARTLEY�����Ķ��⴦��
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

	//����ͼ��У�����������ӳ�����
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

	//�������
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

	//�������̽ǵ����������ֵ
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

	//��Բ����
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
	//����СԲ ����

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
