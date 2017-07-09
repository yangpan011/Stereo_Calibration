#include "stdio.h"
#include "ImageProcess.h"
#include "StereoCalib.h"

#include <iostream>   
#include <fstream>

#include "cv.h"
#include <opencv2/opencv.hpp>

using   namespace   std;
using namespace cv;

int main()
{
	StereoCalib strStereoCalib;
	StereoCalib::CornerDatas cornerDatas;

	std::cout << "开始提取角点………………" << endl;

	/*****************Initialize Parameters****************/
	int image_count = 14;//**** The number of images for each camera****/ 
	cv::Size boardSize(8, 8);  /****    定标板上每行、列的角点数       ****/
	int successImageNum = 0;        /****   成功提取角点的棋盘图对数量   ****/
	int nimages = 0; /****累加总的角点数*****/
//	vector<vector<Mat>> image_seq(2,vector<Mat>(1));
	vector<vector<Mat>> image_seq;
	image_seq.resize(2, vector<Mat>());

	cornerDatas.nImageNum = image_count;
	cornerDatas.nPointsPerImage = 64;
	cornerDatas.imageSize = Size(1280, 960);

	strStereoCalib.detectCorners(boardSize, image_seq, successImageNum, cornerDatas);

	StereoCalib::CameraParams cameraParams;
	StereoCalib::StereoParams stereoParams;

	//双目标定
	stereoParams.cameraParams1.flags = CV_CALIB_FIX_PRINCIPAL_POINT;
	stereoParams.cameraParams2.flags = CV_CALIB_FIX_PRINCIPAL_POINT;
	stereoParams.flags = CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_USE_INTRINSIC_GUESS;
	stereoParams.alpha = 0.5;

	bool needCalibSingleCamera = 1;
	strStereoCalib.calibrateStereoCamera(cornerDatas, stereoParams, needCalibSingleCamera);


	Size imageSize(1280, 960);
	double err_avr = 0;
	int points_total = 0;// (# of points in the individual iamge) * (# of images)
	vector<Point3f> lines[2]; //for the epipolar lines

	Mat intrinsic_matrix_left = stereoParams.cameraParams1.cameraMatrix;
	Mat distortion_coeffs_left = stereoParams.cameraParams1.distortionCoefficients;

	Mat intrinsic_matrix_right = stereoParams.cameraParams2.cameraMatrix;
	Mat distortion_coeffs_right = stereoParams.cameraParams2.distortionCoefficients;

	cv::Mat F = stereoParams.foundational;
	cv::Mat R = stereoParams.rotation;
	cv::Mat T = stereoParams.translation;
	for (int i = 0; i < successImageNum; i++)
	{
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

	/************************************************************************
	RECTIFICATION 06/02/2017
	*************************************************************************/
	Mat R1, R2, P1, P2, Q,
		map11, map12,
		map21, map22;
	Rect validRoi[2];

	cv::stereoRectify(intrinsic_matrix_left, distortion_coeffs_left,
		intrinsic_matrix_right, distortion_coeffs_right,
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	cv::initUndistortRectifyMap(intrinsic_matrix_left, distortion_coeffs_left, R1, P1, imageSize, CV_16SC2, map11, map12);
	cv::initUndistortRectifyMap(intrinsic_matrix_right, distortion_coeffs_right, R2, P2, imageSize, CV_16SC2, map21, map22);

	Mat imageRect_left, imageRect_right;
	cvtColor(image_seq[0][0], imageRect_left, CV_GRAY2BGR);
	cvtColor(image_seq[1][0], imageRect_right, CV_GRAY2BGR);

	remap(imageRect_left, imageRect_left, map11, map12, INTER_LINEAR);
	remap(imageRect_right, imageRect_right, map21, map22, INTER_LINEAR);

	imwrite("rect1.bmp", imageRect_left);
	imwrite("rectr.bmp", imageRect_right);
	/************************************************************************
	保存定标结果
	*************************************************************************/
	bool bsaveYMLResult = 0;
	if (bsaveYMLResult)
	{
		FileStorage fs("intrinsics.yml", FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "Translation" << T << "Rotation" << R;
			fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
			fs.release();
		}
		else
			std::cout << "Error: can not save the Result\n";
	}
	
	
	return 0;
}



int GDcalibration()
{
	int nWidth = 0;
	int nHeight = 0;
	unsigned char* pRGB = NULL;
	StereoCalib strStereoCalib;
	StereoCalib::CornerDatas cornerDatas;
	cv::Size imageSize;
	float fAllImageCornerPoint[2][728] = { 0 };
	int k = 1;//0代表左摄
	for (k = 0; k < 2; k++)
	{
		if (k == 0)
		{
			pRGB = bBMPFileToYUVData("22.bmp", &nWidth, &nHeight);
		}
		else
		{
			pRGB = bBMPFileToYUVData("11.bmp", &nWidth, &nHeight);
		}
		if (!pRGB)
			return 1;
		unsigned char* pYBuffer = (unsigned char*)malloc(nWidth*nHeight*sizeof(unsigned char));
		RGB24ToY(pRGB, pYBuffer, nWidth, nHeight);

		int nNumSmall, nNumBig;
		MyArea strAreaSmall[50], strAreaBig[60];
		DoubleCameraCheckDots(pYBuffer, nWidth, nHeight, nNumSmall, nNumBig, k, strAreaSmall, strAreaBig);

		MyArea strAreaSmallChoose[50], strAreaBigChoose[60];
		ControlSorting(nNumSmall, nNumBig, strAreaSmall, strAreaBig, strAreaSmallChoose, strAreaBigChoose);

		//得到所有角点

		float fAllCornerPoint[728];
		strStereoCalib.GetCornerDatas(fAllCornerPoint, strAreaSmallChoose, strAreaBigChoose);

		memcpy(fAllImageCornerPoint[k], fAllCornerPoint, 728 * sizeof(float));



		Mat iMat(nHeight, nWidth, CV_8UC1, pYBuffer, nWidth);
		imageSize = iMat.size();
#if _DEBUG
		string strIndex;
		for (int i = 0; i < 91; i++)
		{
			ostringstream   ostr;
			ostr << i;
			strIndex = ostr.str();
			cv::putText(iMat, strIndex, cv::Point(fAllImageCornerPoint[k][8 * i + 3], fAllImageCornerPoint[k][8 * i + 4]), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 1, 1);
		}
		namedWindow("效果图");
		imshow("效果图", iMat);
		waitKey();
#endif	

	}
	int nImageNum = 2;
	int nPointsPerImage = 91;
	strStereoCalib.initCornerData(nImageNum, nPointsPerImage, fAllImageCornerPoint[0], fAllImageCornerPoint[1], imageSize, cornerDatas);

	StereoCalib::CameraParams cameraParams;
	StereoCalib::StereoParams stereoParams;
	// 执行单目定标
	//	strStereoCalib.calibrateSingleCamera(cornerDatas, cameraParams);

	//双目标定
	stereoParams.cameraParams1.flags = CV_CALIB_FIX_K3;
	stereoParams.cameraParams2.flags = CV_CALIB_FIX_K3;
	stereoParams.flags = CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_USE_INTRINSIC_GUESS;
	stereoParams.alpha = 0.5;

	bool needCalibSingleCamera = 1;
	strStereoCalib.calibrateStereoCamera(cornerDatas, stereoParams, needCalibSingleCamera);

	cout << stereoParams.translation << endl;
	cout << stereoParams.rotation << endl;

	system("pause");

	// 计算标定误差
	double avgErr = 0;
	strStereoCalib.getStereoCalibrateError(cornerDatas, stereoParams, avgErr);

	return  0;
	// 
	// 	// 执行双目校正
	// 	strStereoCalib.rectifyStereoCamera(cornerDatas, stereoParams, remapMatrixs, optCalib.rectifyMethod);

	//	AfxMessageBox(_T("已完成双目校正"));

	//	strStereoCalib.initObjectCornerData(nImageNum, nPointsPerImage, fAllImageCornerPoint[0], cornerDatas);
}