#ifndef _STEREO_VISION_H_
#define _STEREO_VISION_H_H
#include "ImageProcess.h"
#include "cv.h"
#include <iostream>   
#include <fstream>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
using namespace cv;  //�����ռ�

using namespace std;
using namespace cv;

class StereoCalib
{
public:
	StereoCalib(void);
	~StereoCalib(void);

	typedef struct _InputStereoParams
	{
		int rNum;                          //[in]mark������
		int cNum;                          //[in]mark������
		double DX;                         //[in]mark���ˮƽ����ʵ�ʾ���
		double DY;                         //[in]mark��䴹ֱ����ʵ�ʾ���
		int ipts;                          //[in]mark�����
		int iSpts;                         //[in]СԲmark�����
		int iBpts;                         //[in]��Բmark�����
		double fHorChipsize;               //[in]ˮƽ����Chip��С��λmm
		double fVerChipsize;			   //[in]��ֱ����Chip��С��λmm
		double focallength;				   //[in]���൥λmm
		double pixsize;                    //[in]���ش�С, ��λmm
		double deltH;                      //[in]˫����֮�����
	}pInpara;

	/***
	*	���̽ǵ����� �ṹ��
	*/
	struct CornerDatas
	{
		int			nPoints;			// ���̽ǵ�����
		int			nImageNum;			// ����ͼ����
		int			nPointsPerImage;	// ÿ�����̵Ľǵ���
		cv::Size	imageSize;			// ͼ��ֱ���
		vector<vector<cv::Point3f> >	objectPoints;	// ���̽ǵ�������������
		vector<vector<cv::Point2f> >	imagePoints1;	// ����ͼ�����̽ǵ�������������
		vector<vector<cv::Point2f> >	imagePoints2;	// ����ͼ�����̽ǵ�������������
	};

	/***
	*	��Ŀ�궨���������
	*/
	struct CameraParams
	{
		cv::Size		imageSize;				// ͼ��ֱ���
		cv::Mat			cameraMatrix;			// ���������
		cv::Mat			distortionCoefficients;	// ������������
		vector<cv::Mat> rotations;				// ����ͼƬ����ת����
		vector<cv::Mat> translations;			// ����ͼƬ��ƽ������
		int				flags;					// ��Ŀ�궨���õı�־λ
	};

	/***
	*	˫Ŀ�궨���������
	*/
	struct StereoParams
	{
		cv::Size		imageSize;		// ͼ��ֱ���
		CameraParams	cameraParams1;	// ��������궨����
		CameraParams	cameraParams2;	// ��������궨����
		cv::Mat			rotation;		// ��ת����
		cv::Mat			translation;	// ƽ������
		cv::Mat			essential;		// ���ʾ���
		cv::Mat			foundational;	// ��������
		int				flags;			// ˫Ŀ�궨���õı�־λ
		double          alpha;          // ˫ĿУ��Ч��������ϵ����ȡֵ 0~1 �� -1
	};

	/***
	*	˫ĿУ�����������
	*/
	struct RemapMatrixs
	{
		cv::Mat		mX1;	// ����ͼ X ��������ӳ�����
		cv::Mat		mY1;	// ����ͼ Y ��������ӳ�����
		cv::Mat		mX2;	// ����ͼ X ��������ӳ�����
		cv::Mat		mY2;	// ����ͼ Y ��������ӳ�����
		cv::Mat		Q;		// ���ڼ�����ά���Ƶ� Q ����
		cv::Rect	roi1;	// ����ͼ��Ч����ľ���
		cv::Rect	roi2;	// ����ͼ��Ч����ľ���
	};

	enum RECTIFYMETHOD { RECTIFY_BOUGUET, RECTIFY_HARTLEY };
	/*----------------------------
	* ���� : ִ�е�Ŀ������궨
	*----------------------------
	* ���� : StereoCalib::calibrateSingleCamera
	* ���� : public
	* ���� : 0 - ����ʧ�ܣ�1 - �����ɹ�
	*
	* ���� : cornerDatas			[in]	���̽ǵ�����
	* ���� : cameraParams			[out]	˫Ŀ�궨����
	*/
	int calibrateSingleCamera(CornerDatas& cornerDatas, CameraParams& cameraParams);

	int calibrateStereoCamera(CornerDatas& cornerDatas, StereoParams& stereoParams, bool cameraUncalibrated /* = false */);

	void GetCornerDatas(float* fAllCornerPoint, MyArea* strAreaSmallChoose, MyArea*strAreaBigChoose);

	int initCornerData(int nImageNum, int nPointsPerImage, float* pAllCornerPoint1, float* pAllCornerPoint2, cv::Size imageSize,CornerDatas& cornerDatas);//ʹ�ö�ά����ת��Ϊ˫ָ�뷽��̫�鷳

	int getCameraCalibrateError(vector<vector<cv::Point3f> >& _objectPoints, vector<vector<cv::Point2f> >& _imagePoints, CameraParams& cameraParams, double& err);

	int rectifySingleCamera(CameraParams& cameraParams, RemapMatrixs& remapMatrixs);

	int getStereoCalibrateError(CornerDatas& cornerDatas, StereoParams& sterepParams, double& err);

	int rectifyStereoCamera(CornerDatas& cornerDatas, StereoParams& stereoParams, RemapMatrixs& remapMatrixs, RECTIFYMETHOD method);

	int detectCorners(cv::Size boardSize, vector<vector<Mat>>& image_seq, int& successImageNum, StereoCalib::CornerDatas& cornerDatas);

	int CalCorrespondEpilinesErr(CornerDatas& cornerDatas,StereoParams stereoParams, vector<vector<Mat>> image_seq, int successImageNum, double& err_avr);
};

#endif