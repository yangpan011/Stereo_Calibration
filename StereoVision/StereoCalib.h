#ifndef _STEREO_VISION_H_
#define _STEREO_VISION_H_H
#include "ImageProcess.h"
#include "cv.h"
#include <iostream>   
#include <fstream>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
using namespace cv;  //命名空间

using namespace std;
using namespace cv;

class StereoCalib
{
public:
	StereoCalib(void);
	~StereoCalib(void);

	typedef struct _InputStereoParams
	{
		int rNum;                          //[in]mark点行数
		int cNum;                          //[in]mark点列数
		double DX;                         //[in]mark点间水平方向实际距离
		double DY;                         //[in]mark点间垂直方向实际距离
		int ipts;                          //[in]mark点个数
		int iSpts;                         //[in]小圆mark点个数
		int iBpts;                         //[in]大圆mark点个数
		double fHorChipsize;               //[in]水平方向Chip大小单位mm
		double fVerChipsize;			   //[in]垂直方向Chip大小单位mm
		double focallength;				   //[in]焦距单位mm
		double pixsize;                    //[in]像素大小, 单位mm
		double deltH;                      //[in]双层标板之间距离
	}pInpara;

	/***
	*	棋盘角点数据 结构体
	*/
	struct CornerDatas
	{
		int			nPoints;			// 棋盘角点总数
		int			nImageNum;			// 棋盘图像数
		int			nPointsPerImage;	// 每幅棋盘的角点数
		cv::Size	imageSize;			// 图像分辨率
		vector<vector<cv::Point3f> >	objectPoints;	// 棋盘角点世界坐标序列
		vector<vector<cv::Point2f> >	imagePoints1;	// 左视图的棋盘角点像素坐标序列
		vector<vector<cv::Point2f> >	imagePoints2;	// 右视图的棋盘角点像素坐标序列
	};

	/***
	*	单目标定的输出参数
	*/
	struct CameraParams
	{
		cv::Size		imageSize;				// 图像分辨率
		cv::Mat			cameraMatrix;			// 摄像机矩阵
		cv::Mat			distortionCoefficients;	// 摄像机畸变参数
		vector<cv::Mat> rotations;				// 棋盘图片的旋转矩阵
		vector<cv::Mat> translations;			// 棋盘图片的平移向量
		int				flags;					// 单目标定所用的标志位
	};

	/***
	*	双目标定的输出参数
	*/
	struct StereoParams
	{
		cv::Size		imageSize;		// 图像分辨率
		CameraParams	cameraParams1;	// 左摄像机标定参数
		CameraParams	cameraParams2;	// 右摄像机标定参数
		cv::Mat			rotation;		// 旋转矩阵
		cv::Mat			translation;	// 平移向量
		cv::Mat			essential;		// 本质矩阵
		cv::Mat			foundational;	// 基础矩阵
		int				flags;			// 双目标定所用的标志位
		double          alpha;          // 双目校正效果的缩放系数，取值 0~1 或 -1
	};

	/***
	*	双目校正的输出参数
	*/
	struct RemapMatrixs
	{
		cv::Mat		mX1;	// 左视图 X 方向像素映射矩阵
		cv::Mat		mY1;	// 左视图 Y 方向像素映射矩阵
		cv::Mat		mX2;	// 右视图 X 方向像素映射矩阵
		cv::Mat		mY2;	// 右视图 Y 方向像素映射矩阵
		cv::Mat		Q;		// 用于计算三维点云的 Q 矩阵
		cv::Rect	roi1;	// 左视图有效区域的矩形
		cv::Rect	roi2;	// 右视图有效区域的矩形
	};

	enum RECTIFYMETHOD { RECTIFY_BOUGUET, RECTIFY_HARTLEY };
	/*----------------------------
	* 功能 : 执行单目摄像机标定
	*----------------------------
	* 函数 : StereoCalib::calibrateSingleCamera
	* 访问 : public
	* 返回 : 0 - 操作失败，1 - 操作成功
	*
	* 参数 : cornerDatas			[in]	棋盘角点数据
	* 参数 : cameraParams			[out]	双目标定数据
	*/
	int calibrateSingleCamera(CornerDatas& cornerDatas, CameraParams& cameraParams);

	int calibrateStereoCamera(CornerDatas& cornerDatas, StereoParams& stereoParams, bool cameraUncalibrated /* = false */);

	void GetCornerDatas(float* fAllCornerPoint, MyArea* strAreaSmallChoose, MyArea*strAreaBigChoose);

	int initCornerData(int nImageNum, int nPointsPerImage, float* pAllCornerPoint1, float* pAllCornerPoint2, cv::Size imageSize,CornerDatas& cornerDatas);//使用二维数组转换为双指针方法太麻烦

	int getCameraCalibrateError(vector<vector<cv::Point3f> >& _objectPoints, vector<vector<cv::Point2f> >& _imagePoints, CameraParams& cameraParams, double& err);

	int rectifySingleCamera(CameraParams& cameraParams, RemapMatrixs& remapMatrixs);

	int getStereoCalibrateError(CornerDatas& cornerDatas, StereoParams& sterepParams, double& err);

	int rectifyStereoCamera(CornerDatas& cornerDatas, StereoParams& stereoParams, RemapMatrixs& remapMatrixs, RECTIFYMETHOD method);

	int detectCorners(cv::Size boardSize, vector<vector<Mat>>& image_seq, int& successImageNum, StereoCalib::CornerDatas& cornerDatas);

	int CalCorrespondEpilinesErr(CornerDatas& cornerDatas,StereoParams stereoParams, vector<vector<Mat>> image_seq, int successImageNum, double& err_avr);
};

#endif