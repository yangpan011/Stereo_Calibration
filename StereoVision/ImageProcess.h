#ifndef _IMAGE_PROCESS_H_
#define _IMAGE_PROCESS_H_
#include "gaussianblur.h"

typedef struct _MyArea
{
	int nCx;
	int nCy;
	int nNumber;
}MyArea;

// class  CPoint
// {
// 	int X;
// 	int Y;
// }CPoint;

bool bSaveRawFile(const char* sfilename, unsigned char *pBuffer, int iSize);

int GetImageCornerPoint(unsigned char* pYBuffer, int nWidth, int nHeight, int& nNumSmall, int& nNumBig, MyArea* pAreasSmall, MyArea* pAreasBig);

bool DoubleCameraCheckDots(unsigned char* pYBuffer, int nWidth, int nHeight, int& nNumSmall, int& nNumBig, int bLeftOrRightFlag, MyArea* pAreasSmall, MyArea* pAreasBig);    //双目标定检测所有Mark点位置 

bool ControlSorting(int& nNumSmallSorting, int& nNumBigSorting, MyArea* pAreasSmallSortinginput, MyArea* pAreasBigSortinginput, MyArea* pAreasSmallSortingoutput, MyArea* pAreasBigSortingoutput);

void Histogram(unsigned char* buffer, int buffer_width, int buffer_height, int *Threshold);

void GaussianBlur(unsigned char* buffer, int buffer_width, int buffer_height, int window_size);

void Histogram(unsigned char* buffer, int buffer_width, int buffer_height, int *Threshold);

bool VertFlipBuf(unsigned char *inbuf, int widthBytes, int height);

unsigned char* bBMPFileToYUVData(const char* strPath, int* iWidth, int* iHeight);

void RGB24ToY(unsigned char *pRGB, unsigned char *pY, int width, int height);

bool bSaveRawFile(const char* sfilename, unsigned char *pBuffer, int iSize);

#endif