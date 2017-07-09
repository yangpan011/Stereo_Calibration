#include "ImageProcess.h"
#include <string.h>
#include <stack> 
#include "stdlib.h"
#include <malloc.h>
#include "windows.h"
#include <atltypes.h>

using namespace std;

bool DoubleCameraCheckDots(unsigned char* pYBuffer, int nWidth, int nHeight, int& nNumSmall, int& nNumBig, int bLeftOrRightFlag, MyArea* pAreasSmall, MyArea* pAreasBig)
{
	if (pYBuffer == NULL || pAreasSmall == NULL || pAreasBig == NULL)
		return false;
	nNumBig = 0;
	nNumSmall = 0;
	unsigned char* pBinaryImage = (unsigned char*)malloc(nWidth * nHeight * sizeof(unsigned char));
	if (pBinaryImage == NULL)
		return false;
	memset(pBinaryImage, 255, nWidth * nHeight * sizeof(unsigned char));
	//二值化
	int nThreshold;
	GaussianBlur(pYBuffer, nWidth, nHeight, 13);
	Histogram(pYBuffer, nWidth, nHeight, &nThreshold);
	
	if (bLeftOrRightFlag == 0)//左摄
	{
		nThreshold = nThreshold - 20;
		int top = 80, bottom = 850, left = 150, right = 930;
		for (int i = top; i < bottom; ++i)          //纵坐标
		{
			for (int j = left; j < right; ++j)    //横坐标
			{
				if (pYBuffer[i * nWidth + j] < nThreshold)
					pBinaryImage[i * nWidth + j] = 0;
				else
					pBinaryImage[i * nWidth + j] = 255;
			}
		}

		int x1 = left;
		int y1 = top;
		int x2 = right;
		int y2 = bottom;
		bSaveRawFile("leftbinary.raw", pBinaryImage, nWidth*nHeight);
	}
	else
	{
		nThreshold = nThreshold - 35;
		int top = 80, bottom = 870, left = 350, right = 1100;
		for (int i = top; i < bottom; ++i)          //纵坐标
		{
			for (int j = left; j < right; ++j)    //横坐标
			{
				if (pYBuffer[i * nWidth + j] < nThreshold)
					pBinaryImage[i * nWidth + j] = 0;
				else
					pBinaryImage[i * nWidth + j] = 255;
			}
		}
		int x1 = left;
		int y1 = top;
		int x2 = right;
		int y2 = bottom;
		bSaveRawFile("rightbinary.raw", pBinaryImage, nWidth*nHeight);
	}
	unsigned char* pFlagImage = (unsigned char*)malloc(nWidth * nHeight * sizeof(unsigned char));        //标记像素有没有遍历过
	memset(pFlagImage, 0, nWidth * nHeight * sizeof(unsigned char));

	// 找连通域
	stack<CPoint> sPosition;                     // 记录像素点的坐标
	int nPointNumber = 0;                        // 连通域的面积
	int nLuxSum = 0;                             //  一个连通域的总亮度
	int nXFactorLux = 0, nYFactorLux = 0;         //  一个连通域中，以亮度为权重的亮度和，为了求该连通域的亮度中心坐标
	for (int i = 0; i < nHeight; ++i)
	{
		for (int j = 0; j < nWidth; ++j)
		{
			if (pFlagImage[i * nWidth + j] != 1 && pBinaryImage[i * nWidth + j] == 0)
			{
				sPosition.push(CPoint(j, i));
				pFlagImage[i * nWidth + j] = 1;
				nPointNumber = nLuxSum = nXFactorLux = nYFactorLux = 0;
				while (!sPosition.empty())
				{
					CPoint cCurrentPoint = sPosition.top();
					int nX = cCurrentPoint.x, nY = cCurrentPoint.y;
					if (pFlagImage[nY * nWidth + nX - 1] != 1 && pBinaryImage[nY * nWidth + nX - 1] == 0)                     //判断左边的点
					{
						sPosition.push(CPoint(nX - 1, nY));
						pFlagImage[nY * nWidth + nX - 1] = 1;
					}
					else if (pFlagImage[(nY - 1) * nWidth + nX - 1] != 1 && pBinaryImage[(nY - 1) * nWidth + nX - 1] == 0)        //判断左上方的点
					{
						sPosition.push(CPoint(nX - 1, nY - 1));
						pFlagImage[(nY - 1) * nWidth + nX - 1] = 1;
					}
					else if (pFlagImage[(nY - 1) * nWidth + nX] != 1 && pBinaryImage[(nY - 1) * nWidth + nX] == 0)                     //判断上边的点
					{
						sPosition.push(CPoint(nX, nY - 1));
						pFlagImage[(nY - 1) * nWidth + nX] = 1;
					}
					else if (pFlagImage[(nY - 1) * nWidth + nX + 1] != 1 && pBinaryImage[(nY - 1) * nWidth + nX + 1] == 0)             //判断右上方的点
					{
						sPosition.push(CPoint(nX + 1, nY - 1));
						pFlagImage[(nY - 1) * nWidth + nX + 1] = 1;
					}
					else if (pFlagImage[nY * nWidth + nX + 1] != 1 && pBinaryImage[nY * nWidth + nX + 1] == 0)                     //判断右边的点
					{
						sPosition.push(CPoint(nX + 1, nY));
						pFlagImage[nY * nWidth + nX + 1] = 1;
					}
					else if (pFlagImage[(nY + 1) * nWidth + nX + 1] != 1 && pBinaryImage[(nY + 1) * nWidth + nX + 1] == 0)    //判断右下角的点
					{
						sPosition.push(CPoint(nX + 1, nY + 1));
						pFlagImage[(nY + 1) * nWidth + nX + 1] = 1;
					}
					else if (pFlagImage[(nY + 1) * nWidth + nX] != 1 && pBinaryImage[(nY + 1) * nWidth + nX] == 0)             //判断下边的点
					{
						sPosition.push(CPoint(nX, nY + 1));
						pFlagImage[(nY + 1) * nWidth + nX] = 1;
					}
					else if (pFlagImage[(nY + 1) * nWidth + nX - 1] != 1 && pBinaryImage[(nY + 1) * nWidth + nX - 1] == 0)     //判断左下角的点
					{
						sPosition.push(CPoint(nX - 1, nY + 1));
						pFlagImage[(nY + 1) * nWidth + nX - 1] = 1;
					}
					else
					{
						sPosition.pop();
						nPointNumber++;
						nLuxSum += pYBuffer[nY * nWidth + nX];
						nXFactorLux += pYBuffer[nY * nWidth + nX] * nX;
						nYFactorLux += pYBuffer[nY * nWidth + nX] * nY;
					}
				}
				if (nPointNumber > 420)                  //    区分大小圆的标准
				{
					pAreasBig[nNumBig].nCx = nXFactorLux / nLuxSum;
					pAreasBig[nNumBig].nCy = nYFactorLux / nLuxSum;
					pAreasBig[nNumBig].nNumber = nPointNumber;
					nNumBig++;
				}
				else
				{
					pAreasSmall[nNumSmall].nCx = nXFactorLux / nLuxSum;
					pAreasSmall[nNumSmall].nCy = nYFactorLux / nLuxSum;
					pAreasSmall[nNumSmall].nNumber = nPointNumber;
					nNumSmall++;
				}
			}
		}
	}
	return true;
}

bool ControlSorting(int& nNumSmallSorting, int& nNumBigSorting, MyArea* pAreasSmallSortinginput, MyArea* pAreasBigSortinginput,
	MyArea* pAreasSmallSortingoutput, MyArea* pAreasBigSortingoutput)
{

	//寻找大圆全部排序
	int temp;
	for (int i = 0; i < nNumBigSorting - 1; i++)
	{
		for (int j = 0; j < nNumBigSorting - 1 - i; j++)
		{

			if (pAreasBigSortinginput[j].nCy > pAreasBigSortinginput[j + 1].nCy)
			{
				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;

				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;
			}
		}
	}

	//获取前9个大园，y坐标最小，x坐标依次排序
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8 - i; j++)
		{
			if (pAreasBigSortinginput[j].nCx > pAreasBigSortinginput[j + 1].nCx)
			{
				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;

				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;
			}
		}
	}
	//输出坐标结果
	for (int i = 0; i < 9; i++)
	{
		pAreasBigSortingoutput[i].nCx = pAreasBigSortinginput[i].nCx;
		pAreasBigSortingoutput[i].nCy = pAreasBigSortinginput[i].nCy;
	}

	///获取前9个大园，y坐标最大，x坐标依次排序
	for (int i = 43; i < 51; i++)
	{
		for (int j = 43; j < 94 - i; j++)
		{
			if (pAreasBigSortinginput[j].nCx > pAreasBigSortinginput[j + 1].nCx)
			{
				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;


				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;

			}
		}
	}

	//输出坐标结果
	for (int i = 43; i < 52; i++)
	{
		pAreasBigSortingoutput[i].nCx = pAreasBigSortinginput[i].nCx;
		pAreasBigSortingoutput[i].nCy = pAreasBigSortinginput[i].nCy;
	}

	//////剩余34个大圆点排序
	for (int i = 9; i < 42; i++)
	{
		for (int j = 9; j < 51  - i; j++)
		{
			if (pAreasBigSortinginput[j].nCx > pAreasBigSortinginput[j + 1].nCx)
			{
				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;


				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;

			}
		}
	}

	//获取34个大圆中7个大圆，x最小，y依次增大
	for (int i = 9; i < 15; i++)
	{
		for (int j = 9; j < 24  - i; j++)
		{
			if (pAreasBigSortinginput[j].nCy > pAreasBigSortinginput[j + 1].nCy)
			{
				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;

				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;
			}
		}
	}

	//输出坐标结果	
	for (int i = 9; i < 16; i++)
	{
		pAreasBigSortingoutput[i].nCx = pAreasBigSortinginput[i].nCx;
		pAreasBigSortingoutput[i].nCy = pAreasBigSortinginput[i].nCy;
	}


	//获取34个大圆中7个大圆，x最大，y依次增大

	for (int i = 36; i < 42; i++)
	{
		for (int j = 36; j < 78 - i; j++)
		{
			if (pAreasBigSortinginput[j].nCy > pAreasBigSortinginput[j + 1].nCy)
			{
				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;

				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;
			}
		}
	}
	//输出坐标结果
	for (int i = 36; i < 43; i++)
	{
		pAreasBigSortingoutput[i].nCx = pAreasBigSortinginput[i].nCx;
		pAreasBigSortingoutput[i].nCy = pAreasBigSortinginput[i].nCy;
	}

	//获取20个大圆中20个大圆

	for (int i = 16; i < 20; i++)
	{
		for (int j = 16; j < 36 - i; j++)
		{
			if (pAreasBigSortinginput[j].nCy > pAreasBigSortinginput[j + 1].nCy)
			{
				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;

				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;
			}
		}
	}


	for (int i = 21; i < 25; i++)
	{
		for (int j = 21; j < 46 - i; j++)
		{
			if (pAreasBigSortinginput[j].nCy > pAreasBigSortinginput[j + 1].nCy)
			{
				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;

				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;
			}
		}
	}

	for (int i = 26; i < 30; i++)
	{
		for (int j = 26; j < 56 - i; j++)
		{
			if (pAreasBigSortinginput[j].nCy > pAreasBigSortinginput[j + 1].nCy)
			{
				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;

				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;
			}
		}
	}

	for (int i = 31; i < 35; i++)
	{
		for (int j = 31; j < 66 - i; j++)
		{
			if (pAreasBigSortinginput[j].nCy > pAreasBigSortinginput[j + 1].nCy)
			{
				temp = pAreasBigSortinginput[j].nCx;
				pAreasBigSortinginput[j].nCx = pAreasBigSortinginput[j + 1].nCx;
				pAreasBigSortinginput[j + 1].nCx = temp;

				temp = pAreasBigSortinginput[j].nCy;
				pAreasBigSortinginput[j].nCy = pAreasBigSortinginput[j + 1].nCy;
				pAreasBigSortinginput[j + 1].nCy = temp;
			}
		}
	}



	for (int i = 16; i < 36; i++)
	{
		pAreasBigSortingoutput[i].nCx = pAreasBigSortinginput[i].nCx;
		pAreasBigSortingoutput[i].nCy = pAreasBigSortinginput[i].nCy;
	}

	//寻找小圆排布
	for (int i = 0; i < nNumSmallSorting - 1; i++)
	{
		for (int j = 0; j < nNumSmallSorting - 1 - i; j++)
		{

			if (pAreasSmallSortinginput[j].nCy > pAreasSmallSortinginput[j + 1].nCy)
			{
				temp = pAreasSmallSortinginput[j].nCy;
				pAreasSmallSortinginput[j].nCy = pAreasSmallSortinginput[j + 1].nCy;
				pAreasSmallSortinginput[j + 1].nCy = temp;

				temp = pAreasSmallSortinginput[j].nCx;
				pAreasSmallSortinginput[j].nCx = pAreasSmallSortinginput[j + 1].nCx;
				pAreasSmallSortinginput[j + 1].nCx = temp;
			}
		}
	}


	//寻找小圆 前9个 上排

	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8 - i; j++)
		{
			if (pAreasSmallSortinginput[j].nCx > pAreasSmallSortinginput[j + 1].nCx)
			{
				temp = pAreasSmallSortinginput[j].nCx;
				pAreasSmallSortinginput[j].nCx = pAreasSmallSortinginput[j + 1].nCx;
				pAreasSmallSortinginput[j + 1].nCx = temp;

				temp = pAreasSmallSortinginput[j].nCy;
				pAreasSmallSortinginput[j].nCy = pAreasSmallSortinginput[j + 1].nCy;
				pAreasSmallSortinginput[j + 1].nCy = temp;
			}
		}
	}

	//输出坐标结果
	for (int i = 0; i < 9; i++)
	{
		pAreasSmallSortingoutput[i].nCx = pAreasSmallSortinginput[i].nCx;
		pAreasSmallSortingoutput[i].nCy = pAreasSmallSortinginput[i].nCy;
	}


	for (int i = 30; i < 38; i++)
	{
		for (int j = 30; j < 68 - i; j++)
		{
			if (pAreasSmallSortinginput[j].nCx > pAreasSmallSortinginput[j + 1].nCx)
			{
				temp = pAreasSmallSortinginput[j].nCx;
				pAreasSmallSortinginput[j].nCx = pAreasSmallSortinginput[j + 1].nCx;
				pAreasSmallSortinginput[j + 1].nCx = temp;

				temp = pAreasSmallSortinginput[j].nCy;
				pAreasSmallSortinginput[j].nCy = pAreasSmallSortinginput[j + 1].nCy;
				pAreasSmallSortinginput[j + 1].nCy = temp;
			}
		}
	}
	//输出坐标结果
	for (int i = 30; i < 39; i++)
	{
		pAreasSmallSortingoutput[i].nCx = pAreasSmallSortinginput[i].nCx;
		pAreasSmallSortingoutput[i].nCy = pAreasSmallSortinginput[i].nCy;
	}

	//  输入

	for (int i = 9; i < 29; i++)
	{
		for (int j = 9; j < 38 - i; j++)
		{
			if (pAreasSmallSortinginput[j].nCx > pAreasSmallSortinginput[j + 1].nCx)
			{
				temp = pAreasSmallSortinginput[j].nCx;
				pAreasSmallSortinginput[j].nCx = pAreasSmallSortinginput[j + 1].nCx;
				pAreasSmallSortinginput[j + 1].nCx = temp;

				temp = pAreasSmallSortinginput[j].nCy;
				pAreasSmallSortinginput[j].nCy = pAreasSmallSortinginput[j + 1].nCy;
				pAreasSmallSortinginput[j + 1].nCy = temp;
			}
		}
	}

	for (int i = 9; i < 15; i++)
	{
		for (int j = 9; j < 24  - i; j++)
		{
			if (pAreasSmallSortinginput[j].nCy > pAreasSmallSortinginput[j + 1].nCy)
			{
				temp = pAreasSmallSortinginput[j].nCx;
				pAreasSmallSortinginput[j].nCx = pAreasSmallSortinginput[j + 1].nCx;
				pAreasSmallSortinginput[j + 1].nCx = temp;

				temp = pAreasSmallSortinginput[j].nCy;
				pAreasSmallSortinginput[j].nCy = pAreasSmallSortinginput[j + 1].nCy;
				pAreasSmallSortinginput[j + 1].nCy = temp;
			}
		}
	}

	for (int i = 9; i < 16; i++)
	{
		pAreasSmallSortingoutput[i].nCx = pAreasSmallSortinginput[i].nCx;
		pAreasSmallSortingoutput[i].nCy = pAreasSmallSortinginput[i].nCy;
	}
	for (int i = 16; i < 22; i++)
	{
		for (int j = 16; j < 38  - i; j++)
		{
			if (pAreasSmallSortinginput[j].nCy > pAreasSmallSortinginput[j + 1].nCy)
			{
				temp = pAreasSmallSortinginput[j].nCx;
				pAreasSmallSortinginput[j].nCx = pAreasSmallSortinginput[j + 1].nCx;
				pAreasSmallSortinginput[j + 1].nCx = temp;

				temp = pAreasSmallSortinginput[j].nCy;
				pAreasSmallSortinginput[j].nCy = pAreasSmallSortinginput[j + 1].nCy;
				pAreasSmallSortinginput[j + 1].nCy = temp;
			}
		}
	}

	for (int i = 16; i < 23; i++)
	{
		pAreasSmallSortingoutput[i].nCx = pAreasSmallSortinginput[i].nCx;
		pAreasSmallSortingoutput[i].nCy = pAreasSmallSortinginput[i].nCy;
	}
	for (int i = 23; i < 29; i++)
	{
		for (int j = 23; j < 52  - i; j++)
		{
			if (pAreasSmallSortinginput[j].nCy > pAreasSmallSortinginput[j + 1].nCy)
			{
				temp = pAreasSmallSortinginput[j].nCx;
				pAreasSmallSortinginput[j].nCx = pAreasSmallSortinginput[j + 1].nCx;
				pAreasSmallSortinginput[j + 1].nCx = temp;

				temp = pAreasSmallSortinginput[j].nCy;
				pAreasSmallSortinginput[j].nCy = pAreasSmallSortinginput[j + 1].nCy;
				pAreasSmallSortinginput[j + 1].nCy = temp;
			}
		}
	}

	for (int i = 23; i < 30; i++)
	{
		pAreasSmallSortingoutput[i].nCx = pAreasSmallSortinginput[i].nCx;
		pAreasSmallSortingoutput[i].nCy = pAreasSmallSortinginput[i].nCy;
	}

	return true;
}

//写数据到文件
// bool bSaveRawFile(const char* sfilename, unsigned char *pBuffer, int iSize)
// {
// 
// 	HFILE			 bmpFile;
// 	LPCSTR str = (LPCSTR)sfilename;
// 	bmpFile = _lcreat(str, FALSE);
// 	if (bmpFile < 0)
// 	{
// 		//AfxMessageBox(_T("获取图片失败"));
// 		return FALSE;
// 	}
// 
// 	UINT len;
// 	len = _lwrite(bmpFile, (LPSTR)pBuffer, iSize);  //+4 is for exact filesize
// 
// 	_lclose(bmpFile);
// 
// 	return true;
// }

void GaussianBlur(unsigned char* buffer, int buffer_width, int buffer_height, int window_size)
{
	TGaussianBlur<unsigned char> BlurFilter;
	//高斯平滑
	BlurFilter.Filter(buffer, NULL, buffer_width, buffer_height, window_size);
}

//直方图计算动态阈值
void Histogram(unsigned char* buffer, int buffer_width, int buffer_height, int *Threshold)
{
	double sum = 0.0;
	double w0 = 0.0;
	double w1 = 0.0;
	double u0_temp = 0.0;
	double u1_temp = 0.0;
	double u0 = 0.0;
	double u1 = 0.0;
	double delta_temp = 0.0;
	double delta_max = 0.0;
	int i, j;

	//src_image灰度级
	int pixel_count[256] = { 0 };
	float pixel_pro[256] = { 0 };
	int threshold = 0;
	unsigned char* data = buffer;
	//统计每个灰度级中像素的个数
	for (i = 0; i < buffer_height; i++)
	{
		for (j = 0; j < buffer_width; j++)
		{
			pixel_count[(int)data[i * buffer_width + j]]++;
			sum += (int)data[i * buffer_width + j];
		}
	}
	//cout<<"平均灰度："<<sum / ( buffer_height * buffer_width )<<endl;
	//计算每个灰度级的像素数目占整幅图像的比例
	for (i = 0; i < 256; i++)
	{
		pixel_pro[i] = (float)pixel_count[i] / (buffer_height * buffer_width);
	}
	//遍历灰度级[0,255],寻找合适的threshold
	for (i = 0; i < 256; i++)
	{
		w0 = w1 = u0_temp = u1_temp = u0 = u1 = delta_temp = 0;
		for (int j = 0; j < 256; j++)
		{
			if (j <= i)   //背景部分
			{
				w0 += pixel_pro[j];
				u0_temp += j * pixel_pro[j];
			}
			else   //前景部分
			{
				w1 += pixel_pro[j];
				u1_temp += j * pixel_pro[j];
			}
		}
		u0 = u0_temp / w0;
		u1 = u1_temp / w1;
		delta_temp = (float)(w0 *w1* pow((u0 - u1), 2));
		if (delta_temp > delta_max)
		{
			delta_max = delta_temp;
			threshold = i;
		}
	}

	*Threshold = threshold;
}

unsigned char* bBMPFileToYUVData(const char* strPath, int* iWidth, int* iHeight)
{

	FILE* fp;
	fp = fopen(strPath, "rb");
	if (fp == NULL)
		return NULL;
	fpos_t posstart, posend;
	fseek(fp, 0, SEEK_END);
	fgetpos(fp, &posend);
	fseek(fp, 0, SEEK_SET);
	fgetpos(fp, &posstart);
	DWORD filesize = (DWORD)(posend - posstart);
	BYTE* filebuffer = (BYTE*)malloc(filesize);
	if (filebuffer == NULL)
	{
		return NULL;
	}
	//	memcpy(filebuffer,fp,filesize);
	fread(filebuffer, sizeof(BYTE), filesize, fp);
	fclose(fp);
	//////////////////////////////////////////////////////////////////////////
	/*开始处理图像数据*/
	if (filebuffer[0] != 'B' || filebuffer[1] != 'M')
	{
		return NULL;
	}
	PBITMAPFILEHEADER pbitmapfileheader = (PBITMAPFILEHEADER)filebuffer;
	PBITMAPINFOHEADER pbitmapinfoheader = (PBITMAPINFOHEADER)(filebuffer + 14);
	int width, height, bitsize;
	width = pbitmapinfoheader->biWidth;
	height = pbitmapinfoheader->biHeight;
	*iWidth = (int)width;
	*iHeight = (int)height;
	if (!(pbitmapinfoheader->biBitCount == 24 || pbitmapinfoheader->biBitCount == 32))
	{
		return NULL;
	}
	bitsize = pbitmapinfoheader->biBitCount / 8;
	long datasize = width * height * bitsize;
	unsigned char* lData;
	lData = filebuffer + pbitmapfileheader->bfOffBits;
	VertFlipBuf(lData, width*bitsize, height);
	filebuffer = NULL;
	return lData;
}
bool VertFlipBuf(unsigned char* inbuf, int widthBytes, int height)
{
	unsigned char  *tb1;
	unsigned char  *tb2;

	if (inbuf == NULL)
		return FALSE;

	int bufsize;

	bufsize = widthBytes;

	tb1 = (unsigned char *)new unsigned char[bufsize];
	if (tb1 == NULL) {
		return FALSE;
	}

	tb2 = (unsigned char *)new unsigned char[bufsize];
	if (tb2 == NULL) {
		delete[] tb1;
		return FALSE;
	}

	int row_cnt;
	ULONG off1 = 0;
	ULONG off2 = 0;

	for (row_cnt = 0; row_cnt < (height + 1) / 2; row_cnt++) {
		off1 = row_cnt*bufsize;
		off2 = ((height - 1) - row_cnt)*bufsize;

		memcpy(tb1, inbuf + off1, bufsize);
		memcpy(tb2, inbuf + off2, bufsize);
		memcpy(inbuf + off1, tb2, bufsize);
		memcpy(inbuf + off2, tb1, bufsize);
	}

	delete[] tb1;
	delete[] tb2;

	return TRUE;

}

void RGB24ToY(unsigned char *pRGB, unsigned char *pY, int width, int height)
{
	int green, red, blue;
	long offset = 0;
	long imagesize = height*width * 3;
	for (long i = 0; i < imagesize; i += 3)
	{

		blue = pRGB[i];
		green = pRGB[i + 1];
		red = pRGB[i + 2];
		//Y
		pY[offset] = (unsigned char)(0.2989f	*	red + 0.587f	*	green + 0.114f	*	blue);

		offset++;
	}
}

bool bSaveRawFile(const char* sfilename, unsigned char *pBuffer, int iSize)
{
	HFILE			 bmpFile;
	bmpFile = _lcreat(sfilename, FALSE);
	if (bmpFile < 0)
	{
		return FALSE;
	}

	UINT len;
	len = _lwrite(bmpFile, (LPSTR)pBuffer, iSize);  //+4 is for exact filesize
	_lclose(bmpFile);

	return TRUE;
}