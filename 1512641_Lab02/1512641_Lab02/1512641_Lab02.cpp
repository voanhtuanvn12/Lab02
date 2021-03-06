// 1512641_Lab02.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "GeometricTransformer.h"

int main(int argc, char **argv) {
	/*
	Mat scaleMatrix = (Mat_<float>(3, 3) << 2, 0, 0, 0, 3, 0, 1, 0, 1);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cout << scaleMatrix.at<float>(i, j)<<" ";
		}
		cout << endl;
	}
	*/


	if (argc < 5) {
		cout << "Argument parameter's error !" << endl;
		return -1;
	}

	Mat srcImage, dstImage;
	GeometricTransformer *geometricTransformer = new GeometricTransformer();
	PixelInterpolate *pixelInterpolate;

	// Interpolate 
	if (strcmp(argv[2], "--nn") == 0) {
		pixelInterpolate = new NearestNeighborInterpolate();
	}
	else if (strcmp(argv[2], "--bl") == 0) {
		pixelInterpolate = new BilinearInterpolate();
	}
	else {
		cout << "Error Interpolate : " << argv[2] <<" is not supported "<< endl;
		return -1;
	}

	// Read the source image
	srcImage = imread(argv[3], CV_LOAD_IMAGE_ANYCOLOR);

	if (!srcImage.data) {
		cout << "Can not load the image ! " << endl;
		return -1;
	}

	float num = atof(argv[4]);

	int res = 0;
	if (strcmp(argv[1], "--zoom") == 0) {
		res = geometricTransformer->Scale(srcImage, dstImage, num, num, pixelInterpolate);
	}
	else if (strcmp(argv[1], "--rotate") == 0) {
		res = geometricTransformer->RotateKeepImage(srcImage, dstImage, num, pixelInterpolate);
	}
	else if (strcmp(argv[1], "--rotateN") == 0) {
		res = geometricTransformer->RotateUnkeepImage(srcImage, dstImage, num, pixelInterpolate);
	}
	else {
		cout << "Error : " << argv[1] << endl;
		return -1;
	}

	if (res == 0) {
		cout << "Can not Transform the image !" << endl;
	}

	return 0;
}

