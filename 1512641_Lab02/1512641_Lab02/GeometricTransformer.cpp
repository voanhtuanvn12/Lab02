#include "stdafx.h"
#include "GeometricTransformer.h"

PixelInterpolate::PixelInterpolate()
{
}

PixelInterpolate::~PixelInterpolate()
{
}

void BilinearInterpolate::Interpolate(float tx, float ty, uchar * pSrc, int srcWidthStep, int nChannels, uchar * pDstRow)
{
	int x = round(tx); float a = tx - x;
	int y = round(ty); float b = ty - y;

	for (int i = 0; i < nChannels; ++i) {
		pDstRow[i] = (uchar)((1 - a)*(1 - b)*((int)pSrc[x*srcWidthStep + y * nChannels + i])
			+ a * (1 - b)*((int)pSrc[(x + 1)*srcWidthStep + y * nChannels + i])
			+ b * (1 - b)*((int)pSrc[x*srcWidthStep + (y + 1)*nChannels + i])
			+ a * b*((int)pSrc[(x + 1)*srcWidthStep + (y + 1)*nChannels + i]));
	}
}

BilinearInterpolate::BilinearInterpolate()
{
}

BilinearInterpolate::~BilinearInterpolate()
{
}

void NearestNeighborInterpolate::Interpolate(float tx, float ty, uchar * pSrc, int srcWidthStep, int nChannels, uchar * pDstRow)
{
	int x = round(tx); int y = round(ty);
	for (int i = 0; i < nChannels; ++i) {
		uchar  v = (uchar)(pSrc[x*srcWidthStep + y * nChannels + i]);
		pDstRow[i] = v;
	}
}

NearestNeighborInterpolate::NearestNeighborInterpolate()
{
}

NearestNeighborInterpolate::~NearestNeighborInterpolate()
{
}

void AffineTransform::Translate(float dx, float dy)
{
	Mat translateMatrix = (Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, dx, dy, 1);
	_matrixTransform = _matrixTransform * translateMatrix;
}

void AffineTransform::Rotate(float angle)
{
	float sinx = sin(angle*PI / 180), cosx = cos(angle*PI / 180);
	Mat rotateMatrix = (Mat_<float>(3, 3) << cosx, sinx, 0, -sinx, cosx, 0, 0, 0, 1);
	_matrixTransform = _matrixTransform * rotateMatrix;
}

void AffineTransform::Scale(float sx, float sy)
{
	Mat scaleMatrix = (Mat_<float>(3, 3) << sx, 0, 0, 0, sy, 0, 0, 0, 1);
	_matrixTransform = _matrixTransform * scaleMatrix;
}

void AffineTransform::TransformPoint(float & x, float & y)
{
	//Mat temp = Mat(3, 4, CV_8UC1);

	//cout << " cols " << temp.cols << " step " << temp.step[0] << " step[1] " << temp.step[1] << " channels " << temp.channels() << endl;
	//cout << CV_8UC1 << endl;
	//int row = _matrixTransform.rows;
	//int col = _matrixTransform.cols;
	Mat X = (Mat_<float>(1,3) << x, y, 1);
	//Mat result = Mat(3, 1, X.type());
	/*
	for (int c = 0; c < 3; c++) {
		for (int d = 0; d < 1; d++) {
			float sum = 0;
			for (int k = 0; k < 3; k++) {
				sum += _matrixTransform.at<float>(c, k) * X.at<float>(k, d);
			}
			float *result_data = result.ptr<float>(c);
			*(result_data + d) = sum;
		}
	}
	*/
	Mat result = X*_matrixTransform;
	x = result.at<float>(0, 0);
	y = result.at<float>(0, 1);
}

void AffineTransform::Identify()
{
	_matrixTransform = (Mat_<float>(3, 3) << 1, 0, 0, 0, 1 ,0, 0, 0, 1);
}

void AffineTransform::Inverse()
{
	_matrixTransform = _matrixTransform.inv();
}

AffineTransform::AffineTransform()
{
	_matrixTransform = (Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
}

AffineTransform::~AffineTransform()
{
	_matrixTransform.release();
}

int GeometricTransformer::Transform(const Mat & beforeImage, Mat & afterImage, AffineTransform * transformer, PixelInterpolate * interpolator)
{
	if (!beforeImage.data || !afterImage.data) {
		return 0;
	}

	int src_stepWidth = beforeImage.step[0];
	int src_nchannels = beforeImage.step[1];
	int src_width = beforeImage.cols;
	int src_height = beforeImage.rows;

	int dst_stepWidth = afterImage.step[0];
	int dst_height = afterImage.rows;
	int dst_width = afterImage.cols;
	uchar *src_data = (uchar*)beforeImage.data;
	uchar* dst_data = (uchar*)afterImage.data;

	

	// Chuyển ma trận transform thành ma trận nghịch đảo
	transformer->Inverse();

	for (int y = 0; y < dst_height; ++y, dst_data += dst_stepWidth) {
		uchar *pRow = dst_data;
		for (int x = 0; x < dst_width; ++x, pRow += src_nchannels) {
			float yy = y, xx = x;
			transformer->TransformPoint(yy, xx);

			if (round(xx) >= src_width || round(yy) >= src_height || xx < 0 || yy < 0)
				continue;
			interpolator->Interpolate(yy, xx, src_data, src_stepWidth, src_nchannels, pRow);
		}
	}
	return 1;
}

int GeometricTransformer::RotateKeepImage(const Mat & srcImage, Mat & dstImage, float angle, PixelInterpolate * interpolator)
{
	if (!srcImage.data) {
		return 0;
	}


	int src_width = srcImage.cols;
	int src_height = srcImage.rows;

	float cosx = cos(angle * PI / 180.0);
	float sinx = sin(angle *PI / 180.0);

	// Tìm kích thước cho cửa số mới
	int dst_width = (int)(src_width*cosx + src_height * sinx);
	int dst_height = (int)(src_width*sinx + src_height * cosx);
	dstImage.create(dst_height, dst_width, srcImage.type());

	AffineTransform *aff = new AffineTransform();

	// tạo ma trận biến đổi
	// Tịnh tiến ma trận về tâm của ma trận src
	aff->Translate((dst_height / 2) - (src_height / 2), (dst_width / 2) - (src_width / 2));
	// Tịnh tiến ma trận về gốc của ma trận src
	aff->Translate(-(dst_height / 2), -(dst_width / 2));
	// Xoay ma trận 
	aff->Rotate(angle);
	// Tịnh tiến ma trận về vị trí cũ
	aff->Translate(dst_height / 2, dst_width / 2);
	GeometricTransformer::Transform(srcImage, dstImage, aff, interpolator);

	namedWindow("Rotate keep image", WINDOW_AUTOSIZE);
	cout << dstImage.size() << endl;
	imwrite("Rotate_keep.png", dstImage);
	imshow("Rotate keep image", dstImage);
	waitKey(0);
	return 1;
}

int GeometricTransformer::RotateUnkeepImage(const Mat & srcImage, Mat & dstImage, float angle, PixelInterpolate * interpolator)
{
	if (!srcImage.data) {
		return 0;
	}
	int src_width = srcImage.cols;
	int src_height = srcImage.rows;


	dstImage.create(src_height, src_width, srcImage.type());
	AffineTransform *aff = new AffineTransform();

	aff->Translate(-(src_height / 2), -(src_width / 2));

	aff->Rotate(angle);

	aff->Translate(src_height / 2, src_width / 2);

	GeometricTransformer::Transform(srcImage, dstImage, aff, interpolator);

	

	namedWindow("Rotate unkeep image", WINDOW_AUTOSIZE);
	imshow("Rotate unkeep image", dstImage);
	imwrite("Rotate_unkeep.png", dstImage);
	waitKey(0);
	return 1;
}

int GeometricTransformer::Scale(const Mat & srcImage, Mat & dstImage, float sx, float sy, PixelInterpolate * interpolator)
{
	if (!srcImage.data) {
		return 0;
	}

	int width = srcImage.cols;
	int height = srcImage.rows;

	dstImage.create(width, height, srcImage.type());
	AffineTransform *aff = new AffineTransform();


	// Tịnh tiến về tâm
	aff->Translate(-height / 2, -width / 2);
	// Xoay
	aff->Scale(sx, sy);
	// Tịnh tiến về tọa độ ban đầu
	aff->Translate(height / 2, width / 2);

	GeometricTransformer::Transform(srcImage, dstImage, aff, interpolator);

	namedWindow("Scale", WINDOW_AUTOSIZE);
	imshow("Scale", dstImage);
	imwrite("Scaleimage.png", dstImage);
	waitKey(0);
	return 1;
}

GeometricTransformer::GeometricTransformer()
{
}

GeometricTransformer::~GeometricTransformer()
{
}
