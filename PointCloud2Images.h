#pragma once

#include <map>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <windows.h>

#include <opencv2/opencv.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common_headers.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include "omp.h"
using namespace std;

namespace PointCloud2Image
{
	struct SPoint
	{
		float x, y, z, r, i, g, b, l;
		float angle, height, radius;
	};

	struct SPixel
	{
		bool flag = 0;
		float x, y, z, i, l, depth, n;
		unsigned short BA_val, depth_val, intensity_val, SNA_val, N_val;
		unsigned short r, g, b;
		unsigned int index = -1;
	};

	class Point3d
	{
	public:
		float x, y, z;
		Point3d(void);
		Point3d(const Point3d& tmpP);
		friend Point3d operator - (const Point3d &p1, const Point3d &p2);
		friend float operator *(const Point3d &p1, const Point3d &p2);
		float Dist(void);
		Point3d & Normalize();
	};

	class singleton
	{
	protected:
		singleton() {}
	private:
		static singleton* globaldata;
	public:
		static singleton* instance();
		vector<SPoint> data;
		int picnum = 1;
		vector<vector<SPixel>>vimage;
		vector<vector<vector<SPixel>>>n_image;
	};

	class Io
	{
	public:
		Io();
		~Io();
	public:
		static void Io::data_input(string str, vector<SPoint>&data, float ViewPoint_x, float ViewPoint_y, float ViewPoint_z);
	private:

	};

	class Matrix
	{
	public:
		Matrix();
		~Matrix();
	public:
		static void Matrix::get_Matrix(singleton *globaldata, int n_pixel, float angel_increase);
	private:

	};

	class Feature
	{
	public:
		Feature();
		~Feature();
	public:
		static void Feature::get_Feature(singleton *globaldata);

	private:
		static void Feature::get_feature_BA(vector<SPoint>&data, vector<vector<SPixel>>&vimage);
		static void Feature::get_feature_SNA(vector<SPoint>&data, vector<vector<SPixel>>&vimage);
		static void Feature::get_feature_N(vector<SPoint>&data, vector<vector<SPixel>>&vimage);
		static void Feature::get_feature_Intensity(vector<SPoint>&data, vector<vector<SPixel>>&vimage);
		static void Feature::get_feature_Depth(vector<SPoint>&data, vector<vector<SPixel>>&vimage);
	};

	class Picture
	{
	public:
		Picture();
		~Picture();
	public:

		static void Picture::draw_picture_BA(string path, vector<vector<SPixel>>&vimage);
		static void Picture::draw_picture_Intensity(string path, vector<vector<SPixel>>&vimage);
		static void Picture::draw_picture_SNA(string path, vector<vector<SPixel>>&vimage);
		static void Picture::draw_picture_RGB(string path, vector<vector<SPixel>>&vimage);
		static void Picture::draw_picture_N(string path, vector<vector<SPixel>>&vimage);
		static void Picture::draw_picture_Depth(string path, vector<vector<SPixel>>&vimage);

	private:

	};

}

