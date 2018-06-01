#include "stdafx.h"
#include "PointCloud2Images.h"

namespace PointCloud2Image
{
	singleton* singleton::globaldata = NULL;

	singleton* singleton::instance()
	{
		if (globaldata == NULL)
			globaldata = new singleton();
		return globaldata;
	}
	/**************************************************/

	Io::Io() {}
	Io::~Io() {}
	void Io::data_input(string str, vector<SPoint>&data, float ViewPoint_x, float ViewPoint_y, float ViewPoint_z)
	{
		data.clear();
		ifstream infile;
		infile.open(str);
		float x_, y_, z_, i_;
		float r_, g_, b_, l_;
		int num = 0;
		i_ = 0; l_ = 0;
		while (infile >> x_ >> y_ >> z_ >> i_ >> r_ >> g_ >> b_ >> l_)
		//infile >> num;while (infile >> x_ >> y_ >> z_ >> r_ >> g_ >> b_ >> i_)
		{
			SPoint p;
			p.x = x_ - ViewPoint_x;
			p.y = y_ - ViewPoint_y;
			p.z = z_ - ViewPoint_z;
			p.i = i_;
			p.r = r_;
			p.g = g_;
			p.b = b_;
			p.l = l_;
			data.push_back(p);
		}
		infile.close();

	}
	/**************************************************/

	#define RAD_TO_DEG (180 / (4 * atan(1)))
	#define DEG_TO_RAD ((4 * atan(1)) / 180) 

	Matrix::Matrix() {}
	Matrix::~Matrix() {}

	void Matrix::get_Matrix(singleton *globaldata, int n_pixel, float angel_increase)
	{
		const int w_degree = 60;
		const int h_degree = 180;
		int w_n_pixel = n_pixel * w_degree * 3;
		int h_n_pixel = n_pixel * h_degree;

		globaldata->picnum = int(360 / angel_increase);
		globaldata->n_image.clear();
		globaldata->n_image.resize(globaldata->picnum);
		for (int i = 0; i<globaldata->picnum; i++)
		{
			globaldata->n_image[i].resize(w_n_pixel);
			for (int j = 0; j<w_n_pixel; j++)
			{
				globaldata->n_image[i][j].resize(h_n_pixel);
			}
		}

		map<int, vector<long long>>angle2index_map;
		for (long long i = 0; i < globaldata->data.size(); i++)
		{
			globaldata->data[i].angle = RAD_TO_DEG * atan2(globaldata->data[i].y, globaldata->data[i].x) + 180;
			globaldata->data[i].radius = sqrt(globaldata->data[i].x*globaldata->data[i].x + globaldata->data[i].y*globaldata->data[i].y);
			angle2index_map[int(globaldata->data[i].angle)].push_back(i);
		}

#pragma omp parallel for
		for (int i = 0; i < globaldata->picnum; i++)
		{
			for (multimap<int, vector<long long>>::iterator it = angle2index_map.begin(); it != angle2index_map.end(); it++)
			{
				if (!(fabs((*it).first - i*angel_increase) < (w_degree / 2 + 1) ||
					(360 - fabs((*it).first - i*angel_increase) < (w_degree / 2 + 1))))
					continue;

				for (int t = 0; t < (*it).second.size(); t++)
				{
					int j = (*it).second[t];
					int w, h;
					float ag, depth;
					if (fabs(globaldata->data[j].angle - i*angel_increase)<w_degree / 2)
					{
						ag = globaldata->data[j].angle - i*angel_increase;
					}
					else if (360 - fabs(globaldata->data[j].angle - i*angel_increase)<w_degree / 2)
					{
						if (globaldata->data[j].angle - i*angel_increase >= 0) ag = -(360 - fabs(globaldata->data[j].angle - i*angel_increase));
						else  ag = (360 - fabs(globaldata->data[j].angle - i*angel_increase));
					}
					else
						continue;
					depth = (2 * cos(fabs(ag)*DEG_TO_RAD)*globaldata->data[j].radius);

					w = int(w_n_pixel / 2 * (1 + tan(ag*DEG_TO_RAD) / tan(w_degree / 2 * DEG_TO_RAD)));
					h = floor((1 + globaldata->data[j].z / (globaldata->data[j].radius*cos(ag*DEG_TO_RAD)))*h_n_pixel / 2);

					if (h < 0 || h >= h_n_pixel)
						continue;
					if (globaldata->n_image[i][w][h].flag&&globaldata->n_image[i][w][h].depth_val < depth)
						continue;

					globaldata->n_image[i][w][h].r = unsigned short(globaldata->data[j].r);
					globaldata->n_image[i][w][h].g = unsigned short(globaldata->data[j].g);
					globaldata->n_image[i][w][h].b = unsigned short(globaldata->data[j].b);
					globaldata->n_image[i][w][h].l = unsigned short(globaldata->data[j].l);
					globaldata->n_image[i][w][h].x = (globaldata->data[j].x);
					globaldata->n_image[i][w][h].y = (globaldata->data[j].y);
					globaldata->n_image[i][w][h].z = (globaldata->data[j].z);

					globaldata->n_image[i][w][h].i = unsigned short(globaldata->data[j].i);
					globaldata->n_image[i][w][h].depth = depth;
					globaldata->n_image[i][w][h].flag = true;
					globaldata->n_image[i][w][h].index = j;
				}
			}
		}
	}
	/**************************************************/

	Feature::Feature() {}
	Feature::~Feature() {}
	using namespace cv;

	void Feature::get_feature_BA(vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		unsigned short ba_grey;
		Point3d _position;
		for (int i = 0; i < vimage.size(); i++)
			for (int j = 0; j < vimage[i].size(); j++)
			{
				if (vimage[i][j].flag == 0)
				{
					ba_grey = 0;
					continue;
				}

				int i_nb = i - 1;
				int j_nb = j + 1;
				if (i_nb < 0)i_nb = 0;
				if (j_nb >= vimage[i].size())j_nb = vimage[i].size() - 1;

				Point3d pa, pc, p;
				pa.x = data[vimage[i][j].index].x; pa.y = data[vimage[i][j].index].y; pa.z = data[vimage[i][j].index].z;
				pa = pa - _position;
				if (vimage[i_nb][j_nb].flag != 0)
				{
					pc.x = data[vimage[i_nb][j_nb].index].x; pc.y = data[vimage[i_nb][j_nb].index].y; pc.z = data[vimage[i_nb][j_nb].index].z;
				}

				pc = pc - _position;
				p = pa - pc;

				float angle = acos((p * (pc.Normalize())) / p.Dist());

				ba_grey = 255 - angle / 3.1415927 * 255;
				if (ba_grey < 0) ba_grey = 0;
				if (ba_grey > 255) ba_grey = 255;

				vimage[i][j].BA_val = unsigned short(ba_grey);
			}
	}

	void Feature::get_feature_N(vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		int nei[18] = { -1,-1, 0,-1, 1,-1,
			-1,0,  0,0,  1,0,
			-1,1,  0,1,  1,1 };
		int nei_t = 1;
		for (int i = 0; i < vimage.size(); i++)
		{
			for (int j = 0; j < vimage[i].size(); j++)
			{
				if (vimage[i][j].flag == 0)
				{
					vimage[i][j].N_val = 0;
					continue;
				}
				pcl::PointCloud<pcl::PointXYZL> Cpoints;
				for (int t = 0; t < nei_t; t++)
				{
					for (int p = 0; p < 9; p++)
					{
						int i_, j_;
						i_ = i + (t + 1)*nei[2 * p];
						j_ = j + (t + 1)*nei[2 * p + 1];
						if (i_ < 0 || i_ >= vimage.size())continue;
						if (j_ < 0 || j_ >= vimage[i].size())continue;
						if (vimage[i_][j_].flag == 0)continue;

						pcl::PointXYZL _p;
						_p.x = data[vimage[i_][j_].index].x;
						_p.y = data[vimage[i_][j_].index].y;
						_p.z = data[vimage[i_][j_].index].z;
						Cpoints.push_back(_p);

					}
				}
				if (Cpoints.points.size() < 3)
				{
					vimage[i][j].N_val = 255;
					continue;
				}

				EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
				Eigen::Vector4f xyz_centroid;
				EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
				EIGEN_ALIGN16 Eigen::Vector3f eigen_values;

				if (pcl::computeMeanAndCovarianceMatrix(Cpoints, covariance_matrix, xyz_centroid) == 0)
				{
					vimage[i][j].N_val = 255;
					continue;
				}
				pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
				vimage[i][j].N_val = int(fabs(eigen_vectors(2, 0)) * 255);
				vimage[i][j].n = fabs(eigen_vectors(2, 0));

			}
		}

	}

	void Feature::get_feature_SNA(vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		for (int i = 1; i < vimage.size() - 1; i++)
		{
			for (int j = 1; j < vimage[i].size() - 1; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					vimage[i][j].SNA_val = 0;
					continue;
				}
				SPoint p0; p0.x = 0; p0.y = 0; p0.z = 0;

				SPoint p1, p2, p3;
				int p1_x, p1_y, p3_x, p3_y;
				p2 = data[vimage[i][j].index];

				float d = vimage[0].size() / 180.0;
				float d1 = fabs(j * 1.0 / d - 45);
				float d2 = d1 + 1.0 / d;
				float ref = tan(d2 / 180 * 3.1416) / tan(d1 / 180 * 3.1416);

				if (j > vimage[0].size() / 2)
				{
					p1_x = i - 1;
					p1_y = j + 1;
					p3_x = i + 1;
					p3_y = j + 1;

				}
				else
				{
					p1_x = i - 1;
					p1_y = j - 1;
					p3_x = i + 1;
					p3_y = j - 1;

				}

				if (vimage[p1_x][p1_y].flag == 0)p1 = p0;
				else p1 = data[vimage[p1_x][p1_y].index];
				if (vimage[p3_x][p3_y].flag == 0)p3 = p0;
				else p3 = data[vimage[p3_x][p3_y].index];

				double side[3];
				side[0] = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
				side[1] = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) + pow(p1.z - p3.z, 2));
				side[2] = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) + pow(p3.z - p2.z, 2));

				if ((side[0] * side[2]) == 0) { vimage[i][j].SNA_val = 0; continue; }
				float angle = acos((pow(side[0], 2) + pow(side[2], 2) - pow(side[1], 2)) / 2 / (side[0] * side[2]));

				if ((sqrt(p1.x *p1.x + p1.y*p1.y + p1.z*p1.z) + sqrt(p3.x *p3.x + p3.y*p3.y + p3.z*p3.z))>(2 * sqrt(p2.x *p2.x + p2.y*p2.y + p2.z*p2.z)*0.999))
				{
					angle = 2 * 3.1416 - angle;

				}
				if (j > vimage[0].size() / 2)
				{
					vimage[i][j].SNA_val = (int(angle / 2 / 3.1416 * 255) + 8 ) % 256;
				}
				else
				{
					vimage[i][j].SNA_val = (int(angle / 2 / 3.1416 * 255)) % 256;
				}
				
				
			}
		}

	}

	void Feature::get_feature_Depth(vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		struct  P_coordinate
		{
			int i, j;
		};

		multimap<float, P_coordinate>Depth2color_map;

		for (int i = 0; i < vimage.size(); i++)
		{
			for (int j = 0; j < vimage[i].size(); j++)
			{
				if (vimage[i][j].flag == 0)continue;
				P_coordinate _pc;
				_pc.i = i;
				_pc.j = j;
				Depth2color_map.insert(make_pair(vimage[i][j].depth, _pc));
			}
		}
		int total_P = Depth2color_map.size() + 1;
		float i = 0;
		for (multimap<float, P_coordinate>::iterator it = Depth2color_map.begin(); it != Depth2color_map.end(); it++)
		{
			i++;
			vimage[(*it).second.i][(*it).second.j].depth_val = int(i * 255 / total_P);
		}
	}

	void Feature::get_feature_Intensity(vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		struct  P_coordinate
		{
			int i, j;
		};

		multimap<float, P_coordinate>Intensity2color_map;

		for (int i = 0; i < vimage.size(); i++)
		{
			for (int j = 0; j < vimage[i].size(); j++)
			{
				if (vimage[i][j].flag == 0)continue;
				P_coordinate _pc;
				_pc.i = i;
				_pc.j = j;
				Intensity2color_map.insert(make_pair(vimage[i][j].i, _pc));
			}
		}
		int total_P = Intensity2color_map.size() + 1;
		float i = 0;
		for (multimap<float, P_coordinate>::iterator it = Intensity2color_map.begin(); it != Intensity2color_map.end(); it++)
		{
			i++;
			vimage[(*it).second.i][(*it).second.j].intensity_val = int(i * 255 / total_P);
		}
	}

	void Feature::get_Feature(singleton *globaldata)
	{
#pragma omp parallel for
		for (int i = 0; i < globaldata->n_image.size(); i++)
		{
#pragma omp parallel
		{
			get_feature_N(globaldata->data, globaldata->n_image[i]);
			get_feature_BA(globaldata->data, globaldata->n_image[i]);
			get_feature_SNA(globaldata->data, globaldata->n_image[i]);
			get_feature_Depth(globaldata->data, globaldata->n_image[i]);
			get_feature_Intensity(globaldata->data, globaldata->n_image[i]);
		}
		}
	}

	Point3d::Point3d(void)
	{
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}

	Point3d::Point3d(const Point3d& tmpP)
	{
		this->x = tmpP.x;
		this->y = tmpP.y;
		this->z = tmpP.z;
	}
	Point3d operator - (const Point3d &p1, const Point3d &p2)
	{
		Point3d po;
		po.x = p1.x - p2.x;
		po.y = p1.y - p2.y;
		po.z = p1.z - p2.z;
		return po;
	}

	float operator *(const Point3d  &p1, const Point3d  &p2)
	{
		return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z);
	}

	float Point3d::Dist(void)
	{
		return sqrt(x*x + y*y + z*z);
	}

	Point3d & Point3d::Normalize()
	{
		float n = float(sqrt(x*x + y*y + z*z));
		if (n > float(0))
		{
			x /= n; y /= n; z /= n;
		}
		return *this;
	}
	/**************************************************/

	Picture::Picture() {}

	Picture::~Picture() {}

	void Picture::draw_picture_BA(string path, vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = 255 - vimage[i][j].BA_val;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void Picture::draw_picture_SNA(string path, vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 255;
					continue;
				}
				int rgb_grey = 255 - vimage[i][j].SNA_val;

				if (rgb_grey < 127)rgb_grey = 64;
				else rgb_grey = 192;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void Picture::draw_picture_N(string path, vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = vimage[i][j].N_val;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void Picture::draw_picture_Intensity(string path, vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = 255 - vimage[i][j].intensity_val;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void Picture::draw_picture_Depth(string path, vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = vimage[i][j].depth_val;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;
			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}
	void Picture::draw_picture_RGB(string path, vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = uchar(vimage[i][j].r);
				rgba[1] = uchar(vimage[i][j].g);
				rgba[2] = uchar(vimage[i][j].b);
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	
}
/**************************************************/
using namespace PointCloud2Image;
const char* data_input = "data_xyzirgbl.txt";
//const char* data_input = "F://Wurzburg data//bremen_city//bremen_city//scan004.txt";
const char* pic_output = "Image//";
const int Pic_num = 60;
const float Angle_res = 0.5;
const float ViewPoint_x = 0;
const float ViewPoint_y = 0;
const float ViewPoint_z = 0;

void Data_input(string str, singleton *globaldata)
{
	double start, end;
	start = GetTickCount();
	Io::data_input(str, globaldata->data, ViewPoint_x, ViewPoint_y, ViewPoint_z);
	end = GetTickCount();
	cout << ">>> Data_input ok! Time consuming " << (end - start) / 1000 << "s" << " Data size: " << globaldata->data.size() << endl;

	start = GetTickCount();
	Matrix::get_Matrix(globaldata, 1 / Angle_res, 360 / Pic_num);
	end = GetTickCount();
	cout << ">>> Get_Matrix ok! Time consuming " << (end - start) / 1000 << "s" << endl;

	start = GetTickCount();
	Feature::get_Feature(globaldata);
	end = GetTickCount();
	cout << ">>> Get_Feature ok! Time consuming " << (end - start) / 1000 << "s" << endl;
}

void Pic_output(string path, singleton *globaldata)
{
	double start = GetTickCount();
#pragma omp parallel for
	for (int i = 0; i < globaldata->picnum; i++)
	{
		stringstream ss; string t;
		ss << i; ss >> t;
#pragma omp parallel
		{
			Picture::draw_picture_SNA(path + "SNAB_" + t + ".png", globaldata->n_image[i]);
			/*Picture::draw_picture_BA(path + "BA_" + t + ".png", globaldata->n_image[i]);
			Picture::draw_picture_Intensity(path + "I_" + t + ".png", globaldata->n_image[i]);
			Picture::draw_picture_N(path + "N_" + t + ".png", globaldata->n_image[i]);
			Picture::draw_picture_Depth(path + "Depth_" + t + ".png", globaldata->n_image[i]);
			Picture::draw_picture_RGB(path + "RGB_" + t + ".png", globaldata->n_image[i]);*/
		}
	}
	double  end = GetTickCount();
	cout << ">>> Draw_picture ok! Time consuming " << (end - start) / 1000 << "s" << endl;
}

int main()
{
	singleton *globaldata = singleton::instance();
	Data_input(data_input, globaldata);
	Pic_output(pic_output, globaldata);
	system("pause");
	return 0;
}