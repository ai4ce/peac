//
// Copyright 2014 Mitsubishi Electric Research Laboratories All
// Rights Reserved.
//
// Permission to use, copy and modify this software and its
// documentation without fee for educational, research and non-profit
// purposes, is hereby granted, provided that the above copyright
// notice, this paragraph, and the following three paragraphs appear
// in all copies.
//
// To request permission to incorporate this software into commercial
// products contact: Director; Mitsubishi Electric Research
// Laboratories (MERL); 201 Broadway; Cambridge, MA 02139.
//
// IN NO EVENT SHALL MERL BE LIABLE TO ANY PARTY FOR DIRECT,
// INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
// LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
// DOCUMENTATION, EVEN IF MERL HAS BEEN ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGES.
//
// MERL SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND MERL HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,
// SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
#pragma warning(disable: 4996)
#pragma warning(disable: 4819)
#define _CRT_SECURE_NO_WARNINGS

#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#ifdef USE_OPENNI1
#include <pcl/io/openni_grabber.h>
#else
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
#endif
#include <pcl/io/pcd_io.h>

#include "opencv2/opencv.hpp"


class MainLoop
{
	typedef pcl::PointCloud< pcl::PointXYZRGBA > ColorCloud;
	typedef pcl::PointCloud< pcl::PointXYZRGB > RGBCloud;
protected:
	ColorCloud::ConstPtr cloud_;
	cv::Mat rgb;
	bool done;

	std::vector< ColorCloud::Ptr > recorded_cloud;
	std::vector<cv::Mat> recorded_rgb;
	int record_id;
	boost::mutex record_mutex;

public:
	MainLoop () : done(false), cloud_(), record_id(0)/*, capturing(true), recording(false)*/ {}

	//process a new frame of point cloud
	void onNewCloud (const ColorCloud::ConstPtr &cloud)
	{
		boost::mutex::scoped_lock lock(record_mutex);
		cloud_ = cloud;

		//fill RGB
		if (rgb.empty() || rgb.rows != cloud->height || rgb.cols != cloud->width) {
			rgb.create(cloud->height, cloud->width, CV_8UC3);
		}
		for (int i = 0; i < (int)cloud->height; ++i) {
			for (int j = 0; j < (int)cloud->width; ++j) {
				const pcl::PointXYZRGBA& p = cloud->at(j, i);
				if (!pcl_isnan(p.z)) {
					rgb.at<cv::Vec3b>(i, j) = cv::Vec3b(p.b, p.g, p.r);
				} else {
					static const cv::Vec3b white(255, 255, 255);
					rgb.at<cv::Vec3b>(i, j) = white;//whiten invalid area
				}
			}
		}

		//show frame rate
		std::stringstream stext;
		stext << "id=" << recorded_rgb.size();
		cv::putText(rgb, stext.str(), cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 1));

		//cv::imshow("rgb", rgb); //calling imshow inside capture thread seems not a good idea for tablet
	}

	//start the main loop
	void run ()
	{
#ifdef USE_OPENNI1
		pcl::Grabber* grabber = new pcl::OpenNIGrabber();
#else
		pcl::Grabber* grabber = new pcl::io::OpenNI2Grabber();
#endif

		boost::function<void (const ColorCloud::ConstPtr&)> f =
			boost::bind (&MainLoop::onNewCloud, this, _1);

		grabber->registerCallback(f);

		cv::namedWindow("rgb");
		cv::namedWindow("recorded");
		const int baseX = 80, baseY = 10;
		cv::moveWindow("rgb", baseX, baseY);
		cv::moveWindow("recorded", baseX, baseY + 10 + 480);

		//grabbing loop
		grabber->start();

		//GUI loop
		while (!done)
		{
			onKey(cv::waitKey(50)); //update parameter once per second
		}

		grabber->stop();
		delete grabber;
	}

	void record_current() {
		{
			boost::mutex::scoped_lock lock(record_mutex);
			if (!cloud_) return;
			//recording = true;
			recorded_rgb.push_back(rgb.clone());
			recorded_cloud.push_back(ColorCloud::Ptr(new ColorCloud(*cloud_)));
		}

		if (recorded_rgb.size() <= 0) return;
		cv::imshow("recorded", recorded_rgb.back());
		record_id = recorded_rgb.size() - 1;
		std::cout << "#recorded=" << recorded_rgb.size() << std::endl;
	}

	void save(const std::string& outputPrefix) {
		for (int k = 0; k < (int)recorded_cloud.size(); ++k) {
			std::string name = outputPrefix + cv::format("/record_%05d.pcd", k);
			const ColorCloud::Ptr& cloud = recorded_cloud[k];

			RGBCloud ocloud(cloud->width, cloud->height);
			for (int i = 0; i<(int)cloud->height; ++i) {
				for (int j = 0; j<(int)cloud->width; ++j) {
					const pcl::PointXYZRGBA& p = cloud->at(j, i);
					pcl::PointXYZRGB& po = ocloud.at(j, i);
					po.x = p.x; po.y = p.y; po.z = p.z;
					po.r = p.r; po.g = p.g; po.b = p.b;
				}
			}

			pcl::io::savePCDFileBinary(name, ocloud);
			std::cout << "saved: " << name << std::endl;
		}
	}

	//handle keyboard commands
	void onKey(const unsigned char key)
	{
		static bool stop=false;
		switch(key) {
		case 'q':
			this->done=true;
			break;
		case 'r':
			record_current();
			break;
		case ',':
			if (recorded_rgb.size() > 0) {
				--record_id;
				if (record_id < 0) record_id = recorded_rgb.size() - 1;
				cv::imshow("recorded", recorded_rgb[record_id]);
			}
			break;
		case '.':
			if (recorded_rgb.size()>0) {
				record_id = (record_id + 1) % recorded_rgb.size();
				cv::imshow("recorded", recorded_rgb[record_id]);
			}
			break;
		case '-':
			if (recorded_rgb.size() > 0) {
				recorded_rgb.pop_back();
				recorded_cloud.pop_back();
				record_id = recorded_rgb.size() - 1;
				if (record_id >= 0) {
					cv::imshow("recorded", recorded_rgb[record_id]);
				} else {
					cv::imshow("recorded", cv::Mat::zeros(480,640,CV_8UC1));
				}
				std::cout << "deleted last recorded frame! (" << recorded_rgb.size() << " frames left)." << std::endl;
			}
			break;
		}
		if(!rgb.empty())
			cv::imshow("rgb", rgb);
	}
};

int main (const int argc, const char** argv)
{
	if (argc != 2) {
		std::cout << "usage: " << argv[0] << " <pcd/record/dir/>" << std::endl;
		return 0;
	}

	const std::string outputDir(argv[1]);
	{//create outputDir
#ifdef _WIN32
		std::string cmd = "mkdir " + outputDir;
#else
		std::string cmd = "mkdir -p " + outputDir;
#endif
		system(cmd.c_str());
	}

	MainLoop loop;
	loop.run();

	loop.save(outputDir);
	return 0;
}