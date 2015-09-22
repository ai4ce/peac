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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#ifdef USE_OPENNI1
#include <pcl/io/openni_grabber.h>
#else
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
#endif

#include "opencv2/opencv.hpp"

#include "AHCPlaneFitter.hpp"

using ahc::utils::Timer;

// pcl::PointCloud interface for our ahc::PlaneFitter
template<class PointT>
struct OrganizedImage3D {
	const pcl::PointCloud<PointT>& cloud;
	//NOTE: pcl::PointCloud from OpenNI uses meter as unit,
	//while ahc::PlaneFitter assumes mm as unit!!!
	const double unitScaleFactor;

	OrganizedImage3D(const pcl::PointCloud<PointT>& c) : cloud(c), unitScaleFactor(1000) {}
	int width() const { return cloud.width; }
	int height() const { return cloud.height; }
	bool get(const int row, const int col, double& x, double& y, double& z) const {
		const PointT& pt=cloud.at(col,row);
		x=pt.x; y=pt.y; z=pt.z;
		return pcl_isnan(z)==0; //return false if current depth is NaN
	}
};
typedef OrganizedImage3D<pcl::PointXYZRGBA> RGBDImage;
typedef ahc::PlaneFitter<RGBDImage> PlaneFitter;

class MainLoop
{
protected:
	PlaneFitter pf;
	cv::Mat rgb, seg;
	bool done;

public:
	MainLoop () : done(false) {}

	//process a new frame of point cloud
	void onNewCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		//fill RGB
		if(rgb.empty() || rgb.rows!=cloud->height || rgb.cols!=cloud->width) {
			rgb.create(cloud->height, cloud->width, CV_8UC3);
			seg.create(cloud->height, cloud->width, CV_8UC3);
		}
		for(int i=0; i<(int)cloud->height; ++i) {
			for(int j=0; j<(int)cloud->width; ++j) {
				const pcl::PointXYZRGBA& p=cloud->at(j,i);
				if(!pcl_isnan(p.z)) {
					rgb.at<cv::Vec3b>(i,j)=cv::Vec3b(p.b,p.g,p.r);
				} else {
					rgb.at<cv::Vec3b>(i,j)=cv::Vec3b(255,255,255);//whiten invalid area
				}
			}
		}

		//run PlaneFitter on the current frame of point cloud
		RGBDImage rgbd(*cloud);
		Timer timer(1000);
		timer.tic();
		pf.run(&rgbd, 0, &seg);
		double process_ms=timer.toc();

		//blend segmentation with rgb
		cv::cvtColor(seg,seg,CV_RGB2BGR);
		seg=(rgb+seg)/2.0;
		
		//show frame rate
		std::stringstream stext;
		stext<<"Frame Rate: "<<(1000.0/process_ms)<<"Hz";
		cv::putText(seg, stext.str(), cv::Point(15,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,1));

		cv::imshow("rgb", rgb);
		cv::imshow("seg", seg);
	}

	//start the main loop
	void run ()
	{
#ifdef USE_OPENNI1
		pcl::Grabber* grabber = new pcl::OpenNIGrabber();
#else
		pcl::Grabber* grabber = new pcl::io::OpenNI2Grabber();
#endif

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind (&MainLoop::onNewCloud, this, _1);

		grabber->registerCallback(f);

		//grabbing loop
		grabber->start();

		cv::namedWindow("rgb");
		cv::namedWindow("seg");
		cv::namedWindow("control", cv::WINDOW_NORMAL);
		
		int mergeMSETol=(int)pf.params.stdTol_merge,
			minSupport=(int)pf.minSupport,
			doRefine=(int)pf.doRefine;
		cv::createTrackbar("epsilon","control", &mergeMSETol, (int)pf.params.stdTol_merge*2);
		cv::createTrackbar("T_{NUM}","control", &minSupport, pf.minSupport*5);
		cv::createTrackbar("Refine On","control", &doRefine, 1);
		cv::createTrackbar("windowHeight","control", &pf.windowHeight, 2*pf.windowHeight);
		cv::createTrackbar("windowWidth","control", &pf.windowWidth, 2*pf.windowWidth);

		//GUI loop
		while (!done)
		{
			pf.params.stdTol_merge=(double)mergeMSETol;
			pf.minSupport=minSupport;
			pf.doRefine=doRefine!=0;
			onKey(cv::waitKey(1000));
		}

		grabber->stop();
	}

	//handle keyboard commands
	void onKey(const unsigned char key)
	{
		static bool stop=false;
		switch(key) {
		case 'q':
			this->done=true;
			break;
		}
	}
};

int main ()
{
	MainLoop loop;
	loop.run();
	return 0;
}