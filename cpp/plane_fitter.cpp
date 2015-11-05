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

//#define DEBUG_ADAPT
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
		x=pt.x*unitScaleFactor; y=pt.y*unitScaleFactor; z=pt.z*unitScaleFactor;
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

//#define USE_ADAPT
#if defined (USE_ADAPT)
		pf.runAdaptive(&rgbd, seg);
#else
		pf.run(&rgbd, 0, &seg);
#endif
		double process_ms=timer.toc();

		//blend segmentation with rgb
		cv::cvtColor(seg,seg,CV_RGB2BGR);
		seg=(rgb+seg)/2.0;
		
		//show frame rate
		std::stringstream stext;
		stext<<"Frame Rate: "<<(1000.0/process_ms)<<"Hz";
		cv::putText(seg, stext.str(), cv::Point(15,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0,1));

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
		const int baseX = 80, baseY = 10;
		cv::moveWindow("rgb", baseX, baseY);
		cv::moveWindow("control", baseX + 10 + 640, baseY);
		cv::moveWindow("seg", baseX, baseY + 10 + 480);

		// related to T_mse
		int depthSigma = (int)(pf.params.depthSigma * 1e7);
		int mergeMSETol = (int)pf.params.stdTol_merge;
		int initMSETol = (int)pf.params.stdTol_init;
		cv::createTrackbar("depthSigma", "control", &depthSigma, 5 * depthSigma);
		cv::createTrackbar("epsilon", "control", &mergeMSETol, 2 * mergeMSETol);
		cv::createTrackbar("eps_init", "control", &initMSETol, 2 * initMSETol);

		// related to T_ang
		int z_near = (int)pf.params.z_near;
		int z_far = (int)pf.params.z_far;
		int angle_near = (int)15;
		int angle_far = (int)90;
		int simThMergeDeg = 60;
		int simThRefDeg = 30;
		cv::createTrackbar("z_near", "control", &z_near, 2 * z_near);
		cv::createTrackbar("z_far", "control", &z_far, 2 * z_far);
		cv::createTrackbar("angle_near", "control", &angle_near, 90);
		cv::createTrackbar("angle_far", "control", &angle_far, 90);
		cv::createTrackbar("simThMergeDeg", "control", &simThMergeDeg, 90);
		cv::createTrackbar("simThRefDeg", "control", &simThRefDeg, 90);

		// related to T_dz
		int depthAlpha = (int)(pf.params.depthAlpha * 100);
		int depthChangeTol = (int)(pf.params.depthChangeTol * 100);
		cv::createTrackbar("depthAlpha", "control", &depthAlpha, 2 * depthAlpha);
		cv::createTrackbar("depthChangeTol", "control", &depthChangeTol, 2 * depthChangeTol);

		// other
		int minSupport = (int)pf.minSupport;
		int doRefine = (int)pf.doRefine;
		cv::createTrackbar("T_{NUM}","control", &minSupport, pf.minSupport*5);
		cv::createTrackbar("Refine On","control", &doRefine, 1);
		cv::createTrackbar("windowHeight","control", &(pf.windowHeight), 2*pf.windowHeight);
		cv::createTrackbar("windowWidth","control", &(pf.windowWidth), 2*pf.windowWidth);
		int erodeType = (int)pf.erodeType;
		cv::createTrackbar("erodeType", "control", &erodeType, 2);

		//GUI loop
		while (!done)
		{
			static bool reported = false;
			pf.params.depthSigma = depthSigma*1e-7;
			pf.params.stdTol_init = (double)initMSETol;
			pf.params.stdTol_merge = (double)mergeMSETol;

			pf.params.z_near = (double)z_near;
			pf.params.z_far = (double)z_far;
			pf.params.angle_near = (double)MACRO_DEG2RAD(angle_near);
			pf.params.angle_far = (double)MACRO_DEG2RAD(angle_far);
			pf.params.similarityTh_merge = std::cos(MACRO_DEG2RAD(simThMergeDeg));
			pf.params.similarityTh_refine = std::cos(MACRO_DEG2RAD(simThRefDeg));

			pf.params.depthAlpha = (double)depthAlpha*0.01;
			pf.params.depthChangeTol = (double)depthChangeTol*0.01;

			pf.minSupport=minSupport;
			pf.doRefine=doRefine!=0;
			pf.erodeType = (ahc::ErodeType)erodeType;

			onKey(cv::waitKey(1000)); //update parameter once per second
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