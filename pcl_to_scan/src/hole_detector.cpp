/*                                                                                                                       
 * Copyright (c) 2010, Willow Garage, Inc.                                                                               
 * All rights reserved.                                                                                                  
 *                                                                                                                       
 * Redistribution and use in source and binary forms, with or without                                                    
 * modification, are permitted provided that the following conditions are met:                                           
 *                                                                                                                       
 *     * Redistributions of source code must retain the above copyright                                                  
 *       notice, this list of conditions and the following disclaimer.                                                   
 *     * Redistributions in binary form must reproduce the above copyright                                               
 *       notice, this list of conditions and the following disclaimer in the                                             
 *       documentation and/or other materials provided with the distribution.                                            
 *     * Neither the name of the Willow Garage, Inc. nor the names of its                                                
 *       contributors may be used to endorse or promote products derived from                                            
 *       this software without specific prior written permission.                                                        
 *                                                                                                                       
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"                                           
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE                                             
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE                                            
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE                                              
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR                                                   
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF                                                  
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS                                              
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN                                               
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)                                               
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE                                            
 * POSSIBILITY OF SUCH DAMAGE.                                                                                           
 */
#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"

#include <boost/foreach.hpp>



namespace pcl_to_scan {
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class HoleDetector: public nodelet::Nodelet {
public:
	//Constructor
	HoleDetector() :
		threshold_(0.0) {
	}

private:
	virtual void onInit() {
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& private_nh = getPrivateNodeHandle();


		private_nh.getParam("threshold", threshold_);

		pub_ = nh.advertise<PointCloud> ("output", 10);
		sub_ = nh.subscribe<PointCloud> ("input", 10, &HoleDetector::callback, this);
	}

	void callback(const PointCloud::ConstPtr& msg) {
		PointCloud::Ptr output (new PointCloud);

		output->header = msg->header;
		output->height = msg->height;
		output->width = msg->width;

		BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
			if (pt.z < threshold_)
				output->points.push_back (pt);
		}

		pub_.publish(output);
	}

	double threshold_;

	ros::Publisher pub_;
	ros::Subscriber sub_;

};

PLUGINLIB_DECLARE_CLASS(pcl_to_scan, HoleDetector, pcl_to_scan::HoleDetector, nodelet::Nodelet);
}
