/*
	Example of Cython and catkin integration
	Copyright (C) 2014  Marco Esposito <marcoesposito1988@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cython_catkin_example.h>

#include <pcl/io/pcd_io.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::poses;


using namespace std;
using namespace pcl;


void transferPclPointCloudToXYPointsMap(CCExample::PCloud::Ptr &input_pc,  CSimplePointsMap*  point_map)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  //pcl::fromPCLPointCloud2(*input_pc, cloud);
  cloud = CCExample::specializeCloud<pcl::PointXYZ>(input_pc);
  std::vector<float> xs, ys;
  //std::cout << "transfer: " << cloud->points.size() << std::endl;
  for (size_t next = 0; next < cloud->points.size(); ++next)
  {
    //velodyne_pointcloud::PointXYZIR _point = input_pc->points.at(next);
    pcl::PointXYZ _point = cloud->points.at(next);
    xs.push_back(_point.x);
    ys.push_back(_point.y);
    //if (next == 100) {cout << _point.x << " " << _point.y << endl;}
  }
  point_map->setAllPoints(xs, ys);

}

std::vector<float> CCExample::processICP(float init_x, float init_y, float init_yaw)
{
  stringstream ss;
  std::vector<float> result;
  if ( pointClouds.find("ref_map") == pointClouds.end() || pointClouds.find("que_map") == pointClouds.end() )
  {
    cout << "cannot icp, ref or que not available" << endl;
    return result;
  }
  //cout << "Processing yeah" << endl;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::fromROSMsg(*cloud_msg , *cloud);
  CSimplePointsMap		g_m1,g_m2;
  transferPclPointCloudToXYPointsMap(pointClouds[string("ref_map")], &g_m1);
  transferPclPointCloudToXYPointsMap(pointClouds[string("que_map")], &g_m2);
  float					runningTime;
  CICP::TReturnInfo		info;
  //CPose2D		initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));
  CPose2D		initialPose(init_x,init_y,(float)DEG2RAD(init_yaw));

  CPosePDFPtr pdf = ICP.Align(
      &g_m1,
      &g_m2,
      //g_m1.get(),
      //g_m2.get(),
      initialPose,
      &runningTime,
      (void*)&info);

  //printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
      //runningTime*1000,
      //info.nIterations,
      //runningTime*1000.0f/info.nIterations,
      //info.goodness*100 );

  //cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

  CPosePDFGaussian  gPdf;
  gPdf.copyFrom(*pdf);
  CPosePDFGaussianInf gInf(gPdf);
  mrpt::math::CMatrixDouble33 information_matrix;
  gInf.getInformationMatrix(information_matrix);


  //cout << "Covariance of estimation: " << endl << gPdf.cov << endl;
  //cout << "Information of estimation: " << endl << information_matrix << endl;

  //cout << " std(x): " << sqrt( gPdf.cov(0,0) ) << endl;
  //cout << " std(y): " << sqrt( gPdf.cov(1,1) ) << endl;
  //cout << " std(phi): " << RAD2DEG(sqrt( gPdf.cov(2,2) )) << " (deg)" << endl;

  mrpt::math::CVectorDouble icp_result;
  pdf->getMeanVal().getAsVector(icp_result);
  //cout << icp_result << endl;

  //result[0] = icp_result[0];
  //result[1] = icp_result[1];
  //result[2] = icp_result[2];
  result.push_back(icp_result[0]);
  result.push_back(icp_result[1]);
  result.push_back(icp_result[2]);
  result.push_back(information_matrix(0,0));
  result.push_back(information_matrix(0,1));
  result.push_back(information_matrix(1,1));
  result.push_back(information_matrix(2,2));
  result.push_back(information_matrix(0,2));
  result.push_back(information_matrix(1,2));
  return result;

}


map<string, float> CCExample::getPointXYZCloudDetails(string cloudName) {
    map<string, float> details;
    PCloud::ConstPtr cloud;
    try {
        cloud = pointClouds.at(cloudName);
    } catch (out_of_range) {
        return details;
    }

    details["width"] = cloud->width;
    details["height"] = cloud->height;
    details["data_size"] = cloud->data.size();

    for (unsigned int i = 0; i < cloud->fields.size(); ++i) {
        stringstream ss;
        ss << "Field " << i << " : " << cloud->fields[i] << endl;
        details[ss.str()] = 0;
    }

    return details;
}

bool CCExample::loadPointCloudFromArrays(const string& destCloud,
                                            int n,
                                            float* xarr,
                                            float* yarr,
                                            float* zarr) {
    PointCloud<PointXYZ>::Ptr temp(new PointCloud<PointXYZ>);
	for (int i = 0; i < n; i++) {
		temp->push_back(PointXYZ(xarr[i],yarr[i],zarr[i]));
	}
    pointClouds[destCloud] = generalizeCloud<PointXYZ>(temp);
    return true;
}

bool CCExample::loadPointCloudFrom2DArrays(const string& destCloud,
    int n,
    float* xarr,
    float* yarr) {
  PointCloud<PointXYZ>::Ptr temp(new PointCloud<PointXYZ>);
  for (int i = 0; i < n; i++) {
    temp->push_back(PointXYZ(xarr[i],yarr[i],0));
  }
  pointClouds[destCloud] = generalizeCloud<PointXYZ>(temp);
  return true;
}


void CCExample::deletePointCloud(const string& cloudName) {
	pointClouds.erase(cloudName);
}

// implementation - accessible for debug

bool CCExample::loadPointCloudFromFile(const string& destCloud, const string& filename) {
    PCloud::Ptr cloud(new PCloud);
	if (io::loadPCDFile(filename, *cloud) == -1) {
		return false;
	}
	pointClouds[destCloud] = cloud;
    return true;
}

bool CCExample::savePointCloudToFile(const string& srcCloud, const string& filename) {
	PCloud::Ptr cloud;
	
	try {
        cloud = pointClouds.at(srcCloud);
    } catch (out_of_range) {
        return false;
    }
    
    if (!cloud)
        return false;
	
	std::string extension = filename.substr(filename.find_last_of (".") + 1);
    if (extension == "pcd")
        return !(io::savePCDFile(filename, *cloud));
    else
        return false;

	return true;
}

template<typename T>
typename PointCloud<T>::Ptr CCExample::specializeCloud(const PCloud::ConstPtr &srcCloud) {
    typename PointCloud<T>::Ptr ret(new PointCloud<T>);
    fromPCLPointCloud2(*srcCloud,*ret);
    return ret;
}

template<typename T>
CCExample::PCloud::Ptr CCExample::generalizeCloud(const typename pcl::PointCloud<T>::Ptr& srcCloud) {
    PCloud::Ptr ret(new PCloud);
    toPCLPointCloud2(*srcCloud,*ret);
    return ret;
}
