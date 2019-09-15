/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  
  // TODO: Fill in this function
  for (int i =0; i < maxIterations; ++i)
  {
    auto index = rand()%cloud->points.size();
    std::unordered_set<int> inliers;
    float x[2];
    float y[2];
    int j =0; 
    while(inliers.size() < 2)
    {
      x[j] = cloud->points[index].x; 
      y[j] = cloud->points[index].y;
      if(inliers.count(index)==0)
      {
        inliers.insert(index);
        ++j;
      }
    }
    
    float a = y[1] - y[0];
    float b = x[0] - x[1];
    float c = y[0]*x[1] - y[1]*x[0];
    float denom = sqrt(a*a + b*b);
    
    for(int index = 0; index<cloud->points.size(); ++index)
    {
      float x = cloud->points[index].x;
      float y = cloud->points[index].y;
      
      float distance = fabs(a*x + b*y +c )/denom; 
      if(distance < distanceTol)
      {
        inliers.insert(index);
      } 
    }
    if(inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    } 
  }
  return inliersResult;
}

std::unordered_set<int> ransacplane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int maxIter, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  
  for(int i = 0; i < maxIter; ++i)
  {
    std::unordered_set<int> inliers;
    float x[3];
    float y[3];
    float z[3];
    int j = 0;
    
    while(inliers.size() < 3)
    {
      auto index = rand()%cloud->points.size();
      
      x[j] = cloud->points[index].x;
      y[j] = cloud->points[index].y;
      z[j] = cloud->points[index].z;
   
      if(inliers.count(index)==0)
      {
        inliers.insert(index);
        ++j;
      }  
    }
    
    float v1[3] = {x[1] - x[0], y[1] - y[0], z[1] - z[0]};
    float v2[3] = {x[2] - x[0], y[2] - y[0], z[2] - z[0]};
      
    float a = v1[1]*v2[2] - v1[2]*v2[1];
    float b = v1[2]*v2[0] - v1[0]*v2[2];
    float c = v1[0]*v2[1] - v1[1]*v2[0];
    float d = -(a*x[0] + b*y[0] + c*z[0]);
      
    float denominator = sqrt(a*a + b*b + c*c); 
    
    for(int index = 0; index < cloud->points.size(); ++index)
    { 
      float x = cloud->points[index].x;
      float y = cloud->points[index].y;
      float z = cloud->points[index].z;
      
      float distance = fabs(a*x + b*y + c*z + d) / denominator;
      if(distance < distanceTol)
      {
        inliers.insert(index);
      }
    }
    if(inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }
  }
  return inliersResult; 
}

int main ()
{

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
  // TODO: Change the max iteration and distance tolerance arguments for Ransac function
  //std::unordered_set<int> inliers = Ransac(cloud, 0, 0);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
  std::unordered_set<int> inliers = ransacplane(cloud, 50, 0.5);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
