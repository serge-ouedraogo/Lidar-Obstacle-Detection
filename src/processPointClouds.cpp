// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
//#include "kdtree.h"

using namespace std;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::ransacplane(typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIter, float distanceTol)
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


template<typename PointT>
void ProcessPointClouds<PointT>::clusterhelper(int index, const std::vector< std::vector<float> >&points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree *tree, float distanceTol)  
{
  processed[index] = true;
  cluster.push_back(index);
  std::vector<int> nearby = tree->search(points[index], distanceTol);
  for(int id: nearby)
  {
    if(!processed[id])
    {
      clusterhelper(id, points, cluster, processed, tree, distanceTol);
    }
  }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
  // TODO: Fill out this function to return list of indices for each cluster
  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed(points.size(), false);
  
  int i =0;
  while(i < points.size())
  {
    if(processed[i])
    {
      i++;
      continue;
    }
    std::vector<int> cluster;
    clusterhelper(i, points, cluster, processed, tree, distanceTol);
    clusters.push_back(cluster);
    i++;
  }
  return clusters;

}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();
  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  
  typename pcl::PointCloud<PointT>::Ptr cloudfiltered (new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudfiltered);
  
  typename pcl::PointCloud<PointT>::Ptr cloudregion (new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setInputCloud(cloudfiltered);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.filter(*cloudregion);
  
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setInputCloud(cloudregion);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.4, -4, 1));
  roof.filter(indices);
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  for(int index = 0; index <indices.size(); ++index)
  {
    inliers->indices.push_back(index);
  }
  pcl::ExtractIndices<PointT>extract;
  extract.setInputCloud(cloudregion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudregion);
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime); 
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
  
  return cloudregion;
}
    

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstcloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr planecloud (new pcl::PointCloud<PointT> ());
  for(int index: inliers->indices)
  {
    planecloud->points.push_back(cloud->points[index]);
  }
  
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true); //remove all the inliers points fro the reference cloud
  extract.filter(*obstcloud);
  
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstcloud, planecloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
  /*
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if(inliers->indices.size()==0)
  {
    cout << " could not estimate planar model" << endl;
  }
  */ 
  std::unordered_set<int> inliers_vect = ransacplane(cloud, maxIterations, distanceThreshold);
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  for (auto index:inliers_vect)
  {
    inliers->indices.push_back(index);
  }
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
  
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
  return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize)  
{
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();
  
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  /*
  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  std::vector<pcl::PointIndices>clusterindices;
  typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
  kdtree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(cloud);
  ec.extract(clusterindices);
  */
  
 
  KdTree* tree = new KdTree;
  
  std::vector<std::vector<float>> points;
  for (int i = 0; i < cloud->points.size(); i++) 
  {
    points.push_back(std::vector<float> {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    tree->insert(std::vector<float> {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);
  }
  std::vector<std::vector<int>> clusterindices = euclideanCluster(points, tree, clusterTolerance);
  for (const auto& clusterIndex : clusterindices) 
  {
    typename pcl::PointCloud<PointT>::Ptr cloudcluster(new pcl::PointCloud<PointT>());
    
    for (const auto pointIndex : clusterIndex) 
    //for (auto pointIndex : clusterIndex.indices) 
    {
      cloudcluster->points.push_back (cloud->points[pointIndex]);
      //std::cout << "CLOUDCLUSTER SIZE = " << cloudcluster->points[pointIndex] << std::endl;
    }
    
    cloudcluster->width = cloudcluster->points.size();
    cloudcluster->height = 1;
    cloudcluster->is_dense = true;
    
    if (cloudcluster->width >= minSize && cloudcluster->width <= maxSize) 
    {
      clusters.push_back(cloudcluster);
    }
  }
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
  return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(const typename pcl::PointCloud<PointT>::Ptr& cluster)  
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}