#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <custom_msgs/CustomMessage.h>

using namespace std;

ros::Publisher pub_cluster;          //클러스터링 퍼블리셔
ros::Publisher pub_ransaced;         //랜삭 퍼블리셔
// ros::Publisher pub_cluster_custom;   //커스텀 메시지 퍼블리셔

void roiFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
               pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
    //차량 표면을 제외
    float min_x = -2.5f, max_x = 0.0f;
    float min_y = -1.0f, max_y = 1.0f;
    float min_z = -1.0f, max_z = 1.0f;

    for (const auto& point : input->points) {
        if (!((point.x >= min_x && point.x <= max_x) && 
              (point.y >= min_y && point.y <= max_y) &&
              (point.z >= min_z && point.z <= max_z))) {
            output->points.push_back(point);
        }
    }
}

void downSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& output, float leaf_size) {
    //Voxel Grid 필터링:포인트 클라우드를 균일한 크기의 3D 그리드(격자)로 나눈 후, 각 격자 내에서 대표 점(centroid)만 남기는 방식으로 다운샘플링함
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);//너무 작으면 overflow, 작게하면 연산량 증가(작은물체 감지) / 크면 큰 물체나 평면을 잘 감지
    sor.filter(*output);                             //필터링을 수행하고 cloud_downsampled 객체에 필터링 결과를 저장
}

void ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_z_neg,
              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_z_pos) {
    //====RANSAC을 사용하여 평면 추출===
  pcl::SACSegmentation<pcl::PointXYZ> sac;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//추출된 평면에 속하는 포인트들의 인덱스를 저장

  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setModelType(pcl::SACMODEL_PLANE);
  sac.setDistanceThreshold(0.4);
  sac.setOptimizeCoefficients(true);
  sac.setInputCloud(cloud_z_neg);
  sac.segment(*inliers, *coefficients);   //RANSAC을 통해 평면 추출


  //평면 계수 출력
  double A = coefficients->values[0];
  double B = coefficients->values[1];
  double C = coefficients->values[2];
  double D = coefficients->values[3];
  cout << "(A)" << A << " (B)" << B << " (C)" << C << " (D)" << D << endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  if (abs(C - 1.0) < 0.2 && abs(A)< 0.01&& abs(B)< 0.03) {  // C ≈ 1 (수평 평면)
    // cout<<A<<endl;
     cout << "Detected horizontal plane (C is around 1.0)" << endl;
      // 평면을 클라우드에서 제외하기 위해 인덱스를 사용하여 필터링
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud_z_neg);
      extract.setIndices(inliers);
      extract.setNegative(true);  //평면 제외
      extract.filter(*cloud_filtered);
  } else {
      cout << "Skipping non-horizontal plane" << endl;
  }
  
  //나눈 클라우드 합체 
  *cloud_z_neg = *cloud_filtered + *cloud_z_pos;

  sensor_msgs::PointCloud2 cloud_filtered_ros;
  pcl::toROSMsg(*cloud_z_neg, cloud_filtered_ros);
  cloud_filtered_ros.header.frame_id = "map";
  pub_ransaced.publish(cloud_filtered_ros);
}

void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_final) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_final);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(cloud_final);
    ec.setClusterTolerance(1.0);   //거리 기준(반경)을 설정
    ec.setMinClusterSize(8);       //클러스터 최소 크기 설정
    ec.setMaxClusterSize(20000);
    ec.setSearchMethod(tree);      //Kd-Tree를 이용하여 인접한 포인트들을 효율적으로 검색
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroids(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& it : cluster_indices) {
        pcl::PointXYZ centroid(0, 0, 0);
        for (const auto& pit : it.indices) {
            centroid.x += cloud_final->points[pit].x;
            centroid.y += cloud_final->points[pit].y;
            centroid.z += cloud_final->points[pit].z;
        }
        centroid.x /= it.indices.size();
        centroid.y /= it.indices.size();
        centroid.z /= it.indices.size();

        // custom_msgs::CustomMessage cluster_msg;
        // cluster_msg.x = centroid.x;
        // cluster_msg.y = centroid.y;
        // cluster_msg.z = centroid.z;

        // pub_cluster_custom.publish(cluster_msg);
        cloud_centroids->points.push_back(centroid);
    }

    sensor_msgs::PointCloud2 cloud_centroids_ros;
    pcl::toROSMsg(*cloud_centroids, cloud_centroids_ros);
    cloud_centroids_ros.header.frame_id = "map";
    pub_cluster.publish(cloud_centroids_ros);
}

void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& input) {
    //====ROS 메시지를 PCL 포인트 클라우드로 변환=== 초기 입력 데이터 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    //====ROI 필터링====
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    roiFilter(cloud, cloud_roi);

    //====필터링 및 클러스터링 코드==== 필터링 결과 저장 (다운 샘플링 된)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    downSampling(cloud_roi, cloud_downsampled, 0.07f);

    //최종적으로 평면이 제거된 포인트 클라우드 저장할 변수
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_neg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_pos(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : cloud_downsampled->points) {
        if (point.z <= -0.5) {
            cloud_z_neg->points.push_back(point);
        } else {
            cloud_z_pos->points.push_back(point);
        }
    }

    //====RANSAC을 사용하여 평면 추출===
    ransac(cloud_z_neg,cloud_z_pos);  // Z ≤ -0.5 영역에 대해서만 RANSAC 수행

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_final = *cloud_z_neg; //다시 병합

    //====클러스터링==== 랜삭된 포인트 클라우드에서 클러스터링 수행      //itensity로 색상 지정 가능
    clustering(cloud_final); // 클러스터링 수행
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_clustering_node");
    ros::NodeHandle nh;

    pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/clustered_points", 1);
    pub_ransaced = nh.advertise<sensor_msgs::PointCloud2>("/ransaced_points", 1);
    // pub_cluster_custom = nh.advertise<custom_msgs::CustomMessage>("/clustered_custom", 1);

    ros::Subscriber sub = nh.subscribe("/lidar3D", 1, cloudCallBack);

    ros::spin();
    return 0;
}
