#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <lidar_package/lane_arr.h>
// #include <lidar_package/lane.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>  // 🔹 MovingLeastSquares를 위한 헤더
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h> 
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/Marker.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>


// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/passthrough.h>



#include <cmath> // For atan2 and M_PI

ros::Publisher pub_lane;
ros::Publisher pub_lane_vis;
ros::Publisher pub_lane_filtered;
ros::Publisher pub_cloud_roi;
ros::Publisher pub_test;
ros::Publisher pub_test2;
ros::Publisher pub_cluster;
ros::Publisher pub_stop_lane_vis;
ros::Publisher pub_stop_lane_roi;
ros::Publisher pub_arrow_vis;
ros::Publisher pub_stop_lane_cluster;
ros::Publisher pub_front_marker;
ros::Publisher pub_left_marker;
ros::Publisher pub_right_marker;
ros:: Publisher pub_curve_marker;
ros:: Publisher pub_object;
ros::Publisher pub_object_cluster;
ros::Publisher pub_object_bound_box_marker;

double stop_cluster_x_id;
bool stop_lane = false;
bool right_left_off = false;


using namespace std;

struct ParabolicCurve {
    double a, b, c;  // y = ax^2 + bx + c
};

ParabolicCurve fitParabolicCurve(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) {
    if (cluster->points.size() < 3) {
        std::cerr << "Error: Not enough points for parabolic fitting!" << cluster->points.size()<<endl;
        return {0, 0, 0};
    }

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    // sor.setInputCloud(cluster);
    // sor.setMeanK(40);  // 인접한 50개 점의 평균을 기준으로
    // sor.setStddevMulThresh(1.0);  // 표준편차 기준으로 이상치 제거
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cluster(new pcl::PointCloud<pcl::PointXYZI>());
    // sor.filter(*filtered_cluster);

    // // 2. Pass-Through 필터를 사용하여 특정 범위 내의 점만 사용
    // pcl::PassThrough<pcl::PointXYZI> pass;
    // pass.setInputCloud(filtered_cluster);
    // pass.setFilterFieldName("y");  // Y축을 기준으로 필터링
    // pass.setFilterLimits(-10.0, 10.0);  // Y축 값이 -10에서 10 사이의 점만 선택
    // pass.filter(*filtered_cluster);  // 필터링된 클러스터

    Eigen::MatrixXd A(cluster->points.size(), 3);
    Eigen::VectorXd Y(cluster->points.size());

    for (size_t i = 0; i < cluster->points.size(); ++i) {
        double x = cluster->points[i].x;
        double y = cluster->points[i].y;
        A(i, 0) = x * x;  //x^2 항
        A(i, 1) = x;      //x 항
        A(i, 2) = 1.0;    //상수항
        Y(i) = y;
    }

    //최소제곱법으로 계수 (a, b, c) 계산
    Eigen::VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * Y);

    ParabolicCurve curve;
    curve.a = coeffs(0);
    curve.b = coeffs(1);
    curve.c = coeffs(2);
    cout<<"curve :"<<curve.a<<endl;

    return curve;
}

void publishParabola(const ParabolicCurve& curve, const std::string& ns,
    int id, float r, float g, float b, double min_x,double max_x){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    for (double x = min_x; x <= max_x; x += 0.1)
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = curve.a * x * x + curve.b * x + curve.c;
        p.z = 0.0;
        marker.points.push_back(p);
    }

    pub_curve_marker.publish(marker);
}

void publishFrontLaneFillArea(const ParabolicCurve& left_curve, const ParabolicCurve& right_curve, 
                         const std::string& ns, int id, float r, float g, float b, 
                         double min_x, double max_x) {
    visualization_msgs::Marker fill_marker;
    fill_marker.header.frame_id = "map";
    fill_marker.header.stamp = ros::Time::now();
    fill_marker.ns = ns;
    fill_marker.id = id;
    fill_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 삼각형으로 채우기
    fill_marker.action = visualization_msgs::Marker::ADD;

    fill_marker.pose.orientation.w = 1.0;
    fill_marker.pose.orientation.x = 0.0;
    fill_marker.pose.orientation.y = 0.0;
    fill_marker.pose.orientation.z = 0.0;

    fill_marker.scale.x = 1.0;  
    fill_marker.scale.y = 1.0;
    fill_marker.scale.z = 1.0;

    fill_marker.color.r = r;
    fill_marker.color.g = g;
    fill_marker.color.b = b;
    fill_marker.color.a = 0.5;

    double dx = 0.1;
    // if (fabs(left_curve.a) > 0.01 || fabs(right_curve.a) > 0.01) { 
    //     dx = 0.05;
    // }

    std::vector<geometry_msgs::Point> left_points, right_points;
    
    for (double x = min_x; x <= max_x; x += dx) {
        geometry_msgs::Point left_p, right_p;
        left_p.x = right_p.x = x;
        left_p.y = left_curve.a * x * x + left_curve.b * x + left_curve.c;
        right_p.y = right_curve.a * x * x + right_curve.b * x + right_curve.c;
        left_p.z = right_p.z = 0.0;

        left_points.push_back(left_p);
        right_points.push_back(right_p);
    }

    if (left_points.size() < 2 || right_points.size() < 2) {
        ROS_WARN("Not enough points to create lane fill area.");
        return;
    }

    for (size_t i = 0; i < left_points.size() - 1; i++) {
        fill_marker.points.push_back(left_points[i]);   // 왼쪽 차선 점
        fill_marker.points.push_back(right_points[i]);  // 오른쪽 차선 점
        fill_marker.points.push_back(left_points[i+1]); // 다음 왼쪽 점

        fill_marker.points.push_back(right_points[i]);   // 오른쪽 차선 점
        fill_marker.points.push_back(right_points[i+1]); // 다음 오른쪽 점
        fill_marker.points.push_back(left_points[i+1]);  // 다음 왼쪽 점
    }

    pub_front_marker.publish(fill_marker);
}

void publishRightLaneFillArea(const ParabolicCurve& left_curve, const ParabolicCurve& right_curve, 
                             const std::string& ns, int id, float r, float g, float b, 
                             double min_x, double max_x) {
    visualization_msgs::Marker fill_marker;
    fill_marker.header.frame_id = "map";
    fill_marker.header.stamp = ros::Time::now();
    fill_marker.ns = ns;
    fill_marker.id = id;
    fill_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    fill_marker.action = visualization_msgs::Marker::ADD;

    fill_marker.pose.orientation.w = 1.0;
    fill_marker.pose.orientation.x = 0.0;
    fill_marker.pose.orientation.y = 0.0;
    fill_marker.pose.orientation.z = 0.0;

    fill_marker.scale.x = 1.0;
    fill_marker.scale.y = 1.0;
    fill_marker.scale.z = 1.0;

    fill_marker.color.r = r;
    fill_marker.color.g = g;
    fill_marker.color.b = b;
    fill_marker.color.a = 0.5;

    double dx = 0.1;
    // if (fabs(left_curve.a) > 0.01 || fabs(right_curve.a) > 0.01) {  
    //     dx = 0.05;
    // }

    std::vector<geometry_msgs::Point> inner_left_points, outer_left_points;

    for (double x = min_x; x <= max_x; x += dx) {
        geometry_msgs::Point inner_p, outer_p;
        inner_p.x = outer_p.x = x;

        // 왼쪽 차선과 오른쪽 차선 사이 거리(차선 폭)를 계산
        double left_y = left_curve.a * x * x + left_curve.b * x + left_curve.c;
        double right_y = right_curve.a * x * x + right_curve.b * x + right_curve.c;
        double lane_width = fabs(right_y - left_y);  // 차선 폭

        inner_p.y = right_y;    // 왼쪽 차선을 한 차선 폭만큼 왼쪽으로 이동
        outer_p.y = right_y - 3.2; // 추가로 한 차선 폭 더 이동
        inner_p.z = outer_p.z = 0.0;

        // inner_p.y = left_y - lane_width;    // 왼쪽 차선을 한 차선 폭만큼 왼쪽으로 이동
        // outer_p.y = left_y - 2 * lane_width; // 추가로 한 차선 폭 더 이동
        // inner_p.z = outer_p.z = 0.0;

        inner_left_points.push_back(inner_p);
        outer_left_points.push_back(outer_p);
    }

    if (inner_left_points.size() < 2 || outer_left_points.size() < 2) {
        ROS_WARN("Not enough points to create left lane fill area.");
        return;
    }

    for (size_t i = 0; i < inner_left_points.size() - 1; i++) {
        fill_marker.points.push_back(inner_left_points[i]);
        fill_marker.points.push_back(outer_left_points[i]);
        fill_marker.points.push_back(inner_left_points[i+1]);

        fill_marker.points.push_back(outer_left_points[i]);
        fill_marker.points.push_back(outer_left_points[i+1]);
        fill_marker.points.push_back(inner_left_points[i+1]);
    }

    pub_right_marker.publish(fill_marker);
}

void publishLeftLaneFillArea(const ParabolicCurve& left_curve, const ParabolicCurve& right_curve, 
                             const std::string& ns, int id, float r, float g, float b, 
                             double min_x, double max_x) {
    visualization_msgs::Marker fill_marker;
    fill_marker.header.frame_id = "map";
    fill_marker.header.stamp = ros::Time::now();
    fill_marker.ns = ns;
    fill_marker.id = id;
    fill_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    fill_marker.action = visualization_msgs::Marker::ADD;

    fill_marker.pose.orientation.w = 1.0;
    fill_marker.pose.orientation.x = 0.0;
    fill_marker.pose.orientation.y = 0.0;
    fill_marker.pose.orientation.z = 0.0;

    fill_marker.scale.x = 1.0;
    fill_marker.scale.y = 1.0;
    fill_marker.scale.z = 1.0;

    fill_marker.color.r = r;
    fill_marker.color.g = g;
    fill_marker.color.b = b;
    fill_marker.color.a = 0.5;

    double dx = 0.1;
    if (fabs(left_curve.a) > 0.01 || fabs(right_curve.a) > 0.01) {  
        dx = 0.05;
    }

    std::vector<geometry_msgs::Point> inner_left_points, outer_left_points;

    for (double x = min_x; x <= max_x; x += dx) {
        geometry_msgs::Point inner_p, outer_p;
        inner_p.x = outer_p.x = x;

        // 왼쪽 차선과 오른쪽 차선 사이 거리(차선 폭)를 계산
        double left_y = left_curve.a * x * x + left_curve.b * x + left_curve.c;
        double right_y = right_curve.a * x * x + right_curve.b * x + right_curve.c;
        double lane_width = fabs(right_y - left_y);  // 차선 폭

          inner_p.y = left_y ;    // 왼쪽 차선을 한 차선 폭만큼 오른쪽으로 이동
        outer_p.y = left_y +   3.2; // 추가로 한 차선 폭 더 이동
        inner_p.z = outer_p.z = 0.0;
        // 왼쪽 차선의 마커는 오른쪽 차선의 반대 방향에 위치시킴
        // inner_p.y = right_y + lane_width;    // 왼쪽 차선을 한 차선 폭만큼 오른쪽으로 이동
        // outer_p.y = right_y + 2 * lane_width; // 추가로 한 차선 폭 더 이동
        // inner_p.z = outer_p.z = 0.0;

        inner_left_points.push_back(outer_p);
        outer_left_points.push_back(inner_p);
    }

    if (inner_left_points.size() < 2 || outer_left_points.size() < 2) {
        ROS_WARN("Not enough points to create left lane fill area.");
        return;
    }

    for (size_t i = 0; i < inner_left_points.size() - 1; i++) {
        fill_marker.points.push_back(inner_left_points[i]);
        fill_marker.points.push_back(outer_left_points[i]);
        fill_marker.points.push_back(inner_left_points[i+1]);

        fill_marker.points.push_back(outer_left_points[i]);
        fill_marker.points.push_back(outer_left_points[i+1]);
        fill_marker.points.push_back(inner_left_points[i+1]);
    }

    pub_left_marker.publish(fill_marker);
}

void findMinMaxX(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, double& min_x, double& max_x, double& stop_cluster_x_id, bool& stop_lane) {
    if (cluster->empty()) return;

    min_x = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::lowest();

    for (const auto& point : cluster->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
    }

    if (stop_lane == true){
        max_x = stop_cluster_x_id;
        // stop_cluster_x_id = 100;
    }
            // cout<<"MAX"<<max_x<<endl;

}




//흰선 0.5 보다 큼 
//노랑 0.2~0.33
void roiFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, 
               pcl::PointCloud<pcl::PointXYZI>::Ptr& left_output,
               pcl::PointCloud<pcl::PointXYZI>::Ptr& right_output,
               pcl::PointCloud<pcl::PointXYZI>::Ptr& stop_output,
               bool& stop_lane,
               double& stop_cluster_x_id,
                pcl::PointCloud<pcl::PointXYZI>::Ptr& object_output
                ) {

    //오른쪽 앞 roi
    double min_right_front_x = 1.7f, max_right_front_x = 20.7f;    //16.7
    float min_right_front_y = -2.3f, max_right_front_y = -0.8f;
    float max_right_front_z = 0.0f;

    //왼쪽 앞 roi
    double min_left_front_x = 1.7f, max_left_front_x = 20.7f;     //16.7
    float min_left_front_y = 0.8f, max_left_front_y = 2.3f;
    float max_left_front_z = 0.0f;

    //정지선 roi
    float min_stop_front_x = 0.0f, max_stop_front_x = 25.0f;
    float min_stop_front_y = -1.1f, max_stop_front_y = 1.1f;
    float max_stop_front_z = 0.0f;

    //객체 roi 
    float min_car_front_x = -2.0f, max_car_front_x = 0.0f;
    float min_car_front_y = -1.1f, max_car_front_y = 1.1f;
    float min_car_front_z = -1.0f, max_car_front_z = 1.0f;


    if (stop_lane){
        max_right_front_x = stop_cluster_x_id;
        max_left_front_x = stop_cluster_x_id;
    }
    cout<<"right x "<<max_right_front_x<<endl;
        cout<<"left x "<<max_left_front_x<<endl;
        cout<<"stop x id "<<stop_cluster_x_id<<endl;

    for (const auto& point : input->points) {
        if ((point.x >= min_right_front_x && point.x <= max_right_front_x) && 
              (point.y >= min_right_front_y && point.y <= max_right_front_y) &&
              (point.z <= max_right_front_z)) {

                      //   std::cout << "X: " << point.x 
                      // << ", Y: " << point.y 
                      // << ", Z: " << point.z 
                      // << ", Intensity: " << point.intensity 
                      // << std::endl;
            right_output->points.push_back(point);
        } 
        if ((point.x >= min_left_front_x && point.x <= max_left_front_x) && 
              (point.y >= min_left_front_y && point.y <= max_left_front_y) &&
              (point.z <= max_left_front_z)) {

                      //   std::cout <<", Intensity: " << point.intensity 
                      // << std::endl;
            left_output->points.push_back(point);
        }
        if ((point.x >= min_stop_front_x && point.x <= max_stop_front_x) && 
              (point.y >= min_stop_front_y && point.y <= max_stop_front_y) &&
              (point.z <= max_stop_front_z)) {
                      //   std::cout << "X: " << point.x 
                      // << ", Y: " << point.y 
                      // << ", Z: " << point.z 
                      // << ", Intensity: " << point.intensity 
                      // << std::endl;
            stop_output->points.push_back(point);
        }
   
        if (!((point.x >= min_car_front_x && point.x <= max_car_front_x) && 
              (point.y >= min_car_front_y && point.y <= max_car_front_y) &&
              (point.z >= min_car_front_z && point.z <= max_car_front_z)) &&
               ((point.x>=-5.0&&point.x<=20)&&(point.y >= -10 && point.y <= 10))) {
            object_output->points.push_back(point);
        }
    
    }
}
// void roiFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,pcl::PointCloud<pcl::PointXYZI>::Ptr& object_output){

//     //객체 roi 
//     float min_car_front_x = -2.0f, max_car_front_x = 0.0f;
//     float min_car_front_y = -1.1f, max_car_front_y = 1.1f;
//     float min_car_front_z = -1.0f, max_car_front_z = 1.0f;

//     for (const auto& point : input->points) {
//         if (!((point.x >= min_car_front_x && point.x <= max_car_front_x) && 
//               (point.y >= min_car_front_y && point.y <= max_car_front_y) &&
//               (point.z >= min_car_front_z && point.z <= max_car_front_z)) &&
//                ((point.x>=-5.0&&point.x<=20)&&(point.y >= -10 && point.y <= 10)&&(point.z <= 0))) {
//             object_output->points.push_back(point);
//         }
//     }
// }

void cloudFiltering(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_left_roi,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_right_roi,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_stop_roi,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& stop_white_output,
                pcl::PointCloud<pcl::PointXYZI>::Ptr& left_white_output,
                pcl::PointCloud<pcl::PointXYZI>::Ptr& left_yellow_output,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr& right_white_output,
                pcl::PointCloud<pcl::PointXYZI>::Ptr& right_yellow_output){
    //0.4 이상 화살표 
    for (const auto& point : cloud_stop_roi->points) {
        if((point.intensity>0.3)){
                // std::cout 
                //        << " Intensity: " << point.intensity 
                //        << std::endl;
            stop_white_output->points.push_back(point);
        }
    }

    for (const auto& point : cloud_left_roi->points) {
        if(point.intensity>0.15&&0.17>point.intensity){  //point.intensity>0.2&&0.329>point.intensity
            left_yellow_output->points.push_back(point);
                // std::cout 
                //       << " Intensity: " << point.intensity 
                //       << std::endl;
        }
        if((point.intensity>0.3)){    // 판교 0.3   고속도로 0.5
                 // std::cout 
                       // << " Intensity: " << point.intensity 
                       // << std::endl;
            left_white_output->points.push_back(point);
        }
    }


    for (const auto& point : cloud_right_roi->points) {
        // std::cout 
                      // << " Intensity: " << point.intensity 
                      // << std::endl;
        if(point.intensity>0.15&&0.17>point.intensity){  //point.intensity>0.2&&0.329>point.intensity)
             // std::cout 
             //          << " Intensity: " << point.intensity 
             //          << std::endl;
            right_yellow_output->points.push_back(point);
        }
        if((point.intensity>0.3)){    //판교 0.3    고속도로 0.5

            right_white_output->points.push_back(point);
        }
    }
}


void clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output,
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters,
    bool& yellow_lane,bool& solid_lane,bool& object_detect) {

    if (cloud_input->empty()) {
        return;
    }
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_input);

    float cluster_range ;   //3.0
    float cluster_min_size;  //10

    if (solid_lane == false) {
        cluster_range = 1.0;
        cluster_min_size = 7;
    } else {
        cluster_range = 0.5;
        cluster_min_size = 5;
    }

    if (yellow_lane == true) {
        cluster_range = 0.5;
        cluster_min_size = 5;
    }

    if (object_detect == true) {
        cluster_range = 0.5;
        cluster_min_size = 8; 
    }

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_input);
    ec.setClusterTolerance(cluster_range);   //거리 기준(반경)을 설정
    ec.setMinClusterSize(cluster_min_size);       //클러스터 최소 크기 설정
    ec.setMaxClusterSize(20000);
    ec.setSearchMethod(tree);      //Kd-Tree를 이용하여 인접한 포인트들을 효율적으로 검색
    ec.extract(cluster_indices);

    for (const auto& it : cluster_indices) {
        if (it.indices.empty()) continue;  // 빈 클러스터 예외 처리
        pcl::PointXYZI centroid;
        centroid.x = centroid.y = centroid.z  = 0.0f;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& pit : it.indices) {
              centroid.x += cloud_input->points[pit].x;
            centroid.y += cloud_input->points[pit].y;
            centroid.z += cloud_input->points[pit].z;
            centroid.intensity += cloud_input->points[pit].intensity;
            cluster->points.push_back(cloud_input->points[pit]);
        }
        float cluster_size = static_cast<float>(it.indices.size());
        centroid.x /= cluster_size;
        centroid.y /= cluster_size;
        centroid.z /= cluster_size;

        cloud_output->points.push_back(centroid);
        clusters.push_back(cluster);  // 개별 클러스터 저장
    }


}

double computeMinDistanceBetweenClusters(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters,
                                pcl::PointXYZI& closest_point1,
                                pcl::PointXYZI& closest_point2,
                                bool& solid_lane) {

    if (clusters.size() < 2) {
        solid_lane = false;

        // std::cerr << "Error: At least two clusters are required!" << std::endl;
        return 10.0;
    }

    double min_distance = std::numeric_limits<double>::max();
  

    for (size_t i = 0; i < clusters.size(); ++i) {
        for (size_t j = i + 1; j < clusters.size(); ++j) {
            const auto& cluster1 = clusters[i];
            const auto& cluster2 = clusters[j];

            for (const auto& point1 : cluster1->points) {
                for (const auto& point2 : cluster2->points) {
                    double distance = std::sqrt(
                        std::pow(point1.x - point2.x, 2) +
                        std::pow(point1.y - point2.y, 2) +
                        std::pow(point1.z - point2.z, 2)
                    );

                    if (distance < min_distance) {
                        min_distance = distance;
                        closest_point1 = point1;
                        closest_point2 = point2;
                    }
                }
            }
        }
    }
    return min_distance;
}

void clusteringAndDetectLaneType(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cluster, bool& solid_lane, bool& yellow_lane, bool& object_detect) {
    pcl::PointXYZI closest1, closest2;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    clustering(input_cloud, output_cluster, clusters,yellow_lane,solid_lane,object_detect);

    // cout<<"vector_size:"<<clusters.size()<<endl;

    double min_distance = computeMinDistanceBetweenClusters(clusters, closest1, closest2,solid_lane);
        cout<<"min_distance:"<<min_distance<<endl;
    if (min_distance>3.0){
        solid_lane = false;
        // if (input_cloud -> size()> 140){
        //     solid_lane = true;
        // }
    } else{
        solid_lane = true;
    }
}

void clusteringDetectToStopLane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output,
                                vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters,
                                float& distance_from_stop,
                                double& stop_cluster_x_id,
                                bool& stop_lane){

    if (cloud_input->empty()) {
        stop_lane = false;
        return;
    }
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_input);
    ec.setClusterTolerance(0.5);   //거리 기준(반경)을 설정
    ec.setMinClusterSize(30);       //클러스터 최소 크기 설정
    ec.setMaxClusterSize(2000);
    ec.setSearchMethod(tree);      //Kd-Tree를 이용하여 인접한 포인트들을 효율적으로 검색
    ec.extract(cluster_indices);
   const float MIN_Y_RANGE = 1.5f;  // 최소 Y축 폭 기준

    for (const auto& it : cluster_indices) {
        if (it.indices.empty()){
            continue;  // 빈 클러스터 예외 처리

        }             

        float y_min = std::numeric_limits<float>::max();
        float y_max = std::numeric_limits<float>::lowest();

        pcl::PointXYZI centroid;
        centroid.x = centroid.y = centroid.z = 0.0f;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);

        for (const auto& pit : it.indices) {
            float y_value = cloud_input->points[pit].y;
            y_min = std::min(y_min, y_value);
            y_max = std::max(y_max, y_value);

            centroid.x += cloud_input->points[pit].x;
            centroid.y += cloud_input->points[pit].y;
            centroid.z += cloud_input->points[pit].z;
            
            cluster->points.push_back(cloud_input->points[pit]);
        }

        float y_range = y_max - y_min;  // Y축 범위 계산

        //Y축 폭이 1.5m 이상이면 클러스터로 선택
        if (y_range >= MIN_Y_RANGE) {
            stop_lane = true;
            float cluster_size = static_cast<float>(cluster->points.size());
            centroid.x /= cluster_size;
            centroid.y /= cluster_size;
            centroid.z /= cluster_size;


            distance_from_stop = std::sqrt(
            centroid.x * centroid.x +
            centroid.y * centroid.y
            // centroid.z * centroid.z
            );
            cloud_output->points.push_back(centroid);
            clusters.push_back(cluster);
            stop_cluster_x_id = centroid.x;
            // cout<<"cluster"<<cluster -> points.size()<<endl;
            sensor_msgs::PointCloud2 cluster_msg;
            pcl::toROSMsg(*cluster, cluster_msg);
            cluster_msg.header.frame_id = "map";  // 좌표계 설정 (필요에 따라 변경)
            cluster_msg.header.stamp = ros::Time::now();
            pub_test.publish(cluster_msg);
            // std::cout << "Distance from STOP LANE: " << distance_from_stop << " meters" << std::endl;
        } else {
            // stop_lane = false;
        }
    }
}
          

void downsampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud_input);
    vg.setLeafSize(0.05f, 0.05f, 0.05f); // 가로, 세로, 높이 단위 (최적화 필요)
    vg.filter(*cloud_filtered);
}

void objectDownSampling(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, 
                  pcl::PointCloud<pcl::PointXYZI>::Ptr& output, float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*output);                          
}

void ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_z_neg,
              pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_z_pos,
              pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ground) {
    //====RANSAC을 사용하여 평면 추출===
  pcl::SACSegmentation<pcl::PointXYZI> sac;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//추출된 평면에 속하는 포인트들의 인덱스를 저장

  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setModelType(pcl::SACMODEL_PLANE);
  sac.setDistanceThreshold(0.4);//origin 0.4
  sac.setOptimizeCoefficients(true);
  sac.setInputCloud(cloud_z_neg);
  sac.segment(*inliers, *coefficients);   //RANSAC을 통해 평면 추출


  //평면 계수 출력
  double A = coefficients->values[0];
  double B = coefficients->values[1];
  double C = coefficients->values[2];
  double D = coefficients->values[3];
  cout << "(A)" << A << " (B)" << B << " (C)" << C << " (D)" << D << endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  if (abs(C - 1.0) < 0.2 && abs(A)< 0.01&& abs(B)< 0.03) {  // C ≈ 1 (수평 평면)
    // cout<<A<<endl;
     cout << "Detected horizontal plane (C is around 1.0)" << endl;
     //
     pcl::copyPointCloud<pcl::PointXYZI>(*cloud_z_neg,*inliers,*cloud_ground);
     //
      // 평면을 클라우드에서 제외하기 위해 인덱스를 사용하여 필터링
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(cloud_z_neg);
      extract.setIndices(inliers);
      extract.setNegative(true);  //평면 제외
      extract.filter(*cloud_filtered);
      //
      
      //
  } else {
      cout << "Skipping non-horizontal plane" << endl;
  }
  
  //나눈 클라우드 합체 
  *cloud_z_neg = *cloud_filtered + *cloud_z_pos;

  // sensor_msgs::PointCloud2 cloud_filtered_ros;
  // pcl::toROSMsg(*cloud_z_neg, cloud_filtered_ros);
  // cloud_filtered_ros.header.frame_id = "map";
  // pub_ransaced.publish(cloud_filtered_ros);
}

struct BoundingBox {
    float x_min, x_max;
    float y_min, y_max;
};
struct ObjectBoundingBox {
    float x_min, x_max;
    float y_min, y_max;
    float z_min, z_max;  // z값 추가 (3D 직육면체를 위해)
};

visualization_msgs::Marker createBoundingBoxMarker(const ObjectBoundingBox& bbox, int id) {

    // visualization_msgs::Marker delete_marker;
    // delete_marker.header.frame_id = "map";
    // delete_marker.header.stamp = ros::Time::now();
    // delete_marker.ns = "bounding_boxes";
    // delete_marker.action = visualization_msgs::Marker::DELETEALL; // 기존 마커 삭제

    // // visualization_msgs::MarkerArray marker_array;
    // // marker_array.markers.push_back(delete_marker);
    
    // pub_object_bound_box_marker.publish(delete_marker); // 퍼블리시하여 기존 마커 제거



    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // RViz 기준 좌표계
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_boxes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; // 선 두께
    marker.scale.y = 0.2;
    marker.color.r = 1.0; // 빨간색
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0; // 투명도

       geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    p1.x = bbox.x_min; p1.y = bbox.y_min; p1.z = bbox.z_min;
    p2.x = bbox.x_max; p2.y = bbox.y_min; p2.z = bbox.z_min;
    p3.x = bbox.x_max; p3.y = bbox.y_max; p3.z = bbox.z_min;
    p4.x = bbox.x_min; p4.y = bbox.y_max; p4.z = bbox.z_min;
    p5.x = bbox.x_min; p5.y = bbox.y_min; p5.z = bbox.z_max;
    p6.x = bbox.x_max; p6.y = bbox.y_min; p6.z = bbox.z_max;
    p7.x = bbox.x_max; p7.y = bbox.y_max; p7.z = bbox.z_max;
    p8.x = bbox.x_min; p8.y = bbox.y_max; p8.z = bbox.z_max;

    // 직육면체의 12개 변을 두 점씩 연결하여 추가
    marker.points.push_back(p1); marker.points.push_back(p2);
    marker.points.push_back(p2); marker.points.push_back(p3);
    marker.points.push_back(p3); marker.points.push_back(p4);
    marker.points.push_back(p4); marker.points.push_back(p1);

    marker.points.push_back(p5); marker.points.push_back(p6);
    marker.points.push_back(p6); marker.points.push_back(p7);
    marker.points.push_back(p7); marker.points.push_back(p8);
    marker.points.push_back(p8); marker.points.push_back(p5);

    marker.points.push_back(p1); marker.points.push_back(p5);
    marker.points.push_back(p2); marker.points.push_back(p6);
    marker.points.push_back(p3); marker.points.push_back(p7);
    marker.points.push_back(p4); marker.points.push_back(p8);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;  // 항등 회전

    return marker;
}


// 파라볼라 함수로 y 값을 계산
double getYForX(const ParabolicCurve& curve, double x) {
    return curve.a * x * x + curve.b * x + curve.c;
}

tuple<vector<BoundingBox>, vector<BoundingBox> ,vector<BoundingBox>> computeBoundingBoxes(const vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters,
                                                 const ParabolicCurve& left_curve, const ParabolicCurve& right_curve) {
    vector<BoundingBox> left_bboxes;
    vector<BoundingBox> right_bboxes;
    vector<BoundingBox> front_bboxes;
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (const auto& cluster : clusters) {
        if (cluster->empty()) continue;

        BoundingBox bbox;
        ObjectBoundingBox object_bbox;
        bbox.x_min = bbox.y_min = numeric_limits<float>::max();
        bbox.x_max = bbox.y_max = numeric_limits<float>::lowest();

        object_bbox.x_min = object_bbox.y_min = object_bbox.z_min= numeric_limits<float>::max();
        object_bbox.x_max = object_bbox.y_max = object_bbox.z_max=numeric_limits<float>::lowest();

        for (const auto& point : cluster->points) {
            bbox.x_min = min(bbox.x_min, point.x);
            bbox.x_max = max(bbox.x_max, point.x);
            bbox.y_min = min(bbox.y_min, point.y);
            bbox.y_max = max(bbox.y_max, point.y);

            object_bbox.x_min = min(object_bbox.x_min, point.x);
            object_bbox.x_max = max(object_bbox.x_max, point.x);
            object_bbox.y_min = min(object_bbox.y_min, point.y);
            object_bbox.y_max = max(object_bbox.y_max, point.y);
            object_bbox.z_min = min(object_bbox.z_min, point.z);
            object_bbox.z_max = max(object_bbox.z_max, point.z);
        }
        bbox.x_min -= 2;
        bbox.x_max += 1;

        visualization_msgs::Marker marker = createBoundingBoxMarker(object_bbox, id++);
        marker_array.markers.push_back(marker);

        // bbox의 중심 y 좌표를 계산
        double center_y = (bbox.y_min + bbox.y_max) / 2.0;
        double left_curve_y = getYForX(left_curve, center_y);  // 왼쪽 커브에서 y 값을 계산
        double right_curve_y = getYForX(right_curve, center_y);  // 오른쪽 커브에서 y 값을 계산

        // 커브 기준으로 분류
        if (center_y > left_curve_y) {
            left_bboxes.push_back(bbox);  // 왼쪽 커브보다 클 경우 왼쪽 차선
        } else if (center_y < right_curve_y) {
            right_bboxes.push_back(bbox);  // 오른쪽 커브보다 작을 경우 오른쪽 차선
        } else if (right_curve_y<center_y<left_curve_y){
            front_bboxes.push_back(bbox);
        }
    }

    pub_object_bound_box_marker.publish(marker_array);

    // 왼쪽과 오른쪽 BoundingBox를 각각 반환
    return make_tuple(left_bboxes, right_bboxes,front_bboxes);
}


void updateMinMaxX(double& min_x, double& max_x, const vector<BoundingBox>& bounding_boxes) {
    double filtered_min_x = min_x;
    double filtered_max_x = max_x;

    for (const auto& bbox : bounding_boxes) {
        if (bbox.x_min <= min_x && bbox.x_max >= min_x) {
            filtered_min_x = bbox.x_max;  // min_x가 bbox 내부에 있으면 bbox.x_max 이후로 이동
        }
        if (bbox.x_min <= max_x && bbox.x_max >= max_x) {
            filtered_max_x = bbox.x_min;  // max_x가 bbox 내부에 있으면 bbox.x_min 이전으로 이동
        }
    }
    min_x = filtered_min_x;
    max_x = filtered_max_x;
}

vector<pair<double, double>> getFilteredXRanges(double min_x, double max_x, const vector<BoundingBox>& bounding_boxes) {
    vector<pair<double, double>> x_ranges;
    double current_min = min_x;

    for (const auto& bbox : bounding_boxes) {
      if (bbox.x_min > current_min && bbox.x_min < max_x) {
            x_ranges.push_back({current_min, bbox.x_min});
            current_min = bbox.x_max;  // `current_min`을 갱신
        }
        // bbox.x_max가 current_min보다 작거나 같은 경우에도 current_min을 갱신
        if (bbox.x_max > current_min) {
            current_min = bbox.x_max;
        }
    }

    // 마지막 구간 처리
    if (current_min < max_x) {
        x_ranges.push_back({current_min, max_x});
    }

    return x_ranges;
}

// 장애물이 있는 영역을 제외하고 나머지 영역을 찾는 함수
vector<pair<double, double>> getRemainingXRanges(double min_x, double max_x, const vector<pair<double, double>>& x_ranges_with_obstacles) {
    vector<pair<double, double>> remaining_ranges;

    // 전체 범위에서 장애물 범위를 제외한 나머지 구간을 찾아서 추가
    double last_end = min_x;  // 이전 끝 범위
    for (const auto& range : x_ranges_with_obstacles) {
        double obstacle_start = range.first;
        double obstacle_end = range.second;

        // 장애물 이전 구간
        if (last_end < obstacle_start) {
            remaining_ranges.push_back({last_end, obstacle_start});
        }
        
        // 장애물 후 구간
        last_end = obstacle_end;
    }

    // 마지막 장애물 이후의 구간
    if (last_end < max_x) {
        remaining_ranges.push_back({last_end, max_x});
    }

    return remaining_ranges;
}
//=====================================================================================================================
//=================================================콜백=================================================================
//=====================================================================================================================
void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left_roi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right_roi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stop_roi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object_roi(new pcl::PointCloud<pcl::PointXYZI>);

    //for object clustering roi
    // roiFilter(cloud,cloud_object_roi);
    roiFilter(cloud, cloud_left_roi,cloud_right_roi,cloud_stop_roi,stop_lane,stop_cluster_x_id,cloud_object_roi);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test(new pcl::PointCloud<pcl::PointXYZI>);
    // *cloud_test = *cloud_left_roi + *cloud_right_roi;
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud_test, output);
    // output.header.frame_id = "map";  // 적절한 프레임 ID로 변경
    // pub_cloud_roi.publish(output);

    //================================객체 다운 샘플링========================================
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    objectDownSampling(cloud_object_roi, cloud_downsampled, 0.07f);

    //====================================랜삭============================================
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z_neg(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z_pos(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);

    for (const auto& point : cloud_downsampled->points) {
        if (point.z <= -0.5) {
            cloud_z_neg->points.push_back(point);
        } else {
            cloud_z_pos->points.push_back(point);
        }
    }

    //ransac(cloud_z_neg,cloud_z_pos);  // Z ≤ -0.5 영역에 대해서만 RANSAC 수행
    ransac(cloud_z_neg,cloud_z_pos,cloud_ground);  // Z ≤ -0.5 영역에 대해서만 RANSAC 수행
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_final = *cloud_z_neg + *cloud_z_pos; //다시 병합
    //for ground roi filtering
    // roiFilter(cloud_ground, cloud_left_roi,cloud_right_roi,cloud_stop_roi,stop_lane,stop_cluster_x_id);
    //

    sensor_msgs::PointCloud2 object_msg;
    pcl::toROSMsg(*cloud_final, object_msg);
    // pcl::toROSMsg(*cloud_ground, object_msg);
    object_msg.header.frame_id = "map";  // 좌표계 설정 (필요에 따라 변경)
    object_msg.header.stamp = ros::Time::now();
    pub_object.publish(object_msg);
    

    //================================객체 클러스터링================================================
    bool object_detect = true;
    bool dumy_bool1 = false;
    bool dumy_bool2 = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object_clustered(new pcl::PointCloud<pcl::PointXYZI>);
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> object_clusters;
    clustering(cloud_final,cloud_object_clustered,object_clusters,dumy_bool1,dumy_bool2,object_detect); // 클러스터링 수행
    object_detect = false;



    sensor_msgs::PointCloud2 object_cluster_msg;
    pcl::toROSMsg(*cloud_object_clustered, object_cluster_msg);
    object_cluster_msg.header.frame_id = "map";  // 좌표계 설정 (필요에 따라 변경)
    object_cluster_msg.header.stamp = ros::Time::now();
    pub_object_cluster.publish(object_cluster_msg);


    //================================클라우드 색 구분======================================================
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stop_white_color(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left_white_color(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left_yellow_color(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right_white_color(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right_yellow_color(new pcl::PointCloud<pcl::PointXYZI>);

  
    cloudFiltering(cloud_left_roi,cloud_right_roi,cloud_stop_roi, cloud_stop_white_color,
    cloud_left_white_color,cloud_left_yellow_color,cloud_right_white_color,cloud_right_yellow_color);

    pcl::PointCloud<pcl::PointXYZI>::Ptr right_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr left_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI closest1, closest2;
    bool left_solid_lane ;
    bool right_solid_lane ;
    bool left_yellow_lane = false;
    bool right_yellow_lane = false;
    cout<<cloud_left_yellow_color -> points.size()<<endl;
    //     cout<<cloud_right_yellow_color -> points.size()<<endl;


    if (cloud_left_yellow_color->size() > 81) {
        cout<<"LEFTYELLOW"<<endl;
        left_yellow_lane = true;
        clusteringAndDetectLaneType(cloud_left_yellow_color, left_cluster,left_solid_lane,left_yellow_lane,object_detect);
    } else{
        left_yellow_lane = false;
        clusteringAndDetectLaneType(cloud_left_white_color, left_cluster,left_solid_lane,left_yellow_lane,object_detect);
                        cout<<"white size "<<cloud_left_white_color->size()<<endl;
    }
    if (cloud_right_yellow_color->size() > 81) {

        right_yellow_lane = true;
        clusteringAndDetectLaneType(cloud_right_yellow_color, right_cluster,right_solid_lane,right_yellow_lane,object_detect);
    } else{
        right_yellow_lane = false;
        clusteringAndDetectLaneType(cloud_right_white_color, right_cluster,right_solid_lane,right_yellow_lane,object_detect);
    }


    //===================정지선 검출================================================
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stop_roi_downsampled(new pcl::PointCloud<pcl::PointXYZI>);

    downsampleCloud(cloud_stop_white_color,cloud_stop_roi_downsampled);

    pcl::PointCloud<pcl::PointXYZI>::Ptr stop_lane_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    float distance_from_stop = 100.0;

    clusteringDetectToStopLane(cloud_stop_roi_downsampled,stop_lane_cluster, clusters,distance_from_stop,stop_cluster_x_id,stop_lane);

    sensor_msgs::PointCloud2 output5;
    pcl::toROSMsg(*stop_lane_cluster, output5);
    output5.header.frame_id = "map";  // 적절한 프레임 ID로 변경
    pub_stop_lane_cluster.publish(output5);

    sensor_msgs::PointCloud2 output1;
    pcl::toROSMsg(*cloud_stop_roi_downsampled, output1);
    output1.header.frame_id = "map";  // 적절한 프레임 ID로 변경
    pub_stop_lane_vis.publish(output1);
    // sensor_msgs::PointCloud2 output2;
    // pcl::toROSMsg(*arrow_cloud, output2);
    // output2.header.frame_id = "map";  // 적절한 프레임 ID로 변경
    // pub_arrow_vis.publish(output2);
    if (distance_from_stop < 10.0) {
        left_solid_lane = true;
        right_solid_lane =true;
        right_left_off =true;
    }

    //=================차선 피팅 주행가능 영역 표시=======================================

    double left_min_x, left_max_x;
    double right_min_x, right_max_x;
    if (!left_cluster->empty() && !right_cluster->empty()) {

        ParabolicCurve left_curve;
        ParabolicCurve right_curve;

        if (left_yellow_lane == true){
            cout<<"left yellow"<<endl;
            left_curve = fitParabolicCurve(cloud_left_yellow_color);
            
            findMinMaxX(cloud_left_yellow_color, left_min_x, left_max_x,stop_cluster_x_id,stop_lane);

        } else {
            left_curve = fitParabolicCurve(cloud_left_white_color);
            findMinMaxX(cloud_left_white_color, left_min_x, left_max_x,stop_cluster_x_id,stop_lane);

        }
        if (right_yellow_lane == true){
            right_curve = fitParabolicCurve(cloud_right_yellow_color);
            findMinMaxX(cloud_right_yellow_color, right_min_x, right_max_x,stop_cluster_x_id,stop_lane);

        } else {
            right_curve = fitParabolicCurve(cloud_right_white_color);
            findMinMaxX(cloud_right_white_color, right_min_x, right_max_x,stop_cluster_x_id,stop_lane);
        }

        // if (object_clusters.size()> 0 ){
            auto bounding_boxes = computeBoundingBoxes(object_clusters,left_curve,right_curve);
            // vector<pair<double, double>> x_ranges = getFilteredXRanges(min_x, max_x, bounding_boxes);
            vector<BoundingBox> left_bboxes = get<0>(bounding_boxes);
            vector<BoundingBox> right_bboxes = get<1>(bounding_boxes);
            vector<BoundingBox> front_bboxes = get<2>(bounding_boxes);
            // vector<pair<double, double>> left_x_ranges = getFilteredXRanges(left_min_x, left_max_x, left_bboxes);
            // vector<pair<double, double>> right_x_ranges = getFilteredXRanges(right_min_x, right_max_x, right_bboxes);
            // vector<pair<double, double>> front_x_ranges = getFilteredXRanges(min(left_min_x,right_min_x), max(right_max_x,left_max_x), front_bboxes);

 
            vector<pair<double, double>> front_x_ranges ;

            vector<pair<double, double>> left_x_ranges = getFilteredXRanges(1,15, left_bboxes);
            vector<pair<double, double>> right_x_ranges = getFilteredXRanges(1,15, right_bboxes);
            if (stop_lane ==true ){
                front_x_ranges = getFilteredXRanges(1, max(right_max_x,left_max_x), front_bboxes);
                // front_x_ranges = getFilteredXRanges(min(left_min_x,right_min_x), max(right_max_x,left_max_x), front_bboxes);

            }else {
                front_x_ranges = getFilteredXRanges(1, 15, front_bboxes);

            }




        // }
            // vector<pair<double, double>> remaining_ranges = getRemainingXRanges(min_x, max_x, x_ranges);

            for (const auto& range : left_x_ranges) {
                publishParabola(left_curve, "left_lane", 0, 1.0, 0.0, 0.0,range.first,range.second);
                // publishParabola(right_curve, "right_lane", 1, 1.0, 0.0, 0.0, range.first, range.second);
                            // publishFrontLaneFillArea(left_curve, right_curve, "lane_fill", 2, 1.0, 1.0, 0.0, left_min_x, left_max_x);

                 if (right_left_off == false && left_solid_lane == false) {
                    publishLeftLaneFillArea(left_curve, right_curve, "lane_fill", 3, 0.0, 1.0, 0.0, range.first, range.second);
                } else {
                    visualization_msgs::Marker delete_left_marker;
                    delete_left_marker.header.frame_id = "map";
                    delete_left_marker.header.stamp = ros::Time::now();
                    delete_left_marker.ns = "lane_fill";
                    delete_left_marker.id = 3;
                    delete_left_marker.action = visualization_msgs::Marker::DELETE;
                    pub_left_marker.publish(delete_left_marker); // 마커 삭제
                }
            }

            for (const auto& range : right_x_ranges) {
                // publishParabola(left_curve, "left_lane", 0, 1.0, 0.0, 0.0,range.first,range.second);
                publishParabola(right_curve, "right_lane", 1, 1.0, 0.0, 0.0, range.first, range.second);
                            // publishFrontLaneFillArea(left_curve, right_curve, "lane_fill", 2, 1.0, 1.0, 0.0, right_min_x, right_max_x);

                if (right_left_off == false && right_solid_lane == false) {

                publishRightLaneFillArea(left_curve, right_curve, "lane_fill", 4, 0.0, 1.0, 0.0, range.first, range.second);
                } else {
                    visualization_msgs::Marker delete_right_marker;
                    delete_right_marker.header.frame_id = "map";
                    delete_right_marker.header.stamp = ros::Time::now();
                    delete_right_marker.ns = "lane_fill";
                    delete_right_marker.id = 4;
                    delete_right_marker.action = visualization_msgs::Marker::DELETE;
                    pub_right_marker.publish(delete_right_marker); // 마커 삭제
                }
            }


            for (const auto& range : front_x_ranges) {
                                publishParabola(left_curve, "left_lane", 0, 1.0, 0.0, 0.0,range.first,range.second);

                // // publishParabola(left_curve, "left_lane", 0, 1.0, 0.0, 0.0,range.first,range.second);
                // publishParabola(right_curve, "right_lane", 1, 1.0, 0.0, 0.0, range.first, range.second);
                publishFrontLaneFillArea(left_curve, right_curve, "lane_fill", 2, 0.0, 1.0, 0.0, range.first, range.second);

                // if (right_left_off == false && right_solid_lane == false) {

                // publishRightLaneFillArea(left_curve, right_curve, "lane_fill", 4, 0.0, 0.0, 1.0, range.first, range.second);
                // } else {
                //     visualization_msgs::Marker delete_right_marker;
                //     delete_right_marker.header.frame_id = "map";
                //     delete_right_marker.header.stamp = ros::Time::now();
                //     delete_right_marker.ns = "lane_fill";
                //     delete_right_marker.id = 4;
                //     delete_right_marker.action = visualization_msgs::Marker::DELETE;
                //     pub_right_marker.publish(delete_right_marker); // 마커 삭제
                // }
            }



    }

//===============================================================================================
    //
    
    // std::ofstream fout("/home/autonav/Documents/mydata.txt", std::ios::app);
    // // fout << (left_solid_lane ? "solid lane " : "dotted lane") << std::endl;
    // // fout << (right_solid_lane ? "solid lane " : "dotted lane") <<std::endl;
    // fout << (stop_lane ? " True " : " False ") <<std::endl;

    // fout.close();
    //
    // cout << "\033[2J\033[H"; // 화면 지우고 커서 맨 위로 이동
    // cout << "======================" << endl;
    // cout << "  Lane Detection Log  " << endl;
    // cout << "======================" << endl;
    // cout << "LEFT  : " << (left_solid_lane ? "solid lane " : "dotted lane")
    //     << (left_yellow_lane ? " (yellow lane)" : " (white lane)") << endl;
    // cout << "RIGHT : " << (right_solid_lane ? "solid lane " : "dotted lane")
    //        << (right_yellow_lane ? " (yellow lane)" : " (white lane)") << endl;
    // cout << "DISTANCE FROM STOP : " << distance_from_stop<<" meters"<< endl;

    left_solid_lane = false;
    right_solid_lane = false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_merge_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_merge_cluster = *left_cluster+*right_cluster;

    sensor_msgs::PointCloud2 cloud_centroids_ros;
    pcl::toROSMsg(*cloud_merge_cluster, cloud_centroids_ros);
    cloud_centroids_ros.header.frame_id = "map";
    pub_cluster.publish(cloud_centroids_ros);


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_merge(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_merge = *cloud_right_yellow_color+*cloud_left_yellow_color+*cloud_left_white_color+*cloud_right_white_color;


    sensor_msgs::PointCloud2 output4;
    pcl::toROSMsg(*cloud_stop_white_color, output4);
    output4.header.frame_id = "map";  // 적절한 프레임 ID로 변경
    pub_stop_lane_roi.publish(output4);

    sensor_msgs::PointCloud2 output3;
    pcl::toROSMsg(*cloud_merge, output3);
    output3.header.frame_id = "map";  // 적절한 프레임 ID로 변경
    pub_lane_vis.publish(output3);


    // Lane_Detection(cloud_roi);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_lane_detection_node");
    ros::NodeHandle nh;

    pub_lane_filtered =nh.advertise<sensor_msgs::PointCloud2>("/lane_filtered", 1);
    // pub_lane = nh.advertise<lidar_package::lane_arr>("/lane_coefficients", 1);  // 차선 계수 결과를 퍼블리시
    pub_lane_vis = nh.advertise<sensor_msgs::PointCloud2>("/lane_visualization", 1);
    pub_stop_lane_vis = nh.advertise<sensor_msgs::PointCloud2>("/stop_lane_visualization", 1);
    pub_stop_lane_roi = nh.advertise<sensor_msgs::PointCloud2>("/stop_lane_roi", 1);
    pub_stop_lane_cluster = nh.advertise<sensor_msgs::PointCloud2>("/stop_lane_cluster", 1);
    pub_arrow_vis = nh.advertise<sensor_msgs::PointCloud2>("/arrow_visualization", 1);


    pub_cloud_roi = nh.advertise<sensor_msgs::PointCloud2>("/cloud_roi", 1);
    pub_test = nh.advertise<sensor_msgs::PointCloud2>("/cloud_test", 1);
    pub_test2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud_test2", 1);
    pub_cluster= nh.advertise<sensor_msgs::PointCloud2>("/lane_clustering", 1);

    pub_front_marker = nh.advertise<visualization_msgs::Marker>("/front_marker", 1);
    pub_right_marker = nh.advertise<visualization_msgs::Marker>("/right_marker", 1);
    pub_left_marker = nh.advertise<visualization_msgs::Marker>("/left_marker", 1);
    pub_curve_marker = nh.advertise<visualization_msgs::Marker>("/lane_curve_marker", 1);
    pub_object_bound_box_marker = nh.advertise<visualization_msgs::MarkerArray>("/object_bounding_box_marker", 1);

    pub_object = nh.advertise<sensor_msgs::PointCloud2>("/object", 1);
    pub_object_cluster = nh.advertise<sensor_msgs::PointCloud2>("/object_clusters", 1);



    ros::Subscriber sub = nh.subscribe("/lidar3D", 1, cloudCallBack);

    ros::spin();
    return 0;
}
