/*******************************************************************************
 * ROS2 Hybrid A* Node - 完整版
 *
 * 功能：
 * - 订阅 /map, /initialpose, /goal_pose
 * - 发布 /global_path (map坐标), /global_path_utm (UTM坐标), /global_trajectory (经纬度)
 * - 支持 UTM 坐标转换
 * - 从 map.yaml 解析 PNG origin UTM
 ******************************************************************************/

#include <memory>
#include <chrono>
#include <functional>
#include <fstream>
#include <regex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "hybrid_a_star/hybrid_a_star.h"

// 简单的UTM转换（使用外部库或自己实现）
// 这里提供一个简化版本，实际应用可能需要更精确的转换
struct UTMCoord {
    double easting;
    double northing;
    int zone;
    char letter;
};

// 简化的经纬度转UTM（仅作示例，实际应使用专业库如GeographicLib）
UTMCoord latlon_to_utm(double lat, double lon) {
    UTMCoord utm;
    utm.zone = static_cast<int>((lon + 180.0) / 6.0) + 1;
    utm.letter = (lat >= 0) ? 'N' : 'S';

    // 简化计算，实际应使用更精确的公式
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double central_meridian = ((utm.zone - 1) * 6 - 180 + 3) * M_PI / 180.0;

    double k0 = 0.9996;
    double a = 6378137.0;  // WGS84 major axis
    double e2 = 0.00669438;  // WGS84 eccentricity squared

    double N = a / std::sqrt(1 - e2 * std::sin(lat_rad) * std::sin(lat_rad));
    double T = std::tan(lat_rad) * std::tan(lat_rad);
    double C = e2 * std::cos(lat_rad) * std::cos(lat_rad) / (1 - e2);
    double A = (lon_rad - central_meridian) * std::cos(lat_rad);

    utm.easting = k0 * N * (A + (1 - T + C) * A * A * A / 6.0) + 500000.0;

    double M = a * ((1 - e2/4 - 3*e2*e2/64) * lat_rad);
    utm.northing = k0 * M;
    if (lat < 0) {
        utm.northing += 10000000.0;
    }

    return utm;
}

// UTM转经纬度（简化版）
void utm_to_latlon(double easting, double northing, int zone, char letter, double& lat, double& lon) {
    double k0 = 0.9996;
    double a = 6378137.0;
    double e2 = 0.00669438;

    double x = easting - 500000.0;
    double y = northing;
    if (letter < 'N') {
        y -= 10000000.0;
    }

    double M = y / k0;
    double mu = M / (a * (1 - e2/4 - 3*e2*e2/64));

    double lat_rad = mu + (3*e2/8 + 3*e2*e2/32) * std::sin(2*mu);

    lat = lat_rad * 180.0 / M_PI;

    double central_meridian = ((zone - 1) * 6 - 180 + 3) * M_PI / 180.0;
    lon = central_meridian * 180.0 / M_PI;
}

class HybridAStarNode : public rclcpp::Node
{
public:
    HybridAStarNode() : Node("hybrid_a_star_node")
    {
        // 车辆参数：车长4.53m, 车宽1.9m, 轴距2.85m, 最大转向角60度
        this->declare_parameter("vehicle_length", 4.53);
        this->declare_parameter("vehicle_width", 1.9);
        this->declare_parameter("rear_axle_dist", 1.3);
        this->declare_parameter("wheelbase", 2.85);
        this->declare_parameter("max_steer_angle", 60.0);  // degrees
        this->declare_parameter("segment_length", 1.6);
        this->declare_parameter("segment_length_discrete_num", 8);
        this->declare_parameter("steering_angle_discrete_num", 1);
        this->declare_parameter("steering_penalty", 1.05);
        this->declare_parameter("reversing_penalty", 2.0);
        this->declare_parameter("steering_change_penalty", 1.5);
        this->declare_parameter("shot_distance", 10.0);
        this->declare_parameter("state_grid_resolution", 1.0);
        this->declare_parameter("map_grid_resolution", 0.1);

        // UTM 相关参数
        this->declare_parameter("map_yaml_path", std::string("./maps/map.yaml"));
        this->declare_parameter("utm_zone", 50);
        this->declare_parameter("utm_zone_letter", std::string("N"));

        // 获取参数
        vehicle_length_ = this->get_parameter("vehicle_length").as_double();
        vehicle_width_ = this->get_parameter("vehicle_width").as_double();
        rear_axle_dist_ = this->get_parameter("rear_axle_dist").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        double max_steer_angle = this->get_parameter("max_steer_angle").as_double();
        double segment_length = this->get_parameter("segment_length").as_double();
        int segment_length_discrete_num = this->get_parameter("segment_length_discrete_num").as_int();
        int steering_angle_discrete_num = this->get_parameter("steering_angle_discrete_num").as_int();
        double steering_penalty = this->get_parameter("steering_penalty").as_double();
        double reversing_penalty = this->get_parameter("reversing_penalty").as_double();
        double steering_change_penalty = this->get_parameter("steering_change_penalty").as_double();
        double shot_distance = this->get_parameter("shot_distance").as_double();
        state_grid_resolution_ = this->get_parameter("state_grid_resolution").as_double();
        map_grid_resolution_ = this->get_parameter("map_grid_resolution").as_double();

        map_yaml_path_ = this->get_parameter("map_yaml_path").as_string();
        utm_zone_ = this->get_parameter("utm_zone").as_int();
        utm_zone_letter_ = this->get_parameter("utm_zone_letter").as_string()[0];

        // 创建 Hybrid A* 规划器
        planner_ = std::make_shared<HybridAStar>(
            max_steer_angle,
            steering_angle_discrete_num,
            segment_length,
            segment_length_discrete_num,
            wheelbase_,
            steering_penalty,
            reversing_penalty,
            steering_change_penalty,
            shot_distance,
            72  // grid_size_phi: 360/72 = 5 degrees resolution
        );

        // 设置日志回调
        planner_->SetLogCallback([this](const std::string& msg) {
            RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        });

        // 订阅地图
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            std::bind(&HybridAStarNode::mapCallback, this, std::placeholders::_1));

        // 订阅起点
        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&HybridAStarNode::startCallback, this, std::placeholders::_1));

        // 订阅终点
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&HybridAStarNode::goalCallback, this, std::placeholders::_1));

        // 使用 TRANSIENT_LOCAL QoS，方便 RViz 看到历史路径
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                      .reliable()
                      .transient_local();

        // 发布三个话题
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", qos);
        path_pub_utm_ = this->create_publisher<nav_msgs::msg::Path>("global_path_utm", qos);
        trajectory_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("global_trajectory", 10);

        // 发布搜索树可视化
        tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("search_tree", 10);

        // 加载 map.yaml 以获取 UTM 信息
        if (!map_yaml_path_.empty()) {
            loadPngOriginUTM(map_yaml_path_);
        }

        RCLCPP_INFO(this->get_logger(),
                   "Hybrid A* Node initialized. Vehicle: L=%.2fm W=%.2fm WB=%.2fm. Waiting for map, start and goal...",
                   vehicle_length_, vehicle_width_, wheelbase_);
    }

private:
    void loadPngOriginUTM(const std::string& yaml_path)
    {
        std::ifstream file(yaml_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open map.yaml: %s", yaml_path.c_str());
            return;
        }

        std::string line;
        std::regex pattern(R"(PNG origin UTM.*E\s*=\s*([0-9.+\-eE]+).*N\s*=\s*([0-9.+\-eE]+))");

        while (std::getline(file, line)) {
            std::smatch match;
            if (std::regex_search(line, match, pattern)) {
                png_origin_e_ = std::stod(match[1]);
                png_origin_n_ = std::stod(match[2]);
                has_utm_origin_ = true;
                RCLCPP_INFO(this->get_logger(),
                           "Loaded PNG origin UTM: E=%.3f, N=%.3f",
                           png_origin_e_, png_origin_n_);
                break;
            }
        }

        if (!has_utm_origin_) {
            RCLCPP_WARN(this->get_logger(),
                       "No PNG origin UTM found in map.yaml. UTM path publishing will be disabled.");
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (map_received_) {
            return;  // 只处理一次地图
        }

        RCLCPP_INFO(this->get_logger(), "Received map: %dx%d, resolution: %.2f",
                    msg->info.width, msg->info.height, msg->info.resolution);

        map_ = msg;
        map_received_ = true;

        // 计算 UTM 原点
        if (has_utm_origin_) {
            utm_origin_e_ = png_origin_e_ - msg->info.origin.position.x;
            utm_origin_n_ = png_origin_n_ - msg->info.origin.position.y;
            RCLCPP_INFO(this->get_logger(),
                       "Computed UTM origin: E0=%.3f, N0=%.3f",
                       utm_origin_e_, utm_origin_n_);
        }

        // 初始化规划器地图边界
        double x_lower = msg->info.origin.position.x;
        double y_lower = msg->info.origin.position.y;
        double x_upper = x_lower + msg->info.width * msg->info.resolution;
        double y_upper = y_lower + msg->info.height * msg->info.resolution;

        planner_->Init(x_lower, x_upper, y_lower, y_upper,
                      state_grid_resolution_, msg->info.resolution);

        // 设置车辆形状
        planner_->SetVehicleShape(vehicle_length_, vehicle_width_, rear_axle_dist_);

        // 将障碍物设置到规划器
        for (unsigned int y = 0; y < msg->info.height; ++y) {
            for (unsigned int x = 0; x < msg->info.width; ++x) {
                int idx = y * msg->info.width + x;
                if (msg->data[idx] > 50) {  // 障碍物或未知区域
                    double wx = x_lower + (x + 0.5) * msg->info.resolution;
                    double wy = y_lower + (y + 0.5) * msg->info.resolution;
                    planner_->SetObstacle(wx, wy);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Map processed and planner initialized.");

        // 如果已经有起点和终点，则立即规划
        if (start_received_ && goal_received_) {
            planPath();
        }
    }

    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        start_ = msg->pose.pose;
        start_received_ = true;

        double start_yaw = getYawFromQuaternion(start_.orientation);

        RCLCPP_INFO(this->get_logger(), "Received start: (%.2f, %.2f, %.2f°)",
                    start_.position.x, start_.position.y, start_yaw * 180.0 / M_PI);

        if (map_received_ && goal_received_) {
            planPath();
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = msg->pose;
        goal_received_ = true;

        double goal_yaw = getYawFromQuaternion(goal_.orientation);

        RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f, %.2f°)",
                    goal_.position.x, goal_.position.y, goal_yaw * 180.0 / M_PI);

        if (map_received_ && start_received_) {
            planPath();
        }
    }

    void planPath()
    {
        RCLCPP_INFO(this->get_logger(), "Starting Hybrid A* path planning...");

        double start_yaw = getYawFromQuaternion(start_.orientation);
        double goal_yaw = getYawFromQuaternion(goal_.orientation);

        Vec3d start_state(start_.position.x, start_.position.y, start_yaw);
        Vec3d goal_state(goal_.position.x, goal_.position.y, goal_yaw);

        // 重置规划器
        planner_->Reset();

        // 执行规划
        auto start_time = std::chrono::high_resolution_clock::now();
        bool success = planner_->Search(start_state, goal_state);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        if (success) {
            // 获取路径
            VectorVec3d path = planner_->GetPath();

            // 发布三个话题
            publishPath(path);
            publishPathUTM(path);
            publishTrajectory(path);

            // 发布搜索树
            publishSearchTree();

            RCLCPP_INFO(this->get_logger(),
                       "Path planning succeeded! Points: %d, Time: %ld ms",
                       static_cast<int>(path.size()), duration.count());
        } else {
            RCLCPP_WARN(this->get_logger(),
                       "Path planning failed! Time: %ld ms", duration.count());
        }
    }

    void publishPath(const VectorVec3d& path)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = map_->header.frame_id;

        for (const auto& state : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = state.x();
            pose.pose.position.y = state.y();
            pose.pose.position.z = 0.0;

            double yaw = state.z();
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);

            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published /global_path (map frame)");
    }

    void publishPathUTM(const VectorVec3d& path)
    {
        if (!has_utm_origin_) {
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "utm";

        for (const auto& state : path) {
            double E = utm_origin_e_ + state.x();
            double N = utm_origin_n_ + state.y();

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = E;
            pose.pose.position.y = N;
            pose.pose.position.z = 0.0;

            double yaw = state.z();
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);

            path_msg.poses.push_back(pose);
        }

        path_pub_utm_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published /global_path_utm (UTM frame)");
    }

    void publishTrajectory(const VectorVec3d& path)
    {
        if (!has_utm_origin_) {
            return;
        }

        geometry_msgs::msg::PoseArray trajectory_msg;
        trajectory_msg.header.stamp = this->now();
        trajectory_msg.header.frame_id = "map";

        for (const auto& state : path) {
            double E = utm_origin_e_ + state.x();
            double N = utm_origin_n_ + state.y();

            // 转换为经纬度
            double lat, lon;
            utm_to_latlon(E, N, utm_zone_, utm_zone_letter_, lat, lon);

            geometry_msgs::msg::Pose pose;
            pose.position.x = lon;  // 经度
            pose.position.y = lat;  // 纬度
            pose.position.z = 0.0;

            double yaw = state.z();
            pose.orientation.w = std::cos(yaw / 2.0);
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = std::sin(yaw / 2.0);

            trajectory_msg.poses.push_back(pose);
        }

        trajectory_pub_->publish(trajectory_msg);
        RCLCPP_INFO(this->get_logger(), "Published /global_trajectory (lat/lon)");
    }

    void publishSearchTree()
    {
        VectorVec4d tree = planner_->GetSearchedTree();

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = map_->header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "search_tree";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;

        for (const auto& edge : tree) {
            geometry_msgs::msg::Point p1, p2;
            p1.x = edge[0];
            p1.y = edge[1];
            p1.z = 0.0;
            p2.x = edge[2];
            p2.y = edge[3];
            p2.z = 0.0;

            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }

        marker_array.markers.push_back(marker);
        tree_pub_->publish(marker_array);
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
    {
        double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
        double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // 成员变量
    std::shared_ptr<HybridAStar> planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_utm_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::Pose start_;
    geometry_msgs::msg::Pose goal_;

    bool map_received_ = false;
    bool start_received_ = false;
    bool goal_received_ = false;

    double vehicle_length_;
    double vehicle_width_;
    double rear_axle_dist_;
    double wheelbase_;
    double state_grid_resolution_;
    double map_grid_resolution_;

    // UTM 相关
    std::string map_yaml_path_;
    int utm_zone_;
    char utm_zone_letter_;
    bool has_utm_origin_ = false;
    double png_origin_e_ = 0.0;
    double png_origin_n_ = 0.0;
    double utm_origin_e_ = 0.0;
    double utm_origin_n_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridAStarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
