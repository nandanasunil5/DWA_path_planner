#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

struct State { double x, y, yaw, v, w; };
struct Point { double x, y; };

class CustomDWAPlanner : public rclcpp::Node {
public:
    CustomDWAPlanner() : Node("custom_dwa_planner") {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/best_trajectory", 10);
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&CustomDWAPlanner::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CustomDWAPlanner::odom_callback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&CustomDWAPlanner::goal_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&CustomDWAPlanner::control_loop, this));
        
        global_goal_ = {0.0, 0.0}; 
        goal_received_ = false;
        robot_radius_ = 0.18; // Reduced buffer so it doesn't get stuck falsely
    }

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        global_goal_.x = msg->pose.position.x;
        global_goal_.y = msg->pose.position.y;
        goal_received_ = true;
        RCLCPP_INFO(this->get_logger(), ">>> GOAL RECEIVED! Moving to x:%.2f y:%.2f", global_goal_.x, global_goal_.y);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_ = msg; }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
        double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_state_.yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    std::vector<State> predict_trajectory(double v, double w) {
        std::vector<State> traj;
        State temp = {0, 0, 0, v, w};
        for (double t = 0.0; t < 2.0; t += 0.1) {
            temp.x += v * std::cos(temp.yaw) * 0.1;
            temp.y += v * std::sin(temp.yaw) * 0.1;
            temp.yaw += w * 0.1;
            traj.push_back(temp);
        }
        return traj;
    }

    double calc_obstacle_cost(const std::vector<State>& traj, const std::vector<Point>& obs) {
        double min_dist = 1e6;
        for (const auto& s : traj) {
            for (const auto& o : obs) {
                double d = std::hypot(s.x - o.x, s.y - o.y);
                if (d < robot_radius_) return 1e6; // Collision
                if (d < min_dist) min_dist = d;
            }
        }
        return 1.0 / min_dist;
    }

    void control_loop() {
        if (!last_scan_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for LaserScan...");
            return;
        }
        if (!goal_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Goal Click in RViz...");
            return;
        }

        std::vector<Point> obstacles;
        double angle = last_scan_->angle_min;
        for (auto r : last_scan_->ranges) {
            if (r > last_scan_->range_min && r < last_scan_->range_max)
                obstacles.push_back({r * std::cos(angle), r * std::sin(angle)});
            angle += last_scan_->angle_increment;
        }

        double dx = global_goal_.x - current_state_.x;
        double dy = global_goal_.y - current_state_.y;
        Point local_goal;
        local_goal.x = dx * std::cos(-current_state_.yaw) - dy * std::sin(-current_state_.yaw);
        local_goal.y = dx * std::sin(-current_state_.yaw) + dy * std::cos(-current_state_.yaw);

        if (std::hypot(dx, dy) < 0.25) { 
            publish_velocity(0, 0); 
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached!");
            return; 
        }

        double best_cost = 1e9, best_v = 0.0, best_w = 0.0;
        std::vector<State> best_traj;

        // Low speed (0.15) and Safe Radius (0.18)
        for (double v = 0.01; v <= 0.15; v += 0.02) {
            for (double w = -0.6; w <= 0.6; w += 0.1) {
                auto traj = predict_trajectory(v, w);
                double g_dist = std::hypot(local_goal.x - traj.back().x, local_goal.y - traj.back().y);
                double h_cost = std::abs(std::atan2(local_goal.y - traj.back().y, local_goal.x - traj.back().x) - traj.back().yaw);
                double o_cost = calc_obstacle_cost(traj, obstacles);

                double total_cost = (1.0 * g_dist) + (0.8 * h_cost) + (4.0 * o_cost);

                if (total_cost < best_cost) {
                    best_cost = total_cost; best_v = v; best_w = w; best_traj = traj;
                }
            }
        }

        if (best_v == 0.0 && best_w == 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "STUCK: All paths blocked by obstacles!");
            // Optional Recovery: slowly rotate to find space
            best_w = 0.2; 
        }

        publish_velocity(best_v, best_w);
        visualize_path(best_traj);
    }

    void publish_velocity(double v, double w) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = v; msg.angular.z = w;
        cmd_pub_->publish(msg);
    }

    void visualize_path(const std::vector<State>& traj) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "base_link"; m.header.stamp = this->now();
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.02; m.color.b = 1.0; m.color.a = 1.0;
        for (const auto& s : traj) {
            geometry_msgs::msg::Point p; p.x = s.x; p.y = s.y; m.points.push_back(p);
        }
        marker_pub_->publish(m);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    State current_state_; Point global_goal_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    bool goal_received_; double robot_radius_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomDWAPlanner>());
    rclcpp::shutdown();
    return 0;
}
