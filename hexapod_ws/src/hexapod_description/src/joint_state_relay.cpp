#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <unordered_map>

static const std::vector<std::string> JOINT_ORDER = {
    "leg1_coxa_joint", "leg1_femur_joint", "leg1_tibia_joint",
    "leg2_coxa_joint", "leg2_femur_joint", "leg2_tibia_joint",
    "leg3_coxa_joint", "leg3_femur_joint", "leg3_tibia_joint",
    "leg4_coxa_joint", "leg4_femur_joint", "leg4_tibia_joint",
    "leg5_coxa_joint", "leg5_femur_joint", "leg5_tibia_joint",
    "leg6_coxa_joint", "leg6_femur_joint", "leg6_tibia_joint",
};

class JointStateRelay : public rclcpp::Node
{
public:
    JointStateRelay() : Node("joint_state_relay")
    {
        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/hexapod_joint_controller/commands", 10);
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_commands", 10,
            std::bind(&JointStateRelay::cb, this, std::placeholders::_1));
    }

private:
    void cb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::unordered_map<std::string, double> index;
        for (size_t i = 0; i < msg->name.size(); ++i)
            index[msg->name[i]] = msg->position[i];

        std_msgs::msg::Float64MultiArray out;
        for (const auto & joint : JOINT_ORDER) {
            auto it = index.find(joint);
            if (it != index.end())
                out.data.push_back(it->second);
        }
        pub_->publish(out);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateRelay>());
    rclcpp::shutdown();
    return 0;
}