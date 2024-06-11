#include "example_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace examplePKG {

ExampleNode::ExampleNode(const rclcpp::NodeOptions & options)
: Node("example_node", options) {
    // 初始化节点，创建定时器、订阅者、发布者等
    RCLCPP_INFO(this->get_logger(), "ExampleNode has been started.");
}

void ExampleNode::example_callback() {
    // 示例回调函数
}

}  // namespace examplePKG

RCLCPP_COMPONENTS_REGISTER_NODE(examplePKG::ExampleNode)
