#ifndef EXAMPLEPKG__EXAMPLE_NODE_HPP_
#define EXAMPLEPKG__EXAMPLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace examplePKG {

class ExampleNode : public rclcpp::Node {
private:
    void example_callback();

public:
    explicit ExampleNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

};

}  // namespace examplePKG

#endif  // EXAMPLEPKG__EXAMPLE_NODE_HPP_
