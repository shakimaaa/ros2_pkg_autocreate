import os
import sys

def create_header_file(package_path, package_name, file_name, class_name):
    file_guard = f"{package_name.upper()}__{file_name.upper()}_HPP_"
    header_content = f"""#ifndef {file_guard}
#define {file_guard}

#include "rclcpp/rclcpp.hpp"

namespace {package_name} {{

class {class_name} : public rclcpp::Node {{
private:
    void example_callback();

public:
    explicit {class_name}(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}};

}}  // namespace {package_name}

#endif  // {file_guard}
"""
    header_path = os.path.join(package_path, 'include', package_name, f"{file_name}.hpp")
    os.makedirs(os.path.dirname(header_path), exist_ok=True)
    with open(header_path, 'w') as header_file:
        header_file.write(header_content)
    print(f"Header file created at: {header_path}")

def create_source_file(package_path, package_name, file_name, class_name):
    source_content = f"""#include "{file_name}.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace {package_name} {{

{class_name}::{class_name}(const rclcpp::NodeOptions & options)
: Node("{file_name}", options) {{
    // 初始化节点，创建定时器、订阅者、发布者等
    RCLCPP_INFO(this->get_logger(), "{class_name} has been started.");
}}

void {class_name}::example_callback() {{
    // 示例回调函数
}}

}}  // namespace {package_name}

RCLCPP_COMPONENTS_REGISTER_NODE({package_name}::{class_name})
"""
    source_path = os.path.join(package_path, 'src', f"{file_name}.cpp")
    os.makedirs(os.path.dirname(source_path), exist_ok=True)
    with open(source_path, 'w') as source_file:
        source_file.write(source_content)
    print(f"Source file created at: {source_path}")

def main():
    if len(sys.argv) != 5:
        print("Usage: create_ros2_files.py <package_path> <package_name> <file_name> <class_name>")
        return

    package_path = sys.argv[1]
    package_name = sys.argv[2]
    file_name = sys.argv[3]
    class_name = sys.argv[4]

    create_header_file(package_path, package_name, file_name, class_name)
    create_source_file(package_path, package_name, file_name, class_name)

if __name__ == "__main__":
    main()
