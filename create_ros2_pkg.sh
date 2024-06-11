#!/bin/bash

# 检查是否提供了包名和类名参数
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <package_name> <class_name>"
    exit 1
fi

PACKAGE_NAME=$1
CLASS_NAME=$2

# 将类名转换为文件名格式
FILE_NAME=$(echo "$CLASS_NAME" | awk '{print tolower($0)}' | sed 's/\([a-z0-9]\)\([A-Z]\)/\1_\2/g' | sed 's/ /_/g')

# 将类名转换为驼峰式
CAMEL_CLASS_NAME=$(echo "$CLASS_NAME" | sed -r 's/(^|_)([a-z])/\U\2/g' | sed 's/ /_/g')

# 创建功能包
ros2 pkg create --build-type ament_cmake $PACKAGE_NAME

# 进入功能包目录
cd $PACKAGE_NAME

# 创建必要的目录
mkdir -p scripts include/$PACKAGE_NAME src

# 将生成节点文件的脚本移动到 scripts 目录
cat << 'EOF' > scripts/create_ros2_files.py
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
EOF

# 自动生成节点文件
python3 scripts/create_ros2_files.py $(pwd) $PACKAGE_NAME $FILE_NAME $CAMEL_CLASS_NAME

# 将生成其他普通文件的脚本移动到 scripts 目录
cat << 'EOF' > scripts/create_additional_files.py
import os
import sys

def create_additional_header(package_path, package_name, header_file_name, class_name):
    file_guard = f"{package_name.upper()}__{class_name.upper()}_HPP_"
    header_content = f"""#ifndef {file_guard}
#define {file_guard}

namespace {package_name} {{

class {class_name} {{
private:

public:
    {class_name}();
    ~{class_name}();

    void do_something();
}};

}}  // namespace {package_name}

#endif  // {file_guard}
"""
    header_path = os.path.join(package_path, 'include', package_name, f"{header_file_name}.hpp")
    os.makedirs(os.path.dirname(header_path), exist_ok=True)
    with open(header_path, 'w') as header_file:
        header_file.write(header_content)
    print(f"Additional header file created at: {header_path}")

def create_additional_source(package_path, package_name, source_file_name, class_name):
    source_content = f"""#include "{source_file_name}.hpp"

namespace {package_name} {{

{class_name}::{class_name}() {{
    // 构造函数实现
}}

{class_name}::~{class_name}() {{
    // 析构函数实现
}}

void {class_name}::do_something() {{
    // 方法实现
}}

}}  // namespace {package_name}
"""
    source_path = os.path.join(package_path, 'src', f"{source_file_name}.cpp")
    os.makedirs(os.path.dirname(source_path), exist_ok=True)
    with open(source_path, 'w') as source_file:
        source_file.write(source_content)
    print(f"Additional source file created at: {source_path}")

def main():
    if len(sys.argv) != 6:
        print("Usage: create_additional_files.py <package_path> <package_name> <header_file_name> <source_file_name> <class_name>")
        return

    package_path = sys.argv[1]
    package_name = sys.argv[2]
    header_file_name = sys.argv[3]
    source_file_name = sys.argv[4]
    class_name = sys.argv[5]

    create_additional_header(package_path, package_name, header_file_name, class_name)
    create_additional_source(package_path, package_name, source_file_name, class_name)

if __name__ == "__main__":
    main()
EOF

# 提示用户手动运行生成其他文件的脚本
echo "Node files created for $CAMEL_CLASS_NAME."
echo "To create additional .hpp and .cpp files, run the following command:"
echo "cd $PACKAGE_NAME"
echo "python3 scripts/create_additional_files.py \$(pwd) $PACKAGE_NAME <header_file_name> <source_file_name> <class_name>"

