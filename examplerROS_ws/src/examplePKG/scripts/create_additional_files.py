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
