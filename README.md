# helix

C++20 优化算法库，使用 CMake 构建。

代码风格由 `.clang-format` 定义，格式化：`clang-format -i src/*.cpp tests/*.cpp`。

## 环境依赖

| 工具 | 版本 | 说明 |
| --- | --- | --- |
| g++ | 13.3+ | 系统自带编译器 |
| gdb | - | 调试器 |
| CMake | 3.20+ | 构建系统（本机装在 `~/.local`） |
| VS Code | - | 扩展：C/C++、CMake Tools |

## 目录结构

```
helix/
├── CMakeLists.txt        # 顶层构建脚本（C++20, -Wall -Wextra）
├── src/
│   └── main.cpp          # 程序入口
├── tests/
│   ├── CMakeLists.txt
│   └── test_basic.cpp    # 已接入 ctest
├── .vscode/
│   └── settings.json     # CMake / IntelliSense 配置
└── build/                # 构建产物（可删除，不提交）
```

## 快速开始

```bash
# 配置
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build build -j

# 运行
./build/helix

# 跑测试
ctest --test-dir build --output-on-failure
```

## Debug 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
gdb ./build/helix
```

## VS Code

1. 用 VS Code 打开本目录，CMake Tools 会自动配置。
2. 底部状态栏可切换构建类型、编译、运行、调试。
3. IntelliSense 依赖 `build/compile_commands.json`（配置时自动生成）。

## 添加新代码

- 新增源文件后，在 `CMakeLists.txt` 的 `add_executable` 中加入路径。
- 新增测试：在 `tests/` 下建文件，并在 `tests/CMakeLists.txt` 中 `add_executable` + `add_test`。
