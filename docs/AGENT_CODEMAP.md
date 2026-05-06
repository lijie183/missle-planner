# MissilePlanner 代码结构索引

本文档面向后续维护或改造项目的 agent，目标是用最少的阅读成本快速定位代码入口、数据流和常见修改点。

## 1. 项目定位

MissilePlanner 是一个基于 Qt + OpenSceneGraph + osgEarth 的导弹多弹协同任务规划与三维可视化系统。核心能力包括：

- 多导弹/多目标协同规划。
- 威胁区约束下的路径搜索与平滑。
- 三维地球、导弹、目标、威胁区的可视化展示。
- 导弹推演、遥测采样与图表展示。
- HTML 形式的规划与推演报告导出。

## 2. 启动入口

程序启动路径非常短：

- [main.cpp](../main.cpp) 负责创建 Qt 应用并显示主窗口。
- [ui/MainWindow.h](../ui/MainWindow.h) 和 [ui/MainWindow.cpp](../ui/MainWindow.cpp) 是整个系统的控制中心。
- [render/OsgEarthWidget.h](../render/OsgEarthWidget.h) 和 [render/OsgEarthWidget.cpp](../render/OsgEarthWidget.cpp) 负责三维场景、相机和对象可视化。

启动后默认会在主窗口中装载默认场景，再把导弹、目标和威胁区同步到三维地球上。

## 3. 目录总览

### 3.1 core/

这是规划算法和数据模型层，后续如果要改“规划逻辑”，优先看这里。

- [core/MissionTypes.h](../core/MissionTypes.h) 定义所有核心数据结构，包括威胁区、导弹、目标、任务请求、规划结果、遥测样本、分配结果等。
- [core/AStarAlgorithm.h](../core/AStarAlgorithm.h) 和 [core/AStarAlgorithm.cpp](../core/AStarAlgorithm.cpp) 实现网格化 A* 搜索，是基础路径规划器。
- [core/HybridRoutePlanner.h](../core/HybridRoutePlanner.h) 和 [core/HybridRoutePlanner.cpp](../core/HybridRoutePlanner.cpp) 在 A* 基础上做二次优化与威胁区推离。
- [core/MultiMissilePlanner.h](../core/MultiMissilePlanner.h) 和 [core/MultiMissilePlanner.cpp](../core/MultiMissilePlanner.cpp) 负责多导弹任务分配、冲突检测、时间同步。
- [core/HungarianAlgorithm.cpp](../core/HungarianAlgorithm.cpp)、[core/GeneticAllocator.cpp](../core/GeneticAllocator.cpp)、[core/RRTAlgorithm.cpp](../core/RRTAlgorithm.cpp) 是不同规划或分配策略的实现。

### 3.2 sim/

这是飞行推演层，负责让规划轨迹真正“跑起来”。

- [sim/MissileSim.h](../sim/MissileSim.h) 和 [sim/MissileSim.cpp](../sim/MissileSim.cpp) 管理导弹沿路线的推进、加速度、姿态和阶段切换。

### 3.3 render/

这是三维地球和场景渲染层。

- [render/OsgEarthWidget.h](../render/OsgEarthWidget.h) 和 [render/OsgEarthWidget.cpp](../render/OsgEarthWidget.cpp) 负责地球创建、相机控制、威胁区绘制、导弹/目标标记、航线和尾迹渲染。
- 这里也是修复视角、闪屏、线条样式、悬停信息、对象醒目程度的主要位置。

### 3.4 ui/

这是界面与业务编排层。

- [ui/MainWindow.h](../ui/MainWindow.h) 和 [ui/MainWindow.cpp](../ui/MainWindow.cpp) 组织所有控件、按钮、表格、遥测面板和事件槽函数。
- [ui/TelemetryPlotWidget.h](../ui/TelemetryPlotWidget.h) 和 [ui/TelemetryPlotWidget.cpp](../ui/TelemetryPlotWidget.cpp) 负责遥测六宫格图表。
- [ui/FlightReportExporter.h](../ui/FlightReportExporter.h) 和 [ui/FlightReportExporter.cpp](../ui/FlightReportExporter.cpp) 负责 HTML 报告导出。

### 3.5 data/

这是地球底图、DEM 和资源文件。

- [data/earth/highres_global.earth](../data/earth/highres_global.earth) 是地球数据配置入口。
- [data/earth/README.md](../data/earth/README.md) 描述地球数据目录。
- [data/earth/dem/](../data/earth/dem/) 和 [data/earth/imagery/](../data/earth/imagery/) 存放地形和影像资源。

### 3.6 docs/

目前用于放置项目文档、分析说明和后续 agent 的工作备忘。

## 4. 关键调用链

### 4.1 启动链路

1. [main.cpp](../main.cpp) 创建 QApplication。
2. MainWindow 构造时组装 UI、默认场景和三维控件。
3. [ui/MainWindow.cpp](../ui/MainWindow.cpp) 中的默认场景会生成导弹、目标和威胁区数据。
4. 这些数据通过 sync 函数同步到 [render/OsgEarthWidget.cpp](../render/OsgEarthWidget.cpp)。

### 4.2 规划链路

1. 用户点击规划按钮后，MainWindow 收集导弹、目标、威胁区和算法参数。
2. MultiMissilePlanner 先做任务分配，再调用单航迹规划器生成每个导弹的路线。
3. 结果写回 MultiMissionResult，并刷新三维场景、表格和统计信息。

### 4.3 推演链路

1. 用户点击开始推演后，MainWindow 把每个导弹的路线交给 MissileSim。
2. 定时器周期性调用 MissileSim::update。
3. 更新后的位置信息同步回三维场景和遥测面板。

## 5. 数据模型说明

核心数据都在 [core/MissionTypes.h](../core/MissionTypes.h) 中，后续改结构时优先从这里查。

- ThreatZone：威胁区中心、半径、高度范围。
- MissionRequest：单次路径规划请求，包含起点、终点、威胁区和导弹最大高度。
- RoutePlanResult：单条航线和规划指标。
- MissileConfig / TargetConfig：导弹和目标的静态配置。
- Assignment / MultiMissionResult：多导弹分配与总体结果。
- TelemetrySample：遥测采样结果，供图表和报告使用。

如果要新增字段，优先判断它属于：

- 静态任务配置，放在 MissionTypes。
- 规划器内部状态，放在具体算法类。
- 显示层状态，放在 MainWindow 或 OsgEarthWidget。

## 6. 常见修改入口

### 6.1 想改视角、地球或对象样式

直接看 [render/OsgEarthWidget.cpp](../render/OsgEarthWidget.cpp)。这里处理：

- 相机初始化与聚焦。
- 威胁区几何体。
- 导弹、目标、起点、终点、冲击点。
- 航线、尾迹、跟随视角。

### 6.2 想改页面布局、按钮、表格、统计

直接看 [ui/MainWindow.cpp](../ui/MainWindow.cpp)。这里处理：

- UI 组件构建。
- 默认场景生成。
- 规划按钮、推演按钮、失败模拟、动态重规划。
- 遥测表、规划对比表、任务汇总。

### 6.3 想改路径规划算法

优先看：

- [core/AStarAlgorithm.cpp](../core/AStarAlgorithm.cpp)
- [core/HybridRoutePlanner.cpp](../core/HybridRoutePlanner.cpp)
- [core/MultiMissilePlanner.cpp](../core/MultiMissilePlanner.cpp)

优先级建议：

- 先改 AStarAlgorithm，控制基础可达性和威胁区避让。
- 再改 HybridRoutePlanner，处理平滑和局部修正。
- 最后改 MultiMissilePlanner，处理多机分配、冲突和时间同步。

### 6.4 想改推演速度、姿态、阶段感

看 [sim/MissileSim.cpp](../sim/MissileSim.cpp)。这里控制：

- 推进速度变化。
- 助推、巡航、末制导、完成阶段。
- 俯仰角、航向角、加速度和采样结果。

### 6.5 想改遥测图表

看 [ui/TelemetryPlotWidget.cpp](../ui/TelemetryPlotWidget.cpp)。这里控制图表布局、坐标轴、绘制和悬停信息。

## 7. 当前工程的实际约束和注意事项

- Windows 下直接用普通 PowerShell 跑 CMake 可能会碰到工具链环境问题，优先用 VS Code 的 CMake Tools 或已配置好的开发者命令行。
- osgEarth 相关头文件使用的是 [render/OsgEarthWidget.cpp](../render/OsgEarthWidget.cpp) 里当前的包含方式，避免随意改成错误的旧头文件路径。
- [CMakeLists.txt](../CMakeLists.txt) 使用 file(GLOB_RECURSE) 收集源文件，新增文件后需要确保 CMake 重新配置。
- 没有离线 earth 数据时，系统会走程序化地球或在线数据回退逻辑，三维渲染仍应保持可用。
- 多导弹场景不是单独 demo，而是主路径，修改时不要只盯单导弹逻辑。

## 8. 后续 agent 的推荐工作方式

为了减少 token 和误改范围，建议按下面顺序看代码：

1. 先看 [ui/MainWindow.cpp](../ui/MainWindow.cpp) 中对应按钮或槽函数。
2. 再看它调用的规划器或渲染函数。
3. 最后再深入相关的算法实现或三维节点构建函数。

如果只做局部问题修复，通常只需要同时看一个 UI 槽函数、一个算法实现、一个渲染入口，不要一开始全仓库展开。

## 9. 适合继续补充的内容

如果后续还有时间，建议在本文件继续补两类信息：

- 重要函数清单，按“函数名 -> 作用 -> 适合修改的场景”整理。
- 常见 Bug 清单，记录已验证的失败模式和对应文件位置。

## 10. 2026-05 可视化与算法增强记录

以下改动用于“首帧聚焦战区、样式强化、悬停信息、规划线与飞行线一致、避威胁算法加强、渲染清晰度优化”：

- [render/OsgEarthWidget.h](../render/OsgEarthWidget.h) 与 [render/OsgEarthWidget.cpp](../render/OsgEarthWidget.cpp)
  - 新增 `focusOnMissionArea()`：按导弹起点/目标点/威胁区包围盒自动聚焦任务区域，启动即看战区。
  - 新增悬停信息链路：`updateHoverTooltip()`、`projectGeoToScreen()`、`mapWidgetToOsgEventCoords()`，支持威胁区/导弹/目标实时信息提示。
  - 鼠标事件改为 viewport 比例坐标桥接，并在拖拽期间抑制高频悬停刷新，降低卡顿与闪动风险。
  - 规划线改为灰色基线，飞行轨迹改为按导弹颜色逐步覆盖，视觉上与规划路线保持一致。
  - 威胁区与标记样式升级为更强调高度与层次的结构化几何（分层柱体/帽体/环形强调）。

- [ui/MainWindow.cpp](../ui/MainWindow.cpp)
  - `syncEarthWidgetFromConfig()` 中统一同步威胁区、导弹名称，并触发 `focusOnMissionArea()`。
  - 主窗口初始化后直接聚焦任务区域，避免初始视角偏离作战区域。

- [ui/TelemetryPlotWidget.cpp](../ui/TelemetryPlotWidget.cpp)
  - `TelemetryPlotWidget::missileColor()` 改为复用 `mission::missileColor()`，保证遥测曲线颜色与三维飞行覆盖色一致。

- [core/AStarAlgorithm.cpp](../core/AStarAlgorithm.cpp)
  - 邻接扩展阶段改为“所有边都做线段安全采样”，不再只对部分对角边校验。
  - 平滑后增加全段安全复核；若检测到风险段，回退到基线路径，避免穿威胁区。

- [core/HybridRoutePlanner.cpp](../core/HybridRoutePlanner.cpp)
  - 引入段级安全判定并约束局部优化，防止势场推离阶段把路径推入风险区域。
  - 最终增加整段安全检查，不满足时回退到原始安全路径。
