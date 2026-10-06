# RRTstar 边求解调用简化日志

## 输出副本

本目录由 `..._fixedvel_rewire` 复制得到，原目录未覆盖。规划程序未运行。

## 重构目标

在不改变 PFF 代价、AVX2/float8 SIMD 批量求解、SoA 最近邻和树结构的前提下，减少 `MyRRTstar.cpp` 中重复拼接“求时间、算代价、生成终端状态、构造轨迹”的调用。

## 新的边求解入口

### FreeVel

`solveFreeVelEdge()` / `solveFreeVelEdgeFromDIStates()` 集中完成：

1. `DI_timeFreeVel` 对应的四次方程求解；
2. 最优到达时间和 `DI_costFreeVel`；
3. `DI_stateFreeVel` 终端状态；
4. FreeVel 轨迹 primitive 构造。

用于新节点扩展、父节点候选、目标连接和 `dynamicMotionCost`。

### FixedVel

`solveFixedVelEdgeFromDIStates()` 集中完成：

1. `DI_timeFixedVel` 对应的四次方程求解；
2. 最优到达时间和 `DI_costFixedVel`；
3. 固定末端完整状态；
4. FixedVel 轨迹 primitive 构造。

只用于 rewiring。

## 碰撞检查

保留统一的 `validatePreparedMotionPrimitive()` 执行器，不复制两套完整碰撞实现。新增两个语义入口：

- `checkFreeVelMotionWithTime()`：只接受 `EdgeMode::FreeVel` primitive；
- `checkFixedVelMotionWithTime()`：只接受 `EdgeMode::FixedVel` primitive。

验证器根据 primitive 类型选择状态批量计算路径，然后统一执行状态边界、障碍物和 SIMD 检查。时间和代价不会在碰撞阶段重复计算。

## SIMD 与 PFF 保持项

- `evaluateEdgeBasicBlock8()` 的 AVX2/标量尾处理保留；
- `evaluateRewireCandidateBlock()` 的批量 FixedVel 求解保留；
- SoA 最近邻和邻域搜索保留；
- PFF 代价公式、时间扫描、Newton/bisection 工作参数保留；
- 延迟碰撞检查顺序保留：先候选代价筛选，再按需检查轨迹；
- 子节点重连后的递归过程仍只更新累计 `cost`。

## 安全裕量与采样

- `FastEnvironment2D` 圆障碍物半径统一使用 `radius + 0.1`；
- 矩形障碍物四边各扩展 `0.1`；
- `test.cpp::isStateValid` 使用相同安全裕量；
- `test.cpp` 将碰撞参数设为 `11`，对应起点、10 个内部点和终点；8 点批次使用 SIMD，尾部点使用既有标量路径。

## 兼容性处理

`DIEdgePrimitive` 增加显式 `EdgeMode`，避免根据终端速度数值猜测边模型。原先的 `evaluateEdgeBasic*` 包装已从类接口移除，主循环、目标连接、代价接口和批量尾路径均改用 `solve...Edge` 命名入口。

## 静态检查结果

- 原 `MyRRTstar.cpp` 未被覆盖；
- 新副本中所有 FreeVel 主循环调用均走 `solveFreeVelEdge`；
- rewiring 均走 `solveFixedVelEdge` 和 FixedVel 碰撞入口；
- 未发现旧 `evaluateEdgeBasic` 或 `validateMotionPrimitive` 的主流程调用；
- 未运行 CMake、规划程序或基准测试，避免改变用户运行状态。
