# FixedVel Rewiring 修改说明

## 输出与基线

- 输出目录：`版本3加速（4000-0.27）_rake_avx2_soa_nn_avx2_float8_benchmark_fixedvel_rewire`
- 修改基线：上一轮另存的 `..._refactored` 目录。
- 原 ZIP、原解压目录和 `..._refactored` 目录均未覆盖。

## 目标行为

1. 新节点尝试重连临近旧节点时，固定边的起点和终点完整状态 `(x, y, vx, vy)`。
2. 使用给定的 FixedVel 时间方程、代价公式和固定两端状态的三次轨迹。
3. 重连成功后不改写旧节点及其子孙的位置、速度或缓存。
4. 子孙递归仅传播累计 cost：`child.cost = parent.cost + child.incCost`。

## FixedVel 数学层

`MyRRTstar.cpp/.h` 新增以下独立方法：

- `makeTimeFixedVelCoefficients`：生成给定公式中的四次时间方程系数。
- `DI_costFixedVel`：计算固定起终位置、速度的边代价。
- `computeBestTimeFixedVelFromDIStates`：求时间区间内的最优 FixedVel 时间和代价。
- `prepareEdgePrimitiveFixedVel`：生成满足两端位置、速度约束的三次 Hermite 轨迹原语。
- `evaluateEdgeFixedVelFromDIStates`：把 FixedVel 数学层结果装入现有 `EdgeEval`。

自由末端速度和固定末端速度共用 `computeBestTimeFromQuartic`，避免复制根扫描、Newton 和二分细化逻辑。

## Rewiring 修改

- `evaluateRewireCandidateBlock` 改为调用 FixedVel 边评估；父节点选择、新节点生成和目标直连仍使用原 FreeVel 逻辑。
- FixedVel 评估直接以邻居当前 `cache` 作为终端状态，因此终端速度不再被重新求解。
- 重连提交时仅修改 `parent`、`incCost`、`cost`、`duration` 和边控制缓存；不再执行 `copyState`、`cache` 改写或 `updateMotionSoA`。
- `updateChildCosts` 删除轨迹重算、碰撞验证和状态提交，只递归更新累计 cost。

## 关联文件一致性

- `di_primitive_types.h`：`DIEdgePrimitive` 改为通用三次多项式系数 `x2/y2/x3/y3`。
- `di_simd_backend.h`：标量和 AVX2 状态采样改为评估通用三次多项式，因此 FreeVel 与 FixedVel 共用同一采样实现。
- `di_primitive_helpers.h`：新增“验证已准备原语”的共享内核。原 FreeVel 接口仍构造 FreeVel 原语；rewiring 把 FixedVel 原语交给同一套 SIMD 碰撞采样和状态合法性检查，避免改变公共 FreeVel 接口语义。
- 路径控制离散化和中间状态输出统一根据每条边保存的两端完整状态构造 FixedVel 原语，保证 rewiring 后输出轨迹与验证轨迹一致。

## 静态检查

- 已检查 FixedVel 方法的声明、定义和 rewiring 调用链。
- 已检查旧 `term1/term2/invTs3/invTwoTs3` 原语字段无残留引用。
- 已检查 rewiring 提交及 `updateChildCosts` 不再修改节点状态或速度。
- 已检查 `test.cpp` 依赖的原 FreeVel 公共接口仍保留。

## 未执行项

按要求未运行 CMake、编译、测试程序或 benchmark。可运行性结论来自静态接口、调用链和公式一致性检查，未经过编译器或运行时验证。
