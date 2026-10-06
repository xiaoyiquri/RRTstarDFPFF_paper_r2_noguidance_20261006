# RRTstar2D SoA NN float8 AVX2 修改日志

## 1. 修改范围

- 输入：`版本3加速（4000-0.27）_rake_avx2_soa_nn_avx2`
- 输出：`版本3加速（4000-0.27）_rake_avx2_soa_nn_avx2_float8`
- 输入目录未覆盖、未修改。
- 修改对象为 C++ 代码和配套文档，不涉及 MATLAB。

本副本保留输入版本已有的 SoA 最近邻/邻域查询，并把所有 AVX2 热路径
从 `double + __m256d`（4 lanes）转换为 `float + __m256`（8 lanes）。

## 2. 统一 SIMD 宽度

```text
AVX2 register width   = 256 bit
float width           = 32 bit
lanes per register    = 8
PrimitiveBatchWidth   = 8
RakeBatchWidth        = 8
```

完整块每次处理 8 个元素，尾部不足 8 个元素时使用 float 或既有 double
标量路径，且只读取有效元素。

## 3. 轨迹与碰撞路径

### 3.1 候选边时间和代价

`evaluateEdgeBasicBlock8` 同时处理 8 个父候选或重连候选。以下步骤使用
`__m256` 和 `*_ps` 指令：

- 起点位置、速度和目标位置装载；
- 距离平方、速度平方、点积及方程系数；
- 导数区间扫描；
- Newton 和二分根细化；
- 最优时间对应的边代价；
- lane mask 和最小代价选择。

float 根求解的最低容差为 `1e-6f`。结果写回树节点前提升为 double；树的
累计代价、OMPL 状态、`Motion::cache` 和 edge primitive 继续使用 double。

### 3.2 单轨迹 rake

一条轨迹的连续内部时间点按 8 个一组：

```text
[t1, ..., t8], [t9, ..., t16], ...
```

每组同步计算 `x/y/vx/vy`。临时 `DIStateBlock<8>` 是 float SoA。最后
不足 8 个采样点走标量尾处理，不会把多条轨迹混入同一个 rake。

### 3.3 碰撞检测

`FastEnvironment2D::checkStateBlock8Avx2` 向量化检查：

- 位置边界；
- 速度边界；
- 圆形障碍物距离平方；
- 矩形障碍物范围；
- `_mm256_movemask_ps` 结果归约。

任意 lane 越界或碰撞都会拒绝当前轨迹。外部
`FastBatchValidityChecker(const double*, ...)` API 保持不变，临时 float
状态在调用边界提升为 double。

## 4. SoA 最近邻与邻域查询

### 4.1 存储

```text
soaMotions[]          Motion 指针
soaX[]                float x 坐标
soaY[]                float y 坐标
soaDistanceScratch[]  float 距离平方
soaIndexScratch[]     节点索引
```

树节点本身仍保存 double 状态。节点加入 SoA 或重连更新时显式转换为
float；删除节点仍采用末尾槽交换，并同步修正被移动节点的 `soaIndex`。

### 4.2 最近节点

`nearestNeighborSoA` 每次装载 8 个 x 和 8 个 y，计算 8 个二维距离平方，
再按 lane 顺序归约最小值。尾部不足 8 个节点使用 float 标量计算。

### 4.3 半径邻域

`getNeighborsSoA` 每次计算 8 个距离平方，与 float 半径平方比较，通过
`_mm256_movemask_ps` 得到 8-bit 成员 mask，只收集命中的节点。

### 4.4 K 近邻

K 近邻首先以 8-lane AVX2 填充 float 距离数组，然后对索引执行
`nth_element` 和前 K 个索引排序。距离相等时仍用 SoA 索引确定次序。

这些修改只替换二维距离扫描，不改变 RRT* 的父节点选择、动力学边评价、
重连条件、代价传播和碰撞判定。

## 5. SoA 生命周期核对

- 新节点：`addMotionToNearestNeighbors` 同时写入 NN 容器和 SoA。
- 重连节点状态变化：`updateMotionSoA` 更新 float 坐标。
- 子树状态递归变化：每次提交新状态后更新 SoA。
- 剪枝删除：`unregisterMotionSoA` 删除并修复交换槽索引。
- 清理：`freeMemory` 清空所有 SoA 数组和 scratch。
- 查询：规划主循环不再调用 `nn_->nearest/nearestR/nearestK`，均使用 SoA。

## 6. 逐文件检查

| 文件 | 适配结果 |
|---|---|
| `MyRRTstar.h` | 批宽改为 8；候选函数改名；SoA 坐标和距离缓存改为 float；公共接口保持兼容。 |
| `MyRRTstar.cpp` | 候选边、最近节点、半径邻域、K 近邻改为 8-lane；SoA 生命周期和日志同步适配。 |
| `di_primitive_types.h` | 临时状态块改为 float SoA；规范状态和 primitive 保持 double。 |
| `di_edge_eval_block.h` | 候选终端结果使用独立 double 块，避免树状态缓存被临时 float 块降精度。 |
| `di_simd_backend.h` | 单轨迹状态公式改为 float `__m256` 8-lane，保留标量回退。 |
| `di_primitive_helpers.h` | rake 调度改为 8；尾块和 double validity 回调完成适配。 |
| `fast_environment_2d.h` | 边界、圆、矩形和 mask 归约改为 float8 AVX2。 |
| `DICostUtils.cpp/.h` | 无需修改；独立 double API 和四次方程的 4 个根与 SIMD 宽度无关。 |
| `test.cpp` | 无需修改；现有规划参数和障碍配置继续使用原公共 API。 |
| `profile_perf.sh` | 无需修改；继续用于 Linux perf 对比。 |
| `FAST_VALIDITY_INTEGRATION.md` | 已更新为 float8 rake、碰撞和尾块行为。 |
| `README_RAKE_AVX2_CHANGELOG.md` | 保留输入版本历史，并指向本日志。 |

## 7. 编译与验证

推荐 GCC/Clang 参数：

```bash
-std=c++17 -O3 -mavx2
```

需要 FMA 时增加 `-mfma`。未定义 `__FMA__` 时使用独立乘法和加法。

静态检查项目：

- 当前 `.h/.cpp` 中没有残留 `__m256d`、`*_pd`、`Block4` 或 4-lane
  SoA 循环；
- 四维状态 `[x,y,vx,vy]` 和四次方程的 4 个根保持不变；
- 候选块分配、释放和循环均由 `PrimitiveBatchWidth=8` 驱动；
- SoA 新增、更新、交换删除、剪枝和清理路径保持同步。

当前 Windows 环境没有 `cl`、`clang++`、`g++` 或 `cmake`，WSL 服务也
不可用，因此无法在本机完成 OMPL 编译、链接和性能运行。旧 README 的
double4 性能输出不是本版本数据。

## 8. 精度注意事项

- 最近邻距离、邻域半径比较、候选边求解和内部碰撞采样改为 float 后，
  在距离相近或紧贴边界时可能与 double4 版本选择不同节点或不同轨迹。
- 最终状态、树代价和 OMPL 接口仍为 double，但这不能恢复 SIMD 阶段已经
  舍入的信息。
- 应在原 OMPL/Linux 环境使用相同随机种子分别测试 double4 和 float8，
  对比运行时间、最终代价、成功率、碰撞边界和路径连续性。
