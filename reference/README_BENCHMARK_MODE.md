# 无绘图 Benchmark 模式

## 1. 目录

- 输入：`版本3加速（4000-0.27）_rake_avx2_soa_nn_avx2_float8`
- 输出：`版本3加速（4000-0.27）_rake_avx2_soa_nn_avx2_float8_benchmark`
- 输入目录未覆盖。

普通无参数模式保持原行为，会生成路径、构造 `PlannerData`、绘图、保存
`tree.png` 并调用 `waitKey`。新增的 `--benchmark` 模式不会执行这些操作。

## 2. 编译

沿用原工程的 OMPL/OpenCV 编译命令，建议至少使用：

```bash
-std=c++17 -O3 -mavx2 -DNDEBUG
```

启用 FMA 时增加 `-mfma`。完成后确认编译的是本目录的新 `test.cpp`，建议
先删除旧目标文件并做完整重编译。

## 3. 直接运行

```bash
./test --benchmark --warmup 5 --runs 50 --samples 4000 --seed 40 --timeout 30
```

参数：

| 参数 | 含义 | 默认值 |
|---|---|---:|
| `--benchmark` | 启用无绘图 benchmark | 关闭 |
| `--warmup N` | 不计入统计的预热轮数 | 5 |
| `--runs N` | 计入统计的独立规划轮数 | 50 |
| `--samples N` | 每轮要求接受的节点数 | 4000 |
| `--seed N` | OMPL 基础随机种子 | 40 |
| `--timeout SEC` | 每轮规划时间上限 | 30 |

每轮都会重新构造 `SimpleSetup` 和规划器，避免上一轮树、近邻缓存或代价
污染下一轮。计时区间只包围 `ss.solve()`，不包括问题构造和 `ss.setup()`。

输出包括：

- 成功求解次数；
- 实际接受样本数 min/max；
- mean、median、P95、min、max；
- 所有测量轮次的规划时间总和。

若任意一轮没有达到 `--samples`，程序返回退出码 2。这样可以直接发现
旧二进制、超时或实际只运行 400 节点的问题。

## 4. perf 脚本

```bash
chmod +x profile_benchmark.sh
sudo ./profile_benchmark.sh ./test
```

传入自定义参数时会替换脚本默认参数：

```bash
sudo ./profile_benchmark.sh ./test \
  --warmup 10 --runs 100 --samples 4000 --seed 40 --timeout 30
```

脚本固定 OpenBLAS、OpenMP 和 OpenCV 为单线程，先执行 5 次 `perf stat`
重复测量，再执行足够长的 `perf record`。硬件 PMU 不可用时自动回退到
software counters/CPU-clock。

## 5. 与旧测试相比

benchmark 模式不执行：

- `getPlannerData`；
- 树、路径和轨迹文本导出；
- `drawTreeAndTrajectory`；
- `cv::imshow`；
- `cv::imwrite`；
- `cv::waitKey`。

它还把 OMPL 日志级别设为 `LOG_ERROR`，防止每轮 `OMPL_INFORM` 输出进入
规划计时。程序会自行打印编译到的 primitive backend 和 benchmark 汇总。

## 6. 公平对比要求

double4 和 float8 必须使用：

- 相同编译器和优化参数；
- 相同 `--warmup/--runs/--samples/--seed/--timeout`；
- 相同虚拟机 CPU 数量和负载；
- 相同半径/K近邻模式、障碍物和碰撞采样数。

建议优先比较 median 和 P95，不用单次最小值作为加速结论。虚拟机若不
支持硬件 PMU，则不能从 perf 得到 cycles、instructions、IPC 和 cache
misses，只能比较稳定的 planning-time 统计与软件计数器。
