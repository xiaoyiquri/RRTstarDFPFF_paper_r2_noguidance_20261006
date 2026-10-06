# RRTstar2D AVX2 Rake 修改日志

> 本文件记录输入版本 `_rake_avx2_soa_nn_avx2` 的 `double + __m256d`
> 历史实现。当前 float8 副本的实际修改和验证结果见
> `README_SOA_NN_FLOAT8_AVX2_CHANGELOG.md`；下文中的 k=4 和性能输出不能
> 当作当前版本实测数据。

说明：下面的性能输出是原 `_rake_avx2` 版本留下的历史基线，不代表本次
SoA 最近邻/邻域改造后的新测试结果。本次环境没有可用的 C++/OMPL 构建链，
因此没有伪造新的性能数字。

```
e_avx2$ ./profile_perf.sh ./test
== perf stat ==
Info:    RRTstarDI: Primitive SIMD backend: AVX2
Planning...
Info:    RRTstarDI: Started planning with 1 states. Seeking a solution better than 0.00000.
Info:    RRTstarDI: Initial rewiring radius of 4.00
Info:    RRTstarDI: Found an initial solution with a cost of 32.91 in 0.000661 seconds, 215 iterations (77 vertices in the graph)
Info:    RRTstarDI: Created 400 new states. Checked 8788 rewire options. 7 goal states in tree (7 improved best cost). Final solution cost 18.498
Info:    RRTstarDI: SIMD edge blocks parent full/tail/candidates=1944/310/8374; rewire full/tail/candidates=1018/286/4627; full-block rate 83.2%, average lanes 3.65/4
Info:    Solution found in 0.008842 seconds
Planning elapsed time: 0.00893884 s
Found solution.
State count:   191
Control count: 190
[ WARN:0] global ./modules/imgcodecs/src/loadsave.cpp (771) imwrite_ imwrite_('tree.png'): can't open file for writing: permission denied
Saved image to: tree.png

 Performance counter stats for './test':

   <not supported>      cycles                                                                
   <not supported>      instructions                                                          
   <not supported>      branches                                                              
   <not supported>      branch-misses                                                         
   <not supported>      cache-misses                                                          

      22.789241946 seconds time elapsed

       0.603773000 seconds user
       0.329238000 seconds sys



== perf record ==
WARNING: Kernel address maps (/proc/{kallsyms,modules}) are restricted,
check /proc/sys/kernel/kptr_restrict and /proc/sys/kernel/perf_event_paranoid.

Samples in kernel functions may not be resolved if a suitable vmlinux
file is not found in the buildid cache or in the vmlinux path.

Samples in kernel modules won't be resolved at all.

If some relocation was applied (e.g. kexec) symbols may be misresolved
even with a suitable vmlinux or kallsyms file.

Couldn't record kernel reference relocation symbol
Symbol resolution may be skewed if relocation was used (e.g. kexec).
Check /proc/kallsyms permission or run as root.
Info:    RRTstarDI: Primitive SIMD backend: AVX2
Planning...
Info:    RRTstarDI: Started planning with 1 states. Seeking a solution better than 0.00000.
Info:    RRTstarDI: Initial rewiring radius of 4.00
Info:    RRTstarDI: Found an initial solution with a cost of 32.91 in 0.000821 seconds, 215 iterations (77 vertices in the graph)
Info:    RRTstarDI: Created 400 new states. Checked 8788 rewire options. 7 goal states in tree (7 improved best cost). Final solution cost 18.498
Info:    RRTstarDI: SIMD edge blocks parent full/tail/candidates=1944/310/8374; rewire full/tail/candidates=1018/286/4627; full-block rate 83.2%, average lanes 3.65/4
Info:    Solution found in 0.014288 seconds
Planning elapsed time: 0.0143383 s
Found solution.
State count:   191
Control count: 190
[ WARN:0] global ./modules/imgcodecs/src/loadsave.cpp (771) imwrite_ imwrite_('tree.png'): can't open file for writing: permission denied
Saved image to: tree.png
[ perf record: Woken up 1 times to write data ]
[ perf record: Captured and wrote 0.118 MB perf.data (970 samples) ]

== top hotspots ==
Warning:
Kernel address maps (/proc/{kallsyms,modules}) were restricted.

Check /proc/sys/kernel/kptr_restrict before running 'perf record'.

As no suitable kallsyms nor vmlinux was found, kernel samples
can't be resolved.

Samples in kernel modules can't be resolved as well.

# To display the perf.data header info, please use --header/--header-only options.
#
#
# Total Lost Samples: 0
#
# Samples: 970  of event 'task-clock:ppp'
# Event count (approx.): 970970970
#
# Overhead  Command         Shared Object                Symbol                                                                                                                                                                                                                                                                        
# ........  ..............  ...........................  ..............................................................................................................................................................................................................................................................................
#
     9.48%  test            libc.so.6                    [.] __sched_yield
            |
            ---__sched_yield
               0x4

     9.28%  test            [unknown]                    [k] 0xffffffffa9a27115
            |
            ---0xffffffffa9a27115
               0xffffffffa9c00130
               |          
                --8.76%--__sched_yield
                          0x4

     8.14%  test            ld-linux-x86-64.so.2         [.] do_lookup_x
            |
            ---do_lookup_x
               |          
                --1.55%--0

     5.98%  test            [unknown]                    [k] 0xffffffffa9a3e461
            |
            ---0xffffffffa9a3e461
               |          
                --5.57%--0xffffffffa8993399
                          0xffffffffa96c7102
                          0xffffffffa988202f
                          0xffffffffa96c009f
                          0xffffffffa96c350a
                          0xffffffffa96c3699
                          0xffffffffa96c372d
                          0xffffffffa8806615
                          0xffffffffa9a27151
                          0xffffffffa9c00130
                          __libc_sendmsg
                          |          
                           --5.15%--0x63eb00000000

     2.99%  test            [unknown]                    [k] 0xffffffffa9a35b38
            |
            ---0xffffffffa9a35b38
               0xffffffffa9a35c73
               0xffffffffa895d69d
               0xffffffffa895d6ee
               0xffffffffa880644f
               0xffffffffa9a27151
               0xffffffffa9c00130
               __sched_yield
               0x4

     1.55%  test            libc.so.6                    [.] _IO_fread
            |
            ---_IO_fread

     1.24%  test            libc.so.6                    [.] _int_free
            |
            ---_int_free

     1.03%  test            libgobject-2.0.so.0.7200.4   [.] g_signal_emit_valist
            |
            ---g_signal_emit_valist
               |          
                --0.93%--g_signal_emit

     1.03%  test            [unknown]                    [k] 0xffffffffa895d691
            |
            ---0xffffffffa895d691
               0xffffffffa895d6ee
               0xffffffffa880644f
               0xffffffffa9a27151
               0xffffffffa9c00130
               __sched_yield
               0x4

     0.93%  test            [unknown]                    [k] 0xffffffffa88d7ddb
            |
            ---0xffffffffa88d7ddb
               0xffffffffa9a2d4a3
               0xffffffffa9c00bc7

     0.72%  test            libc.so.6                    [.] _int_malloc
            |
            ---_int_malloc

     0.72%  test            libgdk-3.so.0.2404.29        [.] 0x000000000003232f
            |
            ---0x71062d12b32f
               0
               0x71062cd11450

     0.62%  test            ld-linux-x86-64.so.2         [.] _dl_relocate_object
            |
            ---_dl_relocate_object
               |          
                --0.52%--dl_main
                          _dl_sysdep_start
                          0x40

     0.62%  test            libglib-2.0.so.0.7200.4      [.] g_hash_table_lookup
            |
            ---g_hash_table_lookup

     0.62%  test            [unknown]                    [k] 0xffffffffa8958f02
            |
            ---0xffffffffa8958f02
               0xffffffffa9a35814
               0xffffffffa9a35c73
               |          
                --0.52%--0xffffffffa895d69d
                          0xffffffffa895d6ee
cxt@cxt-virtual-machine:/mnt/data/test/test2/版本3加速（4000-0.27）_rake_avx2$ 


```



## 1. 修改目标

本目录是 `版本3加速（4000-0.27）` 的独立副本，原目录没有被覆盖。
改动目标是：

1. 把二维双积分器的一条轨迹按 AVX2 的双精度 SIMD 宽度进行连续时间块
   碰撞检测。
2. 把约 4000 个二维节点的最近邻和邻域查询改为 SoA + AVX2 批量扫描。

当前代码使用 `double` 和 `__m256d`：

```text
AVX2 register = 256 bit
double         = 64 bit
k              = 256 / 64 = 4
```

因此本版本的 rake 宽度是 `k = 4`，不是 8。`k = 8` 只适用于 `float`
和 `__m256`。

## 2. 轨迹分组规则

设 `N = numCollisionCheckPoints`。起点 `t_0` 和终点 `t_N` 仍由原有的
单状态检查处理；内部采样点 `t_1 ... t_(N-1)` 按连续顺序分组：

```text
[t_1, t_2, t_3, t_4]
[t_5, t_6, t_7, t_8]
...
```

每个完整块进入四个 AVX2 lane：

```text
lane 0: t_i
lane 1: t_(i+1)
lane 2: t_(i+2)
lane 3: t_(i+3)
```

这是一条轨迹内部的 rake/空间交叉布局，不是四条轨迹混合，也不是把
四个远距离轨迹区间分配给四个 lane。

当最后一个块不足四个采样点时，代码只填充有效 lane，并使用标量尾部
路径处理；无效 lane 不参加碰撞判断。

## 3. 碰撞检测链路

`checkDynamicMotionWithTimeFast` 现在构造 FreeVel primitive 后调用
`checkFreeVelMotionWithTime`；FixedVel rewiring 调用
`checkFixedVelMotionWithTime`。两者最终共享
`validatePreparedMotionPrimitive`。

`validatePreparedMotionPrimitive` 的每个 rake 块依次执行：

1. `evaluateEdgeStateBlock` 生成四个时间点的状态。
2. AVX2 后端批量计算 `x/y/vx/vy`。
3. `FastEnvironment2D::checkStateBlock4Avx2` 批量检查状态。
4. 位置边界、速度边界、圆障碍和矩形障碍分别产生 lane mask。
5. 使用 `_mm256_movemask_pd` 归约；任意 lane 碰撞则整条轨迹拒绝。

状态布局是 SoA：

```text
x [t_i, t_(i+1), t_(i+2), t_(i+3)]
y [t_i, t_(i+1), t_(i+2), t_(i+3)]
vx[t_i, t_(i+1), t_(i+2), t_(i+3)]
vy[t_i, t_(i+1), t_(i+2), t_(i+3)]
```

## 4. 最近邻与邻域查询

`RRTstarDI` 在内部维护三个并行数组：

```text
soaMotions[]  -> Motion 指针
soaX[]        -> 节点 x 坐标
soaY[]        -> 节点 y 坐标
```

节点的 `Motion::soaIndex` 把指针映射回 SoA 行。新增节点、重连后状态
更新、子树状态传播、树剪枝和 `clear/freeMemory` 都同步维护这组数组。

最近邻查询 `nearestNeighborSoA`：

- 对 `soaX[]/soaY[]` 每四个 `double` 进行一次 AVX2 距离计算；
- 计算平方距离 `dx*dx + dy*dy`，避免不必要的平方根；
- 把四个 lane 的结果落回小数组后按原顺序归约最小值；
- 尾部不足四个节点时使用标量循环。

半径邻域查询 `getNeighborsSoA`：

- 一次计算四个节点的平方距离；
- 用 `_mm256_cmp_pd` 与 `_mm256_movemask_pd` 获得四 lane 成员掩码；
- 只把命中的 lane 转换为 `Motion *`；
- 最后处理不足四个节点的尾部。

K 近邻查询：

- 距离数组仍由 AVX2 批量生成；
- 只对索引执行 `nth_element` 和前 K 个索引排序；
- 不对 `Motion` 对象做 SIMD 写入，保持树结构和父子关系不变；
- 距离相同时使用 SoA 索引作为稳定的确定性次序。

这层 SoA 只替代二维距离预筛选，不改变 RRT* 的选父节点、重连、代价
传播和轨迹可行性判定。

## 5. 文件逐一检查

| 文件 | 处理结果 |
|---|---|
| `MyRRTstar.h` | 增加 `RakeBatchWidth`、SoA 字段和查询/生命周期接口；保持公共接口兼容。 |
| `MyRRTstar.cpp` | 接入 SoA 最近邻、半径邻域和 K 近邻；维护新增、重连、剪枝和清理；保留边评价及 RRT* 主逻辑。 |
| `di_primitive_helpers.h` | 核心修改。移除跨四个远距离区间的 lane 分配，改为连续 `t_i..t_(i+3)` rake 分组；尾块只检查有效采样点。 |
| `di_simd_backend.h` | 保持 AVX2 状态公式不变，补充说明四个 lane 来自同一条轨迹的连续时间块。 |
| `fast_environment_2d.h` | 保持并确认 AVX2 的边界、圆、矩形检查和 mask 归约；补充单轨迹 rake 语义说明。 |
| `di_primitive_types.h` | 明确 `DIStateBlock` 是轨迹状态 SoA 布局。 |
| `di_edge_eval_block.h` | 无需修改；用于候选边代价批处理，与单轨迹内部碰撞采样分组正交。 |
| `DICostUtils.cpp/.h` | 无需修改；代价和最优时间计算不改变碰撞采样语义。 |
| `test.cpp` | 无需修改；已有的 2D 边界、圆障碍、矩形障碍配置可直接驱动内置 AVX2 环境。 |
| `profile_perf.sh` | 无需修改；仍可用于比较副本与原版本的硬件性能。 |
| `FAST_VALIDITY_INTEGRATION.md` | 已更新为当前内置 AVX2 rake 实现和外部 validity hook 的真实行为。 |
| `README_RAKE_AVX2_CHANGELOG.md` | 本日志，记录 rake、SoA 查询、生命周期适配和验证结果。 |

## 6. 保持不变的行为

- 起点和终点有效性检查。
- 双积分器轨迹公式、控制约束和最优到达时间计算。
- RRT* 最近邻/邻域逻辑、选父节点和重连逻辑。
- 节点父子关系、代价传播和最终控制序列生成。
- 圆形障碍和矩形障碍的边界包含关系：边界上的点仍视为碰撞。

## 7. 验证记录

- 已建立独立副本，原目录未覆盖。
- 已静态核对全部 13 个目录文件的 include、类型、调用关系和修改影响。
- 已确认原有 AVX2 分支使用 `__m256d`，SIMD 宽度为 4。
- 已确认旧的四段 lane 分配代码已移除，副本只保留连续 rake 分组。
- 已确认 `solve()` 不再调用 `nn_->nearest()`，半径模式不再调用 `nn_->nearestR()`，
  K 模式不再调用 `nn_->nearestK()`；三者均走 SoA 距离扫描。
- 已确认节点新增、重连、子树状态传播、剪枝删除和清空操作都维护 SoA。
- `setup()` 会打印 `XY nearest-neighbor backend: SoA AVX2 (4 doubles)`；如果
  显示 `scalar`，说明编译命令没有启用 AVX2。
- 当前工作区未提供可直接调用的 C++/CMake 构建工具，且目录没有
  `CMakeLists.txt` 或 `Makefile`，因此本次不能在此环境完成 OMPL 链接编译。
  编译时应使用 AVX2 选项，例如 GCC/Clang 的 `-mavx2 -O3`，并链接
  工程所需的 OMPL 和 OpenCV。
