# RTOS 知识与本项目实践

本文面向已经会写 Arduino `setup()` / `loop()`、准备理解和维护本项目 FreeRTOS 版本的开发者。先读 [迁移设计](RTOS_MIGRATION.md) 了解任务分工，再用本文理解调度、通信和验证。当前工程仍使用 PlatformIO Arduino 框架；ESP32 的 Arduino 运行在 ESP-IDF/FreeRTOS 之上，应用已经创建自己的任务，尚未改为纯 ESP-IDF 工程。

## 1. 实时性到底是什么

RTOS 的价值是让关键工作在**规定时间内**完成，而不是让 CPU 平均跑得更快。需要分清四个量：

| 量 | 本项目的例子 | 如何判断 |
|---|---|---|
| 周期 | IMU 任务每 5 ms 尝试采集一次 | `vTaskDelayUntil()` 设定目标唤醒间隔 |
| 执行时间 | 一次 I²C 读取、入队所花时间 | 上板测量，须小于周期并留余量 |
| 延迟 | 到唤醒时间后，任务真正开始运行的等待 | 受更高优先级任务、中断、关中断时间影响 |
| 抖动 | 相邻实际采样间隔相对 5 ms 的变化 | `/diagnostics` 的 `imuIntervalUs`、超 10 ms 次数 |

因此“5 ms 周期”只代表调度目标，不能直接写成“稳定 200 Hz”。记录样本的 `micros()` 时间戳，并用**实际间隔**计算积分 `dt`。CPU 空闲率或构建成功也不能证明采样截止时间达标。

## 2. 任务、状态和调度

一个任务通常在 `Running`（运行）、`Ready`（可运行，等 CPU）、`Blocked`（等待时间、队列或事件）之间切换，也可以被显式挂起。高优先级就绪任务会抢占低优先级任务；同优先级任务的时间片行为受 FreeRTOS 配置影响。优先级只表达**紧急性**，不代表工作量大小。

本项目在 [`src/rtos_runtime.cpp`](../src/rtos_runtime.cpp) 创建五个应用任务：

| 任务 | 核 / 优先级 | 主要工作 | 等待方式 |
|---|---|---|---|
| IMU | 1 / 4 | 读取 MPU6050，样本入队 | 5 ms `vTaskDelayUntil()` |
| Model | 1 / 3 | GPS UART、融合、状态机、快照 | 队列首样本最多等约 2 ms |
| Storage | 0 / 2 | SD、录音与命令 | 每轮延时约 5 ms |
| Web | 0 / 1 | HTTP、串口请求 | 每轮延时约 2 ms |
| Display | 1 / 1 | OLED 与按钮 | 50 ms `vTaskDelayUntil()` |

Wi-Fi、TCP/IP、系统事件和空闲任务也由框架创建，并非只有这五个任务。固定到不同 CPU 核能减少部分竞争，但共享内存、总线、Flash、锁和系统任务仍会互相影响；不能把 core 0 当作完全独立的“慢任务处理器”。

### `vTaskDelay()` 与 `vTaskDelayUntil()`

```cpp
TickType_t wake = xTaskGetTickCount();
for (;;) {
    sampleImu();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
}
```

`vTaskDelay(5 ms)` 从**本轮工作结束**开始等待，执行时间会累积到周期里。`vTaskDelayUntil()` 以固定唤醒基准安排周期，更适合周期采样。若执行超时，它不能追回错过的样本；本项目的 `periodicWait()` 在超时后重置基准，避免随后连续突发读取。`pdMS_TO_TICKS(5)` 受系统 tick 分辨率限制；代码用 `static_assert(configTICK_RATE_HZ >= 200)` 保证 5 ms 至少能表示，但仍需实测。

不要在高优先级循环里持续轮询而不阻塞或让出 CPU，否则会饿死低优先级任务，也可能触发任务看门狗。`modelTask()` 每轮通过队列短暂等待，并且最多处理 8 条 IMU 样本，使 GPS UART 有机会被及时处理。

## 3. 跨任务数据如何传

先问“谁拥有数据、是否必须逐条处理、能否丢弃”，再选原语：

| 需求 | 合适机制 | 本项目实例与语义 |
|---|---|---|
| 每条样本都尽量处理 | 有界队列 | `imuQueue` 深度 32；满时计数、丢新样本，不阻塞 IMU |
| 仅需最新值 | 长度 1 队列 + `xQueueOverwrite()` | `sessionQueue`；旧待保存会话会被新值覆盖 |
| 请求所有者执行动作 | 命令队列 | Web 发录音/日志命令；入队成功不等于执行成功 |
| 复制当前一致状态 | 互斥锁 + 内存快照 | `snapshotMutex` 保护 `telemetry` / `storage` |
| 等待系统启动条件 | 事件组 | 所有任务等待 `Ready`，资源创建成功后才启动 |
| 一个任务的轻量唤醒或计数 | 任务通知 | 当前未使用；可在单接收者场景替代部分信号量/队列 |

FreeRTOS 队列按值复制结构体，因此 [`ImuSample`](../src/rtos_runtime.cpp) 等类型有 `trivially_copyable` 编译检查。队列消耗的内存约为“深度 × 元素大小”，还要加队列控制结构；大对象频繁复制会增大 RAM 和 CPU 开销。若以后传指针，必须同时设计缓冲区所有权与归还时机，不能把局部变量地址入队。

`xQueueSend(..., 0)` 表示不等待。这里宁可把 `imuDropped`、`logDropped` 计入诊断，也不允许慢 SD 或网络把采集任务卡住。丢弃策略要与数据用途匹配：实时控制或测量原始轨迹可能需要覆盖最旧值、环形缓冲或流缓冲，不能无条件沿用当前“丢新值”。

### 互斥、优先级反转和锁顺序

共享变量需要保护的是**一次完整读写操作**，不是单个字段。Model 先算好状态，再持锁复制快照；Web/OLED 复制后释放锁，再做 JSON、网络和 I²C。持有 `snapshotMutex` 时不能进行慢 I/O 或等待另一条队列。

低优先级任务持锁时，高优先级任务可能被迫等待，这叫优先级反转。FreeRTOS 互斥量有优先级继承机制；二值信号量不提供相同的互斥语义。继承不能消除长临界区，因此首先缩短持锁时间。若以后一个操作需要多把锁，统一获取顺序，并避免在持有快照锁时获取文件系统锁，防止环形等待。

`filesystemMutex` 隔离 SPIFFS 读写；SD/录音所有权属于 Storage 任务。文件下载等慢操作仍可能让该任务等待，当前录音时拒绝下载。要在硬件上验证高并发场景下的队列积压和丢弃。

## 4. 中断、DMA 与外设所有权

中断服务函数应短小：只做必要的读状态、清中断和通知；使用带 `FromISR` 后缀的 FreeRTOS API，不调用普通阻塞队列、文件系统、WebServer 或耗时串口输出。需要唤醒更高优先级任务时，按端口 API 要求请求从 ISR 切换任务。GPS 当前由 Model 任务轮询 UART，IMU 由 IMU 任务发起 I²C 读取，并没有自定义 ISR 驱动它们。

DMA 让外设把音频采样搬到缓冲区，减少 CPU 逐样本服务的负担，但不能保证写盘永远跟得上。[`src/audio_recorder.cpp`](../src/audio_recorder.cpp) 的 ADC/I²S 路径仍要检查 DMA 溢出、WAV 时长、采样值和存储延迟。I²C、SPI/SD、UART、OLED 分别指定单一操作任务，是为了让驱动状态和调用顺序可推理；双核不意味着同一驱动可以被两个任务无锁调用。

## 5. 状态机与任务的关系

**任务**决定代码何时运行，**状态机**决定收到事件后下一步做什么。两者不应混成“每个状态建一个任务”。本项目 [`src/runtime_state.h`](../src/runtime_state.h) 的 `RuntimeState::observe()` 是一个不依赖 Arduino 的状态转换函数，由 Model 任务独占调用。Web 只读发布的快照。

| 状态机 | 输入事件/条件 | 主要状态 | 必须保持的约束 |
|---|---|---|---|
| GPS | 新字节、有效位置/速度、1500 ms 超时 | `NoSignal`、`Acquiring`、`Fixed`、`Stale` | `Fixed` 之外不能把旧速度当实时速度 |
| 骑行 | 有效速度 ≥5、<2、失效、30 s 冷却 | `Idle`、`Riding`、`Cooling` | 冷却期速度恢复可回到骑行 |
| 健康 | GPS 固定、IMU 100 ms 内采样、启动 10 s | `Booting`、`Ready`、`Degraded` | `Ready` 是数据可用条件，不是精度认证 |

可以用“事件 → 守卫条件 → 新状态 → 副作用”审查每条转换。例如 `Riding` 中速度失效，守卫 `!validSpeed` 成立，进入 `Cooling` 并记录 `coolingSince`。`uint32_t(now - then)` 适合处理毫秒计数器回绕，但时间窗口仍应远小于半个计数范围。性能计时器是另一套独立状态机；GPS 断流应终止**进行中的**计时，不抹去已完成结果。

现阶段骑行状态和 SD 自动记录仍分别实现了 5/2 km/h、30 s 规则。这是后续应统一的边界，否则一次阈值修改可能让诊断状态和实际写盘行为不一致。

## 6. 内存、任务栈与看门狗

本工程所用 ESP-IDF FreeRTOS 端口中，`xTaskCreatePinnedToCore()` 的栈大小参数是**字节**，与上游 FreeRTOS 常见的“字数”不同。当前五个应用任务分别申请 4096、8192、8192、12288、6144 字节栈；创建队列、互斥量和事件组也使用堆。PlatformIO 构建显示的静态 RAM 占用，不能代表运行时剩余堆或栈安全余量。

[`readRtosDiagnostics()`](../src/rtos_runtime.cpp) 使用 `uxTaskGetStackHighWaterMark()` 提供启动以来的栈最低余量；在此 ESP-IDF 端口中按字节理解。它是历史最低**未使用**栈，不等于当前空闲堆，也不能保证未覆盖的路径安全。要在 GPS 有/无信号、网络高频访问、SD 写入、录音和下载等组合负载下观察最小值。任务创建失败应当停止启动；本项目先创建资源和任务，让任务等待 `Ready`，全部成功后再放行。

看门狗超时可能来自长期不让出 CPU、死锁、关中断过久或外设调用卡住。先记录是哪类看门狗、任务/回溯和当时负载，再检查锁和循环，不应靠简单增大超时掩盖问题。

## 7. 本项目上板验证怎么做

本机回放和 `pio run` 只验证纯逻辑与可编译性。建议每次改变优先级、缓冲深度或驱动后，在同一固件版本记录以下数据：

| 场景 | 观察项 | 判断方式 |
|---|---|---|
| 静置 30 分钟 | `imuIntervalUs`、超 10 ms 次数、各栈余量 | 建立基线，确认无持续增长或异常重启 |
| GPS 断开/恢复 | GPS/健康状态、速度新鲜度、进行中的计时 | 超时变 `Stale`，恢复后状态按有效数据推进 |
| 高频请求 `/data` 和 `/diagnostics` | IMU 间隔、Web 延迟、快照年龄 | Web 负载不应持续拖慢采集 |
| SD 插拔、空间不足或写入失败 | `logDropped`、写错误、系统继续运行 | 记录故障，传感器和网页仍可用 |
| 录音 + SD + 网页组合 | DMA 溢出、录音时长、队列水位、栈余量 | 验证音频连续性和采集截止时间 |
| 开机缺少 IMU/GPS | 启动、状态、HTTP 可访问性 | 降级可诊断，不用旧数据伪装正常 |

先建立基线，再为“允许丢多少条”“最大采样间隔”“最小栈余量”定项目阈值。`/diagnostics` 的累计计数只能说明出现过问题；长时间压测应定期取样并计算增量。当前没有完整的运行时剩余堆字段，可在后续诊断中增加；不要把构建报告里的 RAM 百分比当成运行时堆余量。

## 8. 从 Arduino 任务版走向纯 ESP-IDF

当前 `setup()` 完成驱动初始化并调用 `startRtosRuntime()`；`loop()` 不再承载业务调度。ESP32 的调度器由框架启动，应用不应再调用 `vTaskStartScheduler()`。如果下一阶段切到纯 ESP-IDF，可把初始化入口迁移到 `app_main()`，保留任务边界、消息结构和纯 C++ 状态机，逐个替换 Arduino 的 `Wire`、`HardwareSerial`、WebServer、SPIFFS 等驱动。每替换一个驱动，就用同一批回放输入和硬件负载场景核对行为；框架切换本身不会自动解决时间同步、并发或硬件稳定性问题。

## 官方参考

- [ESP-IDF v4.4.8 FreeRTOS API](https://docs.espressif.com/projects/esp-idf/en/v4.4.8/esp32/api-reference/system/freertos.html)：任务创建、双核调度、ESP32 栈大小单位。
- [ESP-IDF v4.4 性能与任务优先级](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-guides/performance/speed.html)：系统任务和运行性能。
- [ESP-IDF v4.4.8 看门狗](https://docs.espressif.com/projects/esp-idf/en/v4.4.8/esp32/api-reference/system/wdts.html)：中断和任务看门狗。
- [ESP-IDF v4.4 内存使用](https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-guides/performance/ram-usage.html)：静态内存、堆和栈。
- [FreeRTOS Reference Manual](https://www.freertos.org/media/2018/FreeRTOS_Reference_Manual_V10.0.0.pdf)：队列、事件组、任务通知和互斥量 API。
