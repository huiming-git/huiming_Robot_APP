# ROBOT_CPP（轮腿机器人固件，STM32F407 + HAL + FreeRTOS + C++23）

这是一套运行在 **STM32F407（RoboMaster C 板）** 上的固件工程。

工程由两部分组成：
- `Core/`：STM32CubeMX 生成的 HAL/FreeRTOS 工程（外设初始化、中断、任务调度、与硬件交互）
- `RobotApp/`：C++23 业务层（状态估计、逻辑、控制、诊断输出）

本 README 目标：用非常明确的、接近“程序逻辑描述”的语言，告诉你：
1) 工程是怎么跑起来的（任务/频率/调用链）  
2) 数据是怎么流动的（从传感器到电机）  
3) 每个目录/关键文件是干什么的  
4) 控制算法接口在哪里、你应该改哪里  
5) 参数怎么保存到 Flash、怎么在线调参  

---

## 项目介绍（长版）

### 0.0 项目在做什么
本项目在 STM32F407 上实现一条完整的实时控制链路。核心输出是“执行器命令”，核心输入来自“遥控、传感器、电机反馈”。

按实际运行行为，这条链路可以概括成：
- 传感器采样（IMU/Mag）→ 标定/单位换算 → 形成可用测量
- 状态估计（姿态/速度/位置）→ 输出 `StateEstimate`
- 逻辑层（遥控/模式/状态）→ 输出高层命令或直接执行器命令
- 控制层（高层命令 + 估计状态）→ 输出轮电流/关节命令 → CAN 下发到电机
- 诊断层把上述关键状态合并成快照，提供 LiveWatch 直接查看

### 0.1 为什么要分 Core / RobotApp
本项目明确把“平台相关”和“业务计算”分开，原因是：
- 平台相关部分（HAL/FreeRTOS）不可避免地依赖具体芯片、外设、回调时序；这部分要稳定、可重复生成、易排查中断问题
- 业务计算部分（估计/逻辑/控制）需要清晰的接口、清晰的数据结构、可替换的算法实现；这部分应尽量不被 HAL/RTOS 细节污染

具体边界是：
- `Core/` 负责把硬件事件转换为“可消费的数据”（放入缓冲/队列），并按固定频率触发 `RobotApp` 的入口函数
- `RobotApp/` 负责消费数据、做计算、输出命令，并把“系统正在发生什么”发布为统一诊断快照

### 0.2 工程的实时运行方式（任务与节奏）
工程运行依赖 FreeRTOS 任务与一个 1kHz 调度节拍：
- 控制任务：1kHz（每 1ms 一次），负责估计 + 控制 + 电机协议栈 tick
- 传感器任务：200Hz（每 5ms 一次），负责采样 BMI088/IST8310 并上报给 RobotApp
- 逻辑任务：100Hz（每 10ms 一次），负责把遥控/状态转换成高层命令
- 遥控任务：事件驱动（由 USART3 DMA+IDLE 回调唤醒），负责解析 SBUS 帧并更新遥控状态
- 后台任务：1ms（`App_Params_Service()`），负责参数持久化服务（保存请求、擦除、写入、退避等）

任务创建位置：`Core/Src/freertos.c`。

### 0.3 主数据流（按“谁产生、谁消费”的关系）
为了让数据跨任务、跨中断传递时延可控，本工程将数据流分成两类：

#### 0.3.1 ISR → 任务：无锁 pipe / 队列
用于“硬件事件驱动的数据流”，典型特征是：数据到来时间由硬件决定，ISR 只做搬运与唤醒。

- SBUS：
  - ISR（USART3 DMA+IDLE 回调）把收到的字节写入无锁 byte pipe
  - sbusTask 被唤醒后从 pipe 取字节并喂给解析器
  - 解析出帧后更新 `RemoteState/OperatorState`
  - 相关文件：`Core/Src/usart.c`、`Core/Inc/app_sbus.h`、`RobotApp/Tasks/sbus_task.cpp`、`RobotApp/Inputs/sbus.*`

- CAN：
  - CAN Rx ISR 把帧写入无锁队列
  - control loop（1kHz）把队列中的帧弹出并路由到电机协议栈
  - 相关文件：`Core/Src/can.c`、`Core/Inc/app_can.h`、`RobotApp/Platform/can_port.hpp`、`RobotApp/Tasks/control_loop.cpp`

#### 0.3.2 任务 → 任务：双缓冲快照（DoubleBuffer）
用于“周期任务之间共享状态”，典型特征是：写端在固定频率发布整份结构体快照，读端随时读最新快照，不依赖互斥锁。

- `RobotApp/Util/double_buffer.hpp` 实现双缓冲：
  - 写端调用 `push()` 发布一个完整结构体
  - 读端调用 `latest()` 获取最后一次发布的完整结构体

本工程使用双缓冲发布的关键对象包括：
- `RemoteState`、`OperatorState`、`RemoteDiagnostics`
- `SensorsSnapshot`（IMU+Mag+MotorFeedback 聚合）
- `StateEstimate`（估计器输出：姿态/速度/位置）
- `SystemHealth`（系统健康与错误计数）
- `DiagnosticsSnapshot`（统一诊断快照）

### 0.4 “估计器/融合/滤波”在工程里是什么
本工程把“从传感器到状态量”的模块称为估计器（Estimator）。它的职责是：
- 输入：IMU（陀螺/加速度）、磁力计（可选）、轮速反馈（可选）
- 输出：`StateEstimate`（姿态/速度/位置/偏置/有效性）

当前主链路使用的估计器是：
- 文件：`RobotApp/Estimation/ins_estimator.hpp` / `RobotApp/Estimation/ins_estimator.cpp`
- 类型：`robotapp::estimation::InsEstimator`
- 调用位置：`RobotApp/Bridge/app_entry.cpp` 的 `RobotApp_ControlTick(ts_us)`，每 1kHz 调用一次

关于姿态表示：
- 内部状态用四元数 `q_wxyz[4]`（连续、适合积分与旋转运算）
- `rpy_rad[3]`（roll/pitch/yaw）是从四元数换算得到的调试输出，范围会环绕到 `[-π, π]`，在 ±π 附近跳变是正常现象

补充：
- `RobotApp/Algo/imu_kalman.hpp` 是占位接口，不是当前主链路在用的估计器

### 0.5 “逻辑”和“控制”的区别
工程中把“高层意图”和“执行器输出”分开：

- 逻辑层（Logic）：
  - 输入：遥控状态、系统健康、传感器快照（用于决策）
  - 输出：
    - 高层命令 `HighLevelCommand`（例如期望前进速度/角速度等归一化指令）
    - 或直接输出执行器级命令 `ChassisCommand`（当你需要完全绕过控制器时）
  - 相关目录：`RobotApp/Logic/`

- 控制层（Control）：
  - 输入：高层命令 + 估计状态 + 传感器快照 + 轮参数/控制参数
  - 输出：执行器级命令 `ChassisCommand`（轮电流、关节命令）
  - 相关目录：`RobotApp/Tasks/`（主循环）与 `RobotApp/Algo/`（可替换控制器实现）

### 0.6 安全策略的角色（SafetyPolicy）
安全策略不负责“控制计算”，只负责“裁决是否允许输出/如何降级输出”。它的输入来自系统健康快照，输出是一组 flags 与裁决结果。

- 文件：`RobotApp/Safety/safety_policy.hpp`
- 调用位置：`RobotApp/Tasks/control_loop.cpp` 的 1kHz tick
- 典型决策：
  - 遥控失联/质量差 → 停机（输出全 0）
  - 某条 CAN 总线拥塞/故障 → 只停止对应总线上的执行器输出
  - SPI/I2C 通信错误累积 → 进入安全模式

### 0.7 诊断输出与 LiveWatch（如何“看到系统状态”）
工程将调试输出当作正式功能，所有关键状态都会被聚合到 `DiagnosticsSnapshot` 并镜像到全局变量，LiveWatch 可直接读。

- 关键结构：`RobotApp/Domain/types.hpp::DiagnosticsSnapshot`
- 聚合位置：`RobotApp/Bridge/app_entry.cpp`（在 ControlTick 内组装并发布）
- LiveWatch 主要入口变量：
  - `g_robotapp_diag`：统一诊断快照（包含 remote/sensors/estimate/actuator_cmd/safety/telemetry 等）
  - `g_robotapp_rc_state`、`g_robotapp_rc_diag`：遥控状态与统计
  - `g_control_dt_us`、`g_control_jitter_us`：控制任务周期与抖动
  - `g_app_sbus_rx_events`、`g_app_sbus_rx_bytes`：USART3 SBUS 接收是否在进数据（回调次数/累计字节）

### 0.8 控制算法应该改哪里（接口与落点）
控制算法的接口定义在：`RobotApp/Algo/chassis_controller.hpp`。

推荐你实现控制算法的位置：
- `RobotApp/Algo/user_controller.cpp`

控制器安装方式：
- 启动时通过 `ChassisControllerOverride()`（弱符号）返回函数指针，`RobotApp` 会自动安装
- 或在初始化阶段显式调用 `robotapp::InstallChassisController(fn)`

---

## 设计思想（问答）

下面是对“这个工程为什么被设计成这样”的推断总结。每一条都对应工程里已经落地的结构或约束。

**Q：为什么要把工程拆成 `Core/` 和 `RobotApp/` 两层？**  
**A：**为了把“平台相关的硬件细节”和“业务计算逻辑”隔离开。`Core/` 负责外设/中断/任务调度，`RobotApp/` 负责估计/逻辑/控制与统一诊断。这样做的直接结果是：`RobotApp/` 的代码可以更稳定地演进，并且更容易迁移到别的 MCU/RTOS（只需要重写端口层）。

**Q：为什么 ISR/回调里只允许搬运数据和唤醒任务？**  
**A：**因为 ISR 的执行时间如果不可控，会直接影响系统实时性：抢占更高优先级、阻塞其它中断、导致控制任务抖动。把 ISR 限制为“写入预分配缓冲 + 发 flag”，可以把复杂计算稳定地放到任务上下文中完成。

**Q：为什么要用 DMA+IDLE + 无锁 byte pipe 来收 SBUS？**  
**A：**SBUS 是连续字节流，DMA+IDLE 可以降低 CPU 中断频率；ISR 把字节推入 pipe，任务侧再解析帧，能保证：ISR 不做解析、不做循环等待；任务解析失败也不会卡死系统。

**Q：为什么要在调度开始后彻底禁止 `malloc/new`？**  
**A：**运行期堆分配会引入不可预测的延迟（分配/碎片/失败路径），并且可能在长期运行后出现碎片化导致崩溃。该工程通过 `heap_guard` + 链接器 `--wrap=malloc/free/...` 做“硬拦截”，把这类问题提前暴露为可调试的断点，而不是线上随机死机。

**Q：为什么要避免互斥锁（mutex）并改用“无锁缓冲 + 双缓冲快照”？**  
**A：**互斥锁会引入优先级反转、不可控等待、以及“拿锁的路径是否覆盖所有情况”的维护成本。该工程在 ISR→任务使用无锁队列/pipe，在任务→任务共享状态使用 `DoubleBuffer` 发布“整份快照”，目的是让数据交换的时延和行为更确定。

**Q：为什么要把数据组织成 `SensorsSnapshot` / `StateEstimate` 这种“快照结构体”？**  
**A：**因为控制和逻辑需要的是“同一时间点可用的一组信息”，而不是多个模块的零散变量。用快照结构体发布可以做到：读取端一次读取就能拿到完整一致的数据视图，避免“读到一半被更新”的问题。

**Q：为什么姿态内部用四元数 `q_wxyz`，还要额外输出 `rpy_rad`？**  
**A：**四元数适合连续积分与旋转运算，不会出现 ±π 的环绕跳变，适合作为内部状态。`rpy_rad` 是为了调试可读性（roll/pitch/yaw 更直观），但它会在 `[-π, π]` 环绕且存在欧拉角奇异性，所以只作为输出观察，不作为内部状态。

**Q：为什么“状态估计”和“控制器”要分开？**  
**A：**估计器负责把传感器转换为状态量（姿态/速度/位置），控制器负责把命令与状态量转换为执行器输出（电流/力矩/位置/速度目标）。两者分开可以独立迭代：你可以替换估计器而不动控制器，也可以替换控制器而不动估计器。

**Q：为什么逻辑层要输出两种命令：`hl_cmd`（高层）和 `cmd`（执行器级）？**  
**A：**为了让系统既支持“走控制器”（高层命令→控制计算→执行器输出），也支持“直通执行器命令”（上层直接给电流/力矩/位置目标）。当你调试或引入高级控制算法时，直通命令可以减少中间环节；而高层命令适合遥控与自动模式的统一接口。

**Q：为什么要有单独的安全策略 `SafetyPolicy`，而不是在各处 if 判断？**  
**A：**安全裁决需要一致的策略、稳定的 flags 位定义、以及可观测的统计。集中在 `SafetyPolicy` 可以做到：输入是系统健康快照，输出是明确的裁决（global stop / wheels stop / joints stop）和 flags；控制环只需要按裁决“裁剪输出”，避免散落的安全逻辑互相冲突。

**Q：为什么要把诊断做成 `DiagnosticsSnapshot` 并暴露 LiveWatch 全局变量？**  
**A：**嵌入式系统排障最难的是“现场到底发生了什么”。该工程把遥控、传感器、估计、命令、健康度、计数器等聚合成快照，并镜像到全局变量，目的是让你在调试器里能直接看到每一段链路是否在工作、数据是否新鲜、是否在降级/停机。

**Q：为什么所有数据都要带 `ts_us` 时间戳？**  
**A：**因为“数据新鲜度”是安全和控制的基础。通过 `ts_us` 可以做超时判定（例如 IMU 超时、遥控超时、电机反馈超时），也能在诊断中解释为什么进入降级（比如 remote_age_us/imu_age_us/motor_age_us）。

**Q：为什么控制环选 1kHz、传感器 200Hz、逻辑 100Hz？**  
**A：**这是一个典型的实时分工：控制输出需要更高频率降低延迟与抖动；传感器采样受硬件与总线限制，200Hz 足够覆盖姿态/加速度变化；逻辑决策（遥控映射、模式切换）不需要 1kHz，100Hz 足够并能减少 CPU 占用。

---

## 0. 这套工程的强制规则（你修改代码时必须遵守）

### 0.1 ISR/回调必须无阻塞
**规则**：中断/回调里只允许做两类动作：
- 写入预先分配好的缓冲（例如无锁环形缓冲、双缓冲）
- 通知任务（设置 flag、释放信号量、触发回调）

**禁止**：
- 等待（锁/信号量/延时）
- 运行“业务逻辑”（比如解算、控制计算）
- 动态内存分配（malloc/new）
- 调用可能有不确定延迟的函数

### 0.2 调度开始后禁止运行期堆分配
本工程已经实现了“硬保护”：
- 一旦进入任务运行状态（我们在 `RobotApp_ControlTick/LogicTick/...` 入口里标记“started”）
- 如果任何代码调用 `malloc/free/calloc/realloc/new/delete`
- 程序会触发断点（BKPT）并停在死循环里

相关实现：
- `RobotApp/Platform/heap_guard.cpp`
- `CMakeLists.txt` 里用了 linker `--wrap=malloc/free/...`

**你要知道的结果**：
- 调度开始后，请不要在任务里写会分配堆的代码
- 也不要随意引入可能在内部 malloc 的库（例如某些 printf/iostream/容器操作）

### 0.3 RobotApp 层尽量不直接依赖 HAL/FreeRTOS
原则是：
- HAL/FreeRTOS 相关的类型和函数调用，尽量都在 `Core/` 和 `Core/Inc/app_*.h` / `Core/Src/app_*.c` 内完成
- `RobotApp/` 通过“端口层”（`app_*.h` 或 `RobotApp/Platform/*.hpp`）来用平台能力

---

## 1. 目录与分层（你打开工程第一眼要看这里）

### 1.1 顶层目录
- `Core/`：CubeMX 生成的代码（HAL/FreeRTOS/外设初始化/中断回调）
- `Drivers/`：ST 的 HAL 驱动、CMSIS 等
- `Middlewares/`：FreeRTOS 源码等
- `RobotApp/`：我们自己的 C++23 业务层
- `linker/`：CMake 构建使用的 linker script（避免被 CubeMX 覆盖）
- `cmake/`：CMake toolchain 与 CubeMX 子工程 glue
- `imu/`：BMI088 官方/第三方 C 驱动文件（底层）

### 1.2 Core 与 RobotApp 的关系（非常重要）

**Core 负责**：
- 外设初始化（SPI/I2C/CAN/UART/DMA/定时器）
- 中断接收数据
- 把数据放进无锁队列/缓冲（ISR 做到这里就结束）
- FreeRTOS 任务创建与调度

**RobotApp 负责**：
- 接收“已经从 ISR/驱动层推上来的数据”
- 做状态估计（姿态/速度/位置）
- 做逻辑（遥控 → 高层命令）
- 做控制（高层命令 → 电机电流）
- 输出诊断快照（给你 LiveWatch/上位机看）

---

## 2. 工程如何运行（任务/频率/调用链）

### 2.1 任务列表与频率
任务在 `Core/Src/freertos.c` 里创建：
- `StartControlTask`：控制任务，**1kHz**（每 1ms 一次）
- `StartSensorTask`：传感器任务，**200Hz**（每 5ms 一次）
- `StartLogicTask`：逻辑任务，**100Hz**（每 10ms 一次）
- `StartSbusTask`：遥控任务，按 SBUS 数据到来频率运行（一般 7ms 左右一帧，任务侧等待 flag）
- `StartDefaultTask`：后台任务，做参数 Flash 保存服务（`App_Params_Service()` 每 1ms 调一次）

### 2.2 控制链路（你最关心的一条链）
下面按实际调用顺序写（没有比喻，只有真实逻辑）：

#### 2.2.1 SBUS → 遥控状态（RemoteState / OperatorState）
1. `Core/Src/usart.c`
   - USART3 使用 DMA+IDLE
   - 中断回调 `HAL_UARTEx_RxEventCallback` 把收到的字节写入无锁 byte pipe
   - 并给 `sbusTask` 发 `APP_SBUS_RX_FLAG`
2. `RobotApp/Tasks/sbus_task.cpp`
   - 等待 `APP_SBUS_RX_FLAG`
   - 调 `App_SbusPipe_Read()` 读出字节
   - 调 `RobotApp_SbusFeedBytes(data,len,ts_us)`
3. `RobotApp/Bridge/app_entry.cpp` 中的 `RobotApp_SbusFeedBytes`
   - 解析 SBUS 帧
   - 更新 `RemoteState`（遥控通道归一化到 -1..1）
   - 更新 `OperatorState`（是否 enable、是否 e-stop、模式等）
   - 把这些状态发布到双缓冲（供控制环/逻辑环读取）

#### 2.2.2 传感器 → IMU/Mag 状态
1. `RobotApp/Tasks/sensor_task.cpp`（200Hz）
   - BMI088：`BMI088_PollOnce()` 读取一次 raw frame
   - 组装成 `domain::ImuSample`，调用 `RobotApp_UpdateImuSample(&sample)`
   - IST8310：50Hz 异步 I2C 触发读取，读取到新数据就调用 `RobotApp_UpdateMagSample(&sample)`
2. `RobotApp/Bridge/app_entry.cpp`
   - `RobotApp_UpdateImuSample` 把 raw sample 写到 `ImuAdapter` 的双缓冲
   - `RobotApp_UpdateMagSample` 把 raw sample 写到 `MagAdapter` 的双缓冲
3. `RobotApp/Drivers/imu_adapter.hpp`
   - 把 BMI088 原始计数转换成物理单位：
     - accel：`m/s^2`
     - gyro：`deg/s`
   - 再应用标定参数（scale/bias）
   - 输出 `domain::ImuState`
4. `RobotApp/Drivers/mag_adapter.hpp`
   - 把 IST8310 原始计数按标定 scale/bias 转成 `domain::MagState`（单位 `uT`）

#### 2.2.3 电机反馈 → MotorFeedback
1. `Core/Src/can.c`
   - CAN Rx 中断把收到的帧写入无锁 Rx 队列
2. `RobotApp/Tasks/control_loop.cpp`（1kHz）
   - 每 tick 调用 `can_port_.rx_pop(frame)` 把队列里的帧弹出
   - `motor_mgr_.route_frame(frame)` 分发到对应电机对象
   - 读取电机对象的状态，组成 `domain::MotorFeedback`（包含两轮 3508 和 4 个 DM4310）
   - 调用 `RobotApp_UpdateMotorFeedback(&fb)` 发布给 RobotApp（并带时间戳 `fb.ts_us = ts_us`）

#### 2.2.4 估计（StateEstimate）
发生在 `RobotApp_ControlTick(ts_us)` 里（由 `StartControlTask` 1kHz 调用）：
1. `RobotApp/Bridge/app_entry.cpp`：`RobotApp_ControlTick(ts_us)`
   - 读取最新 `ImuState` 和 `MagState`
   - 读取最新 `MotorFeedback`
   - 调用 `InsEstimator.step(imu, mag, motor, ts_us)`
   - 得到 `domain::StateEstimate` 并发布到双缓冲
2. `RobotApp/Estimation/ins_estimator.*`
   - 姿态：陀螺积分 + 加速度重力方向校正 + 磁力计航向校正（如果磁力计数据新鲜）
   - 速度/位置：对去重力后的加速度积分得到速度，再积分得到位置（没有外部定位会漂移）
   - 轮速校正：用两轮 3508 的 rpm（按减速比换算到轮速）来校正“前向速度”

#### 2.2.5 逻辑（LogicBus 输出）
1. `RobotApp/Tasks/logic_task.cpp`（100Hz）
   - 每 10ms 调 `RobotApp_LogicTick(now_us)`
2. `RobotApp/Bridge/app_entry.cpp`：`RobotApp_LogicTick(ts_us)`
   - 读取最新 `SensorsSnapshot`、`SystemHealth`
   - 调 `robotapp::logic::logic_tick(...)`
3. `RobotApp/Logic/logic_step.hpp`
   - 当前逻辑很简单：
     - 如果遥控超时/失联 → `hl_valid=false`
     - 如果 operator 没 enable 或 e-stop → `hl_valid=false`
     - 否则把遥控通道映射到 `hl_cmd.vx`、`hl_cmd.wz`（范围 -1..1）
   - 结果发布到 `LogicBus`

#### 2.2.6 控制（Controller 输出电机电流）
控制环在 `RobotApp/Tasks/control_loop.cpp`（1kHz）里：
1. 从 `LogicBus` 读最新命令：
   - 如果 `cmd_valid=true`：直接用 `cmd`（这是“你以后写完整控制算法时可以直接输出执行器命令”的通道）
   - 否则如果 `hl_valid=true`：进入“控制器”计算，得到 `cmd`
   - 否则：输出全 0（安全）
2. 安全策略 `SafetyPolicy` 会根据遥控/传感器/CAN 健康决定是否强制停机或只停某些总线
3. 生成最终 `ChassisCommand` 后，调用电机驱动协议发到 CAN

控制器默认实现（为了先跑通链路）：
- 在 `RobotApp/Algo/chassis_controller.hpp` 的 `chassis_controller_step`
- 逻辑：
  - 将 `hl_cmd.vx/wz` 转成目标线速度/角速度（单位 m/s、rad/s）
  - 用两轮 rpm 转成轮线速度（考虑减速比/轮半径/正方向）
  - 做速度误差 P 控制，输出 `WheelCommand.current`（mA）
  - 如果轮速不可用（超时/没反馈），回退到开环电流映射

---

## 3. 关键数据结构（你必须知道它们里有什么、单位是什么）

这些结构体在 `RobotApp/Domain/types.hpp`。

### 3.1 SensorsSnapshot（传感器快照）
`domain::SensorsSnapshot` 字段含义：
- `imu`：`domain::ImuState`
  - `accel_mps2[3]`：加速度，单位 `m/s^2`
  - `gyro_dps[3]`：角速度，单位 `deg/s`
  - `ts_us`：时间戳，单位 `us`
- `mag`：`domain::MagState`
  - `mag_uT[3]`：磁场，单位 `uT`
  - `ts_us`：时间戳
- `motor`：`domain::MotorFeedback`
  - `wheels[2]`：两轮 DJI3508 的反馈（rpm/current/temperature）
  - `joints[4]`：4 个 DM4310 的反馈（pos/vel/torque）
  - `ts_us`：这份 motor feedback 的时间戳（由 `control_loop` 写入）
- `ts_us`：快照聚合时间戳（由 `RobotApp_ControlTick` 写入）

### 3.2 StateEstimate（估计输出）
`domain::StateEstimate` 字段含义：
- `q_wxyz[4]`：世界坐标系相对机体坐标系的四元数（w,x,y,z）
- `rpy_rad[3]`：roll/pitch/yaw，单位 `rad`（从四元数计算出来，方便调试）
- `vel_mps[3]`：速度（世界系），单位 `m/s`
- `pos_m[3]`：位置（世界系），单位 `m`
- `gyro_bias_rps[3]`：陀螺零偏估计，单位 `rad/s`
- `accel_bias_mps2[3]`：加速度零偏估计，单位 `m/s^2`
- `ts_us`：时间戳
- `valid`：0/1，表示这次估计是否有效（例如 dt 异常/传感器超时会变 0）

### 3.3 LogicBus 的输出
`RobotApp/Logic/logic_bus.hpp` 的快照包含：
- `cmd` + `cmd_valid`：执行器级命令（`domain::ChassisCommand`）
- `hl_cmd` + `hl_valid`：高层命令（`domain::HighLevelCommand`）

`domain::HighLevelCommand` 当前使用字段：
- `vx`：[-1,1] 归一化前向速度指令
- `wz`：[-1,1] 归一化角速度指令

### 3.4 控制器输入/输出接口
接口定义在 `RobotApp/Algo/chassis_controller.hpp`：
- 输入：`robotapp::algo::ChassisControllerInput`
  - `sensors`：指向 `SensorsSnapshot`
  - `estimate`：指向 `StateEstimate`
  - `wheel_odom_cfg`：指向 `WheelOdomConfig`（轮子几何/减速比/方向）
  - `wheel_ctrl_cfg`：指向 `WheelControllerConfig`（最大速度、Kp、电流限幅）
  - `operator_state`：使能/急停状态
  - `hl_cmd`：高层命令
- 输出：`robotapp::algo::ChassisControllerOutput`
  - `cmd`：`domain::ChassisCommand`（两轮电流 + 关节命令）
  - `cmd_valid`：是否有效

---

## 4. 你应该在哪写控制算法（非常明确）

### 4.1 推荐方式：改 `RobotApp/Algo/user_controller.cpp`
文件：`RobotApp/Algo/user_controller.cpp`

现在内容是：
- `robotapp::algo::user_controller_step(...)` 默认直接调用内置 `chassis_controller_step(...)`
- `robotapp::ChassisControllerOverride()` 返回 `&user_controller_step`，RobotApp 启动时会自动安装它

你接下来要做的是：
1. 打开 `RobotApp/Algo/user_controller.cpp`
2. 在 `user_controller_step(const ChassisControllerInput& in)` 内写你的算法
3. 你的算法输出 `ChassisControllerOutput`：
   - `out.cmd.wheels[0/1].current` 写两轮电流（mA）
   - 需要控制关节就写 `out.cmd.joints[i]`（mode/pos/vel/kp/kd/tau）
   - 设置 `out.cmd_valid = true`

### 4.2 另外一种方式：运行时安装控制器函数指针
接口在 `RobotApp/Bridge/controller_hook.hpp`：
- `robotapp::InstallChassisController(algo::ChassisControllerFn fn)`

你可以在初始化阶段调用它替换控制器。

---

## 5. 参数与在线调参（你量完参数以后怎么写进去）

### 5.1 运行期配置结构体
这些结构体在 `RobotApp/Domain/types.hpp`：
- `WheelOdomConfig`：轮半径/轮距/减速比/左右轮方向
- `WheelControllerConfig`：最大速度、速度 Kp、电流限幅

### 5.2 上电加载与 Flash 持久化
后端实现文件：
- `Core/Inc/app_params.h`
- `Core/Src/app_params.c`

保存内容（当前版本 AppParamsPayloadV2）：
- `SafetyConfig`
- `ImuCalibration`
- `MagCalibration`
- `WheelOdomConfig`
- `WheelControllerConfig`

**实际逻辑**：
1. `RobotApp_Init()` 会调用 `App_Params_Init()`
2. 如果 Flash 里找到最新有效记录（magic/version/crc 正确）
   - 就加载并应用
3. 如果没有有效记录
   - 就使用默认值（结构体默认构造值）
4. 当你调用 `RobotApp_SetXXXConfig(...)`
   - 只是在 RAM 里改值，并向参数系统标记“dirty”
   - 参数系统不会立即写 Flash，而是等待 500ms 稳定
5. `Core/Src/freertos.c` 的 `StartDefaultTask` 会持续调用 `App_Params_Service()`
   - 它会在合适的时机写入 Flash
   - 写失败会按退避策略延迟下次写（1s/10s/60s/300s）

### 5.3 在线调参入口（CAN）
实现位置：`RobotApp/Tasks/control_loop.cpp`

规则：
- 只有在 `OperatorState.enabled == false`（未使能）时才允许写参数（防止运动中改参数）

命令：
- **写轮参数**：`StdId=0x7A0`，8 字节
  - `data[0]=0x10`
  - `data[1..2]=wheel_radius_mm`（uint16，小端）
  - `data[3..4]=wheel_track_mm`
  - `data[5..6]=gear_ratio_x100`（例如 19.2 → 1920）
  - `data[7]`：bit0=左轮反向，bit1=右轮反向
  - 写入成功会回 `StdId=0x7A2` 返回同样的 8 字节（用于确认）
- **读轮参数**：`StdId=0x7A1`
  - `data[0]=0x11`
  - 设备回 `StdId=0x7A2` 返回当前值

如果你后续需要“在线调 WheelControllerConfig（最大速度/Kp/限流）”，可以按同样方式再加一组 ID。

---

## 6. 诊断与 LiveWatch（你如何确认系统真的在跑）

### 6.1 最推荐看的快照变量
在 `RobotApp/Bridge/app_bridge.h` 暴露：
- `g_robotapp_diag`：`DiagnosticsSnapshot`（包含绝大多数你想看的东西）
- `g_robotapp_sensors`：最新 `SensorsSnapshot`
- `g_robotapp_estimate`：最新 `StateEstimate`
- `g_robotapp_safety_flags`：安全标志位（bitfield）

### 6.2 调度统计
在 `Core/Src/freertos.c`：
- `g_control_last_ts_us`
- `g_control_dt_us`
- `g_control_jitter_us`

### 6.3 堆分配保护计数器
在 `RobotApp/Platform/heap_guard.hpp` / `RobotApp/Bridge/app_bridge.h`：
- `g_robotapp_heap_started`
- `g_robotapp_heap_malloc_calls` / `g_robotapp_heap_free_calls` / ...
- `g_robotapp_heap_calls_after_start`
- `g_robotapp_heap_alloc_fail`

如果 `g_robotapp_heap_calls_after_start` 变大并且程序 BKPT：
- 表示调度开始后发生了堆分配

---

## 7. CubeMX 重生成（避免你过几天点一下生成就出问题）

### 7.1 为什么会被覆盖
CubeMX 会重写一些文件，例如 linker script（`.ld`）或某些 `Core/Src/*.c`。

### 7.2 本工程如何规避 linker script 覆盖
CMake 构建时固定使用：
- `linker/STM32F407XX_FLASH_app.ld`

因此：
- CubeMX 覆盖根目录 `STM32F407XX_FLASH.ld` 不影响 CMake 构建

如果你用 CubeIDE 自己的工程按钮编译：
- 请确认 CubeIDE 工程里使用的 linker script 与 `linker/STM32F407XX_FLASH_app.ld` 一致

---

## 8. 下一步你最应该做什么（按顺序）

### 8.1 先量轮子参数
你需要提供四个信息：
1. 轮子直径（或半径）
2. 两轮中心距（wheel track）
3. 3508 电机到轮子的真实减速比（例如 19.2:1、36:1 等）
4. 左右轮 rpm 的正方向是否相反（如果相反就需要把 `rpm_sign_left/right` 设置为 -1）

### 8.2 把参数写进去（两种方式）
方式 A（推荐，在线写入并保存）：
- 通过 CAN 发 0x7A0 命令写入
- 观察回包 0x7A2 确认
- 观察 `g_app_params_save_ok` 增加确认已写入 Flash

方式 B（直接在代码里改默认值，不推荐长期用）：
- 改 `RobotApp/Domain/types.hpp` 里 `WheelOdomConfig/WheelControllerConfig` 的默认值
- 重新编译下载

### 8.3 写你自己的控制算法
打开 `RobotApp/Algo/user_controller.cpp`，把 `user_controller_step` 替换成你的算法。

---

## 9. 常见问题（按真实逻辑解释，不用比喻）

### 9.1 电机不动
检查顺序：
1. `OperatorState.enabled` 是否为 true（`g_robotapp_operator_state.enabled`）
2. `OperatorState.e_stop` 是否为 false
3. `LogicBus` 是否发布了 `hl_valid=true` 或 `cmd_valid=true`
4. `SafetyPolicy` 是否触发停机（看 `g_robotapp_safety_flags`）
5. CAN 是否在收发（看 `g_robotapp_can1_rx_count`、`g_robotapp_can1_tx_ok`）

### 9.2 一运行就断点/死循环
最常见原因：调度开始后发生了堆分配
1. 看 `g_robotapp_heap_calls_after_start` 是否增加
2. 如果增加，说明某段代码调用了 malloc/new
3. 解决方法：把堆分配移到初始化阶段，或者改成静态/预分配

---

## 10. 控制算法接口位置总结（你只记这一段也行）
- 接口定义：`RobotApp/Algo/chassis_controller.hpp`
- 默认实现：`RobotApp/Algo/chassis_controller.hpp::chassis_controller_step`
- 你的实现入口：`RobotApp/Algo/user_controller.cpp::user_controller_step`
- 安装方式：
  - 启动自动安装：`RobotApp/Algo/user_controller.cpp` 里实现 `robotapp::ChassisControllerOverride()`
  - 运行时安装：`RobotApp/Bridge/controller_hook.hpp::InstallChassisController`
