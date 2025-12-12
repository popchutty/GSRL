# H7 适配关键改动记录

-作者：张楚清
-版本：v1.0.0
-最新修改日期：2025/12/12

## 1. 硬件抽象层 - board_config.h
**文件**: `GSRL/Include/board_config.h`

- 添加条件编译，根据 `STM32H723xx` / `STM32F407xx` 宏切换头文件
- H7 包含 `fdcan.h`，F4 包含 `can.h`
- 添加 `spi.h` 引入
- CAN Handle 映射：
  - H7: `MOTOR_CAN_HANDLE` → `hfdcan3`
  - F4: `MOTOR_CAN_HANDLE` → `hcan1`
- SPI Handle 映射：
  - H7: `IMU_SPI_HANDLE` → `hspi3`
  - F4: `IMU_SPI_HANDLE` → `hspi1`
- UART Handle 映射：
  - `RC_UART_HANDLE` → `huart3`
  - `H30_UART_HANDLE` → `huart1`

---

## 2. CAN 驱动类型抽象 - drv_can.h
**文件**: `GSRL/Driver/inc/drv_can.h`

- 定义统一类型别名：
  ```c
  #if defined(STM32H723xx)
      typedef FDCAN_HandleTypeDef CAN_Handle_t;
      typedef FDCAN_TxHeaderTypeDef CAN_TxHeader_t;
      typedef FDCAN_RxHeaderTypeDef CAN_RxHeader_t;
  #elif defined(STM32F407xx)
      typedef CAN_HandleTypeDef CAN_Handle_t;
      typedef CAN_TxHeaderTypeDef CAN_TxHeader_t;
      typedef CAN_RxHeaderTypeDef CAN_RxHeader_t;
  #endif
  ```

---

## 3. 电机驱动适配 - dvc_motor.hpp
**文件**: `GSRL/Device/inc/dvc_motor.hpp`

- `m_motorControlHeader` 类型从 `CAN_TxHeaderTypeDef` 改为 `CAN_TxHeader_t`（抽象类型）

---

## 4. FreeRTOS 任务注册 - freertos.c (H7)
**文件**: `CubeMx_BSP_H7/Src/freertos.c`

- 添加 `test_task` 线程定义和创建：
  ```c
  osThreadId_t testHandle;
  const osThreadAttr_t test_attributes = {
    .name = "test",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  void test_task(void *argument);
  // 在 MX_FREERTOS_Init 中：
  testHandle = osThreadNew(test_task, NULL, &test_attributes);
  ```

---

## 构建验证
```bash
cmake --build build/H7-Debug
# 输出:
# [2/2] Linking CXX executable H7_Temp.elf
# FLASH: 120168 B / 1 MB (11.46%)
```

---

## 开发构想

### 方案一：单仓库多 BSP

**结构：**
```
Project/
├── CMakePresets.json          # 预设选择 H7-Debug / F4-Debug
├── CMakeLists.txt             # 根据 TARGET_PLATFORM 切换 BSP
├── GSRL/                      # 共用业务逻辑库（平台无关）
│   └── Include/board_config.h # 硬件抽象层
├── Task/                      # 共用任务代码
├── CubeMx_BSP_H7/             # H7 专用 CubeMX 生成代码
└── CubeMX_BSP_F4/             # F4 专用 CubeMX 生成代码
```

**优点：**
- 业务代码只维护一份，修改同步无延迟
- CMake 预设切换方便，一键选择目标平台
- 编译输出隔离（`build/H7-Debug` / `build/F4-Debug`）
- CubeMX 生成的 `.ioc` 文件独立，互不干扰

**缺点：**
- 仓库体积较大（包含两套 HAL 驱动）
- 需要维护 `board_config.h` 抽象层
- 团队成员需理解条件编译逻辑

---

### 方案二：双仓库 + 共用库 Submodule

**结构：**
```
# 仓库 1: H7_Project
H7_Project/
├── CubeMx_BSP/                # H7 CubeMX 代码
├── GSRL/                      # → git submodule (共用库)
└── Task/                      # → git submodule (共用任务)

# 仓库 2: F4_Project
F4_Project/
├── CubeMx_BSP/                # F4 CubeMX 代码
├── GSRL/                      # → git submodule (同一个共用库)
└── Task/                      # → git submodule (同一个共用任务)

# 仓库 3: GSRL (独立库)
# 仓库 4: Task (独立任务)
```

**优点：**
- 每个项目仓库更精简
- 更符合传统多项目管理习惯
- 共用代码通过 submodule 版本锁定，稳定性更好

**缺点：**
- 共用代码修改需要三步：改 GSRL → 提交 → 更新主项目 submodule
- 开发调试时 submodule 同步麻烦
- 多仓库权限管理复杂

---

### 方案对比

| 维度 | 方案一（单仓库） | 方案二（双仓库） |
|------|------------------|------------------|
| 代码同步 | 实时 | 需手动更新 submodule |
| 仓库体积 | 较大 | 较小（分散） |
| 切换平台 | CMake Preset 一键切换 | 切换项目目录 |
| CubeMX 冲突 | 无（独立 .ioc） | 无 |
| 维护复杂度 | 中等（抽象层） | 较高（submodule） |
| 适合团队规模 | 小团队 / 快速迭代 | 大团队 / 稳定版本 |

---

### 建议

由于项目处于**快速开发阶段**，个人建议使用 **方案一**，减少同步成本。

如果项目需要独立版本管理，建议 **方案二**。