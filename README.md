# 视觉增强与工件智能分拣系统

基于 **安路 PH1P35MDG324 FPGA / HX1P35A** 的实时图像处理与机械臂分拣工程。
图像采集、视觉处理、模板识别、帧差跟踪、HDMI 交互和分拣任务控制运行在 FPGA 上；电脑用于构建、串口诊断和离线算法对照。

本仓库 main 分支发布 **0x3920，已写入板上 Flash 并通过断电重启确认的正式版本**。源码来自 2026-10-30 通过完整布局布线与时序检查的工程快照；发布时仅将 ROM 文件与工具安装路径改为可迁移的相对路径或运行参数。
RAW 显示现有校准画面，不使用 SW2 切换。此前的 SW2 候选版仅保留在历史标签 [`v0.1.0-raw-switch-3922`](https://github.com/gugugugg/fpga-vision-sorter/tree/v0.1.0-raw-switch-3922) 中；正式版本标签为 `v0.1.1-stable-3920`。

## 功能

| 模式 | 功能 |
|---|---|
| RAW | 现有黑电平、颜色校准与白平衡链路的相机画面 |
| AE / AWB | 自动曝光与白平衡观察 |
| CNTR / GAUS | 对比度调整、Gaussian 滤波 |
| SOBL / BIN | Sobel 边缘、固定阈值二值化 |
| EROD / DILA | 灰度腐蚀、膨胀 |
| PCLR / SAT | 亮度伪彩、饱和度调整 |
| EVM | 局部微运动放大，512×256 显示区域、128×64 历史网格 |
| SORT | 长方体/楔形模板分类、帧差运动区域与机械臂定点分拣 |
| CEL | 平滑、色阶量化与黑色轮廓叠加的赛璐璐风格化 |

菜单共 **14 项**，模式位图 `0x1f0db7`。MEDN、HEQ、ADPT、浮世绘、水墨、EVA、三维重建和 CNN 未作为本版菜单功能启用；网口和 TF 卡动图也未启用。
部分通用源码仍保留可配置分支，实际启用项由顶层参数、模式位图及布局后的实例决定。

## 系统结构

```text
SC520CS / MIPI RAW10
        │
  解包、裁剪、黑电平与空间颜色校准
        │
  解拜耳、曝光与白平衡控制
        │
  DDR 帧缓存与源帧元数据
        │
  图像算法 / CEL / EVM ────────────── HDMI 菜单、图像与状态
        │
  ROI 背景分割 → 32×32 二值模板 → 稳定类别
        │              帧差 → 运动区域框
        ▼
  分拣状态机 → UART → 电平转换 → ZLink → 机械臂
        ▲                                  │
        └──────────── 位置回读 ─────────────┘

SW1 + K1/K2/K3 → 背景/模板采集 → 独立用户 SPI Flash
```

识别对有效前景归一化后，与两类实拍模板逐位比较，选择差异更小的一类；匹配分数和分差拒绝门槛已关闭。
空台无目标、轮廓触边等前景有效性处理仍保留。模板采集依赖固定相机、工位与照明。

机械臂按已标定点位执行：回展示台、张爪下降、闭合、抬升、分类转运、下降、松爪、返回右侧上方待机。
闭合时使用夹爪位置回读与目标差值形成接触候选；它属于位置反馈判据，不是力传感器测量。
长方体向左放置，楔形向右放置；识别后和放下后各提示一次，并等待 1 秒进入下一段动作。

## 硬件与接线

| 项目 | 配置 |
|---|---|
| FPGA | PH1P35MDG324，速度等级 -3 |
| 开发板 | HX1P35A |
| 摄像头 | SC520CS，MIPI RAW10 |
| 图像输出 | 1024×600 HDMI；52 MHz 像素时钟，1344×635 总时序，名义约 60.93 Hz |
| 控制通信 | 115200 bit/s，8N1 UART |
| 模板 | 两类 32×32 二值轮廓，每类 128 字节 |
| 背景 | 128×75 RGB565，共 19200 字节 |
| 标定存储 | 独立用户 SPI Flash，背景与两类模板分别保存并上电恢复 |

HDMI 刷新率与相机新源帧率分别统计；上述刷新率由时钟与总时序计算，并非新的相机帧率实测。

| 连接 | 方向 |
|---|---|
| J2 第 2 针 D15 | FPGA TX → 电平转换 → ZLink RX |
| J2 第 1 针 C18 | FPGA RX ← 电平转换 ← ZLink TX |
| 电平转换 LV | FPGA 3.3V |
| 电平转换 HV | ZLink 逻辑 5V |
| GND | FPGA、转换模块、ZLink 共地 |
| ZLink 跳帽 | TTL ↔ ZX |

开发板与机械臂各自供电。**不要将 FPGA 的 5V 输出与已有独立电源的 ZLink 5V 并接。**
机械臂动作点位对应本项目的实物装配；更换机械结构或工位后应先调整点位。

## 板上操作

| 控件 | 操作 |
|---|---|
| SW4 ON + K3 | 复位 |
| SW4 OFF + K3 | 打开菜单、确认选择 |
| 菜单中的 K1 / K2 | 上 / 下选择 |
| K4 | 退出菜单；机械臂控制停止入口 |
| SW1 ON | 进入采集模式，屏幕下方显示按键说明 |
| 采集模式 K1 / K2 / K3 | 更新空台背景 / 长方体模板 / 楔形模板 |
| 采集模式 K4 | 取消采集 |
| SORT、菜单关闭、SW1 OFF 时 K2 | 启动一次分拣 |
| SW5 ON | 板载蜂鸣器硬件使能 |

首次布置工位时，先清空展示台采背景，再分别放入两类工件采模板。
等待屏幕显示采集、写入完成后再更换工件或断电。三项数据均支持断电保存，可按需单独更新。
上电保持待机，不会自行开始分拣。

## 布局布线结果

工具：Tang Dynasty **6.2.178840**，placement seed **2**，保留原时钟与 DDR 时序约束。

| 资源 | 使用量 |
|---|---:|
| Slice | 21023 / 21216，99.09% |
| ERAM | 100 / 108 |
| DSP | 18 / 40 |
| IO | 86 / 160 |

| 时序 | 最差余量 | 总负余量 |
|---|---:|---:|
| Setup | +0.070 ns | 0 ns |
| Hold | +0.020 ns | 0 ns |

DDR 校准路径两个方向分别检查 115 和 43 个端点，建立与保持时间均无违例。
原生结果位于 [`results/0x3920`](results/0x3920)，包含资源报告、时序摘要、DDR 双向报告、模板匹配仿真结果及 Flash 冷启动记录。

## 目录

```text
fpga/                 0x3920 原生 TD 工程、项目 RTL、ROM 数据、引脚与时序约束
dependencies/         厂商 BSP/IP 导入清单
tools/                导入依赖和构建脚本
host/                 UART 版本、识别、存储与机械臂诊断工具
results/0x3920/        实际编译结果
docs/                 接线操作说明和可打印演示图卡
```

## 构建

环境：Windows、Python 3.10+、Tang Dynasty 6.2.178840。原生 FPGA 工程依赖开发板配套 BSP 与安路 IP。
仓库保存项目源码和工程配置，厂商依赖按 [`dependencies/vendor_files.json`](dependencies/vendor_files.json) 导入，见 [第三方依赖说明](dependencies/README.md)。

如果已有本版完整原生工程目录：

```powershell
python tools/import_vendor.py --reference-project E:/fpga30nearest
python tools/build.py --td-root D:/anlu --prepare-only
python tools/build.py --td-root D:/anlu
```

从官方参考资料恢复依赖的方式见依赖说明。`build/` 是构建输出目录。
也可以用 TD 打开 `fpga/contest_board_system.al` 查看工程；重现本次 seed 和 route 设置请使用上述构建脚本。
脚本只编译，不烧录、不向机械臂发送指令。

### 串口诊断

```powershell
python -m pip install -r host/requirements.txt
python host/zlink_board_console.py hello --port COM5
python host/template_board_console.py status --port COM5 --output build/template-status.json
```

将 COM5 换成实际板载串口。`hello` 预期读到 `0x3920`，识别与存储可分别用 `status`、`storage-status` 查看。
[`docs/demo-cards.pdf`](docs/demo-cards.pdf) 提供 A4 打印图卡，25 mm / 40 mm 两种图芯，以及微运动与跟踪附件。

## 开源协议

本项目编写的 RTL、Python 工具及文档按 [MIT License](LICENSE) 开源。
未随仓库发布的安路 IP、开发板 BSP 按其原授权获取与使用；厂商标识及第三方图像素材不因本项目采用 MIT 而改变权属，详见 [THIRD_PARTY.md](THIRD_PARTY.md)。
