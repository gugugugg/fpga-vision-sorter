# 厂商依赖导入

工程使用 HX1P35A 官方 lab1 的相机、DDR、HDMI BSP/IP，以及 SC520CS 相机配套寄存器表。
这些文件与项目编写的图像算法分开管理，不将其重新授权为 MIT。仓库中的 `vendor_files.json` 逐项列出目标位置和官方资料来源位置。

## 已有完整原生工程

```powershell
python tools/import_vendor.py --reference-project E:/fpga30rawsw
```

该目录应包含 `inputs/` 与 `vendor_ip/`，版本对应本次编译工程。

## 官方资料目录

将开发板官方参考工程及 SC520CS 参考包解压后，指定这些目录：

```powershell
python tools/import_vendor.py --vendor-root D:/HX1P35A-reference D:/SC520CS-reference
```

导入器优先按清单中的官方相对路径匹配；解压目录层级变化时按文件名检索。
同名候选内容一致时取其中一份，存在不同内容的候选时输出位置，改用更精确的资料目录或完整原生工程目录导入。
官方参考资料需覆盖 `lab_hd_1_mipi_hdmi/user_source` 下的 RTL、IP 与相机初始化文件；SC520CS 表使用 `uics520reg_720p60.v`。

`fpga/contest_board_system.al` 中保留相同的 IP 配置引用；`vendor_ip` 和导入的 BSP 文件在 `.gitignore` 中排除。
