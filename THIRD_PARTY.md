# 第三方依赖与标识

项目编写的代码按根目录 MIT License 发布。

## 开发板 BSP 与安路 IP

安路生成的 PLL、RAM、FIFO、DDR IP，以及开发板提供的相机、HDMI、视频基础模块，保留厂商原授权和署名。
144 个外部依赖文件未纳入 Git，目标路径与官方参考路径列于 `dependencies/vendor_files.json`。
导入工具仅从使用者持有的官方资料或完整原生工程中复制这些文件。

## 图形资源

`logo_indices.dath`、`logo_palette.hex`、`anlogic_logo_160x94.hex` 及对应输入目录数据用于板上展示安路标识。
安路名称与标识的权属不变，不作为 MIT 许可的项目原创图形。

本版未启用的第三方角色动图数据未随仓库发布。`mascot_overlay.v` 是项目编写的可选叠加模块；本版主工程不实例化该模块。
