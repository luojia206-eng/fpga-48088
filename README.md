# 应急信息发布终端（基于安路 EG4S20）

嵌入式芯片与系统设计竞赛 · FPGA 选题一

- 平台：安路 EG4S20BG256（HX4S20C 核心板）
- 开发工具：Anlogic TD 6.2
- 显示：HDMI 720p60，音频：HDMI 内嵌 PCM

## 目录结构

| 目录 | 说明 |
| --- | --- |
| `project_code/board/` | 顶层模块、引脚约束、时序约束 |
| `project_code/rtl/` | 自主编写的 RTL 源码（按功能分类） |
| `project_code/dependencies/` | 平台配套 SDRAM 控制器、异步 FIFO、PLL（厂商 IP） |
| `project_code/vendor_hdmi/` | HDMI 发送核与 PHY（厂商加密 IP） |
| `project_code/project/` | 安路 TD 工程文件 `media_player_excellence.al` |

## 构建方法

用安路 TD 6.2 打开 `project_code/project/media_player_excellence.al`，
依次执行 **综合 → 布局布线 → 生成比特流**。

## 主要功能

- FAT32 文件系统扫描，从 SD 卡读取 BMP24 图片与 PCM 音频
- 图片轮播 + 淡入淡出转场，音量/亮度/周期参数可调并掉电保存（SPI Flash 日志）
- HDMI 输出：视频、音频、OSD 叠加
- 串口（115200 8N1）实时接收文字，滚动字幕显示；`!1`~`!8` 切换 8 条预置中文标语
