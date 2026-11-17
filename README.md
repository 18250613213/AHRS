# AHRS

## V0.1

Windows AHRS 串口数据解析和三维姿态显示工具，支持 T02 `$BDFPDL` 和 EB90 二进制协议。

默认选择 COM31、460800 波特率、T02。启动后点击“打开串口”。协议选择在连接或保存期间锁定。

“开始保存”会持续写入本地 CSV。“停止保存”会后台导出 Excel，也可用“导出 Excel”重试。关闭窗口会保留 CSV。发布版文件位于桌面，源码运行时位于项目目录；界面显示具体路径。Excel 导出时会等待完成后再关闭。

两秒无有效数据会显示超时。断连后可重新打开串口。三维姿态默认直接跟随报文，可手动开启平滑。状态字以原始数值显示。

T02 烧录需要另行提供设备编程协议。文件发送完成表示字节写入串口，设备接收结果尚无应答校验。

### 开发和构建

已验证 Windows / Python 3.14.3。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py
.\.venv\Scripts\python.exe -m unittest discover -v
.\build.bat
```

构建结果位于 `dist/AHRS_T02/AHRS_T02.exe`，运行时保留整个目录。

### 验证范围

17 项自动化测试覆盖解析、显示、录制、超时、写入失败和文件传输失败。COM31 连续 10 秒实测收到 2001 有效帧、零解析错误，保存区间 CSV 与 Excel 均为 2000 条。尚需小时级采集及设备实际转动方向验证。


### RS232 混合数据

飞机姿态右侧为独立 RS232 显示栏，默认 COM15、UART1。可选择 UART0、UART1 或 UART2，点击“打开 RS232”连接并进入选定通道；连接后可用“进入 UART”切换通道。进入流程先在 460800 发送 `exit\r` 恢复命令通道，再发送 `debug\r`，确认 `Debug mode` 应答后发送对应的 `uart N\r` 并切到 230400 接收。

“退出 UART”在 460800 发送 `exit\r`，保持串口连接供后续进入操作；“关闭 RS232”释放串口。界面会区分命令已发送和已收到数据。UART0、UART2 在当前设备实测无输出，仍可退出和切换。

定位摘要显示 NAV-PVT 的经纬度、海拔、卫星数、定位类型和有效标志。下拉列表可选择 UBX 或 NMEA 报文，查看解析字段、最近一帧原始数据及接收数量。字段使用 pyubx2 / pynmeagps 定义的名称和单位，NAV-PVT 摘要中的海拔已换算为米。关闭 RS232 会释放端口，设备的输出模式继续保持，重新打开时重新进入选定 UART。

COM31 的 T02 数据与 COM15 的 RS232 数据独立显示。RS232 当前提供实时查看，现有 CSV/Excel 按钮对应 T02/EB90 主数据流。
