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
