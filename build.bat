@echo off
chcp 65001 >nul
echo ========================================
echo AHRS 数据解析器 - 打包发布脚本
echo ========================================
echo.

cd /d "%~dp0"

echo [1/3] 选择 Python 环境...
set "AHRS_PYTHON=python"
if exist ".venv\Scripts\python.exe" set "AHRS_PYTHON=%~dp0.venv\Scripts\python.exe"

echo.
echo [2/3] 开始打包（这可能需要几分钟）...
"%AHRS_PYTHON%" -m PyInstaller AHRS_T02.spec --workpath build\T02 --noconfirm

if %errorlevel% neq 0 (
    echo.
    echo [错误] 打包失败！
    pause
    exit /b %errorlevel%
)

echo.
echo [3/3] 检查资源文件...
if not exist "dist\AHRS_T02\_internal\3D model" (
    echo [警告] 3D model 文件夹未正确包含，请检查打包配置
)

echo.
echo ========================================
echo 打包完成！
echo 输出程序: dist\AHRS_T02\AHRS_T02.exe
echo ========================================
echo.
pause
