#!/bin/bash

# 一键录制脚本
# 默认: 10秒，20帧 (2 FPS)，1080x720 分辨率

echo "=================================================="
echo "🎬 一键视频录制启动器"
echo "=================================================="

# 激活conda环境
if [ -f ~/anaconda3/etc/profile.d/conda.sh ]; then
    source ~/anaconda3/etc/profile.d/conda.sh
elif [ -f ~/miniconda3/etc/profile.d/conda.sh ]; then
    source ~/miniconda3/etc/profile.d/conda.sh
else
    echo "⚠️ 找不到conda，尝试直接运行..."
fi

# 尝试激活vision_gps_estimator环境
conda activate vision_gps_estimator 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️ 无法激活 vision_gps_estimator 环境，使用当前环境"
fi

# 设置脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 运行录制脚本
python3 "$SCRIPT_DIR/scripts/quick_record.py" "$@"








