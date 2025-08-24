#!/bin/bash

# TWIP服务器控制脚本

case "$1" in
    start)
        echo "启动TWIP网页服务器..."
        cd /Users/youpeixing/ws/TWIP_teleop
        source .venv/bin/activate
        python twip_web_server.py &
        echo $! > /tmp/twip_server.pid
        echo "服务器已在后台启动，PID: $(cat /tmp/twip_server.pid)"
        echo "访问地址: http://localhost:8000"
        echo "使用 './server.sh stop' 停止服务器"
        ;;
    stop)
        if [ -f /tmp/twip_server.pid ]; then
            PID=$(cat /tmp/twip_server.pid)
            echo "停止TWIP服务器 (PID: $PID)..."
            kill -TERM $PID
            rm /tmp/twip_server.pid
            echo "服务器已停止"
        else
            echo "未找到服务器PID文件"
        fi
        ;;
    *)
        echo "用法: $0 {start|stop}"
        ;;
esac