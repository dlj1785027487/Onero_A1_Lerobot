#!/bin/bash
# Onero A1 数据集可视化脚本
# 用法: bash scripts/viz.sh [选项]

set -Eeuo pipefail

usage() {
    echo "用法: bash scripts/viz.sh [选项]"
    echo ""
    echo "必填参数:"
    echo "  --repo-id REPO_ID       数据集 ID，如 my_name/a1_test"
    echo ""
    echo "可选参数:"
    echo "  --episode N              可视化的 episode 序号（默认 0）"
    echo "  --root PATH               缓存根目录，包含 <repo-id> 子目录（默认 ~/lerobot_datasets）"
    echo "  --mode MODE               显示模式: local | distant（默认 local）"
    echo "  --save 0|1                保存为 rrd 文件（默认 0，直接显示）"
    echo "  --output-dir PATH         保存文件的输出目录"
    echo "  --grpc-port PORT          远程模式 GRPC 端口（默认 9876）"
    echo ""
    echo "示例:"
    echo "  # 本地查看 episode 0 的所有帧"
    echo "  bash scripts/viz.sh --repo-id my/test --episode 0"
    echo ""
    echo "  # 保存为 rrd 文件"
    echo "  bash scripts/viz.sh --repo-id my/test --episode 0 --save 1 --output-dir ./output"
    echo ""
    echo "  # 远程流式查看"
    echo "  bash scripts/viz.sh --repo-id my/test --episode 0 --mode distant --grpc-port 9876"
    echo ""
    exit 0
}

# --- 默认值 ---
REPO_ID=""
EPISODE=0
ROOT=""
MODE="local"
SAVE=0
OUTPUT_DIR=""
GRPC_PORT=9876
DATASET_ROOT="$HOME/lerobot_datasets"

# --- 解析参数 ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo-id)     REPO_ID="$2"; shift 2 ;;
        --episode)     EPISODE="$2"; shift 2 ;;
        --root)        ROOT="$2"; shift 2 ;;
        --mode)        MODE="$2"; shift 2 ;;
        --save)        SAVE="$2"; shift 2 ;;
        --output-dir)  OUTPUT_DIR="$2"; shift 2 ;;
        --grpc-port)   GRPC_PORT="$2"; shift 2 ;;
        -h|--help)     usage ;;
        *) echo "未知参数: $1"; usage ;;
    esac
done

# --- 检查必填参数 ---
if [ -z "$REPO_ID" ]; then
    echo "[错误] --repo-id 为必填参数"
    usage
fi

# 数据集根目录
ACTUAL_ROOT="${ROOT:-$DATASET_ROOT}"

echo "数据集: $REPO_ID"
echo "Episode: $EPISODE"
echo "显示模式: $MODE"
echo "存储路径: $ACTUAL_ROOT"
echo ""

# --- 构建并执行命令 ---
CMD=(lerobot-dataset-viz --repo-id "$REPO_ID" --episode-index "$EPISODE" --mode "$MODE" --save "$SAVE")
if [[ -n "$OUTPUT_DIR" ]]; then
    CMD+=(--output-dir "$OUTPUT_DIR")
fi
if [[ "$MODE" == distant ]]; then
    CMD+=(--grpc-port "$GRPC_PORT")
fi

printf '执行:'
printf ' %q' "${CMD[@]}"
printf '\n'
echo ""

export HF_LEROBOT_HOME="$ACTUAL_ROOT"
exec "${CMD[@]}"
