#!/usr/bin/env bash
# 不安装包、不修改系统环境；使用项目 .venv，重建并执行 theory 下五本笔记本。
set -euo pipefail
THEORY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$THEORY_DIR/../.venv/bin/python" "$THEORY_DIR/refresh_notebooks.py" "$@"
