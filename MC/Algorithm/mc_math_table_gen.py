#!/usr/bin/env python3
"""生成 256 点整周期正弦表 C 数组文本（mc_math 查表法用）"""
import math

N = 256
vals = [math.sin(2.0 * math.pi * i / N) for i in range(N)]
lines = []
for i in range(0, N, 8):
    lines.append("    " + ", ".join(f"{v:.7f}f" for v in vals[i:i+8]) + ",")
print("\n".join(lines))
