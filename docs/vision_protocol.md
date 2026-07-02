# 上位机 → 下位机 通信协议 (planArray)

**帧长:** 22 字节 | **帧头:** `0xAA` | **帧尾:** `0x55` | **校验:** XOR (索引 1\~19)

---

## 帧结构

| 索引 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | header | uint8 | `0xAA` |
| 1 | cmd | uint8 | `0` = 坐标模式, `1` = 误差模式 |
| 2-3 | len | uint16 | 数据长度 (固定 `20`) |
| 4-7 | target_x | float32 | 目标 X (**模式相关**) |
| 8-11 | target_y | float32 | 目标 Y (**模式相关**) |
| 12-15 | target_yaw | float32 | 目标朝向 (度, **预留**) |
| 16-19 | target_time | float32 | 预测时间 (s, **预留**) |
| 20 | xor_val | uint8 | 索引 1\~19 逐字节异或 |
| 21 | tail | uint8 | `0x55` |

---

## 模式 0: 坐标导航 (`cmd = 0`)

| 字段 | 含义 | 单位 |
|---|---|---|
| target_x | 世界坐标系目标 X 坐标 | m |
| target_y | 世界坐标系目标 Y 坐标 | m |

### 下位机行为

1. 在线检查: 视觉/光流均在线 + target ≠ (0, 0)
2. 安全边界: `dist > 3.0m` → 不动
3. 到达判定: `dist < 0.1m` → 停
4. 速度规划:

```
speed = min(NAV_MAX_SPEED, dist × NAV_SPEED_GAIN)
vx = (err_x / dist) × speed
vy = (err_y / dist) × speed
```

---

## 模式 1: 误差跟踪 (`cmd = 1`)

| 字段 | 含义 | 单位 |
|---|---|---|
| target_x | 视觉 X 轴像素误差 | pixel |
| target_y | 视觉 Y 轴像素误差 | pixel |

### 下位机行为

1. 在线检查: 视觉/光流均在线 + target ≠ (0, 0)
2. PID 跟踪:

```
vx = PID(−target_x, 0)   // MaxOut 限幅
vy = PID(−target_y, 0)
```

3. 模式切换时: 自动清零 PID 积分 + 速度归零

---

## 模式切换

两种模式可在运行中随时切换。`cmd` 值变化时下位机自动：

- 切换到模式 1 时: 重新初始化 PID（清零积分累积）
- 速度清零一帧，下一帧开始正常控制
- 防止上一模式数据残留导致速度跳变

---

## 参数速查

| 参数 | 值 |
|---|---|
| NAV_MAX_SPEED | 14000 |
| NAV_SPEED_GAIN | 40000 |
| NAV_ARRIVAL_DIST | 0.10 m |
| PID X Kp / Ki / Kd | 0.5 / 0.01 / 0.0 |
| PID Y Kp / Ki / Kd | 0.5 / 0.01 / 0.0 |

---

## XOR 校验示例 (Python)

```python
def calc_xor(data: bytes) -> int:
    result = 0
    for b in data[1:20]:  # 索引 1~19
        result ^= b
    return result
```
