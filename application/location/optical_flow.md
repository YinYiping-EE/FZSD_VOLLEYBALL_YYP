# 光流模块使用

根据Jeffrey的要求添加md注释文件，只保留最简单的说明

这个模块只需要关心两个值：

- `position_x`：X 方向累计位移，单位 `m`
- `position_y`：Y 方向累计位移，单位 `m`

## 1. 包含头文件

```c
#include "optical_flow.h"
```

## 2. 定义光流实例

```c
static OpticalFlowInstance *flow;
```

## 3. 初始化

把下面代码放到初始化函数里。

`usart_handle` 按实际接线改，比如接在 `USART6` 就填 `&huart6`。

```c
OpticalFlow_Init_Config_s flow_conf = {
    .usart_handle = &huart6,
    .protocol = OPTICAL_FLOW_UPIXELS,
    .flow_scale = OPTICAL_FLOW_DEFAULT_SCALE,
    .swap_xy = 0,
    .x_direction = 1,
    .y_direction = 1,
    .min_valid_threshold = 50,
};

flow = OpticalFlowInit(&flow_conf);
```

## 4. 读取 X/Y

在循环或任务里这样读：

```c
const OpticalFlow_Data_s *data = OpticalFlowGetData(flow);

if (data->updated)
{
    float x = data->position_x; // 单位: m
    float y = data->position_y; // 单位: m

    OpticalFlowClearUpdated(flow);
}
```

如果你想用 `mm`：

```c
float x_mm = data->position_x * 1000.0f;
float y_mm = data->position_y * 1000.0f;
```

## 5. 清零

需要重新从当前位置开始计位移时调用：

```c
OpticalFlowResetPosition(flow);
```

## 6. 方向不对时怎么改

只改初始化参数，不用改读取代码。

```c
.swap_xy = 1,      // X/Y 对调
.x_direction = -1, // X 方向取反
.y_direction = -1, // Y 方向取反
```

常见情况：

- X/Y 反了：改 `.swap_xy`
- X 正方向反了：改 `.x_direction`
- Y 正方向反了：改 `.y_direction`
