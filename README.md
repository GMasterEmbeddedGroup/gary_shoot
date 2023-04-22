# gary_shoot

gary_shoot控制发射模块的包，主要接受上层节点传来的发射指令，经过发射系统解算和针对热量的弹频弹速控制后对下层硬件节点进行控制，并带有切枪功能。

## dr16_forwarder

DR16数据解算，带有状态机切换，通过接收上层节点传来的DR16遥控器数据，分析其拨杆状态进行发射器状态切换，如启停摩擦轮、拨弹轮等，并带有安全控制。

本节点后期可被其他节点替代，通过键鼠、自动控制节点或其他方式发送相同数据。

`dr16_forwarder`发送的数据如下:  

- `/rc_speed_set` 弹速向下层转发的设定值。
- `/rc_freq_set` 弹频向下层转发的设定值。

`dr16_forwarder`可配置参数如下

- `bullet_speed_target` 弹速最大值。
- `bullet_freq_target`弹频最大值。

_*部分参数及topic未列出_

### Usage

使用生命周期节点的相关指令开启或关闭该节点。也可以在gary_bringup中直接配置或更改参数。

- 直接启动节点

```bash
ros2 run gary_common DR16_forwarder
```

- 作为component加载

```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager gary_shoot gary_shoot::DR16_forwarder
```

## heat_controller

`heat_controller` 是中间层节点，具有针对热量的弹频弹速控制功能，并决定是否切枪。

本节点通过接收上层的数据，并获取裁判系统相关数据，经过逻辑处理后向下层继续发出弹频、弹速数据，并控制枪管切换。

在实际热量低于某个阈值时，将使用设定的最大弹频来发射弹丸；在实际热量高过阈值时，将使用线性函数拟合剩余热量与弹频，使弹频随热量上升而下降；当剩余热量只允许一发弹丸发射时，将根据枪口冷却速度实时计算弹频，使枪口实际热量维持在限度之下。

本节点可被直接绕过，使弹频弹速不经过热量控制直接传递到下层，便于调试。

`heat_controller` 接收的数据如下:  

- `/rc_speed_set` 上层弹速设定值。
- `/rc_freq_set`上层弹频设定值。
- `/ShootData`裁判系统回传的相关数据

`heat_controller` 可配置参数如下

- `heat_limit` 热量控制阈值
- `barrel` 枪管数量

_*部分参数及topic未列出_

### Usage

***同上***

## shoot_controller

`shoot_controller` 是中间层节点，具有针对热量的弹频弹速控制功能，并进行切枪和单发控制。

本节点接受上层传来的弹频和弹速控制，并根据获取到的*param*计算应当设置的PID值，传递给下层的硬件控制器。

单发控制器采用了基于电机编码器位置的pid控制算法，该算法能够精准控制电机转动，确保每次电机转动只拨出一发弹丸。同时，其还采用枪口热量检测，通过对枪口热量变化的监测，能够实时判断每次发射是否只发出一发弹丸。

本控制器还带有卡弹处理功能，通过计算pid反馈值和设计值之差来实时计算卡弹状态，并且累计卡弹时间。当卡弹时间超过设定的阈值时，模块会自动触发拨弹轮倒转，使弹路回退。

本节点拥有断线重连的能力, 能够在任意发射相关电机掉线时停止其他电机的动作，确保安全。

`shoot_controller` 接收的数据如下:  

- `/speed_set` 上层弹速设定值。
- `/freq_set`上层弹频设定值。
- `/ShootData`裁判系统回传的相关数据
- `/diagnostic_aggs`硬件自检信息

`shoot_controller` 可配置参数为下层硬件控制器所接收的topic名。

_*部分参数及topic未列出_

### Usage

***同上***

