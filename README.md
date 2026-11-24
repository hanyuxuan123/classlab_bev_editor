# Classlab BEV 标注映射与修订工具

这是一个本地网页标注工具，可随仓库下载后独立运行。它读取一段 Classlab 自动标注后的数据，
把已验收的 3D 目标框映射为nuScenes 使用的类别，把 `road.bin` 中的线显示在
点云 BEV 和相机图像上，供人工指定 `divider` / `boundary`、修正线条并绘制
`ped_crossing`。默认把修订保存在**所打开片段目录的 `reviews/`**，原始采集文件
（如 `GT.csv`、`road.bin`、图像和点云）不会被覆盖。工具目录和数据片段目录可以放在不同位置。

## Stage2 五类轨迹标签

页面第二行工具栏可审查并修改 `TURN_RIGHT`、`TURN_LEFT`、`KEEP_STRAIGHT`、
`LANE_CHANGE_LEFT`、`LANE_CHANGE_RIGHT` 五类自车指令。青色虚线表示
`command_valid=true` 的未来 0.5–3.0 秒轨迹，橙色表示自动证据不足、需要人工确认。
自动建议会直接检测未来轨迹是否与当前左右 `divider` 相交：穿左线为左换道、穿右线
为右换道；不使用跨帧 line ID，也不使用 `boundary` 判断换道。
“保存本帧标签”写入 `reviews/maneuvers.json`；确认换道后使用“应用换道窗口”，
程序会反求自车中心到达 divider 中心线的时刻，并覆盖当前审核帧前 2.5 秒至跨线完成帧，
不会再固定向后多标 1 秒；“导出 2 Hz 五类标签”生成
`reviews/maneuver_labels_2.0hz.json`。此功能不会改写源数据或 SparseDrive PKL。算法与
导出字段详见下文“Stage2 未来轨迹和五类指令如何得到”。

## 当前 main 的主要更新

- **schema 9 semantic 道路边界**：保留由 `lane_type=110/112/114` 确认的实黄线
  boundary，以及通过严格几何校验的 `road_edge` boundary；删除“最外层 road.bin
  divider 向外偏移”的默认兜底，避免路口选错分支。手工或全局 boundary 优先。
- **按 track ID 删除全片段误框**：在 BEV 或相机图中选中 bbox 后，右侧可点
  “删除同 ID 全片段”，一次移除所有帧中相同 `track_id` 的框；操作立即生效、影响导出，
  且可在当前页面撤销。原始 `GT.csv` 不会被修改。
- **更顺畅的框与相机联动**：BEV 和相机框显示映射类别及 track ID；点击相机框会选中
  对应 BEV 框并在必要时移动视野，几何修改会刷新相机投影。
- **地图线完整编辑流程**：支持补画、加点、拖点、重画、删除、撤销/重做，并可把手画
  `divider`、`boundary` 和 `ped_crossing` 映射到相邻帧。
- **更稳定的审阅操作**：提供固定“前方 80 m”视野、绝对帧号跳转、快捷键保存和
  2 Hz 合并标签导出；所有全片段规则和局部修订均保存在 `reviews/`。

下面各节给出完整安装、操作和数据格式说明。

```text
<下载的仓库>/
└── classlab_bev_editor/       本工具
    ├── server.py              读取标注、提供 GUI/API、保存修订、导出标签
    ├── geometry.py            road.bin/PCD 解码、坐标变换、相机投影
    ├── maneuver_labels.py     Stage2 轨迹、五类建议和换道事件窗口
    ├── requirements.txt       Python 依赖
    ├── static/
    │   ├── index.html         界面结构
    │   ├── app.js             BEV、相机叠加和编辑交互
    │   └── style.css          界面样式
    └── README.md

<自动标注已完成的片段目录>/
├── car__2024-12-12.yaml       该片段的相机标定
├── result/
│   ├── GT.csv                 逐帧 3D 框
│   ├── road.bin               逐帧道路折线
│   ├── mapping/mapping_pose_quaterniond.txt  跨帧地图位姿
│   └── local_motion_pcdbin/   逐帧点云，例如 0.pcd
├── raw/
│   ├── sync_time_list.txt     相机与 LiDAR 时间同步表
│   └── image/<相机名>/         时间戳命名的 .jpeg 图像
└── reviews/                   首次启动时自动创建，保存修订与导出
```

## 1. 安装与启动

程序使用 Python 标准路径接口，启动命令分别适用于 Linux、macOS 和 Windows；目前已在 Linux 上实测，Windows/macOS 尚未做实机验证。建议使用 **Python 3.10 或更新版本**和现代浏览器。
Python 依赖为 Flask、PyYAML、NumPy；用仓库内的 `requirements.txt` 安装。片段目录应包含上表所列的自动标注结果。下面的 `<片段目录>` 是占位符，
**不是工具自带的数据路径**。例如`/media/classlab/2E0CECBF0CEC82E7/2025-07-17-17-38-26` 仅是实验室某台 Linux
机器上一个已跑完自动标注的片段位置；Windows 可用 `D:\datasets\片段名`，macOS 可用`/Volumes/数据盘/片段名`。把示例换成自己机器上的实际目录。

下载或克隆仓库后，在终端进入 `classlab_bev_editor`。Linux/macOS：

```bash
cd "/path/to/downloaded-repo/classlab_bev_editor"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python server.py "/实际路径/片段目录" --port 8765
```

Windows PowerShell：

```powershell
cd "C:\实际路径\下载的仓库\classlab_bev_editor"
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe server.py "D:\datasets\片段目录" --port 8765
```

在运行服务的同一台机器上打开 <http://127.0.0.1:8765>。服务只监听本机的`127.0.0.1`。首次启动会在**传给 `server.py` 的片段目录下**创建 `reviews/`；
打开不同片段时，各自的修订保存在各自的 `reviews/`。该目录必须可写。如果数据盘只读或想集中存储，增加 `--edits-dir "<可写的修订根目录>"`，实际输出会在 `<可写的修订根目录>/<片段目录名>/`，而非数据片段内。

## 2. 界面怎么操作

1. 用“上一帧/下一帧”或 **Shift+左/右方向键**逐帧查看。帧号框输入绝对帧号（例如`100`），按回车或点“跳转”直接到该帧。GUI 可看原始 10 Hz 帧；最终标签默认每五帧取一帧，即约 2 Hz。
2. 左侧 BEV 的灰点是该帧运动补偿点云；彩色线是道路折线，彩色矩形是 3D 框在地面的俯视轮廓。**红色三角是自车，中心位于车体坐标 `(0,0)`，尖端指向前方 `+x`**。它是编辑视图标识，不写入标注或导出。**框上直接显示映射后的类别与 track ID**。“前方 80 m”使用固定显示比例，切换帧时不再随最远 3D 框自动缩放；点击“适配全部目标”可把本帧所有框和线放进视野。滚轮缩放，右键拖动画布。
3. 右侧可切换七路相机。相机中的线和框由同一组 3D 标注投影得到；相机框也显示类别与 track ID。**点击相机框会选中对应 BEV 框**，若框在当前 BEV 视野之外，视野会移到它。
4. 选中框后，在“所选标注”里改类别、中心、长宽高及 yaw；或在 BEV 拖动框中心。相机投影会随修改刷新，保存前即可对照检查。漏框可切换“补画 3D 框”，点击中心后补全高度、尺寸、朝向和类别。
5. 选中线后，可改为 `divider`、`boundary`、`ped_crossing` 或 `ignore`；拖白色顶点修形状。“给选中线加点”会在最近的线段插点。新线可用“绘制车道线/道路边界”；至少点两点，双击或点“完成绘制”结束。若只是类别选错，在“所选标注”的下拉框改回正确类别；若这条线本身就是误标，点“删除所选”，再点“保存本帧”。
6. 人行横道选“绘制人行横道”，依次点击轮廓角点，至少三点，完成后自动闭合。点“所选静态线映射到相邻帧”会借助建图位姿将它存到全局坐标并自动投回邻近帧。**删除错误横道**：保持“选择/拖动”模式，在 BEV 中点击粉色线段或白色顶点，点“删除所选”。全片段横道会弹出确认，并删除全局记录及同位置重复记录；局部手绘横道删除后需点“保存本帧”。全局删除在确认后立即生效，其他未保存改动仍需保存。
7. 类别修正可只改当前帧，也可点“所选类别应用到同 ID 全片段”。目标按 `track_id` 匹配；源线按道路段 ID 加相对车道 ID 匹配。全片段规则会作用于已有单帧 JSON；某帧明确改过的类别保留本帧选择。改动后点“保存本帧”。切换未保存帧会有提示。
8. 若同一目标在整个片段中都是误标，选中 bbox 后可在右侧点“删除同 ID 全片段”。工具按 `track_id` 删除所有帧中的对应框，确认后立即生效且无需逐帧保存；当前页面可点“撤销”恢复。删除规则写入 `reviews/deleted_tracks.json`，不会改动原始 `GT.csv`。
9. “导出 2 Hz 标签”把修订和未修订的预标注合并为 `mapped_labels_2hz.json`。

“保存本帧”可用 **Ctrl+S**；输入框正在编辑时也会先提交输入值再保存。全片段类别规则、删除的目标 ID、人行横道和手画地图线分别保存在 `class_rules.json`、`deleted_tracks.json`、`crossings_global.json`、`map_lines_global.json`，后续帧读取及导出时自动应用，因而不需要为每个受影响的帧生成 `<帧号>.json`。`<帧号>.json` 只保存该帧的局部修订；全片段记录创建后已立即写盘，当前帧还有局部修改时仍要保存本帧。受到全片段记录影响的帧也显示“已修订”，导出中的 `reviewed` 同样为 `true`。

### 误删后撤销

在**同一帧**点顶部“撤销”，或按 `Ctrl+Z`，可恢复误删的框或线；“重做”或`Ctrl+Y` 可重新执行。撤销后要点“保存本帧”才能更新磁盘中的当前帧标注。按 track ID 删除的全片段目标、全片段横道和全局地图线也可在当前页面撤销，其全局记录会立即恢复。切换帧或刷新网页会清空当前页的撤销记录；因此发现误删时应先撤销，再翻帧。快捷键在数字输入框聚焦时保留给输入框本身，可直接点击顶部按钮。

### 画错线后重新选点

在 BEV 中选中画错的 `divider`、`boundary` 或 `ped_crossing`，按 **Ctrl+Shift+Z**（或点“重画所选线”）。原线会暂时隐藏；按正确顺序重新点击点位，双击或点“完成绘制”后替换原线。重画过程中按 **Ctrl+Z** 可撤销最后一个选点，再按 Ctrl+Shift+Z 会清空刚选的全部点，**Esc** 会取消重画并保留原线。完成后按 **Ctrl+S** 保存；若原线已经是全局静态线，保存也会更新相邻帧。若刚完成的线整体不需要，直接按 Ctrl+Z 撤销即可。快捷键需在画布上操作；数字输入框有焦点时，先点击画布或顶部按钮。

### 自动生成道路边界

默认 `--boundary-policy semantic` 只保留有语义或独立几何证据、可以进入训练的
`boundary`。它与 Stage2 换道判断相互独立，处理顺序如下：

1. 从 `road.bin` 读取折线及其 `line_id`、`lane_id`、`lane_type`。候选线至少覆盖
   8 m 纵向范围，并能在车前 `x=0/10/20 m` 中至少两个位置取得横向值。
2. `lane_type=110/112/114` 视为明确的道路边界语义。程序分别在左右侧选择离自车最近的
   合格候选，保留原始几何，不做横向偏移；该折线既可作为 divider 显示，也以
   `source=road.bin-semantic` 生成训练用 boundary。
3. 左右侧按车前 0–20 m 的局部几何确认：左线保持 `y>0.1 m`，右线保持
   `y<-0.1 m`。不要求整条弯道折线永远与 `y=0` 同侧，否则急弯中的正确双黄线会被误删。
4. 某侧没有 lane-type 语义线时，才检查 `road_edge.csv` 候选。只有拟合连续、近场侧别
   稳定、位于最外 divider 外侧，并且间距与相邻采样跳变均通过限制的线，才以
   `source=road_edge.csv-validated` 进入训练。
5. 手画或全局 boundary 优先；自动线靠近人工线的重叠部分会被裁掉，避免重复 GT。

只根据“最外层 road.bin 线”向外偏移产生的 `road.bin-semantic-outer` 或
`road.bin-offset` 已从默认策略删除，因为它在路口、分叉或逆向车道附近容易连接错误分支。
某一侧缺少可靠来源时，该侧保持 unresolved；另一侧可靠 boundary 仍可导出，因此
`boundary_status.valid=true`。`boundary_status.complete` 只有左右两侧都解析成功时才为
true；两侧均无可靠来源时 boundary GT 才无效。当前规则版本为 schema 9。

旧规则完整保留，可用 `--boundary-policy outer-offset` 启动进行 A/B 对照：

```bash
python server.py /path/to/clip --boundary-policy outer-offset
```

已经保存过的帧默认保留当时审核的 boundary。需要在界面中预览新规则时加
`--regenerate-auto-boundaries`；它只替换旧的自动 `road.bin-offset` /
`road.bin-semantic` / `road.bin-semantic-outer` 线，保留手画线和明确改过类别的源线。只有再次点击“保存本帧”
才会写回该帧 JSON：

```bash
python server.py /path/to/clip --boundary-policy semantic --regenerate-auto-boundaries
```

例如当前片段可直接预览：

```bash
python server.py /media/classlab/2E0CECBF0CEC82E7/2025-07-17-17-38-26 \
  --port 8765 --boundary-policy semantic --regenerate-auto-boundaries
```

SparseDrive 转换器也使用同一策略。若要让已经保存过旧自动边界的帧也按新规则重算，
重新生成 PKL 时同样传入 `--regenerate-auto-boundaries`；不加该参数则尊重已审核结果：

```bash
/home/classlab/anaconda3/envs/sparsedrive/bin/python \
  tools/data_converter/shenzhen_clip_converter.py \
  --clip /media/classlab/2E0CECBF0CEC82E7/2025-07-17-17-38-26 \
  --output-dir data/shenzhen --split train --stride 5 \
  --boundary-policy semantic --regenerate-auto-boundaries --rewrite-infos
```

#### 旧规则（仅用于 A/B）：从最外侧车道线外推道路边界

旧策略从每帧 `road.bin` 在车前 0–20 m 选择最左、最右有效线，沿外法向默认偏移
0.5 m 生成 boundary。它适合与历史结果做 A/B 比较，不建议作为默认训练标签。
`--boundary-offset-m` 只影响这个旧策略。默认 semantic 策略中的 `road_edge.csv` 必须通过
上述独立校验，不会无条件进入 GUI 或导出。

SparseDrive 地图头使用 `ped_crossing`、`divider`、`boundary` 三类。这里的 divider 表示
车道或道路分隔线，boundary 表示可行驶区域外轮廓；人行横道应画成完整闭合轮廓。

### Stage2 未来轨迹和五类指令如何得到

#### 1. 构造未来轨迹

程序从 `result/mapping/mapping_pose_quaterniond.txt` 读取每帧全局平移和四元数。对当前时刻
`t`，在 `t+0.5, 1.0, …, 3.0 s` 各找最近帧；时间误差超过 0.2 s 或缺少位姿时，该采样点
无效。未来全局位置用当前姿态的逆旋转变到当前车体坐标：

```text
p_local(τ) = R(q_t)^-1 · (p_global(τ) - p_global(t))
```

轨迹采用 `x` 向前、`y` 向左，起点为自车中心 `(0,0)`。未来帧的朝前向量也变换到
当前坐标系，最终航向角用于区分持续转弯与普通换道。页面中的青色或橙色虚线就是这组
0.5–3.0 秒轨迹点。

#### 2. 识别当前车道和跨线

Stage2 **只使用检测得到且当前类别为 divider 的折线**，不使用 boundary，也不依赖相邻帧
line ID 是否相同。在 `x=0 m` 与 `x=5 m` 都保持同侧的最近左右 divider 构成当前车道，
两线间距须为 2–6 m。随后逐段计算未来轨迹与这两条 divider 的二维交点。

自动建议按以下优先级产生：

- 未来六个采样点不全：`command_valid=false`，交给人工确认。
- 最终航向变化绝对值不小于 20°：按符号给出左转或右转。
- 只穿过一条当前 divider 且航向变化小于 15°：穿左线为左换道，穿右线为右换道。
- 穿过多条当前 divider：拓扑有歧义，`command_valid=false`。
- 没有跨线、车道走廊可靠且航向变化小于 12°：保持直行。
- 其余情况默认显示直行建议，但置 `command_valid=false`，必须人工复核。

类别固定顺序为 `TURN_RIGHT, TURN_LEFT, KEEP_STRAIGHT, LANE_CHANGE_LEFT,
LANE_CHANGE_RIGHT`。导出同时提供 `command_index_5`、`command_onehot_5`、
`command_valid` 和与其一致的 `planning_mask`。

#### 3. 换道何时算完成

点击“应用换道窗口”时，程序不再使用固定的“后 1 秒”。它在包含 `(0,0)` 的连续未来轨迹
线段上求与所选 divider 的精确交点，并用线段内插比例得到跨线时间：

```text
crossing_time = 当前帧时间 + (轨迹段序号 + 段内比例) × 0.5 s
```

时间戳不早于该时刻的第一帧是 `crossing_frame`；这样可以确保自车中心已经到达 divider
中心线，而不是误取交点之前的“最近帧”。事件标签从当前审核帧前 2.5 s 开始，到
`crossing_frame` 为止，之后不再继续标换道。
保存记录包含 `event_anchor_frame`、`crossing_time_s`、`source_review_frame` 和
`review_note=lane_change_until_divider_center_crossing`，便于回溯。若未来轨迹没有穿过
所选 divider，按钮会拒绝批量写入，避免凭固定时间误标；仍可逐帧人工修改。

“导出 2 Hz 五类标签”默认每 5 个 10 Hz 帧取一帧，写入
`reviews/maneuver_labels_2.0hz.json`。自动结果是建议，`reviews/maneuvers.json` 中的人工结果
读取时优先。

### 手画静态地图线如何映射到后续帧

`divider`、`boundary`、`ped_crossing` 都是静态地图类别，都可用建图位姿跨帧映射。**Ctrl+S 保存当前帧的局部修订；手画线要跨帧显示，还要点映射按钮。** 操作流程：

1. 回到画线的帧，例如图示片段已经保存手画边界的 **1018 帧**。切到“选择/拖动”，在 BEV 点击一条手画的地图线，确认右侧类别是 `divider`、`boundary` `ped_crossing`。
2. 点击顶部 **“所选静态线映射到相邻帧”**。每条手画线分别点一次。工具利用当前帧建图位姿，把 `divider`/`boundary` 存入 `reviews/map_lines_global.json`，把`ped_crossing` 存入 `reviews/crossings_global.json`。无需为 1019、1020 等帧手动生成 JSON。若当前帧还有其他局部修改，再按 **Ctrl+S**。
3. 跳到 1019、1020 帧检查 BEV 和相机投影。只要这条线落在相邻帧的局部视野内，就会由全局坐标变换到该帧车体坐标；车辆驶离该位置后不再显示。
4. 若全局线位置需调整，在任一显示该线的帧拖动顶点，然后按 **Ctrl+S**；保存会同步更新全局折线，再回看相邻帧核对。若只是当前帧的单独修正，可另画局部线，只按 Ctrl+S，不点映射按钮。全局 `divider` 与 `boundary` 可在右侧互相改类；`ped_crossing` 仍须保持人行横道类别。
5. 删除全局线时，先选中这条线，再点“删除所选”；删除立即作用于相邻帧，可在同一帧撤销。全部核对完，重新点“导出 2 Hz 标签”更新已有导出文件。

源 `road.bin` 车道线、默认策略生成的 `road.bin-semantic` 和通过校验的
`road_edge.csv-validated` boundary 都是逐帧生成，不需要点映射按钮；该按钮用于手画静态线。
`road.bin-semantic-outer` 与 `road.bin-offset` 只会在旧 `outer-offset` 策略或历史修订中出现。
已经保存过的旧手画线不会自动跨帧，按第 1–2 步逐条应用即可。旧版
`boundaries_global.json` 会继续读取，下一次写入时会合并到 `map_lines_global.json`。

## 3. 自动标注类别的默认映射

数据说明表中的 `type` 是 Classlab 自动标注类别，和 SparseDrive 的 10 类不完全同名。本工具默认：

| Classlab type | 数据说明表含义 | 默认训练类别 | 处理建议 |
|---:|---|---|---|
| 1 | 行人 | `pedestrian` | 直接使用已验收框 |
| 2 | 骑行者 | `bicycle` | 若实际是摩托车，改为 `motorcycle` |
| 3 | 小车 | `car` | 直接使用已验收框 |
| 4 | 大车 | `truck` | 若实际是拖挂车，改为 `trailer` |
| 5 | 公交车 | `bus` | 直接使用已验收框 |
| 6 | 卡车 | `truck` | 直接使用已验收框 |
| 7 | 工程车辆 | `construction_vehicle` | 直接使用已验收框 |
| 8 | 三轮车 | `ignore` | 逐例决定是否可归入现有类别 |
| 11 | 交通锥 | `traffic_cone` | 若其他片段出现即可使用 |
| 13 | 卡车头 | `truck` | 可按目标实例复核 |
| 14 | 交通灯 | 不进入 3D 框 | 原始框 `length=-1`，当前检测头没有交通灯类 |

其他未定义类别默认 `ignore`。灰色的 `ignore` 框仍可选中并改类别。原始类别、置信度、跟踪 ID 和速度会保留在导出记录中；类别规则位于当前片段的`reviews/class_rules.json`。源 `GT.csv` 的行 ID 在同一帧可能重复，界面为重复框加内部后缀以保证选择和编辑准确，原始 `track_id` 不变。

## 4. 线条和投影是怎么来的

`road.bin` 是 protobuf wire 格式。当前片段验证到外层逐帧记录有 3039 个；每帧包含多条线，每条线有道路段 ID、相对车道 ID、线型和逐点 `x,y,z`。`geometry.py` 只读取这些已观察到的字段，不改写二进制文件。普通折线初始显示为 `divider`；当前已验证的
`lane_type=110/112/114` 可以额外生成训练用 boundary。其他 lane type 不能仅因
处于最外侧就当作道路边界。人工确认一条线后，可用组合 ID 批量设置同一段的类别。
`lane.csv` 仍保留在源目录，可作为线型、有效范围的交叉核对资料。

点云底图来自 `result/local_motion_pcdbin/<帧号>.pcd`，使用 LZF 解压并抽样显示。BEV 使用 Classlab 车体坐标：`x` 向前、`y` 向左、`z` 向上；矩形由`center + length/width + yaw` 计算四个地面角点。

相机图像按 `raw/sync_time_list.txt` 对齐。点或框角点先由车体坐标减去相机位置，再乘相机外参旋转的逆矩阵，得到相机坐标；随后使用标定 YAML 的内参及 8 参数有理畸变模型投到原始 JPEG。框的 12 条棱和地图折线使用同一投影。相机图上的框本来就是这些 3D 框的投影，BEV 上显示的是**同一个框的俯视轮廓**，没有把单独的 2D图像检测结果反算为 3D。本片段已有实际图像抽检，换标定后应重新抽检。

人行横道在当前帧以闭合折线绘制；跨帧映射时用`result/mapping/mapping_pose_quaterniond.txt` 的位姿变换到全局坐标，其他帧再变回车体坐标。若一个修订只应作用于单帧，不点击“所选静态线映射到相邻帧”即可。

## 5. 修订文件放在哪里，输出什么

默认输出路径是启动命令中指定的 `<片段目录>/reviews/`。例如传入`D:\datasets\2025-07-17-17-38-26`，Windows 上的修订文件就在`D:\datasets\2025-07-17-17-38-26\reviews\`；Linux/macOS 同理。若传入前述实验室 Linux 示例路径，输出就是`/media/classlab/2E0CECBF0CEC82E7/2025-07-17-17-38-26/reviews/`。使用 `--edits-dir` 时则按第 1 节所述写到指定的可写目录。文件按使用情况生成，因此刚启动时目录可以是空的。

```text
<片段目录>/reviews/
├── <帧号>.json               按“保存本帧”后生成：该帧局部框与线修订
├── class_rules.json          按“所选类别应用到同 ID 全片段”后生成：类别规则
├── deleted_tracks.json       按“删除同 ID 全片段”后生成：隐藏的 track ID
├── crossings_global.json     按映射按钮后生成：全局人行横道
├── map_lines_global.json     按映射按钮后生成：全局 divider / boundary
├── maneuvers.json            人工确认的逐帧 Stage2 五类标签
├── maneuver_labels_2.0hz.json  Stage2 轨迹、证据和 2 Hz 五类标签
└── mapped_labels_2hz.json    按“导出 2 Hz 标签”后生成：框和地图线合并标签
```

`<帧号>.json` 不会为每个受全局记录影响的帧自动生成；读取帧和导出时会应用全局记录。重新修订后，需再次点击“导出 2 Hz 标签”，覆盖旧的导出文件。

当前导出是**标签中间件**：框和地图线仍在 Classlab 车体坐标中，包含三类 `map_annos`和已映射的目标框。它还不是可以直接训练的 nuScenes PKL。进入下游数据转换脚本前，还须完成车体与 LiDAR 坐标/外参统一、点云转五维 float32 `.bin`、明确采集相机与训练配置的对应方案、写入各帧 pose 和相机矩阵，并按完整片段划分训练/验证集。
