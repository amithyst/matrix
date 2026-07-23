# Matrix House/Furniture 箱庭 V1

House/Furniture V1 使用 Matrix 官方 `HouseWorld` cooked PAK 作为第一版室内家具箱庭。
它不是自研木屋，也不是可编辑 UE 工程；第一版目标是把一个真实家具密集的室内空间接入
同一条 Matrix + SONIC 主启动链路，并形成可复现截图/视频 smoke。

## 场景边界

- 启动入口：`scripts/run_matrix_sonic_house_v1.sh`
- Matrix scene id：`6`
- UE 地图：`/Game/Maps/HouseWorld`
- MuJoCo 场景：`scene_terrain_house.xml`
- 官方包：`HouseWorld-0.1.2.tar.gz`
- PAK chunk：`pakchunk17-Linux.*`

`Home / ApartmentWorld` 也是 Matrix 原生家居场景，但当前 locked runtime 曾出现
`/Game/Maps/ApartmentWorld` 缺包风险。因此本次可验收主目标固定为 `HouseWorld`；
`ApartmentWorld` 暂不作为 Home/Furniture V1 的主线合并门禁。

## 物理语义

HouseWorld 的视觉层来自官方 cooked map，家具、地板、厨房、沙发等由 UE 渲染。物理层使用
Matrix 原生 `scene_terrain_house.xml`，当前是 97 个环境几何体：

- `1` 个 floor/plane；
- `88` 个 box；
- `8` 个 cylinder；
- `0` 个动态环境物体。

这意味着第一版家具主要是静态碰撞代理：机器人应该能被墙体、家具、柜台等挡住或绕行；
但不能宣称可以开抽屉、拖椅子、拿杯子，或编辑 UE 内部 Actor/材质/Blueprint。

## 启动

```bash
bash scripts/run_matrix_sonic_house_v1.sh \
  --profile zza \
  --control-source planner \
  --walk-after 5 \
  --vx 0.15 \
  --max-seconds 80
```

包装脚本拒绝外部 `--scene` 参数，确保 smoke、截图和记录都指向同一个 HouseWorld 箱庭。
需要调别的 Matrix 原生场景时，直接使用 `scripts/run_matrix_sonic.sh --scene <id>`。

## 录屏验收

```bash
bash scripts/record_matrix_sonic_video.sh \
  --display :1 \
  --xauthority /run/user/1000/gdm/Xauthority \
  --ready active \
  --ready-active-seconds 3 \
  --ready-timeout 240 \
  --output outputs/acceptance/house-v1/houseworld-main-smoke.mp4 \
  --metadata outputs/acceptance/house-v1/houseworld-main-smoke.json \
  --duration 12 \
  --fps 30 \
  --notes "House/Furniture V1 main-chain active SONIC smoke" \
  -- \
  bash scripts/run_matrix_sonic_house_v1.sh \
    --profile zza \
    --control-source planner \
    --walk-after 5 \
    --vx 0.15 \
    --max-seconds 80
```

通过标准：

- metadata 中 repository branch/commit 绑定到待验收 main 或 PR commit，且 dirty=false；
- video quality `passed=true`，无 failures；
- SONIC `active_lowcmd=true`，无 fall/reset；
- physics step 接近 200 Hz，render 接近 50 Hz，RTF 接近 1；
- 截图中能看到 HouseWorld 室内家具和 G1，镜头不丢机器人。

## 推荐任务

第一批 benchmark 任务应该保持纯物理和主链路语义：

1. 室内窄通道行走；
2. 绕过沙发、桌子、柜台；
3. 接近指定家具/门口/厨房区域；
4. 视觉记录和碰撞审计；
5. 后续再接入可交互拾取物，不在 V1 里混入游戏机制。
