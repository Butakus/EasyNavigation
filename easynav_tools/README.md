
# easynav_tools (legacy Textual compatible)

Herramientas ROS 2 para EasyNav:
- **TUI** (Textual antiguo): `ros2 run easynav_tools tui`
- **CLI ros2cli**: `ros2 easynav echo|metrics`

## Build

```bash
cd ~/ros2_ws/src
# descomprime aquí
cd ..
colcon build --packages-select easynav_tools --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 run easynav_tools tui
ros2 easynav echo --duration 5 --rate 20
ros2 easynav metrics --duration 10
```
