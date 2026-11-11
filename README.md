# shin-spider-ros

ROS1 (Noetic) port of the Processing spider controller (rendering is not ported).
The catkin package name is `shin_spider`.

## Build

```bash
cd ~/catkin_ws/src
git clone https://github.com/Harlequinade1123/shin-spider-ros.git
cd .. && catkin build shin_spider && source devel/setup.bash
```

## Run

```bash
roslaunch shin_spider shin_spider.launch                  # serial: /dev/ttyUSB0, gamepad: /dev/input/js0
roslaunch shin_spider shin_spider.launch dry_run:=true    # without hardware
```

## Test

```bash
catkin build shin_spider --no-deps --make-args tests
catkin run_tests shin_spider --no-deps
```

## License

MIT
