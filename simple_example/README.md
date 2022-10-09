### Use Case 0: Simple Robot (an example of a ~~BAD~~ improvable BT )

In this simple example, we will use some basic actions and conditions from the [BehaviorTree.CPP](https://www.behaviortree.dev/) to design a simple robot behavior. We will then see how to improve it.

The robot has the following task: _The robot has to visit two points on the map, whenever the robot's battery is below 10% of its capacity, it goes to a charging station until the battery is fully charged_

Let's consider the BT below:


<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/191736753-ea657029-7f95-4b50-a814-b30350f560eb.png" />
</p>

Where we assume that the poses `charging_station`, `pose_of_interest1`, and  `pose_of_interest2` are present in the [Blackboard](https://www.behaviortree.dev/tutorial_02_basic_ports/). In this example, we add them in them the [bt executable](https://github.com/miccol/bt_use_cases/blob/9a66b5e2b1d4705235227096c92303de2baf4724/simple_example/bts/src/run_bt.cpp#L77).

Let's first analyze the architecture of this use case:

- leaf nodes: 
  - IsBatteryLevelAbove {`reference_value`} that reads the battery value from a topic and compares it with the reference value
  -GoTo {`pose`} that sends the pose to a navigation stack via [ROS Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- components: 
  - [Nav2](https://navigation.ros.org/index.html) as navigation stack. 
  - *fakeBatteryReader* to simulate a battery reading. In publishes the [BatteryState message](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/BatteryState.msg) on the topic `/fake_battery_reader/battery_state` with the battery value. It also can receive the following commands through ROS services for debugging: `SetBatteryValue`, `GetBatteryValue`, and `ChargeBattery`.
- Interfaces: 
  - we interface the GoTo action in the BT with the Nav2 via the [NavigateToPose.action](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action)
  - we interface the IsBatteryLevelAbove with the battery reader component via the [BatteryState message](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/BatteryState.msg).



>**Warning**
> In this BT we trust the success conditions of Nav2 Action for the BT Actions _Goto_. In general, it is better to follow the design principle _Improving Readability using Explicit Success Conditions_ from Chapter 3.1 of the book [Behavior Trees in Robotics and AI](https://arxiv.org/pdf/1709.00084.pdf) 

However, the BT above has some issues: 


- While charging, as soon as the battery reach aches above 10% the robot will leave the charging station.
- Once at the charging station, the BT will keep ticking the action `GoTo{charging_station}`.
- The more waypoints you have, the larger the tree (this is arguable, but still).
- The sequence of the waypoint is strict. In some applications, the number and order of waypoints may change at runtime. As is an interactive tour guide robot, where we have a scheduler that skips some less interesting waypoints if the tour takes too long.

So, we don't bother trying to run that BT.
## Use Case 0.1: Simple Robot (a better example)

We now address the first two issues above, we will address the other two in the next use case Tour Robot.

Consider the modified BT below:
<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/192754551-32f215dc-45d8-4019-828d-98acb93fa315.png" />
</p>

Where:
- the BT Action _Wait_ returns only the status of BT::Running
- the BT Condition _BatterIsCharging_ checks if the status of the battery is [POWER_SUPPLY_STATUS_CHARING](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html), reading from the topic `/fake_battery_reader/battery_state `
- the BT Condition IsAt{`pose`},
{`linear_threshold`} , {`angular_threshold`} checks if the current pose of the robot is close to `pose`, within the related thresholds.



Let's run this BT and play with the battery value.

We first run the simulation environment:

```console
ros2 launch bt_uc_sim simple_simulation_launch.py
```
> **Note**
> If the simulation is too heavy for your computer, run the command above with the option `headless:=True` (i.e., `ros2 launch bt_uc_sim simple_simulation_launch.py headless:=True`. That will not launch the Gazebo Client.

We then run all the components (the launch file runs the custom-made _fakeBatteryReader_ and _Nav2_).
```console
ros2 launch simple_components all_components_launch.py
```

At this point windows should appear as below:


<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/192756657-e394730c-5435-4695-b369-afe43c27b93a.png" />
  <img src="https://user-images.githubusercontent.com/8132627/192756952-c7b01c72-fdf4-4578-9268-115877b7a17f.png" />
</p>


And finally, run the BT

```console
ros2 run simple_bts run_bts
```



The robot should happily roam between two poses, until the battery goes below 10%. When that happens, the goes to the another pose (the charging station) and wait for manual recharge.

If you are eager to let the robot go to the charging station, open [RQt](https://docs.ros.org/en/humble/Concepts/About-RQt.html)


```console
rqt
```

and call the service `fake_battery_reader/set_battery_level` with a value of `0.1` (or a lower one).

<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/192758248-3a2a8fb2-c7fe-4b5a-a920-f56bb328ee08.png" />
</p>


Once it reaches the charging station should stay there until we charge the battery. We do that by calling the service `fake_battery_reader/charge_battery`. The robot will leave the charging station when the battery is full.
Check the battery level still with RQt:


<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/192758765-9253acad-04b1-4ebf-bd4d-fa220524640b.png" />
</p>


> **Note**
>  If you have [ZeroMQ](https://zeromq.org/download/)  installed you can visualize the BT while it runs via  [Groot](https://github.com/BehaviorTree/Groot)

Now take a look at the next use case [Tour Robot](../tour_robot/)