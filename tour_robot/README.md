### Use Case 1: Tour Robot

In this simple example, we show a more sophisticated BT interacting with several components. The robot acts as a museum guide

The robot has the following task: _The robot has to visit a list of poses on the map, At each pose, it tells a description of the point of interest at the pose (like a museum guide). Whenever the robot's battery is below 10% of its capacity, it goes to a charging station until the battery is fully charged_

Let's consider the BT below:

<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/192753951-74677ead-4bc9-4b97-bd97-73d5231028cb.png" />
</p>


Where we assume that the poses `charging_station` is present in the [Blackboard](https://www.behaviortree.dev/tutorial_02_basic_ports/). In this example, we add it from the [bt executable](https://github.com/miccol/bt_use_cases/blob/9a66b5e2b1d4705235227096c92303de2baf4724/tour_robot/bts/src/run_bt.cpp#L77).


Let's first analyze the architecture of this use case:

Leaf nodes: 
- The BT Condition _IsBatteryLevelAbove_ {`reference_value`} that reads the battery value from a topic and compares it with the reference value
- The BT Action _GoTo_ {`pose`} that sends the pose to a navigation stack via [ROS Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- The BT Action _Wait_ returns only the status of BT::Running
- The BT Condition _BatterIsNotCharging_ checks if the status of the battery is [POWER_SUPPLY_STATUS_NOT_CHARING](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html), reading from the topic `/battery_state `
- The BT Condition IsAt{`pose`},
{`linear_threshold`} , {`angular_threshold`} checks if the current pose of the robot is close to `pose`, within the related thresholds.
- The BT Action _WriteCurrentPOIOnBB_ takes the current POI from a tour scheduler component and writes it on the blackboard.
- The BT Action _SayText_ reads a std::string with the description of the POI from the blackboard and sends it to a talker component.
- The BT Action _UpdateScheduler_ sends a request to a tour scheduler component to update the current poi.

- Components: 
  - [Nav2](https://navigation.ros.org/index.html) as navigation stack. 
  - _FakeBatteryReader_ to simulate a battery reading. In publishes the [BatteryState message](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/BatteryState.msg) on the topic `/fake_battery_reader/battery_state` with the battery value. It also provides the service `charge_battery` to charge the battery.
  - _SimpleTourScheduler_ a simple tour scheduler that takes a set of POIs from a config file and schedules them in sequence.
  - _FakeTalker_ to simulate the speech synthesis capability of the robot. It prints on the console the text received.
  - 
- Interfaces: 
- we interface the GoTo action in the BT with the Nav2 via the [NavigateToPose.action](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action)
- we interface the IsBatteryLevelAbove with the battery reader component via the [BatteryState message](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/BatteryState.msg).
- we interface the  BT Actions _WriteCurrentPOIOnBB_ and _UpdateScheduler_ with _SimpleTourScheduler_ via the services: [GetCurrentPOI](interfaces/srv/GetCurrentPOI.srv) and [UpdatePOI](interfaces/srv/UpdatePOI.srv)

- we interface the  BT Actions _Say_  via the services: [SayText](interfaces/srv/SayText.srv)
>**Warning**
> In this BT we trust the success conditions of Nav2 Action for the BT Actions _Goto_. In general, it is better to follow the design principle _Improving Readability using Explicit Success Conditions_ from Chapter 3.1 of the book [Behavior Trees in Robotics and AI](https://arxiv.org/pdf/1709.00084.pdf)


Let's run this BT and play with the battery value.

We first run the simulation environment:

```console
ros2 launch bt_uc_sim simple_simulation_launch.py
```

> **Note**
> If the simulation is too heavy for your computer, run the command above with the option `headless:=True` (i.e., `ros2 launch bt_uc_sim simple_simulation_launch.py headless:=True`. That will not launch the Gazebo Client.



We then run all the components (the launch file runs the custom-made _FakeBatteryReader_, _Nav2_,  _SimpleTourScheduler_, and _FakeTalker_ ).
```console
ros2 launch tour_components all_components_launch.py
```

A new terminal with the output of the component _fake_talker_ should.

At this point windows should appear as below:

<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/192756657-e394730c-5435-4695-b369-afe43c27b93a.png" />
  <img src="https://user-images.githubusercontent.com/8132627/192756952-c7b01c72-fdf4-4578-9268-115877b7a17f.png" />
</p>

And finally, run the BT

```console
ros2 run tour_bts run_bts
```

The robot should happily roam between four poses. At Each pose it provides the description of the related POI (check the terminal of the _fake_talker_). Whenever  the battery goes below 10% the robot goes to the another pose (the charging station) and wait for manual recharge.

> *Note*
> It you are running it from [WSL](https://ubuntu.com/wsl), you must install `gnome-terminal` to let the terminal of _fake_talker_ pop up.
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
