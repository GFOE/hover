# hover

GFOE's fork of CCOM respository.

Implements the `hover` ROS node.

## `hover`

ROS

### Waypoint Control Algorithm

* Implements action server that takes lat/lon ([geographic_msgs/GeoPoint Message](https://docs.ros.org/en/noetic/api/geographic_msgs/html/msg/GeoPoint.html)) goal point (note, no heading).
* When a new goal is received, transforms the lat/lon geopoint to the map frame using Project11::Transformations which are implemented here: https://github.com/GFOE/project11/blob/gfoe-devel/include/project11/tf2_utils.h
* Each update cycle control uses Project11::Transformations (which uses ROS tf) to determine range and bearing from the base_link frame to the map frame.
* Range dependent speed command
    * If range < threshold (currently hardcoded as 2.0 m), then speed set to zero.
    * else if range >= maximum_distance, target speed maximum_speed.  Both maximum_distance and maximum_speed are dynamic parameters (see below.)  Note, the default config seems to have the defaults fot these parameters set to zero.  I suspect the defaults are perhaps sent by a Project 11 GUI, maybe CAMP?
    * else if range > minimum_distance, target speed is set in linear proportion to range up to maximum_distance.  minimum_distance is another dynamic parameter.
* Yaw velocity is set proportional to bearing from vessel to goal.  Proportional gain is hardcoded as 0.25.

Sends goal location visualization to CAMP via publication to "project11/display" topic. 



### Subscribed Topics

<std_msgs::Bool>("enable"

### Published Topics

<geometry_msgs::TwistStamped>("cmd_vel"
geographic_visualization_msgs::GeoVizItem>("project11/display


### Action Server

https://github.com/GFOE/hover/blob/gfoe-devel/action/hover.action

```
# goal
geographic_msgs/GeoPoint target
---
# result
bool ok # flag indicating that hover ended for non-error reasons (i.e. user request)
```

### Parameters

    nh_private.param<std::string>("map_frame", m_map_frame, "map");
    nh_private.param<std::string>("base_frame", m_base_frame, "base_link");

### Dynamic Parameters

Includes [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure) for operating parameters: https://github.com/GFOE/hover/blob/gfoe-devel/cfg/hover.cfg

```
gen.add("minimum_distance", double_t, 0, "Distance from hover target to slow to 0, in m",              5.0, 0.0, 50.0)
gen.add("maximum_distance", double_t, 0, "Distance from hover target where max speed is used, in m",  25.0, 0.0, 250.0)
gen.add("maximum_speed",    double_t, 0, "Maximum speed to use to approach target, in m/s",            1.5, 0.0, 10.0)
```

### Theory of Operation

### Examples

#### [fy21_step2_annie_common.launch](https://bitbucket.org/gfoe/project12/src/gfoe-devel/launch/fy21_step2_annie_common.launch)

```
  <!-- hover -->
  <node pkg="hover" type="hover_node" name="hover_action" ns="$(arg namespace)">
    <param name="base_frame" value="$(arg namespace)/base_link"/>
    <param name="map_frame" value="$(arg namespace)/map"/>
    <remap from="cmd_vel" to="piloting_mode/autonomous/cmd_vel"/>
    <remap from="enable" to="piloting_mode/autonomous/active"/>
  </node>
```
