# Day4_5 Final Parking Test

Run `setup_final_parking_test.m` in MATLAB from this folder:

```matlab
cd('C:\Users\User\Desktop\IVS\motionPlanning\26HL_IVS_ADAS\02_Carmaker_project\Practice_sample\src_cm4sl\Models\Day4_5')
setup_final_parking_test
```

The script backs up `Day4_5_Scenario_1.slx`, updates the Day4_5 model constants, and creates:

```text
Data/TestRun/day4_5_final_parking_only
```

Use that TestRun with the Day4_5 Simulink scenario.

## Values Applied

```text
Start point        = [5.5, -36.5]
Goal point         = [35.0, -30.0]
Goal yaw           = 2*pi/3
Map boundary       = [4.0, -4.0;
                      4.0, -46.8;
                      48.0, -4.0;
                      48.0, -46.8]
Traffic size       = [1.97; 4.47]   % [width; length]
Scene obstacles    = T00..T20
Map obstacle input = T00..T20
Road               = day7_final.rd5
TestRun start      = global [5.5, -36.5, 2.5], yaw 180 deg
Traffic.N          = 21
```

## Simulink Blocks Changed

```text
Start_Point constants       : SID 3, 4
Finish_Point constants      : SID 5, 6
Goal yaw constant           : SID 4009
Map_Boundary constants      : SID 7, 9, 8, 10, 3775, 3776, 3778, 3779
Traffic_size constants      : SID 2, 1
Traffic Object readers      : SID 77..83
Traffic Object ID constants : SID 42, 41, 43, 44, 45, 46, 47
```
