ToDo
22/09/2025
Livox and Camera FOVs should be included
The camera view 

Wherever the scan position might be, the system should point at the hub, and give the pan tilt angle at each position.




Further on...
The no of zones calculation steps
And the pan tilt angle for each zones

The number of pulses hit by the moving blade for a laser of a particular frequency and beam width at the given working distance.

To send a pop up message when the laser beam hits the blade


Have a cylinder with a heght of 1 m and radius 0.3m color solid opaque black  - for system height. and have two one at sp and t0 for scanner and tracker. 
the sp is dynamic and the tracker one is static... even the FOVs

Ok now i want a variable, tracker_ht = 1 m, and the previously done caluclations for Tracker position and the laser sensor should be from a height = tracker_ht above the ground.

Now assume i have a pan tilt setup with the tilt on top of the pan axis by offset_x = 0.05 m and offset_vertical = 0.01 m, and the laser sensor is on top of the tilt, update this in
our program and give me the value of the pan and tilt motors when aiming at the hub center. This should be dynamic, so the pan tilt angle toward the hub should update even if the 
system pos is changed. The heirarchy of operations will be First - Pan, then followed by tilt, for aiming at each position.



Now add another sensor
Name: Livox
FOV: 70.4 degree H X 77.2 V degree
Position: on top of the tracker above the laser sensor above the tilt motor
It should also be aimed at the hub ctr...



Scanner

Name: Camera
FOV: 70.4 degree H X 77.2 V degree
Position: on top of the tracker above the laser sensor above the tilt motor




*********************************************************************************************************************************************************************************************************************

Table on a separate panel that contains the info:
Rotor dia
Hub ht
System height
Turbine Coorindates
SP1 Location Coorindates
SP2 Location Coorindates
SP3 Location Coorindates
SP4 Location Coorindates


Rotor RPM
Yaw

Blade count global
Blade count blade 1,2,3

tracker pan tilt angle (for hub aim)
scanner pan tilt angle (for hub aim)


There should be a table containing the live parameters updateing properly at the top
There should be a red indicator light that blincs when the laser hits the blade and stays on until the laser leaves it.
The number of pulses hit by the laser should also reflect on the table. - there should be a global counter, and a blade number counter that gives the current blade that was hit, like blade 1,2,3.
and there should be indication on the table regarding the blade colour like rgb, as r=blade 1, g=blade2, and b=blade3

*********************************************************************************************************************************************************************************************************************






the scanner current position variable should be present such that when i give scanner_current_po = sp2 or sp2 or sp3; it should move to the given position all the while, the scanner sensors should aim the hub
from their new postion, and output the respective solved pan and tilt angles, at that pos.




*********************************************************************************************************************************************************************************************************************


CAMERA ZONE VIEW
the camera with the total zone count should be visible on the horizontal portion/line, starting from the hub_ctr coordinates and rightward or leftward depending on the sp1-3 or sp4 position, according to the rotor_dia/2
The number of zones should all be displayed at once in a very transluscent form stating from the camera upto the wtg blade position, and the current_fov should be highlighted



*********************************************************************************************************************************************************************************************************************

Coordinates: 
When facing the turbine
X right left
Y forward reverse
Z height 






a sweep pattern is needed for the camera with sweep speed corresponding to the WTG rotational speed (downward linear) at a particular spot


HUD Removed.










- presets: save/load full site configs (WTG, blade params, scanner SP) as JSON.

- auto-scan modes: sweep yaw or run SP1→SP4 sequences with dwell times.

+ keyboard shortcuts: 1–4 to jump scanner to SP1–SP4, ←/→ tweak yaw, ↑/↓ tweak RPM.

- CSV/GeoJSON import: load WTG GPS + SP offsets from a file.

- data logging: stream status (yaw, RPM, pan/tilt, distances) to CSV for post-run analysis.

- screenshot hotkey: trigger Foxglove 3D panel snapshots with a button.





SIMULATION CONTROL PANEL:
- auto-scan modes: sweep yaw or run SP1→SP4 sequences with dwell times.
+ yaw oscillation mode: random, sine type, with yaw rate, and max limits
- Camera view
- Reset Button
- N S E W direction and cartesian coordinates direction legend 
- Top heading banner resizing


DASHBORD:
    N S E W direction and cartesian coordinates direction legend 


    
    
    
    Based on GPS location and time, from the internet:

    Display: 

        Site/Location name

        Weather data: 
            From online, current and future based on enetered location
                Temp
                Humidicty
                Precipitaiton now and for next 3 hours
                Cloud cover



System stats 
    System Temperature
    Battery status - in the form icon

Self levelling info (for both scanner and tracker)

Scanner position visual map indication (scanner)
    
Warnings (with icon)
    Yaw out of limit
    Rotor speed out of limit - either high or low below a threshold value that can be set (eg. 12 rpm - upper limit and 1 rpm - for lower limit)
    Low ambient light
    Sensor blocked 
    System warnings - like motor, camera, processor, comm, etc.,


Subdivide the scanner and tracker and simulation controls
    Data input horizontal card
    Scanner  dashboard visual/stats
        Top banner with battery status, network connectivity
        Warning Banner (with warning icon/errors/alarms)
        Power button
        Visuals
    Tracker dashboard visual/stats
    Simulation controls

Include Power button