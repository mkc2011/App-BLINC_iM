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
FOV: 70.4 H X 77.2 V
Position: on top of the tracker above the laser sensor above the tilt motor
It should also be aimed at the hub ctr...










Coordinates: 
When facing the turbine
X right left
Y forward reverse
Z height 






a sweep pattern is needed for the camera with sweep speed corresponding to the WTG rotational speed at a particular spot


HUD Removed.