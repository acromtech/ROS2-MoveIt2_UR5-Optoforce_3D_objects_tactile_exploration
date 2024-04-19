# Tactile exploration of 3D surfaces/objects with an Optoforce sensor and the UR5 robotic arm

In this case the Optoforce sensor is moved and touched to the object directly with the UR5 robotic arm. The knowledge of the object’s rough boundary can be assumed, the main focus should be on the more detailed / precise surface information of the given object. The UR5’s endeffector (EE) can be directly moved to different location-orientation combinations.
This spatial data (location +orientation) has to be co-registered with the measurement of the Optoforce sensor.

Some of the estimated subtasks:
* maybe 3D printing or mechanical design to fix the Optoforce to the UR5’s EE;
* control of the UR5 arm (most probably you will roughly learn it from the infra measurement);
* preliminary trajectory generation to the EE (on the basis of the rough surface information of the object);
* data fusion to the EE data and the Optoforce data.
