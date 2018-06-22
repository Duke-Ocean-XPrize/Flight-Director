# Flight-Director
This is the core control logic for the drone - it is responsible for:
* Guiding the drone to precoded locations for pod dropoff/pickup
* Communicating with the pods and the ground station over LoRa
* Communicating with the vision subsystem in order to pinpoint the location of the pod
* Handling emergency situations including, but not limited to:
  * Gas running low
  * Hybrid engine failure
