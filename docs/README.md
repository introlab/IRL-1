# IRL-1

![IRL-1](images/irl1_full.jpg)

## Introduction

IRL-1 is wheeled, humanoid robot built at Université de Sherbrooke's IntRoLab.
It is a combination of two fully independent robots:

 - Johnny-0, the humanoid torso with compliant arms and an expressive head;
 - A choice of two mobile bases, AZIMUT-3 (omnidirectional and compliant) or Telerobot (differential drive)

The software is entirely based on ROS with custom packages found in this repository.
The aim of this guide is to describe how to use the robot, from startup to the development of higher-level capabilities.

**This is a living document and is not meant to be fully complete from the beginning.
Feel free to suggest improvements or guides of your own.**

## Startup

Since IRL-1 is two independent robots, they both have to be started separately. 
The order is not important, except if you require the network switch installed on the back of Johnny-0, as
it is powered by the torso.

### Johnny-0

Starting Johnny-0, while relatively easy, is a multistep process on the source of power.

#### Choosing the power source
Depending on the work you plan to do, the power source you will use is important.
Both power sources, the external one and the batteries, cannot be used at the same time.
Furthermore, they cannot be switched seamlessly.
Indeed, ** the Johnny-0, including the computer, has to be fully powered off before switching power sources **.

For development and light payloads on a single arm, the external power source can provide enough current.
Otherwise, the batteries are necessary.
While not dangereous by itself, a insufficient current delivery shuts down the whole Johnny-0 torso, which
means the computer will turn off and the arms might fall and collide with the rest of the robot or the environnement.

#### Using the external power source

(TODO: image of the power source)

1. **Power up the source itself.** The power source does not have a on/off switch, and always provide 24V when 
 
2. Connect the external power cable, located in the middle of the torso. **Do not plug the external power source in one of the battery inputs! (TODO: images)**

3. Turn on the torso power switch (TODO: Image). The display should show 24V or so (TODO: confirm, image)

#### Using the batteries

1. **Make sure the robot is power off!**

2. Connect both batteries in their terminals (TODO: picture)

3. Turn on the torso power switch (TODO: Image). The display show show 12V or so on each battery (TODO: confirm, image)

#### Charging the batteries

Each battery has a LiFePO4 charger with its own charging connector.
Charging can be done safely when the robot is used on the external power source.
**Again, switching power sources first require a complete shutdown of the Johnny-0 torso !**
Note that charging the batteries can be done without turning on the power management of the torso.

1. **Disconnected the batteries from the torso.** You have to make sure that the batteries are not currently powering the robot before charging them. Turning the robot off is not enough, **you need to disconnect each battery**.

3. Connect the chargers to an AC source, and leave them off. The chargers have their own on/off switches (TODO: confirm, image).

4. Connect the batteries to their chargers. (TODO: image)

5. Turn on the chargers. The LEDs should turn to (TODO: colors, images).

#### Turning on the computer

Once power is applied to the Johnny-0 torso, you can turn its computer on.
The power microswitch is located below the Mini-ITX case (TODO: picture).
Once powered on, you should see blue LEDs flashing near the microswitch, and see the CPU fan starting to turn through
the air vents of the case.

## Shutdown

To shutdown the robot, you simply need to revert all the steps you took to power on the robot.
The PCs have to be shut down first, then the power management of each robot (power switch on the torso, power key on the mobile base).
Finally, you can disconnect the power sources from the robot.
