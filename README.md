## 2022 Rapid React Code
This repository contains the code for  FRC team Robotiators 888's 2022 rapid react robot. This code is written in java using wpilib's command based structure as its [documented Here](https://docs.wpilib.org/en/stable/index.html). 

## Hardware

**Drivetrain**

This year we were able to aquire a [roboRIO 2.0 from Andymark](https://www.andymark.com/products/ni-roborio-2) on wich we have a [navX-MXP](https://pdocs.kauailabs.com/navx-mxp/) to allow for more accurate tracking of the robot's position. additionally our robot utelizes two three wheel driverails from [Plummer Industries](https://plummerindustries.com/) in a differential drive configureation, each side is powered by two brushless NEO motors and spark maxes. 


**Intake and indexing**

Our intake and indexing solutions utelizelize a series of belts and rollers to manage the balls within our robot. The intake is powered by one 775 motor on a talon srx, and acuated by two pneumatic pistons. following the intake, the canal is two separate belt systems powered by an additional two 775 motors and talon srxs, one per side.  Next, the index, which is the belt directly below the shooter and is run by a NEO 550 and spark max. Our indexing system is automatically controlled by two banner sensors that can tell us if there is a ball in one of the two storage positions. 


**Shooter**

Fed by the Indexing system, is the shooter, Our shooter is a flywheel design powered by two NEOs and spark maxes in tandem.


**Climber**

Our Climber is powered by a NEO and spark max which we use to spool the climber down. On extension, the climber makes use of two Vulcan springs to raise the hook. the extension and retraction of the climber are limited by two limit switches wired into the Spark max via a breakout board. The climber uses a friction brake to take the load of the robot off of the motor when hanging, this brake is actuated by a pneumatic piston. 


**Vision**

For our vision systems, we have a [Limelight](https://limelightvision.io/) mounted over the shooter for getting distance from the goal and aligning to it. in addition, we have two raspberry pis that allow us to track balls behind and in front of the robot with their x and y coordinates relative to the robot. each pi is dedicated to watching one of either the front or rear cameras. finally, we have a Microsoft life cam mounted to the front and plugged into the Rio for driver vision.

## Software
**Trajectory Following**

This year, the robot utilizes Trajectory following and path planning through path weaver, this allows us to quickly and easily create complex autos. Much of the grunt work for this portion of our robot code is from the [wpilib library](https://docs.wpilib.org/en/stable/docs/software/pathplanning/index.html). The odometry code is contained in the drivetrain subsystem class and is updated in the periodic method. Autos and trajectories are created in robot container assisted by the Autonomous class which has methods to easily load and convert path weaver files to trajectories and get a ramsete controller for a provided trajectory. 


**Ball Tracking**

Our ball tracking is written in python using open cv, which we can use to adjust our autonomous routines so the robot always picks up the ball we intend to. The data from the vision system is sent to the roboRIO via network tables which also allows us to monitor the information for debugging purposes. All of the python code for vision is stored in [this separate Repository](https://github.com/Robotiators-888/2022CoprocessorVision).


**Shooter**

The shooter on our robot is controlled by a PID control loop built into the spark maxes. In order to use two motors on our shooter we had to set one as a follower of the primary via the spark max firmware.

