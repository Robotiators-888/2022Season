/* 
* Kevin Hwang
* Lasted edited on 10/26/2021
* Makes robot move using Mecanum wheels.
*/

package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SpeedControllerGroup;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;





public class DriveSubsystem extends SubsystemBase{

    // Left
    private final WPI_VictorSPX frontLeft = new WPI_VictorSPX(Constants.FRONT_LEFT_ID);
    private final WPI_VictorSPX rearLeft = new WPI_VictorSPX(Constants.REAR_LEFT_ID);
    public final SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeft, rearLeft);

    // Right 
    private final WPI_VictorSPX frontRight = new WPI_VictorSPX(Constants.FRONT_RIGHT_ID);
    private final WPI_VictorSPX rearRight = new WPI_VictorSPX(Constants.REAR_RIGHT_ID);
    public final SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRight,rearRight);


    
    public final MecanumDrive driveTrain = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    public DriveSubsystem(){
        rearLeft.setInverted(true);
        frontRight.setInverted(true);
    }

    public void drive(double ySpeed, double xSpeed,double zRotation){
        // ySpeed, xSpeed, and zRotation are all supplier values from the joystick object
        driveTrain.driveCartesian(ySpeed,xSpeed,zRotation);
    }
    
}
