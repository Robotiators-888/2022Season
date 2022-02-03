/* 
* Kevin Hwang
* Lasted edited on 10/26/2021
* Makes robot move using Mecanum wheels.
*/

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {

    // Left
    private final CANSparkMax Left_1 = new CANSparkMax(Constants.Drive_Left_1, MotorType.kBrushless);
    private final CANSparkMax Left_2 = new CANSparkMax(Constants.Drive_Left_2, MotorType.kBrushless);
    private final CANSparkMax Left_3 = new CANSparkMax(Constants.Drive_Left_3, MotorType.kBrushless);

    public final MotorControllerGroup leftGroup = new MotorControllerGroup(Left_1, Left_2, Left_3);

    // Right
    private final CANSparkMax Right_1 = new CANSparkMax(Constants.Drive_Right_1, MotorType.kBrushless);
    private final CANSparkMax Right_2 = new CANSparkMax(Constants.Drive_Right_2, MotorType.kBrushless);
    private final CANSparkMax Right_3 = new CANSparkMax(Constants.Drive_Right_3, MotorType.kBrushless);

    public final MotorControllerGroup rightGroup = new MotorControllerGroup(Right_1, Right_2, Right_3);

    public final DifferentialDrive driveTrain = new DifferentialDrive(leftGroup, rightGroup);

    public DriveSubsystem() {
       // rearLeft.setInverted(true);
       // frontRight.setInverted(true);
    }

    public void drive(double ySpeed, double xSpeed) {
        // ySpeed, xSpeed, and zRotation are all supplier values from the joystick
        // object
       try(DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup)){
           drive.arcadeDrive(xSpeed, ySpeed);
       }
    }

}
