package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase{
    private final CANSparkMax driveMotorLeftMaster = new CANSparkMax(Constants.DrivetrainConstants.driveMotorLeftMaster, MotorType.kBrushless);
    private final CANSparkMax driveMotorLeftSlave = new CANSparkMax(Constants.DrivetrainConstants.driveMotorLeftSlave, MotorType.kBrushless);
    private final Victor driveMotorLeftVictor = new Victor(Constants.DrivetrainConstants.driveMotorLeftVictor);

    private final CANSparkMax driveMotorRightMaster = new CANSparkMax(Constants.DrivetrainConstants.driveMotorRightMaster, MotorType.kBrushless);
    private final CANSparkMax driveMotorRightSlave = new CANSparkMax(Constants.DrivetrainConstants.driveMotorRightSlave, MotorType.kBrushless);
    private final Victor driveMotorRightVictor = new Victor(Constants.DrivetrainConstants.driveMotorRightVictor);

    /*
     * DrivetrainSubsystem() is the constructor for the DrivetrainSubsystem class
     * 
     * This constructor sets the right slave motor to follow the right master motor, and the left slave motor to follow the left master motor
     * be mind that the victors cannot directly follow the sparkmaxes, so we have to set the output manually
     */
    public DrivetrainSubsystem() {
        driveMotorLeftSlave.follow(driveMotorLeftMaster);
        driveMotorRightSlave.follow(driveMotorRightMaster);
    }

    /*
     * arcadeDrive() is the method that drives the robot
     * 
     * This method takes in two doubles, forward and rotation, and sets the motor outputs accordingly
     * 
     * The left motor output is the forward value plus the rotation value
     * The right motor output is the forward value minus the rotation value
     *
     */
    public void arcadeDrive(double forward, double rotation){

        double leftMotorOutput = forward + rotation;
        double rightMotorOutput = forward - rotation;

        driveMotorLeftMaster.set(leftMotorOutput);
        driveMotorRightMaster.set(rightMotorOutput);
        driveMotorLeftVictor.set(leftMotorOutput);
        driveMotorRightVictor.set(rightMotorOutput);

    }
}
