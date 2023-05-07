package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.print.CancelablePrintJob;

import com.fasterxml.jackson.annotation.Nulls;
import com.revrobotics.*;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax driveLeftMaster = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax driveLeftSlave1 = new CANSparkMax(14, MotorType.kBrushless);
    private final Victor driveLeftSlave2 = new Victor(15);

    private final CANSparkMax driveRightMaster = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax driveRightSlave1 = new CANSparkMax(9, MotorType.kBrushless);
    private final Victor driveRightSlave2 = new Victor(2);


    // configure slave follow master
    public DriveSubsystem() {  
        driveLeftSlave1.follow(driveLeftMaster);
        driveRightSlave1.follow(driveRightMaster);
    }

    /*
     * private final Spark driveLeftMotor = new Spark(0);
     * private final Spark driveRightMotor = new Spark(1);
     * private final Encoder driveLeftEncoder = new Encoder(0, 1);
     * private final Encoder driveRightEncoder = new Encoder(2, 3);
     */

    // private final double kEncoderTick2Meter = 1.0/4096.0*0.128*Math.PI;

    // public double getEncoderMeters(){
    // return (driveLeftEncoder.get() + driveRightEncoder.get()) / 2.0 *
    // kEncoderTick2Meter;
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Drive encoder value: ", getEncoderMeters());
    }

    public void setMotorOutput(double left, double right) {
        driveLeftMaster.set(left);
        driveRightMaster.set(right);

        // victor cannot follow sparkmax, so let them run at same rate
        driveLeftSlave2.set(left);
        driveRightSlave2.set(right);
    }

}
