package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    /*
     * gettargetvalidity() returns 1 if a target is detected, 0 if no target is detected
     */
    public double getTargetValidity(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    }

    /*
     * gettargetx() returns the horizontal offset from the crosshair to the target
     */
    public double getTargetX(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    /*
     * gettargety() returns the vertical offset from the crosshair to the target
     */
    public double getTargetY(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    /*
     * gettargetarea() returns the area of the target
     */
    public double getTargetArea(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }
    
}
