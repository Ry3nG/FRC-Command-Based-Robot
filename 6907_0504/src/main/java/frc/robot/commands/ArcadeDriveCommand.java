package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase{

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;

    public ArcadeDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction){
        this.driveSubsystem = driveSubsystem;
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        addRequirements(driveSubsystem);
    }

    @Override 
    public void initialize(){
        System.out.println("ArcadeDriveCommand initialized");
    }

    @Override
    public void execute(){
        double realTimeSPeed = speedFunction.get();
        double realTimeTurn = turnFunction.get();        

        double leftMotorOutput = realTimeSPeed + realTimeTurn;
        double rightMotorOutput = realTimeSPeed - realTimeTurn;

        // safety threhold, better revamp with motion profiling
        leftMotorOutput = 0.3*leftMotorOutput*Math.abs(leftMotorOutput);
        rightMotorOutput = 0.3*rightMotorOutput*Math.abs(rightMotorOutput);

        driveSubsystem.setMotorOutput(leftMotorOutput, rightMotorOutput);
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("ArcadeDriveCommand ended");
    }

    @Override
    public boolean isFinished(){
        return false;
    }


    
}
