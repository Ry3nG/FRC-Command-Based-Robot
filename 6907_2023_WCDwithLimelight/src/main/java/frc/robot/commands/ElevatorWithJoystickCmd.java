package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorWithJoystickCmd extends CommandBase {
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final DoubleSupplier target_height;

    public ElevatorWithJoystickCmd(ElevatorSubsystem elevatorSubsystem, DoubleSupplier height
        ) {
        m_ElevatorSubsystem = elevatorSubsystem;
        target_height = height;
        addRequirements(m_ElevatorSubsystem);
    }

    @Override
    public void execute() {
        double height = target_height.getAsDouble();

        m_ElevatorSubsystem.setHeight(height); 

        SmartDashboard.putNumber("Elevator Target Height", height*m_ElevatorSubsystem.ENCODER_TICKS_PER_METER);
        SmartDashboard.putNumber("Elevator Sensor Tick", m_ElevatorSubsystem.getElevatorEncoder());
    }

}
