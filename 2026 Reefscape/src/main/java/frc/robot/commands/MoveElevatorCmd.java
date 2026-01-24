package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCmd extends Command {

 
    private final ElevatorSubsystem elevatorSubsystem;

    public MoveElevatorCmd (ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
   }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
