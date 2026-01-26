package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;

public class ClimbDownCmd extends Command {

 
    private final ClimbingSubsystem climbingSubsystem;

    public ClimbDownCmd (ClimbingSubsystem climbingSubsystem) {
        this.climbingSubsystem = climbingSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climbingSubsystem.goDown();
   }

    @Override
    public void end(boolean interrupted) {
        climbingSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
