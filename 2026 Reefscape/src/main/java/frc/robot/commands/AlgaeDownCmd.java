package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;

public class AlgaeDownCmd extends Command {

 
    private final AlgaeArmSubsystem algaeSubsystem;

    public AlgaeDownCmd (AlgaeArmSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        algaeSubsystem.goDown();
   }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
