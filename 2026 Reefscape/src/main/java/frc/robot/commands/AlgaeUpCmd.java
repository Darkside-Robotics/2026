
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class AlgaeUpCmd extends Command {

 
    private final AlgaeArmSubsystem algaeSubsystem;

    public AlgaeUpCmd (AlgaeArmSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        algaeSubsystem.goUp();
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
