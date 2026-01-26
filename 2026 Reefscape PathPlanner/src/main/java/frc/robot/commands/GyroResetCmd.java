package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroResetCmd extends Command {

    static SwerveSubsystem swerveSubsystem;
    
    public GyroResetCmd(SwerveSubsystem swerveSubsystem_in){
        swerveSubsystem = swerveSubsystem_in;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveSubsystem.zeroHeading();
        end(true);
    }
    
    @Override
    public void end(boolean interrupted) {
        //swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
