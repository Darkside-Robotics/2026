package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class GyroResetCmd extends Command {

    static DriveSubsystem driveSubsystem;
    
    public GyroResetCmd(DriveSubsystem swerveSubsystem_in){
        driveSubsystem = swerveSubsystem_in;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveSubsystem.zeroHeading();
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
