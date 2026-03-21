package frc.robot.commands;

import java.util.Date;

//import static edu.wpi.first.units.Units.Rotation;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
//import frc.robot.library.field.FieldColor;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveLeftGentlyCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private final TimedRobot robot;

    private final ProfiledPIDController turningPidController;
    private final long milliseconds; // TIME TO END MOVING
    private long endMoving; // TIME TO END MOVING
    private boolean finished = false;

    public MoveLeftGentlyCmd(SwerveSubsystem swerveSubsystem, long milliseconds,
            TimedRobot robot) {
        this.robot = robot;
        this.swerveSubsystem = swerveSubsystem;

        this.turningPidController = new ProfiledPIDController(
                4.2, 0, 0.07, AutoConstants.kThetaControllerConstraints);
        this.turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
        this.milliseconds = milliseconds;
    }

    @Override
    public void initialize() {
        finished = false;
        this.endMoving = (new Date()).getTime() + milliseconds;
        swerveSubsystem.drive(.25, 0, 0, false, robot.getPeriod());
    }

    @Override
    public void execute() {
        if ((new Date()).getTime() > endMoving) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}