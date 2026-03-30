package frc.robot.commands;

import java.util.Date;

//import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.JoystickConstants;
//import frc.robot.library.field.FieldColor;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurnToShootCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final TargetingSubsystem targetingSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final TimedRobot robot;
    private final ProfiledPIDController turningPidController;

    private final long milliseconds; // TIME TO END MOVING
    private long endMoving; // TIME TO END MOVING
    private boolean finished = false;

    public TurnToShootCmd(TargetingSubsystem targetingSubsystem, SwerveSubsystem swerveSubsystem,
            TurretSubsystem turretSubsystem, long milliseconds,
            TimedRobot robot) {

        this.robot = robot;
        this.turretSubsystem = turretSubsystem;
        this.milliseconds = milliseconds;

        this.swerveSubsystem = swerveSubsystem;
        this.targetingSubsystem = targetingSubsystem;

        this.turningPidController = new ProfiledPIDController(
                4.2, 0, 0.07, AutoConstants.kThetaControllerConstraints);
        this.turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        if(turretSubsystem != null)
            addRequirements(turretSubsystem);

        addRequirements(swerveSubsystem);
        addRequirements(targetingSubsystem);
    }

    @Override
    public void initialize() {
        finished = false;
        this.endMoving = (new Date()).getTime() + milliseconds;
        this.turningPidController.reset(swerveSubsystem.getPose().getRotation().getRadians());
        
    }

    @Override
    public void execute() {

        Pose2d robotPose2d = swerveSubsystem.getPose();
        Rotation2d targetRotation2d = targetingSubsystem.getTargetRotation2d();
        double turningSpeed = this.turningPidController.calculate(robotPose2d.getRotation().getRadians(),
                targetRotation2d.getRadians());

                
        SmartDashboard.putNumber("robotCurrentTargetRotation", targetRotation2d.getDegrees());
        SmartDashboard.putNumber("robotCurrentAngle", robotPose2d.getRotation().getDegrees());
        SmartDashboard.putNumber("robotTurningSpeed", turningSpeed);
        SmartDashboard.putBoolean("Shoot At Setpoint", this.turningPidController.atSetpoint());
        swerveSubsystem.drive(0, 0, turningSpeed, true, robot.getPeriod());

        if (this.turningPidController.atGoal() || Math.abs(turningSpeed) < 0.08) {;
            if(turretSubsystem != null)
                turretSubsystem.startFiring();
        }
        if ((new Date()).getTime() > endMoving)
        {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        if(turretSubsystem != null)
            turretSubsystem.stopFiring();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}