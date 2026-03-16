package frc.robot.commands;

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

public class TurnToShootCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final TargetingSubsystem targetingSubsystem;
    
    private final TimedRobot robot;

    private final ProfiledPIDController turningPidController;

    double targetDistance = 10;
    Rotation2d targetRotation2d = Rotation2d.fromDegrees(180);


    public TurnToShootCmd(TargetingSubsystem targetingSubsystem,SwerveSubsystem swerveSubsystem,
            TimedRobot robot) {
        this.robot = robot;
        this.swerveSubsystem = swerveSubsystem;
        this.targetingSubsystem = targetingSubsystem;

        this.turningPidController = new ProfiledPIDController(
                4.2, 0, 0.07, AutoConstants.kThetaControllerConstraints);
        this.turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
        addRequirements(targetingSubsystem);
    }

    @Override
    public void initialize() {
    }

    private double calculateAutoRotation() {
        Pose2d robotPose2d = swerveSubsystem.getPose();

        Rotation2d targetRotation2d = targetingSubsystem.getTargetRotation2d();

        double turningSpeed = this.turningPidController.calculate(robotPose2d.getRotation().getRadians(),
                targetRotation2d.getRadians());

        SmartDashboard.putNumber("robotCurrentAngle", robotPose2d.getRotation().getDegrees());
        SmartDashboard.putNumber("robotTurningSpeed", turningSpeed);

        return turningSpeed;
    }

    @Override
    public void execute() {

 Pose2d robotPose2d = swerveSubsystem.getPose();

        Rotation2d targetRotation2d = targetingSubsystem.getTargetRotation2d();

        double turningSpeed = this.turningPidController.calculate(robotPose2d.getRotation().getRadians(),
                targetRotation2d.getRadians());

        SmartDashboard.putNumber("robotCurrentAngle", robotPose2d.getRotation().getDegrees());
        //SmartDashboard.putNumber("robotTargetAngle", targetRotation2d.getDegrees());
        SmartDashboard.putNumber("robotTurningSpeed", turningSpeed);
        SmartDashboard.putBoolean("Shoot At Setpoint", this.turningPidController.atSetpoint());

        if (Math.abs(turningSpeed) < 0.08) {
            end(false);
        }
        else {
            double autoRotation = calculateAutoRotation();
                    swerveSubsystem.drive(0, 0, turningSpeed, true, robot.getPeriod());
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}