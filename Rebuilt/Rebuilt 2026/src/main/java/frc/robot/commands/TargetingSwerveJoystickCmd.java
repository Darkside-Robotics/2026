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

public class TargetingSwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final TargetingSubsystem targetingSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Supplier<Boolean> boost;
    private final Supplier<Boolean> autoTarget;
    private double speed;
    private final TimedRobot robot;

    private final ProfiledPIDController turningPidController;

    double targetDistance = 10;
    Rotation2d targetRotation2d = Rotation2d.fromDegrees(180);


    public TargetingSwerveJoystickCmd(TargetingSubsystem targetingSubsystem,SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction,
            Supplier<Boolean> boost, Supplier<Boolean> autoTarget, TimedRobot robot) {
        this.robot = robot;
        this.swerveSubsystem = swerveSubsystem;
        this.targetingSubsystem = targetingSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.boost = boost;
        this.autoTarget = autoTarget;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

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

        // double hubX = 11.8;
        // double hubY = 4.0;

        // double yDistance = Math.abs(robotPose2d.getY() - hubY);
        // double xDistance = Math.abs(robotPose2d.getX() - hubX);
        // double relativeTargetAngle = Math.toDegrees(Math.atan(yDistance / xDistance));

        // double shootingLineX = 12.5;
        // double middleLineY = 4;

        // boolean positiveSide = shootingLineX < robotPose2d.getX();
        // boolean leftSide = middleLineY < robotPose2d.getY();

        // double correctedAngle = ((leftSide ? -1 : 1) * (positiveSide ? 180 : 0)) +
        //         ((leftSide ? 1 : -1) * relativeTargetAngle);

        // targetDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        // targetRotation2d = Rotation2d.fromDegrees(correctedAngle).rotateBy(Rotation2d.fromDegrees(180));

        Rotation2d targetRotation2d = targetingSubsystem.getTargetRotation2d();

        double turningSpeed = this.turningPidController.calculate(robotPose2d.getRotation().getRadians(),
                targetRotation2d.getRadians());

        SmartDashboard.putNumber("robotCurrentAngle", robotPose2d.getRotation().getDegrees());
        //SmartDashboard.putNumber("robotTargetAngle", targetRotation2d.getDegrees());
        SmartDashboard.putNumber("robotTurningSpeed", turningSpeed);

        return turningSpeed;
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > JoystickConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > JoystickConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > JoystickConstants.kDeadband ? turningSpeed : 0.0;

        if (boost.get()) {
            speed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        } else {
            speed = 1;
        }

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * speed;
        ySpeed = yLimiter.calculate(ySpeed) * speed;

        double autoRotation = calculateAutoRotation();
        if (autoTarget.get()) {
            turningSpeed = autoRotation;
        } else {
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        }

        swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, true, robot.getPeriod());
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
