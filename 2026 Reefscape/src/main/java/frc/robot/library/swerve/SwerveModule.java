package frc.robot.library.swerve;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModule {

    public static final class SwerveModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDistancePerWheelRotation = Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * kDistancePerWheelRotation;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kDriveMotorFeedforwardVoltsPerRotationsPerMinute = 1 / 473;
        public static final double kDriveMotorFeedforwardVoltsPerRotationsPerSecond = 1 / (473 / 60);
        public static final double kDriveMotorFeedforwardVoltsPerMetersPerSecond = 1 / (473 / 60 * kDistancePerWheelRotation * kDriveMotorGearRatio);
        public static final double kModuleMaxAngularVelocity = Math.PI;
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
    }

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    // private final SparkClosedLoopController

    // private final RelativeEncoder driveEncoder;
    // private final RelativeEncoder turningEncoder;

    private final SparkMaxConfig turningMaxConfig;
    private final SparkMaxConfig driveMaxConfig;

    private final PIDController turningPidController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
    /*
     * private final ProfiledPIDController turningPidController = new
     * ProfiledPIDController(
     * SwerveModuleConstants.kPTurning,
     * 0,
     * 0,
     * new TrapezoidProfile.Constraints(
     * SwerveModuleConstants.kModuleMaxAngularVelocity,
     * SwerveModuleConstants.kModuleMaxAngularAcceleration));
     */

    private final AnalogEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private SwerveModuleState targetState = null;

    private final String name;
    private boolean disabled;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name,
            boolean disabled) {

        this.disabled = disabled;
        this.name = name;

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new AnalogEncoder(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        driveMaxConfig = new SparkMaxConfig();

        driveMaxConfig.idleMode(IdleMode.kBrake);
        driveMaxConfig
                .smartCurrentLimit(DriveConstants.CurrentStalledLimit, DriveConstants.CurrentFreeLimit);
        driveMaxConfig.closedLoop.pidf(0.025, 0, 0.05,
                SwerveModuleConstants.kDriveMotorFeedforwardVoltsPerMetersPerSecond);
        driveMaxConfig
                .inverted(driveMotorReversed);
        driveMaxConfig.encoder
                .positionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveMotor.configure(driveMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turningMotor = new SparkMax(turningMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        turningMaxConfig = new SparkMaxConfig();

        turningMaxConfig.idleMode(IdleMode.kBrake);
        turningMaxConfig
                .inverted(true);
        turningMaxConfig.closedLoop.pid(0.05, 0, 0.025);

        // turningMaxConfig.signals.primaryEncoderPositionPeriodMs(5);
        turningMaxConfig.encoder
                .positionConversionFactor(SwerveModuleConstants.kTurningEncoderRot2Rad)
                .velocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPM2RadPerSec);
        turningMotor.configure(turningMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    // TODO : VERIFY getDrivePosition() IS SIMILAR TO getDistance() in EXAMPLE CODE
    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, false);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean velocityControlled) {

        if (!this.disabled) {

            if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
                stop();
                return;
            }
            // Optimize the reference state to avoid spinning further than 90 degrees
            desiredState.optimize(new Rotation2d(getTurningPosition()));

            // SAVE OUR GOAL FOR REPORTING
            targetState = desiredState;

            if (true || !velocityControlled) {
                // FOR DIRECT COMMAND
                driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
                turningMotor.set(turningPidController.calculate(getTurningPosition(), desiredState.angle.getRadians()));
            } else {
                // FOR VELOCITY CONTROLLED
                driveMotor.getClosedLoopController().setReference(
                        desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                        ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                turningMotor.getClosedLoopController().setReference(
                        turningPidController.calculate(getTurningPosition(), desiredState.angle.getRadians()),
                        ControlType.kPosition, ClosedLoopSlot.kSlot0);
            }
        }
    }

    public void reportToDashboard() {
        SmartDashboard.putString("Wheel Velocity (" + this.name + ")", Double.toString(getDriveVelocity()));
        if (targetState != null) {
            SmartDashboard.putNumber("Wheel Target V (" + this.name + ")", targetState.speedMetersPerSecond);
        }
        SmartDashboard.putString("Turning Position (" + this.name + ")", Double.toString(getTurningPosition()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", getState().toString());
        SmartDashboard.putNumber("Calculation", SwerveModuleConstants.kDriveMotorFeedforwardVoltsPerMetersPerSecond);
        SmartDashboard.putNumber("Current (" + this.name + ")", driveMotor.getOutputCurrent());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    /* TURNING MOTOR ENCODER INFORMATION */
    private double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    private double getTurningPosition() {
        SmartDashboard.putString("Encoder Position(" + name + ")",
                Double.toString(turningMotor.getEncoder().getPosition()));

        return turningMotor.getEncoder().getPosition();
    }

    /* DRIVE ENCODER INFORMATION */
    private double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    private double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    /*
     * Uses the analog encoder to set the wheels to point straight forward
     */
    private void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());

        try {
            while (Math.abs(turningMotor.getEncoder().getPosition() - getAbsoluteEncoderRad()) > .001) {
                Thread.sleep(100);
            }
        } catch (Exception e) {
            SmartDashboard.putString("Exception Resetting Swerve Module Encoder (" + this.name + ")", e.getMessage());
        }

    }

    /*
     * Only used in resetEncoders for initializing the wheel position to 0 (straight
     * forward)
     */
    private double getAbsoluteEncoderRad() {

        double value = absoluteEncoder.get();
        double angle = value * 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;

        SmartDashboard.putNumber("Absolute Encoder Value(" + name + ") " + absoluteEncoder.getChannel(), value);
        SmartDashboard.putNumber("Absolute Encoder Angle(" + name + ")" + absoluteEncoder.getChannel(), angle);

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
}
// */