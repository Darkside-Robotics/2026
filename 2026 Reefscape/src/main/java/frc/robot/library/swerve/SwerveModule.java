package frc.robot.library.swerve;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    // private final SparkClosedLoopController

    // private final RelativeEncoder driveEncoder;
    // private final RelativeEncoder turningEncoder;

    private final SparkMaxConfig turningMaxConfig;
    private final SparkMaxConfig driveMaxConfig;

    private final PIDController turningPidController;

    private final AnalogEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private SwerveModuleState targetState = null;

    private final String name;
    private boolean disabled;

    
    private boolean velocityControlled  = false;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name,
            boolean disabled) {

        this.disabled = disabled;
        this.name = name;

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        // try {
        absoluteEncoder = new AnalogEncoder(absoluteEncoderId);
        // } catch (Exception e) {

        // }
        driveMotor = new SparkMax(driveMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        driveMaxConfig = new SparkMaxConfig();
        
        driveMaxConfig.idleMode(IdleMode.kBrake);
        driveMaxConfig
                .smartCurrentLimit(DriveConstants.CurrentStalledLimit, DriveConstants.CurrentFreeLimit);
                driveMaxConfig.closedLoop.pidf(0.025, 0, 0.05, ModuleConstants.kDriveMotorFeedforwardVoltsPerMetersPerSecond);
                driveMaxConfig
                .inverted(driveMotorReversed);
        driveMaxConfig.encoder
                .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveMotor.configure(driveMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turningMotor = new SparkMax(turningMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        turningMaxConfig = new SparkMaxConfig();
        
        turningMaxConfig.idleMode(IdleMode.kBrake);
        turningMaxConfig
                .inverted(true);
                turningMaxConfig.closedLoop.pid(0.05, 0, 0.025);
        
        // turningMaxConfig.signals.primaryEncoderPositionPeriodMs(5);
        turningMaxConfig.encoder
                .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turningMotor.configure(turningMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(1.0, 0.0, 0.0);

        // SparkClosedLoopController turningPidCiController =
        // turningMotor.getClosedLoopController();

        // driveEncoder = driveMotor.getEncoder();
        // turningEncoder = turningMotor.getEncoder();

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition() {
        SmartDashboard.putString("Encoder Position(" + name + ")",
                Double.toString(turningMotor.getEncoder().getPosition()));

        return turningMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.get();// getAbsolutePosition();
        // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        SmartDashboard.putNumber("Absolute Encoder Value(" + name + ") " + absoluteEncoder.getChannel(), angle);
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        SmartDashboard.putNumber("Absolute Encoder Angle(" + name + ")" + absoluteEncoder.getChannel(), angle);
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());

        try {
            while (Math.abs(turningMotor.getEncoder().getPosition() - getAbsoluteEncoderRad()) > .001) {
                Thread.sleep(100);
            }
        } catch (Exception e) {
            System.getLogger(name);
        }

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, boolean velocityControlled) {

        if (!this.disabled) {

            if (Math.abs(state.speedMetersPerSecond) < 0.001) {
                stop();
                return;
            }
            state = SwerveModuleState.optimize(state, getState().angle);
            targetState=state;
            
           if(true || !velocityControlled)
           {
                //FOR DIRECT COMMAND
                driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
                turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
           }else{
           //FOR VELOCITY CONTROLLED
                driveMotor.getClosedLoopController().setReference(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                turningMotor.getClosedLoopController().setReference(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()), ControlType.kPosition, ClosedLoopSlot.kSlot0);
           }
        }
    }

    public void reportToDashboard() { 
        SmartDashboard.putString("Wheel Velocity (" + this.name + ")", Double.toString(getDriveVelocity()));
        
        if(targetState!=null)
        {

            SmartDashboard.putNumber("Wheel Target V (" + this.name + ")", targetState.speedMetersPerSecond);
        
        }
        SmartDashboard.putString("Turning Position (" + this.name + ")", Double.toString(getTurningPosition()));
       SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", getState().toString());
        
       SmartDashboard.putNumber("Calculation", ModuleConstants.kDriveMotorFeedforwardVoltsPerMetersPerSecond);
       
       SmartDashboard.putNumber("Current (" + this.name + ")", driveMotor.getOutputCurrent());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
//*/