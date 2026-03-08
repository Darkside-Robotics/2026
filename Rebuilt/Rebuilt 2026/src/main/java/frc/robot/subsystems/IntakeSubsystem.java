// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.config.FeedForwardConfig;
//import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public static final class ArmConstants {
        public static final boolean ArmInverted = true;

        public static final class PID {
            public static final double P = 0.5;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final class Motor {
            public static final int MotorPort = 18;
            public static final int CurrentFreeLimit = 20;
            public static final int CurrentStalledLimit = 20;
            public static final int Power = 10;
        }
    }

    private final SparkMax armMotor;
    private final SparkMaxConfig armMotorConfig;
    private final SparkClosedLoopController armController;
    private double armPosition = 0.0;
    private double extendedArmPosition = 0.0;
    private boolean armOut = false;

    // wheels:
    private SparkMax intakeWheelMotor;
    private SparkMaxConfig intakeWheelMotorConfig;
    private SparkClosedLoopController intakeWheelController;
    private double intakeWheelSpeed = 0.0;
    private double spinningIntakeWheelSpeed = 900.0;
    private boolean spin = false;

    public static final class IntakeWheelConstants {
        public static final class PID {
            public static final double P = 0.0005;
            public static final double I = 0;
            public static final double D = 0.012;
        }

        public static final class Motor {
            public static final int MotorPort = 20;
            public static final int CurrentFreeLimit = 20;
            public static final int CurrentStalledLimit = 20;
            public static final int Power = 10;
        }
    }

    private final double kClosedLoopRampRate = 5;

    /** Creates a new Subsystem. */
    public IntakeSubsystem() {

        armMotor = new SparkMax(ArmConstants.Motor.MotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(IdleMode.kBrake)
                .closedLoopRampRate(kClosedLoopRampRate);
        armMotorConfig.closedLoop.pid(ArmConstants.PID.P,
                ArmConstants.PID.I, ArmConstants.PID.D)
                .maxOutput(0.8);
        armMotorConfig.encoder.positionConversionFactor(1);

        armMotorConfig.smartCurrentLimit(ArmConstants.Motor.CurrentStalledLimit,
                ArmConstants.Motor.CurrentFreeLimit);
        armMotor.configure(armMotorConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armController = armMotor.getClosedLoopController();
        armMotor.getEncoder().setPosition(0);

        // /*****************************************************************************************/
        intakeWheelMotor = new SparkMax(IntakeWheelConstants.Motor.MotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        intakeWheelMotorConfig = new SparkMaxConfig();
        intakeWheelMotorConfig.idleMode(IdleMode.kCoast)
                .closedLoopRampRate(kClosedLoopRampRate);
        intakeWheelMotorConfig.closedLoop
                .pid(IntakeWheelConstants.PID.P, IntakeWheelConstants.PID.I, IntakeWheelConstants.PID.D)
                .maxOutput(0.8).feedForward.kV(1.0 / 917.0);
        intakeWheelMotorConfig.encoder.velocityConversionFactor(1.0 / 2.0 / 2.0 / 3.0);

        intakeWheelMotorConfig.smartCurrentLimit(IntakeWheelConstants.Motor.CurrentStalledLimit,
                IntakeWheelConstants.Motor.CurrentFreeLimit);
        intakeWheelMotor.configure(intakeWheelMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        intakeWheelController = intakeWheelMotor.getClosedLoopController();

        intakeWheelMotor.getEncoder().setPosition(0);

    }

    /* Arm ACTIONS */
    public void setArmPosition(double armPosition) {
        this.armPosition = armPosition;
    }

    /* FLYWHEEL ACTIONS */
    public void setIntakeWheelSpeed(double intakeWheelSpeed) {
        this.intakeWheelSpeed = intakeWheelSpeed;
    }

    public double calculateFlywheelSpeed(double distance) {
        return 0;
    }

    @Override
    public void periodic() {
        if (armOut == true && spin == true) {

            armController.setSetpoint((ArmConstants.ArmInverted ? -1 : 1) * extendedArmPosition, ControlType.kPosition);
            intakeWheelController.setSetpoint(spinningIntakeWheelSpeed, ControlType.kVelocity);
        } else {
            if (armOut == true) {
                armController.setSetpoint((ArmConstants.ArmInverted ? -1 : 1) * extendedArmPosition,
                        ControlType.kPosition);
            } else {
                armController.setSetpoint(0, ControlType.kPosition);
            }
            intakeWheelController.setSetpoint(0, ControlType.kVelocity);
        }
        SmartDashboard.putNumber("Arm Position", (ArmConstants.ArmInverted ? -1 : 1) * armPosition);
        SmartDashboard.putNumber("Intake Wheel Speed", intakeWheelSpeed);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public Command ArmInIncrementCmd() {
        return runOnce(
                () -> {
                    setArmPosition(armPosition - 1);
                });
    }

    public Command ArmOutIncrementCmd() {
        return runOnce(
                () -> {
                    setArmPosition(armPosition + 1);
                });
    }

    public Command IntakeToggleCmd() {
        return runOnce(
                () -> {
                    armOut = !armOut;
                    spin = spin && armOut;
                });
    }

    public Command SpinIntakeWheelUpCmd() {
        return runOnce(
                () -> {
                    setIntakeWheelSpeed(intakeWheelSpeed + 1);
                });
    }

    public Command SpinIntakeWheelDownCmd() {
        return runOnce(
                () -> {

                    setIntakeWheelSpeed(intakeWheelSpeed - 1);
                });
    }

    public Command IntakeOnCmd() {
        return runOnce(
                () -> {
                    armOut = true;
                    spin = true;
                });
    }

    
    public Command IntakeOffCmd() {
        return runOnce(
                () -> {
                    armOut = true;
                    spin = false;
                });
    }
}