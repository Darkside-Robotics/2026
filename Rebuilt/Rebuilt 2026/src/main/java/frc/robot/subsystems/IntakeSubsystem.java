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
        public static final class PID {
            public static final int P = 1;
            public static final int I = 0;
            public static final int D = 0;
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

    private SparkMax intakeWheelMotor;
    private SparkMaxConfig intakeWheelMotorConfig;
    private SparkClosedLoopController intakeWheelController;
    private double intakeWheelSpeed = 0.0;

    public static final class IntakeWheelConstants {
        public static final class PID {
            public static final int P = 1;
            public static final int I = 0;
            public static final int D = 0;
        }

        public static final class Motor {
            public static final int MotorPort = 19;
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
                .maxOutput(0.3);
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
        intakeWheelMotorConfig.idleMode(IdleMode.kBrake)
                .closedLoopRampRate(kClosedLoopRampRate);
        intakeWheelMotorConfig.closedLoop
                .pid(IntakeWheelConstants.PID.P, IntakeWheelConstants.PID.I, IntakeWheelConstants.PID.D).maxOutput(0.3);
        intakeWheelMotorConfig.encoder.positionConversionFactor(.05);

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

        SmartDashboard.putNumber("Intake Wheel Speed", intakeWheelSpeed);
        intakeWheelController.setSetpoint(intakeWheelSpeed * -1, ControlType.kVelocity);

        armController.setSetpoint(armPosition, ControlType.kPosition);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public Command ArmInCmd() {
        return runOnce(
                () -> {
                    setArmPosition(armPosition - 1);
                });
    }

    public Command ArmOutCmd() {
        return runOnce(
                () -> {
                    setArmPosition(armPosition + 1);
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

}