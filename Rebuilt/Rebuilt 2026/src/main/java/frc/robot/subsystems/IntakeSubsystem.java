// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax armMotor;
  private final SparkMax wheelMotor;
  private final SparkMaxConfig armMotorConfig;
  private final SparkMaxConfig wheelMotorConfig;

  private final SparkClosedLoopController armController;

  private static class IntakePIDConstants {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kFF = 0;
  }

  private double wheelSpeed = 0.0;
  private double armAngle = 0.0;
  private boolean intakeExtended = false;

  public static final class IntakeConstants {
    public static final class Arms {
      public static final class Motor {
        public static final int MotorPort = 0;
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
      }
    }

    public static final class Wheels {
      public static final class Motor {
        public static final int MotorPort = 0;
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
      }
    }
  }

  private static class WheelConstant {
    public static double FAST_SPEED = 0;
    public static double SLOW_SPEED = 0;
  }

  private static class ArmConstant {
    public static double ARM_OUT_ANGLE = 0;
    public static double ARM_IN_ANGLE = 0;
  }

  private final double kClosedLoopRampRate = 0;

  public IntakeSubsystem() {
    armMotor = new SparkMax(IntakeConstants.Arms.Motor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode(IdleMode.kBrake)
        .closedLoopRampRate(kClosedLoopRampRate);
    armMotorConfig.closedLoop.pid(IntakePIDConstants.kP, IntakePIDConstants.kI, IntakePIDConstants.kD).maxOutput(0.3);
    armMotorConfig.encoder.positionConversionFactor((2 * Math.PI) * 1.25);

    armMotorConfig.smartCurrentLimit(IntakeConstants.Arms.Motor.CurrentStalledLimit,
        IntakeConstants.Arms.Motor.CurrentFreeLimit);
    armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armController = armMotor.getClosedLoopController();

    armMotor.getEncoder().setPosition(0);

    wheelMotor = new SparkMax(IntakeConstants.Wheels.Motor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    wheelMotorConfig = new SparkMaxConfig();
    wheelMotorConfig.idleMode(IdleMode.kBrake);
    wheelMotorConfig.encoder.velocityConversionFactor(1);
    wheelMotorConfig.smartCurrentLimit(IntakeConstants.Wheels.Motor.CurrentStalledLimit,
        IntakeConstants.Wheels.Motor.CurrentFreeLimit);
    wheelMotor.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command IntakeMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /* ARM ACTIONS */
  public void setArmAngle(double angleInDegrees) {
    this.armAngle = angleInDegrees;
  }

  public void extendArm() {
    setArmAngle(ArmConstant.ARM_OUT_ANGLE);
  }

  public void retractArm() {
    setArmAngle(ArmConstant.ARM_IN_ANGLE);
  }

  /* WHEEL ACTIONS */
  public void setWheelSpeed(double wheelSpeed) {
    this.wheelSpeed = wheelSpeed;
  }

  public void spinWheelFast() {
    setWheelSpeed(WheelConstant.FAST_SPEED);
  }

  public void spinWheelSlow() {
    setWheelSpeed(WheelConstant.SLOW_SPEED);
  }

  public void stopWheel() {
    setWheelSpeed(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /* COMMANDS */
  public Command SpinFasterCommand() {
    return runOnce(
        () -> {
          setWheelSpeed(wheelSpeed + 1);
        });
  }

  public Command SpinSlowerCommand() {
    return runOnce(
        () -> {
          setWheelSpeed(wheelSpeed - 1);
        });
  }

  public Command IncreaseArmAngleCommand() {
    return runOnce(
        () -> {
          setArmAngle(armAngle + 1);
        });
  }

  public Command DecreaseArmAngleCommand() {
    return runOnce(
        () -> {
          setArmAngle(armAngle - 1);
        });
  }

  public Command ToggleIntakeCommand() {
    return runOnce(
        () -> {
          // Extend arm if it is in, and spin wheels to slow, or pull arm in and stop
          // wheels,
          intakeExtended = !intakeExtended;
          if (intakeExtended) {
            extendArm();
            spinWheelSlow();
          } else {
            retractArm();
            stopWheel();
          }

        });
  }
}
