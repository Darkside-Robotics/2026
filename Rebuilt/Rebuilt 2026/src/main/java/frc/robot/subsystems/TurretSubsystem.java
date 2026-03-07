// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TurretSubsystem.TurretConstants.FlyWheel;
import frc.robot.subsystems.TurretSubsystem.TurretConstants.Hood;

public class TurretSubsystem extends SubsystemBase {
  // private final SparkMax turntableMotor;
  // private final SparkMaxConfig turntableMotorConfig;
  // private final SparkClosedLoopController turntableController;

  private SparkFlex leadflywheelMotor;
  private SparkFlexConfig leadflywheelMotorConfig;
  private SparkFlex followflywheelMotor;
  private SparkFlexConfig followflywheelMotorConfig;
  private SparkClosedLoopController flywheelController;

  private SparkMax hoodMotor;
  private SparkMaxConfig hoodMotorConfig;
  private SparkClosedLoopController hoodController;

  private double hoodAngle = 0.0;

  private double flywheelSpeed = 0.0;
  private double flywheelSpeedFactor = 50.0;

  // private double turnTableHomeAngle = 0.0;
  // private double turnTableAngle = 0.0;
  // private boolean intakeExtended = false;

  public static final class TurretConstants {
    public static final class Turntable {
      public static final class PID {
        public static final int P = 1;
        public static final int I = 0;
        public static final int D = 0;
      }

      public static final class Motor {
        public static final int MotorPort = 0;
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
      }
    }

    public static final class Hood {
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

    public static final class FlyWheel {
      public static final class PID {
        public static final double P = 0.0001;
        public static final double I = 0;
        public static final double D = 0;
        public static final double FF = (1.0 / 565.0);
      }

      public static final class LeadMotor {
        public static final int MotorPort = 13; // USED TO BE 14 BUT MOTOR WENT BAD
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
      }

      public static final class FollowMotor {
        public static final int MotorPort = 14; // disabled
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
      }
    }
  }

  private final double kClosedLoopRampRate = 5;

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem() {

    /*
     * turntableMotor = new SparkMax(TurretConstants.Turntable.Motor.MotorPort,
     * com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
     * turntableMotorConfig = new SparkMaxConfig();
     * turntableMotorConfig.idleMode(IdleMode.kBrake)
     * .closedLoopRampRate(kClosedLoopRampRate);
     * turntableMotorConfig.closedLoop.pid(TurntablePIDConstants.kP,
     * TurntablePIDConstants.kI, TurntablePIDConstants.kD)
     * .maxOutput(0.3);
     * turntableMotorConfig.encoder.positionConversionFactor((2 * Math.PI) * 1.25);
     * 
     * turntableMotorConfig.smartCurrentLimit(TurretConstants.Turntable.Motor.
     * CurrentStalledLimit,
     * TurretConstants.Turntable.Motor.CurrentFreeLimit);
     * turntableMotor.configure(turntableMotorConfig,
     * ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     * 
     * turntableController = turntableMotor.getClosedLoopController();
     * turntableMotor.getEncoder().setPosition(0);
     */

    leadflywheelMotor = new SparkFlex(TurretConstants.FlyWheel.LeadMotor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    leadflywheelMotorConfig = new SparkFlexConfig();
    leadflywheelMotorConfig.closedLoop.pid(FlyWheel.PID.P, FlyWheel.PID.I, FlyWheel.PID.D).outputRange(-0.8, 0.8);
    leadflywheelMotorConfig.closedLoop.feedForward.kV(FlyWheel.PID.FF);
    leadflywheelMotorConfig.idleMode(IdleMode.kCoast).closedLoopRampRate(kClosedLoopRampRate);
    leadflywheelMotorConfig.encoder.velocityConversionFactor(1);
    leadflywheelMotorConfig.smartCurrentLimit(TurretConstants.FlyWheel.LeadMotor.CurrentStalledLimit,
        TurretConstants.FlyWheel.LeadMotor.CurrentFreeLimit);
    leadflywheelMotor.configure(leadflywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    followflywheelMotor = new SparkFlex(TurretConstants.FlyWheel.FollowMotor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    followflywheelMotorConfig = new SparkFlexConfig();
    followflywheelMotorConfig.idleMode(IdleMode.kCoast).follow(FlyWheel.LeadMotor.MotorPort, true);
    followflywheelMotorConfig.encoder.velocityConversionFactor(1);
    followflywheelMotorConfig.smartCurrentLimit(TurretConstants.FlyWheel.LeadMotor.CurrentStalledLimit,
        TurretConstants.FlyWheel.LeadMotor.CurrentFreeLimit);
    followflywheelMotor.configure(leadflywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    flywheelController = leadflywheelMotor.getClosedLoopController();

    /*****************************************************************************************/
    hoodMotor = new SparkMax(TurretConstants.Hood.Motor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    hoodMotorConfig = new SparkMaxConfig();
    hoodMotorConfig.idleMode(IdleMode.kBrake)
        .closedLoopRampRate(kClosedLoopRampRate);
    hoodMotorConfig.closedLoop.pid(Hood.PID.P, Hood.PID.I, Hood.PID.D).maxOutput(0.3);
    hoodMotorConfig.encoder.positionConversionFactor(2);

    hoodMotorConfig.smartCurrentLimit(TurretConstants.Hood.Motor.CurrentStalledLimit,
        TurretConstants.Hood.Motor.CurrentFreeLimit);
    hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hoodController = hoodMotor.getClosedLoopController();

    hoodMotor.getEncoder().setPosition(0);

  }

  /* ACTIONS */
  /* TURN TABLE ACTIONS */
  public void setHoodAngle(double angle) {
    this.hoodAngle = angle;
  }

  /* TURN TABLE ACTIONS */
  // public void rotateToAngle(double angle) {
  // turnTableAngle = angle;
  // }

  // public void setHome() {
  // turnTableHomeAngle = turntableMotor.getEncoder().getPosition();
  // }

  /* FLYWHEEL ACTIONS */
  public void setFlywheelSpeed(double flywheelSpeed) {
    this.flywheelSpeed = flywheelSpeed;
  }

  public double calculateFlywheelSpeed(double distance) {
    return 0;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Flywheel Speed", flywheelSpeed);
    flywheelController.setSetpoint(flywheelSpeed * -1, ControlType.kVelocity);

    hoodMotor.set(hoodAngle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // /* WHEEL ACTIONS */
  // public void setWheelSpeed(double wheelSpeed) {
  // this.wheelSpeed=wheelSpeed;
  // }

  // public void shootSpeed() {
  // setShooterSpeed(0);
  // }

  // public void stopShooting() {
  // setWheelSpeed(0);

  // }

  // /* ROTATING ACTIONS */
  // public void setRotateSpeed(double angleInDegrees) {
  // this.armAngle=angleInDegrees;
  // }

  // public void rotateTurret() {
  // setArmAngle(ArmConstant.ARM_OUT_ANGLE);
  // }

  // /* HOOD ACTIONS */
  // public void setHoodAngle(double angleInDegrees) {
  // this.armAngle=angleInDegrees;
  // }

  public Command HoodUpCmd() {
    return runOnce(
        () -> {
          setHoodAngle(hoodAngle + 1);
        });
  }

  public Command HoodDownCmd() {
    return runOnce(
        () -> {
          setHoodAngle(hoodAngle - 1);
        });
  }

  public Command SpinFlywheelUpCmd() {
    return runOnce(
        () -> {
          setFlywheelSpeed(flywheelSpeed + flywheelSpeedFactor);
        });
  }

  public Command SpinFlywheelDownCmd() {
    return runOnce(
        () -> {

          setFlywheelSpeed(flywheelSpeed - flywheelSpeedFactor);
        });
  }

}