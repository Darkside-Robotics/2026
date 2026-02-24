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

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final SparkMax turntableMotor;
  private final SparkMaxConfig turntableMotorConfig;
  private final SparkClosedLoopController turntableController;

  private SparkMax flywheelMotor;
  private SparkMaxConfig flywheelMotorConfig;  
  private SparkClosedLoopController flywheelController;

  private SparkMax hoodMotor;
  private SparkMaxConfig hoodMotorConfig;
  private SparkClosedLoopController hoodController;

  private static class TurntablePIDConstants {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
  }

  private double flywheelSpeed = 0.0;
  private double turnTableHomeAngle = 0.0;
  private double turnTableAngle = 0.0;
  private boolean intakeExtended = false;

  public static final class TurretConstants {
    public static final class Turntable {
      public static final class Motor {
        public static final int MotorPort = 9;
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
      }
    }

    public static final class Hood {
      public static final class Motor {
        public static final int MotorPort = 0;
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
      }
    }

    public static final class FlyWheel {
      public static final class Motor {
        public static final int MotorPort = 0;
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
      }
    }
  }

  private final double kClosedLoopRampRate = 0;

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem() {

    turntableMotor = new SparkMax(TurretConstants.Turntable.Motor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    turntableMotorConfig = new SparkMaxConfig();
    turntableMotorConfig.idleMode(IdleMode.kBrake)
        .closedLoopRampRate(kClosedLoopRampRate);
    turntableMotorConfig.closedLoop.pid(TurntablePIDConstants.kP, TurntablePIDConstants.kI, TurntablePIDConstants.kD)
        .maxOutput(0.3);
    turntableMotorConfig.encoder.positionConversionFactor((2 * Math.PI) * 1.25);

    turntableMotorConfig.smartCurrentLimit(TurretConstants.Turntable.Motor.CurrentStalledLimit,
        TurretConstants.Turntable.Motor.CurrentFreeLimit);
    turntableMotor.configure(turntableMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turntableController = turntableMotor.getClosedLoopController();
    turntableMotor.getEncoder().setPosition(0);

    /*
     * flywheelMotor = new SparkMax(TurretConstants.FlyWheel.Motor.MotorPort,
     * com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
     * flywheelMotorConfig = new SparkMaxConfig();
     * flywheelMotorConfig.idleMode(IdleMode.kBrake);
     * flywheelMotorConfig.encoder.velocityConversionFactor(1);
     * flywheelMotorConfig.smartCurrentLimit(TurretConstants.FlyWheel.Motor.
     * CurrentStalledLimit,
     * TurretConstants.FlyWheel.Motor.CurrentFreeLimit);
     * flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters,
     * PersistMode.kPersistParameters);
     * 
     * hoodMotor = new SparkMax(TurretConstants.Hood.Motor.MotorPort,
     * com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
     * hoodMotorConfig = new SparkMaxConfig();
     * hoodMotorConfig.idleMode(IdleMode.kBrake)
     * .closedLoopRampRate(kClosedLoopRampRate);
     * hoodMotorConfig.closedLoop.pid(IntakePIDConstants.kP, IntakePIDConstants.kI,
     * IntakePIDConstants.kD).maxOutput(0.3);
     * hoodMotorConfig.encoder.positionConversionFactor((2 * Math.PI) * 1.25);
     * 
     * hoodMotorConfig.smartCurrentLimit(TurretConstants.Hood.Motor.
     * CurrentStalledLimit,
     * TurretConstants.Hood.Motor.CurrentFreeLimit);
     * hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters,
     * PersistMode.kPersistParameters);
     * 
     * hoodController = hoodMotor.getClosedLoopController();
     * 
     * hoodMotor.getEncoder().setPosition(0);
     */
  }

  /* ACTIONS */
  /* TURN TABLE ACTIONS */
  public void rotateToAngle(double angle) {
    turnTableAngle = angle;
  }

  public void setHome() {
    turnTableHomeAngle = turntableMotor.getEncoder().getPosition();
  }

  /* FLYWHEEL ACTIONS */
  public void setFlywheelSpeed(double flywheelSpeed) {
    this.flywheelSpeed = flywheelSpeed;
  }

  public double calculateFlywheelSpeed(double distance) {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

}
