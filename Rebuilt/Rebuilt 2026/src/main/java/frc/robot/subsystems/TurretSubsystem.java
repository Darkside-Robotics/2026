// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Date;

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
import frc.robot.subsystems.TurretSubsystem.TurretConstants.FlyWheelConstants;
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

  private double flywheelDefaultSpeed = 250.0;
  private double flywheelSpeedFactor = 50.0;

  private double flywheelSpeed = 50.0;

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
        public static final double P = 0.07;
        public static final double I = 0;
        public static final double D = 0.012;
      }

      public static final class Motor {
        public static final int MotorPort = 19;
        public static final int CurrentFreeLimit = 20;
        public static final int CurrentStalledLimit = 20;
        public static final int Power = 10;
      }
    }

    public static final class FlyWheelConstants {

      public static final double ClosedLoopRampRate = 1.0;

      public static final class PID {
        public static final double P = 0.0002;
        public static final double I = 0;
        public static final double D = 0.005;
        public static final double FF = (12.0 / 5767.0) / 13.25; // (1.0 / 565.0) / 10;
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

  private static double reduction = 0.0;
  private static final double[][] firingTable = new double[][] {
      { 1.7344, 63, 8.5 - reduction },
      { 1.7755, 63, 8.6 - reduction },
      { 1.8192, 62, 8.6 - reduction },
      { 1.8617, 62, 8.7 - reduction },
      { 1.9007, 61, 8.7 - reduction },
      { 1.9287, 63, 8.2 - reduction },
      { 1.9638, 62, 8.2 - reduction },
      { 1.9681, 63, 8 - reduction },
      { 2.0176, 63, 8.1 - reduction },
      { 2.0388, 63, 7.9 - reduction },
      { 2.0615, 60, 8.6 - reduction },
      { 2.0957, 63, 7.8 - reduction },
      { 2.122, 62, 8 - reduction },
      { 2.1753, 62, 8.1 - reduction },
      { 2.1798, 62, 7.9 - reduction },
      { 2.2353, 62, 8 - reduction },
      { 2.2614, 61, 8 - reduction },
      { 2.2829, 62, 7.9 - reduction },
      { 2.3183, 61, 8.1 - reduction },
      { 2.3401, 60, 8.1 - reduction },
      { 2.3669, 61, 8 - reduction },
      { 2.4046, 61, 7.9 - reduction },
      { 2.448, 60, 8.1 - reduction },
      { 2.4658, 61, 8 - reduction },
      { 2.4952, 61, 7.9 - reduction },
      { 2.5384, 63, 7.8 - reduction },
      { 2.5686, 62, 7.8 - reduction },
      { 2.5805, 61, 7.9 - reduction },
      { 2.6348, 62, 7.9 - reduction },
      { 2.6421, 62, 7.8 - reduction },
      { 2.68, 63, 7.8 - reduction },
      { 2.7115, 62, 7.8 - reduction },
      { 2.745, 63, 7.8 },
      { 2.7815, 62, 7.9 },
      { 2.8087, 61, 7.9 },
      { 2.8488, 62, 7.9 },
      { 2.8768, 61, 7.9 },
      { 2.903, 60, 8 },
      { 2.9172, 59, 8.1 },
      { 2.9414, 61, 7.9 },
      { 2.9732, 62, 7.9 },
      { 3.0307, 62, 7.9 },
      { 3.0489, 63, 7.9 },
      { 3.0792, 61, 8 },
      { 3.104, 60, 8 },
      { 3.139, 61, 8 },
      { 3.1646, 60, 8 },
      { 3.196, 61, 8 },
      { 3.2175, 62, 8 },
      { 3.2685, 62, 8 },
      { 3.2767, 63, 8 },
      { 3.3222, 63, 8 },
      { 3.36, 60, 8.1 },
      { 3.3853, 61, 8.1 },
      { 3.4141, 60, 8.1 },
      { 3.4361, 61, 8.1 },
      { 3.4847, 61, 8.1 },
      { 3.4938, 62, 8.1 },
      { 3.5223, 59, 8.2 },
      { 3.5761, 59, 8.2 },
      { 3.6026, 60, 8.2 },
      { 3.6277, 59, 8.2 },
      { 3.6512, 60, 8.2 },
      { 3.6824, 58, 8.3 },
      { 3.7313, 56, 8.4 },
      { 3.7561, 55, 8.5 },
      { 3.79, 56, 8.4 },
      { 3.8193, 55, 8.5 },
      { 3.8362, 54, 8.6 },
      { 3.8802, 55, 8.5 },
      { 3.9015, 54, 8.6 },
      { 3.9386, 55, 8.5 },
      { 3.9644, 54, 8.6 },
      { 3.9942, 56, 8.5 },
      { 4.0257, 58, 8.4 },
      { 4.0468, 59, 8.4 },
      { 4.0895, 59, 8.4 },
      { 4.1048, 53, 8.7 },
      { 4.1449, 55, 8.6 },
      { 4.165, 53, 8.7 },
      { 4.1984, 55, 8.6 },
      { 4.2499, 55, 8.6 },
      { 4.2559, 58, 8.5 },
      { 4.296, 56, 8.6 },
      { 4.3433, 56, 8.6 },
      { 4.3449, 54, 8.7 },
      { 4.3965, 54, 8.7 },
      { 4.4334, 53, 8.8 },
      { 4.4574, 52, 8.9 },
      { 4.4868, 53, 8.8 },
      { 4.5147, 52, 8.9 },
      { 4.5385, 53, 8.8 },
      { 4.57, 52, 8.9 },
      { 4.5985, 54, 8.8 },
      { 4.6462, 54, 8.8 },
      { 4.6615, 58, 8.7 },
      { 4.6982, 58, 8.7 },
      { 4.7371, 55, 8.8 },
      { 4.7429, 53, 8.9 },
      { 4.7908, 53, 8.9 },
      { 4.8116, 56, 8.8 },
      { 4.8508, 56, 8.8 },
      { 4.8699, 57, 8.8 },
      { 4.9325, 54, 8.9 },
      { 4.9719, 55, 8.9 },
      { 5.0116, 55, 8.9 },
      { 5.0381, 56, 8.9 },
      { 5.0538, 57, 8.9 },
      { 5.0885, 57, 8.9 },
      { 5.1278, 54, 9 },
      { 5.1679, 54, 9 },
      { 5.2023, 55, 9 },
      { 5.2393, 55, 9 },
      { 5.2608, 56, 9 },
      { 5.2951, 56, 9 },
      { 5.3191, 53, 9.1 },
      { 5.3618, 54, 9.1 },
      { 5.3993, 54, 9.1 },
      { 5.429, 55, 9.1 },
      { 5.4637, 55, 9.1 },
      { 5.4804, 56, 9.1 },
      { 5.516, 53, 9.2 },
      { 5.5541, 53, 9.2 },
      { 5.5923, 54, 9.2 },
      { 5.6276, 54, 9.2 },
      { 5.6663, 56, 9.2 },
      { 5.6974, 56, 9.2 },
      { 5.786, 53, 9.3 },
      { 5.8199, 54, 9.3 },
      { 5.8532, 54, 9.3 },
      { 5.874, 55, 9.3 },
      { 5.9049, 55, 9.3 },
      { 5.9383, 52, 9.4 },
      { 5.9815, 53, 9.4 },
      { 6.0154, 53, 9.4 },
      { 6.0453, 54, 9.4 },
      { 6.0768, 54, 9.4 },
      { 6.0933, 55, 9.4 },
      { 6.1369, 52, 9.5 },
      { 6.1713, 52, 9.5 },
      { 6.2049, 52, 9.5 },
      { 6.2428, 53, 9.5 },
      { 6.2688, 54, 9.5 },
      { 6.2987, 54, 9.5 },
      { 6.4024, 52, 9.6 },
      { 6.4382, 53, 9.6 },
  };

  private boolean fire = false;
  private double flywheelAdjustableConstant = 2.087;

  private boolean homing = false;

  private final IndexingSubsystem indexingSubsystem;
  private final TargetingSubsystem targetingSubsystem;

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem(IndexingSubsystem indexingSubsystem, TargetingSubsystem targetingSubsystem) {
    this.indexingSubsystem = indexingSubsystem;
    this.targetingSubsystem = targetingSubsystem;

    leadflywheelMotor = new SparkFlex(TurretConstants.FlyWheelConstants.LeadMotor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    leadflywheelMotorConfig = new SparkFlexConfig();
    leadflywheelMotorConfig.closedLoop.pid(FlyWheelConstants.PID.P, FlyWheelConstants.PID.I, FlyWheelConstants.PID.D)
        .outputRange(-0.8, 0.8);
    leadflywheelMotorConfig.closedLoop.feedForward.kV(FlyWheelConstants.PID.FF);
    leadflywheelMotorConfig.idleMode(IdleMode.kCoast)
        .closedLoopRampRate(TurretConstants.FlyWheelConstants.ClosedLoopRampRate);
    leadflywheelMotorConfig.encoder.velocityConversionFactor(1.0);
    leadflywheelMotorConfig.smartCurrentLimit(TurretConstants.FlyWheelConstants.LeadMotor.CurrentStalledLimit,
        TurretConstants.FlyWheelConstants.LeadMotor.CurrentFreeLimit);
    leadflywheelMotor.configure(leadflywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    followflywheelMotor = new SparkFlex(TurretConstants.FlyWheelConstants.FollowMotor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    followflywheelMotorConfig = new SparkFlexConfig();
    followflywheelMotorConfig.idleMode(IdleMode.kCoast).follow(FlyWheelConstants.LeadMotor.MotorPort, true);
    followflywheelMotorConfig.smartCurrentLimit(TurretConstants.FlyWheelConstants.LeadMotor.CurrentStalledLimit,
        TurretConstants.FlyWheelConstants.LeadMotor.CurrentFreeLimit);
    followflywheelMotor.configure(followflywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    flywheelController = leadflywheelMotor.getClosedLoopController();

    /*****************************************************************************************/
    hoodMotor = new SparkMax(TurretConstants.Hood.Motor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    hoodMotorConfig = new SparkMaxConfig();
    hoodMotorConfig.idleMode(IdleMode.kBrake)
        .closedLoopRampRate(1);
    hoodMotorConfig.closedLoop.pid(Hood.PID.P, Hood.PID.I, Hood.PID.D).maxOutput(0.4);
    hoodMotorConfig.encoder.positionConversionFactor(1.058);

    hoodMotorConfig.smartCurrentLimit(TurretConstants.Hood.Motor.CurrentStalledLimit,
        TurretConstants.Hood.Motor.CurrentFreeLimit);
    hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hoodController = hoodMotor.getClosedLoopController();
    hoodMotor.getEncoder().setPosition(0);
  }

  /**************************************************************** */
  /* ACTIONS */
  /**************************************************************** */

  public void startFiring(){
        fire = true;
  }
    public void stopFiring(){
        fire = false;
  }
  /* HOOD ACTIONS */
  public void setHoodAngle(double angle) {
    this.hoodAngle = angle;
  }

  /* FLYWHEEL ACTIONS */
  public void setFlywheelSpeed(double flywheelSpeed) {
    this.flywheelSpeed = flywheelSpeed;
  }

  public void setFlywheelConstant(double flywheelAdjustableConstant) {
    this.flywheelAdjustableConstant = flywheelAdjustableConstant;
  }

  public static double[] getTurrentSettings(double distance) {
    int fireTableIndex = 0;
    while (fireTableIndex < firingTable.length - 1 && firingTable[fireTableIndex][0] < distance) {
      fireTableIndex++;
    }
    return firingTable[fireTableIndex];
  }

   @Override
  public void periodic() {
    if (!this.homing) {

      double[] targetingValues = getTurrentSettings(targetingSubsystem.getTargetDistance());

      SmartDashboard.putNumber("Target distance", targetingSubsystem.getTargetDistance());
      SmartDashboard.putNumberArray("Target table", targetingValues);

      double invertFlywheel = -1;
      double hoodAngle = targetingValues[1];
      double wheelSpeed = targetingValues[2];
      double finalAdjustmentConstant = flywheelAdjustableConstant;

      if (targetingSubsystem.getTargetDistance() < 2.4)
        finalAdjustmentConstant = finalAdjustmentConstant - (Math.abs(52 - hoodAngle) * .005);

      double targetRPM = (wheelSpeed * 60.0 / 0.31918 * finalAdjustmentConstant);

      hoodController.setSetpoint(Math.abs(64.0 - hoodAngle), ControlType.kPosition);

      if (fire) {

        flywheelController.setSetpoint(invertFlywheel * targetRPM, ControlType.kVelocity);

        double currentRPM = Math.abs(leadflywheelMotor.getEncoder().getVelocity());

        double error = Math.abs(currentRPM - targetRPM);
        double maxError = targetRPM * .05;

        SmartDashboard.putNumber("Current Flywheel Speed (rpm)", currentRPM);
        SmartDashboard.putNumber("Flywheel Error (rpm)", error);

        if (error < maxError) {
          indexingSubsystem.setIndexerVelocity(2.5);
        } else {
          indexingSubsystem.setIndexerVelocity(0);
        }
      } else {

        flywheelController.setSetpoint(invertFlywheel * this.flywheelDefaultSpeed, ControlType.kVelocity);
        indexingSubsystem.setIndexerVelocity(0);
      }
      SmartDashboard.putNumber("Flywheel Speed (m/s)", wheelSpeed);
      SmartDashboard.putNumber("Flywheel Target (rpm)", targetRPM);
      SmartDashboard.putNumber("Flywheel Adjustment Constant", flywheelAdjustableConstant);
      SmartDashboard.putNumber("Hood Angle", hoodAngle);
      SmartDashboard.putNumber("Hood Set Point", Math.abs(64.0 - hoodAngle));
      SmartDashboard.putNumber("Hood Encoder Position", hoodMotor.getEncoder().getPosition());
    } else {

      hoodMotor.set(-0.4);
      if (hoodMotor.getReverseLimitSwitch().isPressed()) {
        hoodMotor.stopMotor();
        homing = false;
      }
    }
    SmartDashboard.putBoolean("Homing", this.homing);
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

  public Command FlywheelAdjustmentConstantUpCmd() {
    return runOnce(
        () -> {
          setFlywheelConstant(flywheelAdjustableConstant + .01);
        });
  }

  public Command FlywheelAdjustmentConstantDownCmd() {
    return runOnce(
        () -> {
          setFlywheelConstant(flywheelAdjustableConstant + .01);
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

  public Command FireCmd() {
    return runOnce(
        () -> {
          fire = true;
        });
  }

  public Command StopFiringCmd() {
    return runOnce(
        () -> {
          fire = false;
        });
  }

  public Command ResetHoodCmd() {
    return runOnce(
        () -> {
          this.homing = true;
        });
  }
}