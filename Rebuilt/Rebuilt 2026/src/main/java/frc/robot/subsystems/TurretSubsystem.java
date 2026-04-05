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

  private double indexerDefaultSpeed = 0;

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
        public static final double P = 0.0004;
        public static final double I = 0;
        public static final double D = 0.008;
        public static final double FF = (1.0 / 565.0);// / 12; // (1.0 / 565.0) / 10;
      }

      public static final class LeadMotor {
        public static final int MotorPort = 13; // USED TO BE 14 BUT MOTOR WENT BAD
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 80;
        public static final int Power = 10;
      }

      public static final class FollowMotor {
        public static final int MotorPort = 14; // disabled
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 80;
        public static final int Power = 10;
      }

    }
  }

  private double flywheelAdjustableConstant = 2.368;

  private static double reduction = 0.008;
  
  private double throttleClose(double distance, double targetVelocity)
  {
    if(distance < 3.2)
    return targetVelocity - Math.abs(((-1.0*(3.2-distance))/3.2 * 5.0));
    else
    return targetVelocity;

  }

  private static final double[][] firingTable = new double[][] {
      { 2.3324, 47, 10.26 },
      { 2.3643, 47, 10.33 },
      { 2.4201, 47, 9.91 },
      { 2.425, 47, 9.92 },
      { 2.4544, 47, 9.98 },
      { 2.5122, 47, 9.65 },
      { 2.5174, 47, 9.66 },
      { 2.5724, 47, 9.39 },
      { 2.5779, 47, 9.4 },
      { 2.6109, 47, 9.46 },
      { 2.662, 47, 9.23 },
      { 2.6677, 47, 9.24 },
      { 2.7254, 47, 9.06 },
      { 2.7314, 47, 9.07 },
      { 2.7616, 47, 9.12 },
      { 2.8142, 47, 8.96 },
      { 2.8205, 47, 8.97 },
      { 2.8763, 47, 8.84 },
      { 2.8829, 47, 8.85 },
      { 2.9155, 47, 8.9 },
      { 2.9657, 47, 8.78 },
      { 2.9724, 47, 8.79 },
      { 3.0315, 47, 8.7 },
      { 3.0385, 47, 8.71 },
      { 3.0664, 47, 8.75 },
      { 3.1236, 47, 8.67 },
      { 3.1308, 47, 8.68 },
      { 3.1793, 47, 8.6 },
      { 3.1867, 47, 8.61 },
      { 3.2164, 47, 8.65 },
      { 3.2726, 47, 8.59 },
      { 3.2803, 47, 8.6 },
      { 3.3375, 47, 8.55 },
      { 3.3453, 47, 8.56 },
      { 3.3688, 47, 8.59 },
      { 3.4285, 47, 8.55 },
      { 3.4365, 47, 8.56 },
      { 3.4828, 47, 8.51 },
      { 3.491, 47, 8.52 },
      { 3.5238, 47, 8.56 },
      { 3.5737, 47, 8.52 },
      { 3.5821, 47, 8.53 },
      { 3.6359, 47, 8.5 },
      { 3.6445, 47, 8.51 },
      { 3.6788, 47, 8.55 },
      { 3.7292, 47, 8.52 },
      { 3.7379, 47, 8.53 },
      { 3.7933, 47, 8.51 },
      { 3.8022, 47, 8.52 },
      { 3.829, 47, 8.55 },
      { 3.8815, 47, 8.53 },
      { 3.8906, 47, 8.54 },
      { 3.9399, 47, 8.52 },
      { 3.9491, 47, 8.53 },
      { 3.9863, 47, 8.57 },
      { 4.0331, 47, 8.55 },
      { 4.0425, 47, 8.56 },
      { 4.096, 47, 8.55 },
      { 4.1056, 47, 8.56 },
      { 4.1345, 47, 8.59 },
      { 4.1859, 47, 8.58 },
      { 4.1957, 47, 8.59 },
      { 4.2449, 47, 8.58 },
      { 4.2548, 47, 8.59 },
      { 4.2845, 47, 8.62 },
      { 4.3419, 47, 8.62 },
      { 4.352, 47, 8.63 },
      { 4.3974, 47, 8.62 },
      { 4.4076, 47, 8.63 },
      { 4.4383, 47, 8.66 },
      { 4.4923, 47, 8.66 },
      { 4.5027, 47, 8.67 },
      { 4.555, 47, 8.67 },
      { 4.5655, 47, 8.68 },
      { 4.5972, 47, 8.71 },
      { 4.6376, 47, 8.7 },
      { 4.6483, 47, 8.71 },
      { 4.6977, 47, 8.71 },
      { 4.7193, 47, 8.73 },
      { 4.741, 47, 8.75 },
      { 4.7893, 47, 8.75 },
      { 4.8112, 47, 8.77 },
      { 4.8582, 47, 8.77 },
      { 4.8693, 47, 8.78 },
      { 4.9027, 47, 8.81 },
      { 4.9487, 47, 8.81 },
      { 4.9599, 47, 8.82 },
      { 5.0046, 47, 8.82 },
      { 5.016, 47, 8.83 },
      { 5.0501, 47, 8.86 },
      { 5.0939, 47, 8.86 },
      { 5.1169, 47, 8.88 },
      { 5.1596, 47, 8.88 },
      { 5.1712, 47, 8.89 },
      { 5.2061, 47, 8.92 },
      { 5.248, 47, 8.92 },
      { 5.2597, 47, 8.93 },
      { 5.3004, 47, 8.93 },
      { 5.3242, 47, 8.95 },
      { 5.36, 47, 8.98 },
      { 5.4, 47, 8.98 },
      { 5.412, 47, 8.99 },
      { 5.451, 47, 8.99 },
      { 5.4753, 47, 9.01 },
      { 5.5118, 47, 9.04 },
      { 5.5501, 47, 9.04 },
      { 5.5747, 47, 9.06 },
      { 5.6122, 47, 9.06 },
      { 5.6246, 47, 9.07 },
      { 5.6618, 47, 9.1 },
      { 5.6986, 47, 9.1 },
      { 5.7237, 47, 9.12 },
      { 5.7597, 47, 9.12 },
      { 5.7849, 47, 9.14 },
      { 5.8103, 47, 9.16 },
      { 5.8456, 47, 9.16 },
      { 5.8712, 47, 9.18 },
      { 5.9058, 47, 9.18 },
      { 5.9316, 47, 9.2 },
      { 5.9703, 47, 9.23 },
      { 6.0044, 47, 9.23 },
      { 6.0304, 47, 9.25 },
      { 6.0638, 47, 9.25 },
      { 6.0901, 47, 9.27 },
      { 6.1164, 47, 9.29 },
      { 6.1492, 47, 9.29 },
      { 6.1757, 47, 9.31 },
      { 6.2079, 47, 9.31 },
      { 6.2346, 47, 9.33 },
      { 6.2747, 47, 9.36 },
      { 6.3065, 47, 9.36 },
      { 6.3335, 47, 9.38 },
      { 6.3646, 47, 9.38 },
      { 6.3917, 47, 9.4 },
      { 6.419, 47, 9.42 },
      { 6.4496, 47, 9.42 },
      { 6.477, 47, 9.44 },
      { 6.5209, 47, 9.45 },
      { 6.5485, 47, 9.47 },
      { 6.5762, 47, 9.49 },
      { 6.6059, 47, 9.49 },
      { 6.6338, 47, 9.51 },
      { 6.6629, 47, 9.51 },
      { 6.6909, 47, 9.53 },
      { 6.7331, 47, 9.56 },
  };

  private boolean fire = false;
  private boolean manual = false;

  private boolean homing = false;

  private final IndexingSubsystem indexingSubsystem;
  private final TargetingSubsystem targetingSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem(IndexingSubsystem indexingSubsystem, TargetingSubsystem targetingSubsystem, IntakeSubsystem intakeSubsystem) {
    this.indexingSubsystem = indexingSubsystem;
    this.targetingSubsystem = targetingSubsystem;
    this.intakeSubsystem = intakeSubsystem;

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
    hoodMotorConfig.encoder.positionConversionFactor(0.81038961);

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

  public void startFiring() {
    fire = true;
  }

  public void stopFiring() {
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

    this.intakeSubsystem.setShooting(fire);
    
    if (spinIndexerBackward) {
      indexingSubsystem.setIndexerVelocity(-3);

    }
    SmartDashboard.putBoolean("Spin Backward", spinIndexerBackward);
    if (spinIndexerForward) {
      indexingSubsystem.setIndexerVelocity(3);
    }
    SmartDashboard.putBoolean("Spin Forward", spinIndexerForward);

    if (!this.homing) {

      double targetDistance = (false && manual ? 3.3 : targetingSubsystem.getTargetDistance());

      double[] targetingValues = getTurrentSettings(targetDistance);

      SmartDashboard.putNumber("Target distance", targetingSubsystem.getTargetDistance());
      SmartDashboard.putNumberArray("Target table", targetingValues);

      double invertFlywheel = -1;
      double hoodAngle = targetingValues[1];
      double originalWheelSpeed = targetingValues[2];
      double wheelSpeed = throttleClose(targetDistance, targetingValues[2]);
      double finalAdjustmentConstant = flywheelAdjustableConstant;

      //if (targetingSubsystem.getTargetDistance() < 2.2)
      //  finalAdjustmentConstant = finalAdjustmentConstant - (Math.abs(52 - hoodAngle) * .005);

      double targetRPM = (wheelSpeed * 60.0 / 0.31918 * finalAdjustmentConstant);

      //DISABLED HOOD CONTROL
      //hoodController.setSetpoint(Math.abs((60.5 - hoodAngle) > 0 ? (60.5 - hoodAngle) : 0), ControlType.kPosition);
 
      
      if (fire) {
       

        flywheelController.setSetpoint(invertFlywheel * targetRPM, ControlType.kVelocity);

        double currentRPM = Math.abs(leadflywheelMotor.getEncoder().getVelocity());

        double error = Math.abs(currentRPM - targetRPM);
        double maxError = targetRPM * .05;

        SmartDashboard.putNumber("Current Flywheel Speed (rpm)", currentRPM);
        SmartDashboard.putNumber("Flywheel Error (rpm)", error);

        if (error < maxError) {
          indexingSubsystem.setIndexerVelocity(3);
        } else {
          if (!spinIndexerBackward && !spinIndexerForward)
            indexingSubsystem.setIndexerVelocity(0);
        }
      } else {

        flywheelController.setSetpoint(invertFlywheel * this.flywheelDefaultSpeed, ControlType.kVelocity);

        if (!spinIndexerBackward && !spinIndexerForward)
          indexingSubsystem.setIndexerVelocity(0);
      }
      SmartDashboard.putNumber("Original Flywheel Speed", originalWheelSpeed);
      SmartDashboard.putNumber("Flywheel Speed (mps)", wheelSpeed);
      SmartDashboard.putNumber("Flywheel Target (rpm)", targetRPM);
      SmartDashboard.putNumber("Flywheel Adjustment Constant", flywheelAdjustableConstant);
      SmartDashboard.putNumber("Hood Angle", hoodAngle);
      SmartDashboard.putNumber("Hood Set Point", Math.abs((60.5 - hoodAngle) > 0 ? (60.5 - hoodAngle) : 0));
      SmartDashboard.putNumber("Hood Encoder Position", hoodMotor.getEncoder().getPosition());
    } else {

      //DISABLED HOOD CONTROL
      // hoodMotor.set(-0.4);
      // if (hoodMotor.getReverseLimitSwitch().isPressed()) {
      //   hoodMotor.stopMotor();
        homing = false;
      //}
    }
    SmartDashboard.putBoolean("Homing", this.homing);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


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

  public Command FireAutoCmd() {
    return runOnce(
        () -> {
          fire = true;
          manual = false;
        });
  }

  public Command FireManualCmd() {
    return runOnce(
        () -> {
          fire = true;
          manual = true;
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

  boolean spinIndexerForward = false;
  boolean spinIndexerBackward = false;

  public Command spinIndexerForwardCommand() {
    return runOnce(
        () -> {
          this.spinIndexerForward = true;
        });
  }

  public Command spinIndexerBackwardCommand() {
    return runOnce(
        () -> {
          this.spinIndexerBackward = true;
        });
  }

  public Command stopSpinIndexCommand() {
    return runOnce(
        () -> {
          this.spinIndexerForward = false;
          this.spinIndexerBackward = false;
        });
  }

}