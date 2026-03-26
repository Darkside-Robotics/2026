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
import frc.robot.subsystems.TurretSubsystem.TurretConstants;
import frc.robot.subsystems.TurretSubsystem.TurretConstants.FlyWheelConstants;

public class IndexingSubsystem extends SubsystemBase {
  // private final SparkMax turntableMotor;
  // private final SparkMaxConfig turntableMotorConfig;
  // private final SparkClosedLoopController turntableController;

  private SparkMax indexerMotor;
  private SparkMaxConfig indexerMotorConfig;
  private SparkClosedLoopController indexerController;

  private boolean indexerOn = false;
  private double indexerVelocity = 0.0;

  public static final class IndexerConstants {

    public static final double ClosedLoopRampRate = 1;

    public static final class PID {
      public static final double P = 0.0005;
      public static final double I = 0;
      public static final double D = 0.12;
      public static final double FF = 1.0 / 917.0;
    }

    public static final class Motor {
      public static final int MotorPort = 16;
      public static final int CurrentFreeLimit = 20;
      public static final int CurrentStalledLimit = 20;
      public static final int Power = 10;
    }
  }

  /** Creates a new ExampleSubsystem. */
  public IndexingSubsystem() {

    /*****************************************************************************************/
    indexerMotor = new SparkMax(IndexerConstants.Motor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    indexerMotorConfig = new SparkMaxConfig();
    indexerMotorConfig.idleMode(IdleMode.kBrake)
        .closedLoopRampRate(IndexerConstants.ClosedLoopRampRate);
    indexerMotorConfig.closedLoop.pid(IndexerConstants.PID.P, IndexerConstants.PID.I, IndexerConstants.PID.D)
        .outputRange(-0.8, 0.8);
    indexerMotorConfig.closedLoop.feedForward.kV(IndexerConstants.PID.FF);
    indexerMotorConfig.encoder.velocityConversionFactor(1.0 / 4.0);
    indexerMotorConfig.smartCurrentLimit(IndexerConstants.Motor.CurrentStalledLimit,
        IndexerConstants.Motor.CurrentFreeLimit);
    indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    indexerController = indexerMotor.getClosedLoopController();

  }

  /* INDEXER ACTIONS */
  public void setIndexerVelocity(double indexerVelocity) {
    this.indexerVelocity = indexerVelocity;
  }

  private int failedRequestedVelocity = 0;
  private boolean reversed = false;

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Indexer Velocity", indexerVelocity * 1000);

    if (indexerVelocity > 0) {
      // MOVEMENT REQUESTED
      if (Math.abs(indexerMotor.getEncoder().getVelocity()) < 40) {
        //IF STUCK START COUNTING
        failedRequestedVelocity++;
        if (failedRequestedVelocity > 5) 
        {
          //IF STUCK OVER 5 CLICKS REVERSE THE MOVEMENT
          reversed = true; //BACKWARD DIRECTION
        }
      } 
      else 
      {
        //WE ARE MOVING COUNT DOWN TO 0 BEFORE RESTORING FORWARD DIRECTION
        if (failedRequestedVelocity > 0) {          
          failedRequestedVelocity--;
        }
        else
        {
          reversed = false; //FORWARD DIRECTION
        }
      }
    } else {
      // NO MOVEMENT REQUESTED RESET THE COUNTER
      failedRequestedVelocity = 0;
      reversed = false;  //FORWARD DIRECTION
    }
    indexerController.setSetpoint((reversed ? -1.0 : 1.0) * indexerVelocity * 1000, ControlType.kVelocity);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command IndexerUpCmd() {
    return runOnce(
        () -> {
          setIndexerVelocity(indexerVelocity + 1);
        });
  }

  public Command IndexerDownCmd() {
    return runOnce(
        () -> {
          setIndexerVelocity(indexerVelocity - 1);
        });
  }

}