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

  private double flywheelDefaultSpeed = 50.0;
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
        public static final double P = 0.025;
        public static final double I = 0;
        public static final double D = 0.001;
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
        public static final double P = 0.0001;
        public static final double I = 0;
        public static final double D = 0.12;
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


  private static final double[][] firingTable = new double[][]{{ 1.97855480669544, 60, 8.8 },
{ 2.02377745659021, 60, 8.9 },
{ 2.05229919599899, 59, 8.9 },
{ 2.06147713718133, 60, 8.6 },
{ 2.11893501437614, 60, 8.4 },
{ 2.16968615063316, 60, 8.5 },
{ 2.20715741127838, 60, 8.3 },
{ 2.23385778603523, 57, 9.2 },
{ 2.26673997598778, 57, 8.9 },
{ 2.29315366602036, 56, 9.3 },
{ 2.31796412138631, 57, 9 },
{ 2.34976836202329, 58, 8.5 },
{ 2.37723096217951, 57, 8.8 },
{ 2.41468544027858, 57, 8.6 },
{ 2.4251147833136, 58, 8.4 },
{ 2.48319930689353, 58, 8.5 },
{ 2.48723271463479, 58, 8.3 },
{ 2.52575493454939, 59, 8.2 },
{ 2.56572359775245, 59, 8.1 },
{ 2.57981435100718, 60, 8 },
{ 2.62946585448674, 59, 8.2 },
{ 2.66113773223191, 59, 8.1 },
{ 2.66753139501133, 60, 8 },
{ 2.71206272426009, 57, 8.3 },
{ 2.73873385818534, 58, 8.2 },
{ 2.77780731345321, 57, 8.4 },
{ 2.81215036580624, 57, 8.3 },
{ 2.83121514458465, 58, 8.2 },
{ 2.86373240552014, 52, 9.2 },
{ 2.9004118417721, 53, 9 },
{ 2.91427078988342, 54, 8.8 },
{ 2.96353047061474, 54, 8.7 },
{ 2.97805310582273, 56, 8.4 },
{ 3.00856004854453, 57, 8.2 },
{ 3.04399145117283, 55, 8.5 },
{ 3.07032474652129, 56, 8.4 },
{ 3.10430481260994, 54, 8.6 },
{ 3.13854712038984, 55, 8.5 },
{ 3.16496277894266, 56, 8.3 },
{ 3.20111300088596, 54, 8.6 },
{ 3.22860013869176, 55, 8.5 },
{ 3.25753343929261, 53, 8.7 },
{ 3.29341848273054, 54, 8.6 },
{ 3.3169804008309, 55, 8.4 },
{ 3.35206011598177, 53, 8.7 },
{ 3.3815282608549, 54, 8.6 },
{ 3.40402844710434, 52, 8.8 },
{ 3.44238560704031, 53, 8.7 },
{ 3.46426366817758, 54, 8.5 },
{ 3.49653871164523, 52, 8.8 },
{ 3.52878390283543, 53, 8.7 },
{ 3.5638286816088, 50, 9.1 },
{ 3.58511236918437, 52, 8.8 },
{ 3.63420266468649, 51, 8.9 },
{ 3.66999545765938, 52, 8.8 },
{ 3.69078070745128, 53, 8.7 },
{ 3.72121829495454, 51, 8.9 },
{ 3.75141393027827, 52, 8.8 },
{ 3.76466356006548, 50, 9 },
{ 3.80475330001187, 51, 8.9 },
{ 3.82957566399241, 52, 8.8 },
{ 3.88501242251793, 51, 8.9 },
{ 3.90467223167854, 52, 8.8 },
{ 3.93261650631243, 50, 9 },
{ 3.96218465569684, 51, 8.9 },
{ 3.99432675359731, 48, 9.4 },
{ 4.01026088046147, 47, 9.5 },
{ 4.05964436639066, 49, 9.2 },
{ 4.08973471638599, 48, 9.3 },
{ 4.10594838055325, 47, 9.5 },
{ 4.14268732728467, 49, 9.2 },
{ 4.17479801202616, 48, 9.3 },
{ 4.19815633518715, 47, 9.5 },
{ 4.23253248175341, 50, 9 },
{ 4.25682333282205, 48, 9.3 },
{ 4.29987578897691, 49, 9.2 },
{ 4.33597057218649, 48, 9.3 },
{ 4.37286614396717, 47, 9.5 },
{ 4.38649267735931, 46, 9.6 },
{ 4.41238859640044, 48, 9.3 },
{ 4.44622366710416, 49, 9.2 },
{ 4.48621617911561, 48, 9.3 },
{ 4.51567689740184, 49, 9.2 },
{ 4.55323533944965, 46, 9.6 },
{ 4.56867142371728, 45, 9.8 },
{ 4.61309213103971, 47, 9.5 },
{ 4.63250644109915, 46, 9.6 },
{ 4.65426714125765, 45, 9.8 },
{ 4.69340971770335, 48, 9.3 },
{ 4.71059402759211, 49, 9.2 },
{ 4.760327413439, 47, 9.5 },
{ 4.78349901566963, 46, 9.6 },
{ 4.83043945267676, 47, 9.5 },
{ 4.85545641448837, 46, 9.6 },
{ 4.87610488374287, 42, 10.4 },
{ 4.90701651629587, 43, 10.2 },
{ 4.92519973949731, 46, 9.6 },
{ 4.96419165060725, 47, 9.5 },
{ 4.99282963041507, 46, 9.6 },
{ 5.02802792689226, 47, 9.5 },
{ 5.04718334567607, 48, 9.4 },
{ 5.08995864269115, 47, 9.5 },
{ 5.10555066208128, 48, 9.4 },
{ 5.16220129270988, 48, 9.4 },
{ 5.18294657312464, 45, 9.8 },
{ 5.22303057246782, 44, 10 },
{ 5.2497156111037, 45, 9.8 },
{ 5.26995988749666, 51, 9.2 },
{ 5.31523745620172, 51, 9.2 },
{ 5.31912423878291, 53, 9.1 },
{ 5.36178771187151, 54, 9.1 },
{ 5.39934331431934, 54, 9.1 },
{ 5.42897902261059, 55, 9.1 },
{ 5.46374748692629, 55, 9.1 },
{ 5.48041163181749, 56, 9.1 },
{ 5.51603231037915, 53, 9.2 },
{ 5.55408254285007, 53, 9.2 },
{ 5.59227831160241, 54, 9.2 },
{ 5.60397827852857, 51, 9.3 },
{ 5.65167515424488, 50, 9.4 },
{ 5.66441760623259, 52, 9.3 },
{ 5.70295371312822, 52, 9.3 },
{ 5.7313067018242, 49, 9.5 },
{ 5.77576667078583, 49, 9.5 },
{ 5.80092475488831, 48, 9.6 },
{ 5.81116112026753, 47, 9.7 },
{ 5.85994954302607, 47, 9.7 },
{ 5.87397096483327, 55, 9.3 },
{ 5.90487919506244, 55, 9.3 },
{ 5.93830832927968, 52, 9.4 },
{ 5.98152414551117, 53, 9.4 },
{ 6.01540630087529, 53, 9.4 },
{ 6.04527113968229, 54, 9.4 },
{ 6.07678393771944, 54, 9.4 },
{ 6.09333332500969, 55, 9.4 },
{ 6.13685726070912, 52, 9.5 },
{ 6.17129757679985, 52, 9.5 },
{ 6.18484514119768, 50, 9.6 },
{ 6.22224668725877, 50, 9.6 },
{ 6.24283069092141, 53, 9.5 },
{ 6.26878350687093, 54, 9.5 },
{ 6.29867197130141, 54, 9.5 },
{ 6.32785018981971, 43, 10.3 },
{ 6.38094877693918, 43, 10.3 },
{ 6.39669359910852, 48, 9.8 },
{ 6.42595485567963, 47, 9.9 }
  };



  private boolean fire = false;

  private final IndexingSubsystem indexingSubsystem;
  private final TargetingSubsystem targetingSubsystem;

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem(IndexingSubsystem indexingSubsystem, TargetingSubsystem targetingSubsystem) {
    this.indexingSubsystem = indexingSubsystem;
    this.targetingSubsystem = targetingSubsystem;

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

    leadflywheelMotor = new SparkFlex(TurretConstants.FlyWheelConstants.LeadMotor.MotorPort,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    leadflywheelMotorConfig = new SparkFlexConfig();
    leadflywheelMotorConfig.closedLoop.pid(FlyWheelConstants.PID.P, FlyWheelConstants.PID.I, FlyWheelConstants.PID.D)
        .outputRange(-0.8, 0.8);
    leadflywheelMotorConfig.closedLoop.feedForward.kV(FlyWheelConstants.PID.FF);
    leadflywheelMotorConfig.idleMode(IdleMode.kCoast)
        .closedLoopRampRate(TurretConstants.FlyWheelConstants.ClosedLoopRampRate);
    leadflywheelMotorConfig.encoder.velocityConversionFactor(1.0).positionConversionFactor(1);
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


  public static double[] getTurrentSettings(double distance)
  {
    int fireTableIndex = 0;
    while(fireTableIndex < firingTable.length && firingTable[fireTableIndex][0] < distance)
    {      
      fireTableIndex++;
    }
    return firingTable[fireTableIndex];

  }


  // public double calculateFlywheelSpeed(double distance) {
  //   //return (850/20) * (distance*3.28);
  //   //turn 400;
  // }
  // public double calculateHoodAngle(double distance) {
  //   return 60-;
  //   //return 650;
  // }

  @Override
  public void periodic() {

    double[] targetingValues = getTurrentSettings(targetingSubsystem.getTargetDistance());
    
    SmartDashboard.putNumber("Target distance", targetingSubsystem.getTargetDistance()); 
    SmartDashboard.putNumberArray("Target table", targetingValues); 

    double invertFlywheel = -1;
    double hoodAngle = targetingValues[1];
    double wheelSpeed = targetingValues[2];
    double convertedWheelSpeed = wheelSpeed * 60.0 / 0.31918 / 4;

    hoodController.setSetpoint(Math.abs(60 - hoodAngle), ControlType.kPosition);

    if (fire) {
      flywheelController.setSetpoint(invertFlywheel * convertedWheelSpeed, ControlType.kVelocity);

      if (Math.abs(leadflywheelMotor.getEncoder().getVelocity() - 10) > convertedWheelSpeed) {

        indexingSubsystem.setIndexerVelocity(2.5);

      } else {
        indexingSubsystem.setIndexerVelocity(0);
      }
    } else {

      flywheelController.setSetpoint(invertFlywheel * this.flywheelDefaultSpeed, ControlType.kVelocity);
      indexingSubsystem.setIndexerVelocity(0);
    }
    SmartDashboard.putNumber("Flywheel Speed (m/s)", wheelSpeed); 
    SmartDashboard.putNumber("Converted Flywheel Speed (rpm)", convertedWheelSpeed);    
    SmartDashboard.putNumber("Hood Angle", hoodAngle);
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
}