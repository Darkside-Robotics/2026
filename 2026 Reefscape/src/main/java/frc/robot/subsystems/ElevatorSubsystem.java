package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;

//import java.time.LocalDateTime;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.spline.PoseWithCurvature;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.ModuleConstants;
//import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.VisionSubsystem.FiducialAndPose;
import frc.robot.commands.BeamBreakDropCoralCmd;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax primaryElevatorMotor;
    private final SparkMaxConfig primaryElevatorMotorConfig;

    private final SparkMax secondaryElevatorMotor;
    private final SparkMaxConfig secondaryElevatorMotorConfig;

    
    private final SparkClosedLoopController primaryController;

    private final double kP = 0.18;
    private final double kD = 0.03;
    private final double kI = 0;
    private final double kFF = 0.015;
    private final double kClosedLoopRampRate = 0.12;
    
    private final double setPoints[] = new double[]{4, 16, 31, 55};
    
    private int elevatorHeight = 0; //INDEX OF THE SETPOINT

    private final IdleMode idleMode = IdleMode.kBrake;

    private final LEDSubsystem ledSubsystem;
    
    private final OutfeedSubsystem outfeedSubsystem;

    public ElevatorSubsystem(LEDSubsystem ledSubsystem, OutfeedSubsystem outfeedSubsystem) {

      this.outfeedSubsystem = outfeedSubsystem;
        this.ledSubsystem = ledSubsystem;      

        primaryElevatorMotor = new SparkMax(ElevatorConstants.PrimaryMotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        primaryElevatorMotorConfig = new SparkMaxConfig();
        primaryElevatorMotorConfig.idleMode(idleMode).closedLoopRampRate(kClosedLoopRampRate);
        primaryElevatorMotorConfig.closedLoop.pid(kP, kI, kD).maxOutput(0.8);
        primaryElevatorMotorConfig.encoder.positionConversionFactor(0.68/1);
        primaryElevatorMotorConfig.smartCurrentLimit(ElevatorConstants.CurrentStalledLimit,ElevatorConstants.CurrentFreeLimit);

        primaryElevatorMotor.configure(primaryElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        primaryController = primaryElevatorMotor.getClosedLoopController();


        secondaryElevatorMotor = new SparkMax(ElevatorConstants.SecondaryMotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        secondaryElevatorMotorConfig = new SparkMaxConfig();
        secondaryElevatorMotorConfig.idleMode(idleMode).follow(ElevatorConstants.PrimaryMotorPort, true);
        secondaryElevatorMotorConfig.smartCurrentLimit(ElevatorConstants.CurrentStalledLimit,ElevatorConstants.CurrentFreeLimit);
        secondaryElevatorMotor.configure(secondaryElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public double getElevatorPosition() {
        return primaryElevatorMotor.getEncoder().getPosition();
    }

    public double getElevatorVelocity() {
        return primaryElevatorMotor.getEncoder().getVelocity();
    }

    public void resetEncoders() {
        primaryElevatorMotor.getEncoder().setPosition(0);
        secondaryElevatorMotor.getEncoder().setPosition(0);
    }

    public void elevatorUp() {
          //if(outfeedSubsystem.hasCoral()==true){
            elevatorHeight = elevatorHeight < 3 ? elevatorHeight + 1 : elevatorHeight;
            ledSubsystem.elevatorUp();
          //}
      }
    
      public void elevatorDown() {
        elevatorHeight = elevatorHeight > 0 ? elevatorHeight - 1 : elevatorHeight;
        ledSubsystem.elevatorDown();
      }

      @Override
      public void periodic() {        

        SmartDashboard.putNumber("Elevator Height", elevatorHeight);
        
    SmartDashboard.putNumber("Elevator SetPoint", setPoints[elevatorHeight]);
    
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
    
    SmartDashboard.putNumber("Elevator Velocity", getElevatorVelocity());

 if(Math.abs(setPoints[elevatorHeight]) < 6 && Math.abs(getElevatorPosition()) < 4)
 {
    primaryElevatorMotor.set(.09);
 }else{

    primaryController.setReference(-1 * setPoints[elevatorHeight], ControlType.kPosition, ClosedLoopSlot.kSlot0, kFF);
 }
      }

    public void stop() {
        primaryElevatorMotor.set(0);
        secondaryElevatorMotor.set(0);
    }


    
    
  public Command ElevatorUpCmd() {
    return runOnce(
        () -> {
          elevatorUp();
        });
  }

  public Command ElevatorDownCmd() {
    return runOnce(
        () -> {
          elevatorDown();
        });
  }
}
