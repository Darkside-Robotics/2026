package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;

public class ClimbingSubsystem extends SubsystemBase{
     private final SparkMax climbingMotor;    
    private final SparkMaxConfig climbingMotorConfig;

public ClimbingSubsystem(){

 climbingMotor = new SparkMax(ClimbingConstants.MotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        climbingMotorConfig = new SparkMaxConfig();
        climbingMotorConfig.idleMode(IdleMode.kBrake).encoder
                .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        climbingMotorConfig.smartCurrentLimit(ClimbingConstants.CurrentStalledLimit,ClimbingConstants.CurrentFreeLimit);
        climbingMotor.configure(climbingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //climbingMotor.getLimitSwitchState().
}
public double getClimbPosition() {
    return climbingMotor.getEncoder().getPosition();
}
public double getClimbVelocity() {
    return climbingMotor.getEncoder().getVelocity();
}
public void resetEncoders() {
    climbingMotor.getEncoder().setPosition(0);
}


public void goUp(){
    climbingMotor.set(ClimbingConstants.Power);
}

public void goDown(){
    climbingMotor.set(-1 * ClimbingConstants.Power);
}

public void stop() {
    climbingMotor.set(0);
}
}
