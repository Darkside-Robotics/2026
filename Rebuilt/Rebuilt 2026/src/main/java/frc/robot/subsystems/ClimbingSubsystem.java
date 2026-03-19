package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {
    public static final class ClimbingConstants {
        // public static final class PID {
        // public static final int P = 1;
        // public static final int I = 0;
        // public static final int D = 0;
        // }

        public static final class RightMotor {
            public static final int MotorPort = 11;
            public static final int CurrentFreeLimit = 60;
            public static final int CurrentStalledLimit = 40;
            public static final double Power = 0.6;
        }

        public static final class LeftMotor {
            public static final int MotorPort = 17;
            public static final int CurrentFreeLimit = 60;
            public static final int CurrentStalledLimit = 40;
            public static final double Power = 0.6;
        }
    }

    private final SparkMax rightClimbingMotor;
    private final SparkMaxConfig rightClimbingMotorConfig;
    private final SparkMax leftClimbingMotor;
    private final SparkMaxConfig leftClimbingMotorConfig;

    public ClimbingSubsystem() {

        rightClimbingMotor = new SparkMax(ClimbingConstants.RightMotor.MotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        rightClimbingMotorConfig = new SparkMaxConfig();
        rightClimbingMotorConfig.idleMode(IdleMode.kBrake).encoder
                .velocityConversionFactor(1);
        rightClimbingMotorConfig.smartCurrentLimit(ClimbingConstants.RightMotor.CurrentStalledLimit,
                ClimbingConstants.RightMotor.CurrentFreeLimit);
        rightClimbingMotor.configure(rightClimbingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        leftClimbingMotor = new SparkMax(ClimbingConstants.LeftMotor.MotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        leftClimbingMotorConfig = new SparkMaxConfig();
        leftClimbingMotorConfig.idleMode(IdleMode.kBrake).encoder
                .velocityConversionFactor(1);
        leftClimbingMotorConfig.smartCurrentLimit(ClimbingConstants.LeftMotor.CurrentStalledLimit,
                ClimbingConstants.LeftMotor.CurrentFreeLimit);
        leftClimbingMotor.configure(leftClimbingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        // climbingMotor.getLimitSwitchState().
    }

    public void leftGoUp() {
        leftClimbingMotor.set(-1 *ClimbingConstants.LeftMotor.Power);
    }

    public void leftGoDown() {
        leftClimbingMotor.set( ClimbingConstants.LeftMotor.Power);
    }

    public void rightGoUp() {
        rightClimbingMotor.set(-1 * ClimbingConstants.RightMotor.Power);
    }

    public void rightGoDown() {
        rightClimbingMotor.set(ClimbingConstants.RightMotor.Power);
    }

    public void stop() {
        rightClimbingMotor.set(0);
        leftClimbingMotor.set(0);
        
        rightClimbingMotor.stopMotor();
        leftClimbingMotor.stopMotor();
    }

    public Command LeftClimbUpCommand() {
        return runOnce(
                () -> {
                    leftGoUp();
                });
    }

    public Command LeftClimbDownCommand() {
        return runOnce(
                () -> {
                    leftGoDown();
                });
    }

    public Command RightClimbUpCommand() {
        return runOnce(
                () -> {
                    rightGoUp();
                });
    }

    public Command RightClimbDownCommand() {
        return runOnce(
                () -> {
                    rightGoDown();
                });
    }


    public Command ClimbUpCommand() {
        return runOnce(
                () -> {
                    rightGoUp();
                    leftGoUp();
                });
    }

    public Command ClimbDownCommand() {
        return runOnce(
                () -> {
                    rightGoDown();
                    leftGoDown();
                });
    }

    public Command StopCommand() {
        return runOnce(
                () -> {
                    stop();
                });
    }

}