package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.security.AlgorithmConstraints;

import com.fasterxml.jackson.databind.util.internal.PrivateMaxEntriesMap;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.AlgaeSpinnerConstants;
import frc.robot.Constants.ModuleConstants;

public class AlgaeArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final SparkMax wheelMotor;
    private final SparkMaxConfig armMotorConfig;
    private final SparkMaxConfig wheelMotorConfig;

    private final SparkClosedLoopController armController;

    private final double kP = 0.008;
    private final double kD = 0.9;
    private final double kI = 0;
    private final double kFF = -0.03;
    private final double kClosedLoopRampRate = 0.12;

    public AlgaeArmSubsystem() {

        armMotor = new SparkMax(AlgaeArmConstants.MotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(IdleMode.kBrake)
                .closedLoopRampRate(kClosedLoopRampRate);
        armMotorConfig.closedLoop.pid(kP, kI, kD).maxOutput(0.3);
        armMotorConfig.encoder.positionConversionFactor((2 * Math.PI) * 1.25);

        armMotorConfig.smartCurrentLimit(AlgaeArmConstants.CurrentStalledLimit, AlgaeArmConstants.CurrentFreeLimit);
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armController = armMotor.getClosedLoopController();

        armMotor.getEncoder().setPosition(0);

        wheelMotor = new SparkMax(AlgaeSpinnerConstants.MotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        wheelMotorConfig = new SparkMaxConfig();
        wheelMotorConfig.idleMode(IdleMode.kBrake);
        wheelMotorConfig.encoder.velocityConversionFactor(1);
        wheelMotorConfig.smartCurrentLimit(AlgaeSpinnerConstants.CurrentStalledLimit,
                AlgaeSpinnerConstants.CurrentFreeLimit);
        wheelMotor.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double getArmPosition() {
        return armMotor.getEncoder().getPosition();
    }

    public double getArmVelocity() {
        return armMotor.getEncoder().getVelocity();
    }

    public void resetEncoders() {
        armMotor.getEncoder().setPosition(0);
    }

    private double goal = 0.0;

    public void goUp() {
        if (goal - AlgaeArmConstants.ChangeInPosition <= 0)
            goal = 0;
        else
            goal -= AlgaeArmConstants.ChangeInPosition;
    }

    public void goDown() {
        if (goal + AlgaeArmConstants.ChangeInPosition > 90)
            goal = 90;
        else
            goal += AlgaeArmConstants.ChangeInPosition;
    }

    private double spin = 0.0;

    public void stop() {
        // armMotor.set(0);
    }

    @Override
    public void periodic() {

        if (goal * 0.03 >= 0.7)
            spin = 0.7;
        else
            spin = goal * 0.03;

        if (spin < 0)
            spin = 0;

        wheelMotor.set(-1 * spin);

        if (goal == 0 && armMotor.getEncoder().getPosition() < 5 && armMotor.getEncoder().getPosition() > 0) {
            //PARK - the position reqires negative end location because of loose drive mechanism
            armMotor.set(-0.09);
        } else {
            armController.setReference(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0, kFF);
        }

        SmartDashboard.putNumber("Arm Encoder Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Position Goal", goal);

        SmartDashboard.putNumber("Arm Spin", wheelMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Arm Spin Goal", spin);

    }
}
