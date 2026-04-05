// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Date;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
//import com.revrobotics.spark.config.FeedForwardConfig;
//import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public static final class ArmConstants {
        public static final boolean ArmInverted = true;

        public static final class PID {
            public static final double P = 0.015;
            public static final double I = 0;
            public static final double D = 0.012;
        }

        public static final class Motor {
            public static final int MotorPort = 18;
            public static final int CurrentFreeLimit = 30;
            public static final int CurrentStalledLimit = 40;
            public static final int Power = 10;
        }
    }

    private final SparkMax armMotor;
    private final SparkMaxConfig armMotorConfig;
    private final SparkClosedLoopController armController;
    private double armPosition = 0.0;
    private boolean armOut = false;

    // wheels:
    private SparkMax intakeWheelMotor;
    private SparkMaxConfig intakeWheelMotorConfig;
    private SparkClosedLoopController intakeWheelController;
    private double intakeWheelSpeed = 0.0;
    private double spinningIntakeWheelSpeed = 3250.0;
    private boolean spin = false;
    private boolean shooting = false;

    public static final class IntakeWheelConstants {
        public static final class PID {
            public static final double P = 0.0008;
            public static final double I = 0;
            public static final double D = 0.012;
        }

        public static final class Motor {
            public static final int MotorPort = 20;
            public static final int CurrentFreeLimit = 20;
            public static final int CurrentStalledLimit = 40;
            public static final int Power = 10;
        }
    }

    private final double kClosedLoopRampRate = 0.5;

    private final double InPosition = 0;

    /** Creates a new Subsystem. */
    public IntakeSubsystem() {

        armMotor = new SparkMax(ArmConstants.Motor.MotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(IdleMode.kBrake)
                .closedLoopRampRate(kClosedLoopRampRate);


        armMotorConfig.closedLoop.pid(ArmConstants.PID.P,
                ArmConstants.PID.I, ArmConstants.PID.D);

        armMotorConfig.closedLoop.outputRange(-0.2, 0.8);
        armMotorConfig.encoder.positionConversionFactor(360/100);//* (40/18).positionConversionFactor();

        armMotorConfig.smartCurrentLimit(ArmConstants.Motor.CurrentStalledLimit,
                ArmConstants.Motor.CurrentFreeLimit);
        armMotor.configure(armMotorConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armController = armMotor.getClosedLoopController();
        armMotor.getEncoder().setPosition(InPosition);

        // /*****************************************************************************************/
        intakeWheelMotor = new SparkMax(IntakeWheelConstants.Motor.MotorPort,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        intakeWheelMotorConfig = new SparkMaxConfig();
        intakeWheelMotorConfig.idleMode(IdleMode.kCoast)
                .closedLoopRampRate(kClosedLoopRampRate);
        intakeWheelMotorConfig.closedLoop
                .pid(IntakeWheelConstants.PID.P, IntakeWheelConstants.PID.I, IntakeWheelConstants.PID.D)
                .maxOutput(0.8).feedForward.kV(1.0 / 917.0);
        intakeWheelMotorConfig.encoder.velocityConversionFactor(1.0 / 4.0);

        intakeWheelMotorConfig.smartCurrentLimit(IntakeWheelConstants.Motor.CurrentStalledLimit,
                IntakeWheelConstants.Motor.CurrentFreeLimit);
        intakeWheelMotor.configure(intakeWheelMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        intakeWheelController = intakeWheelMotor.getClosedLoopController();

        intakeWheelMotor.getEncoder().setPosition(0);

    }

    /* Arm ACTIONS */
    public void setArmPosition(double armPosition) {
        this.armPosition = armPosition;
    }

    /* FLYWHEEL ACTIONS */
    public void setIntakeWheelSpeed(double intakeWheelSpeed) {
        this.intakeWheelSpeed = intakeWheelSpeed;
    }

    public double calculateFlywheelSpeed(double distance) {
        return 0;
    }

    public boolean getShooting(){
        return shooting;
    }
    public void setShooting(boolean shooting)
    {
        this.shooting = shooting;
    }

    private boolean extendedSetpointFound = false;
    private Date outSetPointTimer = null;

    public void pushOutToHardStop() {
        armMotor.set((ArmConstants.ArmInverted ? -1 : 1) * 0.2);
        if (Math.abs(armMotor.getAppliedOutput()) - 0.2 < .025) {
            if (outSetPointTimer == null) {
                outSetPointTimer = new Date();
                SmartDashboard.putNumber("Intake Arm Out Limit Timer (start)", outSetPointTimer.getTime());

            } else {
                long currentTime = (new Date()).getTime();

                SmartDashboard.putNumber("Intake Arm Out Limit Elapsed Time",
                        (currentTime - outSetPointTimer.getTime()));

                if ((currentTime - outSetPointTimer.getTime()) > 2000) {
                    extendedSetpointFound = true;
                    armMotor.stopMotor();
                    armMotor.getEncoder()
                            .setPosition((ArmConstants.ArmInverted ? -1.0 : 1.0) * 55.0);
                    armPosition=50;
                                    
                }
            }
        }

        SmartDashboard.putBoolean("Intake Arm Out Limit Set Point Found", extendedSetpointFound);
    }

    Date lastShootingTime = new Date();

    @Override
    public void periodic() {

        if(shooting && ((new Date()).getTime()-2000) > lastShootingTime.getTime())
        {
            armOut = !armOut;
            lastShootingTime = new Date();
        }
        

        if (armOut == true && spin == true) {
            if (extendedSetpointFound == false) {
                pushOutToHardStop();
            } else {
       
                armController.setSetpoint(InPosition + (ArmConstants.ArmInverted ? -1 : 1) * armPosition,
                        ControlType.kPosition, ClosedLoopSlot.kSlot0, -Math.cos(Units.radiansToDegrees(90 - Math.abs(armMotor.getEncoder().getPosition()))) * 0.8);
            }
            intakeWheelController.setSetpoint(spinningIntakeWheelSpeed, ControlType.kVelocity);
        } else {
            if (armOut == true) {
                if (extendedSetpointFound == false) {
                    pushOutToHardStop();
                } else {
                    armController.setSetpoint(InPosition + (ArmConstants.ArmInverted ? -1 : 1) * armPosition,
                            ControlType.kPosition, ClosedLoopSlot.kSlot0, -Math.cos(Units.radiansToDegrees(90 - Math.abs(armMotor.getEncoder().getPosition()))) * 0.8);
                }
            } else {
                armController.setSetpoint(InPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, -Math.cos(Units.radiansToDegrees(90 - Math.abs(armMotor.getEncoder().getPosition()))) * 0.8);
            }
            
            intakeWheelController.setSetpoint(200, ControlType.kVelocity);
        }
    
        SmartDashboard.putNumber("Arm Expected Position", armController.getSetpoint());
        SmartDashboard.putNumber("Intake Arm Encoder Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Wheel Speed", intakeWheelSpeed);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public Command ArmInIncrementCmd() {
        return runOnce(
                () -> {
                    setArmPosition(armPosition - 1);
                });
    }

    public Command ArmOutIncrementCmd() {
        return runOnce(
                () -> {
                    setArmPosition(armPosition + 1);
                });
    }

    public Command IntakeToggleCmd() {
        return runOnce(
                () -> {
                    armOut = !armOut;
                    if(armOut == false){
                        extendedSetpointFound=false;
                        outSetPointTimer = null;
                    }
                    spin = spin && armOut;
                });
    }

    public Command SpinIntakeWheelUpCmd() {
        return runOnce(
                () -> {
                    setIntakeWheelSpeed(intakeWheelSpeed + 1);
                });
    }

    public Command SpinIntakeWheelDownCmd() {
        return runOnce(
                () -> {

                    setIntakeWheelSpeed(intakeWheelSpeed - 1);
                });
    }

    public Command IntakeOnCmd() {
        return runOnce(
                () -> {
                    armOut = true;
                    spin = true;
                });
    }

    public Command IntakeOffCmd() {
        return runOnce(
                () -> {
                    armOut = true;
                    spin = false;
                });
    }
}