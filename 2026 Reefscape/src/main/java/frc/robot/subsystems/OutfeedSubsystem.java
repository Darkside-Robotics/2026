// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OutfeedConstants;

public class OutfeedSubsystem extends SubsystemBase {
  private final Servo intakeServo;

  
  private double angle;

  private final DigitalInput beambreak;
  
  public OutfeedSubsystem(int outfeedServoChannel) {
    beambreak = new DigitalInput(OutfeedConstants.OutfeedBeamBreakPort);
    intakeServo = new Servo(0);
    angle = intakeServo.getAngle();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command MoveRampSetCommand(double setPoint) {

    return runOnce(
        () -> {
          intakeServo.set(setPoint);
        });
  }

     

  public Command CloseOutfeedCmd() {

    return runOnce(
        () -> {
          closeOutfeed();
        });
  }


  public boolean isOpen() {

    return intakeServo.getAngle()==Constants.OutfeedConstants.OpenAngle;

  }

  
  public boolean isClosed() {

    return intakeServo.getAngle()==Constants.OutfeedConstants.CloseAngle;

  }

  public boolean hasCoral() {
    
          // intakeServo.getAngle()+1
          SmartDashboard.putBoolean("Outfeed Has Coral", !beambreak.get());
    return !beambreak.get();
  }

  public void closeOutfeed() {
    intakeServo.setAngle(Constants.OutfeedConstants.CloseAngle);
  }

  public Command OpenOutfeedCmd() {

    return runOnce(
        () -> {
          openOutFeed();
        });
  }

  public void openOutFeed() {
    intakeServo.setAngle(Constants.OutfeedConstants.OpenAngle);
  }

  public Command MoveRampAngleUp() {

    return runOnce(
        () -> {
          // intakeServo.getAngle()+1
          SmartDashboard.putNumber("Outfeed Servo Angle", intakeServo.getAngle());

          intakeServo.setAngle(intakeServo.getAngle() + 1);
        });
  }

  public Command MoveRampAngleDown() {

    return runOnce(
        () -> {
          SmartDashboard.putNumber("Outfeed Servo Angle ", intakeServo.getAngle());

          intakeServo.setAngle(intakeServo.getAngle() - 1);
        });
  }
  public Command SetRampAngleZero() {
    return runOnce(
      () -> {
        intakeServo.setAngle(90);
      });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Outfeed Has Coral ", !beambreak.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
