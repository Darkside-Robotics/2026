// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.field.FieldColor;
import frc.robot.library.field.FieldHelper;
import frc.robot.library.field.FieldLocation;

public class TargetingSubsystem extends SubsystemBase {
  private final int[] hubtagids;
  private final VisionSubsystem visionSubsystem;

  public TargetingSubsystem(VisionSubsystem visionSubsystem, FieldColor color) {
    this.visionSubsystem = visionSubsystem;
    hubtagids = new int[] { 
      FieldHelper.lookup(color, FieldLocation.HUB_LEFT_FRONT),
        FieldHelper.lookup(color, FieldLocation.HUB_RIGHT_FRONT),
        FieldHelper.lookup(color, FieldLocation.BUMP_LEFT_BACK),
        FieldHelper.lookup(color, FieldLocation.BUMP_LEFT_FRONT),
        FieldHelper.lookup(color, FieldLocation.BUMP_RIGHT_BACK),
        FieldHelper.lookup(color, FieldLocation.BUMP_RIGHT_FRONT) 
      };

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
