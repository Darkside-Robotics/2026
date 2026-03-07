package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.field.FieldColor;
import frc.robot.library.field.FieldHelper;
import frc.robot.library.field.FieldLocation;

public class ClimbingTargetSubsystem extends SubsystemBase {
  public final int[] fieldLocation;
  public final VisionSubsystem visionSubsystem;

  /** Creates a new ExampleSubsystem. */
  public ClimbingTargetSubsystem(FieldColor fieldColor, VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.fieldLocation = new int[] {
        FieldHelper.lookup(fieldColor, FieldLocation.TOWER_LEFT),
        FieldHelper.lookup(fieldColor, FieldLocation.TOWER_RIGHT)
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
