// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.field.FieldColor;
import frc.robot.library.field.FieldHelper;
import frc.robot.library.field.FieldLocation;

public class TargetingSubsystem extends SubsystemBase {

  private final VisionSubsystem visionSubsystem;
  private final SwerveSubsystem swerveSubsystem;

  private double targetDistance = 10;
  private Rotation2d targetRotation2d = null;

  public double getTargetDistance() {
    return targetDistance;
  }

  public Rotation2d getTargetRotation2d() {
    if(targetRotation2d == null)
    {
      return calculateTargetAngleAndDistance();
    }
    return targetRotation2d;
  }

  private Alliance allyAlliance;

  public TargetingSubsystem(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
  }

  public void setAllyAlliance(Alliance alliance) {
    allyAlliance = alliance;
  }

  public Alliance getAllyAlliance() {
    if (allyAlliance == null) {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        allyAlliance = alliance.get();
        return allyAlliance;
      } else {
        return null;
      }
    } else {
      return allyAlliance;
    }
  }

  private Rotation2d calculateTargetAngleAndDistance() {
    Pose2d robotPose2d = swerveSubsystem.getPose();

    double hubX = 4.6;
    double hubY = 4.0;
    double shootingLineX = 12.5;
    double middleLineY = 4;

    if (allyAlliance == Alliance.Red) {
      hubX = 12.0;
      hubY = 4.0;
      shootingLineX = 12.5;
    } else {
      hubX = 4.0;
      hubY = 4.0;
      shootingLineX = 4.0;
    }

    double yDistance = Math.abs(robotPose2d.getY() - hubY);
    double xDistance = Math.abs(robotPose2d.getX() - hubX);
    double relativeTargetAngle = Math.toDegrees(Math.atan(yDistance / xDistance));

    boolean positiveSide = shootingLineX < robotPose2d.getX();
    boolean leftSide = middleLineY < robotPose2d.getY();

    double correctedAngle;
    if (allyAlliance == Alliance.Red) {
      correctedAngle = ((leftSide ? -1 : 1) * (positiveSide ? 180 : 0)) +
          ((leftSide ? 1 : -1) * relativeTargetAngle);
    } else {
      correctedAngle = ((leftSide ? -1 : 1) * (positiveSide ? 180 : 0)) +
          ((leftSide ? -1 : 1) * relativeTargetAngle);
    }

    targetDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    targetRotation2d = Rotation2d.fromDegrees(correctedAngle).rotateBy(Rotation2d.fromDegrees(180));

    SmartDashboard.putNumber("Robot Current Angle (Targeting)",
        robotPose2d.getRotation().rotateBy(Rotation2d.fromDegrees(180)).getDegrees());
    SmartDashboard.putNumber("Robot Target Angle", correctedAngle);
    SmartDashboard.putNumber("Robot Target Distance", targetDistance);
    SmartDashboard.putNumber("Robot Target Distance (f)", Units.metersToFeet(targetDistance));

    return targetRotation2d;
  }

  @Override
  public void periodic() {
    calculateTargetAngleAndDistance();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
