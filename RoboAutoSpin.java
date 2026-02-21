package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.vision.LimelightHelpers;


public class RoboAutoSpin extends SubsystemBase {

    @Override
    public void periodic() {
    }

    private final PIDController turnPID = new PIDController(0.02, 0.0, 0.001);

    public RoboAutoSpin() {
    }

    public double getRotationCorrection() {
        if (!LimelightHelpers.getTV("limelight")) {
            return 0.0; // no tag detected
        }
        double tx = LimelightHelpers.getTX("limelight");
        return turnPID.calculate(tx, 0);
    }

}

// this needs cut and pasted into the drive command when this script is done:
// Whether any of it is already there is unknown to me, so sorry for amy
// duplicates

// double rotation = RoboAutoSpin.getRotationCorrection();
// drive.drive(xSpeed, ySpeed, rotation, true);



// double xSpeed = -controller.getLeftY();
// double ySpeed = -controller.getLeftX();

// double rotation;

// if (controller.getRightBumper()) {
// rotation = RoboAutoSpin.getRotationCorrection();
// } else {
// rotation = controller.getRightX();
// }

// driveSubsystem.drive(xSpeed, ySpeed, rotation, true);

// All this next stuff does is auto-rotate only when a button is held:

// @Override
// public void execute() {

// double xSpeed = -controller.getLeftY();
// double ySpeed = -controller.getLeftX();

// double rotation;

// if (controller.getRightBumper()) {
// // Auto-aim while held
// rotation = visionAim.getRotationCorrection();
// } else {
// // Manual rotation
// rotation = controller.getRightX();
// }

// driveSubsystem.drive(xSpeed, ySpeed, rotation, true);
// }