package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.vision.LimelightHelpers;
import frc.robot.library.vision.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {

    public VisionSubsystem() {

        LimelightHelpers.setLEDMode_ForceOff("limelight-dark");

        // Set a custom crop window for improved performance (-1 to 1 for each value)
        // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        LimelightHelpers.setCameraPose_RobotSpace("limelight-dark",
                Units.inchesToMeters(14), // Forward offset (meters)
                Units.inchesToMeters(0), // Side offset (meters)
                Units.inchesToMeters(5), // Height offset (meters)
                0.0, // Roll (degrees)
                45.0, // Pitch (degrees)
                0.0 // Yaw (degrees)
        );

        // Set AprilTag offset tracking point (meters)
        // LimelightHelpers.setFiducial3DOffset("",
        // 0.0, // Forward offset
        // 0.0, // Side offset
        // 0.5 // Height offset
        // );

        // Configure AprilTag detection
        // LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] { 1, 2, 3, 4 });
        // // Only track these tag IDs
        // LimelightHelpers.SetFiducialDownscalingOverride("limelight-dark", 2.0f); //
        // Process at half resolution for improved framerate
        // and reduced range

        LimelightHelpers.setPipelineIndex("limelight-dark", 0);
    }

    public class FiducialAndPose {

        private RawFiducial fiducial;

        public RawFiducial getFiducial() {
            return fiducial;
        }

        public void setFiducial(RawFiducial fiducial) {
            this.fiducial = fiducial;
        }

        private Pose3d pose;

        public Pose3d getPose() {
            return pose;
        }

        public void setPose(Pose3d pose) {
            this.pose = pose;
        }

        public FiducialAndPose(RawFiducial fiducial, Pose3d pose) {
            this.fiducial = fiducial;
            this.pose = pose;
        }

    }

    public FiducialAndPose foundTargetDetailed(int targetId) {

        LimelightHelpers.setPriorityTagID("limelight-dark", targetId);

        RawFiducial[] detected = LimelightHelpers.getRawFiducials("limelight-dark");
        SmartDashboard.putNumber("Detected ", detected.length);

        RawFiducial targetDetected = null;
        Pose3d targetPose = null;

        int DetetectionIndex = 0;
        for (DetetectionIndex = 0; DetetectionIndex < detected.length; DetetectionIndex++) {
            SmartDashboard.putNumber("Detection " + DetetectionIndex, detected[DetetectionIndex].id);
            if (detected[DetetectionIndex].id == targetId) {

                targetDetected = detected[DetetectionIndex];
                targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-dark");

            }
        }
        return targetDetected != null ? new FiducialAndPose(targetDetected, targetPose): null;
    }

    public RawFiducial foundTarget(int targetId) {

        LimelightHelpers.setPriorityTagID("limelight-dark", targetId);

        RawFiducial[] detected = LimelightHelpers.getRawFiducials("limelight-dark");
        SmartDashboard.putNumber("Detected ", detected.length);

        RawFiducial targetDetected = null;

        int DetetectionIndex = 0;
        for (DetetectionIndex = 0; DetetectionIndex < detected.length; DetetectionIndex++) {
            SmartDashboard.putNumber("Detection " + DetetectionIndex, detected[DetetectionIndex].id);
            if (detected[DetetectionIndex].id == targetId) {

                targetDetected = detected[DetetectionIndex];
            }
        }
        return targetDetected;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Detection ", "Running");

        LimelightHelpers.setLEDMode_ForceOn("limelight-dark");

        RawFiducial[] detected = LimelightHelpers.getRawFiducials("limelight-dark");
        SmartDashboard.putNumber("Detected ", detected.length);

        int DetetectionIndex = 0;
        for (DetetectionIndex = 0; DetetectionIndex < detected.length; DetetectionIndex++) {
            SmartDashboard.putNumber("Detection " + DetetectionIndex, detected[DetetectionIndex].id);
        }
    }

}
