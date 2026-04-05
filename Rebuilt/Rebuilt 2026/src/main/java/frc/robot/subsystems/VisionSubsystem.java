package frc.robot.subsystems;

import java.util.Date;
import java.util.Vector;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.vision.LimelightHelpers;
import frc.robot.library.vision.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {

    private double limeLightDarkAdjustmentTranslationX = 0.0;
    private double limeLightDarkAdjustmentTranslationY = 0.0;

    public VisionSubsystem() {
        LimelightHelpers.setLEDMode_ForceOff("limelight-dark");

        // Set a custom crop window for improved performance (-1 to 1 for each value)
        // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)

        LimelightHelpers.setCameraPose_RobotSpace("limelight-dark",
                Units.inchesToMeters(9.5), // Forward offset (meters) 0.1651
                Units.inchesToMeters(7.25), // Side offset (meters) 0.28575
                Units.inchesToMeters(21.0), // Height offset (meters) 0.4191
                0, // Roll (degrees)
                15.0, // Pitch (degrees)
                180.0 // Yaw (degrees)
        );

        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        LimelightHelpers.setCameraPose_RobotSpace("limelight-bin",
                Units.inchesToMeters(2.5), // Forward offset (meters)
                Units.inchesToMeters(0), // Side offset (meters)
                Units.inchesToMeters(19), // Height offset (meters)
                0.0, // Roll (degrees)
                0, // Pitch (degrees)
                0 // Yaw (degrees)
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
        LimelightHelpers.setPipelineIndex("limelight-bin", 0);
    }

    private int rejectedDarkVisionUpdateCount =0;
    public void updateRobotPoseTurretSide(SwerveDrivePoseEstimator poseEstimator, AHRS gyro) {

             Pose2d currentPose = poseEstimator.getEstimatedPosition();
        LimelightHelpers.SetRobotOrientation("limelight-dark",
                currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-dark");

        boolean doRejectUpdate = false;

        // if our angular velocity is greater than 360 degrees per second, ignore vision
        // updates
        if (Math.abs(gyro.getRate()) > 360) {
            doRejectUpdate = true;
        }
        if (mt2 != null) {
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }

                        double yDistance = Math.abs(currentPose.getY() - mt2.pose.getY());
            double xDistance = Math.abs(currentPose.getX() - mt2.pose.getX());    
            double changeInPosition = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
            if(changeInPosition > 1.6)
            {
                doRejectUpdate = true;
                rejectedDarkVisionUpdateCount++;
                SmartDashboard.putNumber("Rejected Dark Vision Update", rejectedDarkVisionUpdateCount);
            }
           
            if (!doRejectUpdate) {

                SmartDashboard.putString("Updating with Dark Vision ", mt2.pose.toString());
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                poseEstimator.addVisionMeasurement(
                        new Pose2d(mt2.pose.getX() + limeLightDarkAdjustmentTranslationX,
                                mt2.pose.getY() + limeLightDarkAdjustmentTranslationY, mt2.pose.getRotation()),
                        mt2.timestampSeconds);
            }
        }
    };

    
    private int rejectedBinVisionUpdateCount =0;
    public void updateRobotPoseBinSide(SwerveDrivePoseEstimator poseEstimator, AHRS gyro) {

        
             Pose2d currentPose = poseEstimator.getEstimatedPosition();
        LimelightHelpers.SetRobotOrientation("limelight-bin",
                currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bin");

        boolean doRejectUpdate = false;

        // if our angular velocity is greater than 360 degrees per second, ignore vision
        // updates
        if (Math.abs(gyro.getRate()) > 360) {
            doRejectUpdate = true;
        }
        if (mt2 != null) {
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            
            double yDistance = Math.abs(currentPose.getY() - mt2.pose.getY());
            double xDistance = Math.abs(currentPose.getX() - mt2.pose.getX());    
            double changeInPosition = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
            if(changeInPosition > 1.6)
            {
                doRejectUpdate = true;
                rejectedBinVisionUpdateCount++;
                SmartDashboard.putNumber("Rejected Bin Vision Update", rejectedBinVisionUpdateCount);
            }

            if (!doRejectUpdate) {

                SmartDashboard.putString("Updating with Bin Vision", mt2.pose.toString());
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                poseEstimator.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }
    };

    public void setLimeLightDarkAdjustmentTranslationX(double limeLightDarkAdjustmentTranslationX) {
        this.limeLightDarkAdjustmentTranslationX = limeLightDarkAdjustmentTranslationX;
    }

    // public class FiducialAndPose {

    // private RawFiducial fiducial;

    // public RawFiducial getFiducial() {
    // return fiducial;
    // }

    // public void setFiducial(RawFiducial fiducial) {
    // this.fiducial = fiducial;
    // }

    // private Pose3d pose;

    // public Pose3d getPose() {
    // return pose;
    // }

    // public void setPose(Pose3d pose) {
    // this.pose = pose;
    // }

    // public FiducialAndPose(RawFiducial fiducial, Pose3d pose) {
    // this.fiducial = fiducial;
    // this.pose = pose;
    // }

    // }

    // public FiducialAndPose foundTargetDetailed(int targetId) {

    // LimelightHelpers.setPriorityTagID("limelight-dark", targetId);

    // RawFiducial[] detected = LimelightHelpers.getRawFiducials("limelight-dark");
    // SmartDashboard.putNumber("Detected ", detected.length);

    // RawFiducial targetDetected = null;
    // Pose3d targetPose = null;

    // int DetetectionIndex = 0;
    // for (DetetectionIndex = 0; DetetectionIndex < detected.length;
    // DetetectionIndex++) {
    // SmartDashboard.putNumber("Detection " + DetetectionIndex,
    // detected[DetetectionIndex].id);
    // if (detected[DetetectionIndex].id == targetId) {

    // targetDetected = detected[DetetectionIndex];
    // targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-dark");

    // }
    // }
    // return targetDetected != null ? new FiducialAndPose(targetDetected,
    // targetPose) : null;
    // }

    // public RawFiducial foundTarget(int targetId) {

    // LimelightHelpers.setPriorityTagID("limelight-dark", targetId);

    // RawFiducial[] detected = LimelightHelpers.getRawFiducials("limelight-dark");
    // SmartDashboard.putNumber("Detected ", detected.length);

    // RawFiducial targetDetected = null;

    // int DetetectionIndex = 0;
    // for (DetetectionIndex = 0; DetetectionIndex < detected.length;
    // DetetectionIndex++) {
    // SmartDashboard.putNumber("Detection " + DetetectionIndex,
    // detected[DetetectionIndex].id);
    // if (detected[DetetectionIndex].id == targetId) {

    // targetDetected = detected[DetetectionIndex];
    // }
    // }
    // return targetDetected;
    // }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Vision Offset X", limeLightDarkAdjustmentTranslationX);
        SmartDashboard.putNumber("Vision Offset Y", limeLightDarkAdjustmentTranslationY);
        /*
         * SmartDashboard.putString("Detection ", "Running");
         * 
         * LimelightHelpers.setLEDMode_ForceOn("limelight-dark");
         * 
         * RawFiducial[] detected = LimelightHelpers.getRawFiducials("limelight-dark");
         * SmartDashboard.putNumber("Detected ", detected.length);
         * 
         * int DetetectionIndex = 0;
         * for (DetetectionIndex = 0; DetetectionIndex < detected.length;
         * DetetectionIndex++) {
         * SmartDashboard.putNumber("Detection " + DetetectionIndex,
         * detected[DetetectionIndex].id);
         * }
         */
    }

    public Command MoveVisionLeftCmd() {
        return runOnce(
                () -> {
                    setLimeLightDarkAdjustmentTranslationX(limeLightDarkAdjustmentTranslationX + 0.01);
                });
    }

    public Command MoveVisionRightCmd() {
        return runOnce(
                () -> {

                    setLimeLightDarkAdjustmentTranslationX(limeLightDarkAdjustmentTranslationX - 0.01);
                });
    }

}
