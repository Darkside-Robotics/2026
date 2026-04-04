package frc.robot.subsystems;

import java.util.Date;
import java.util.Vector;

import com.studica.frc.AHRS;

//import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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

    
         LimelightHelpers.setCameraPose_RobotSpace("limelight-dark",
                Units.inchesToMeters(-5), // Forward offset (meters)
                Units.inchesToMeters(12), // Side offset (meters)
                Units.inchesToMeters(17), // Height offset (meters)
                0.0, // Roll (degrees)
                18.0, // Pitch (degrees)
                180.0 // Yaw (degrees)
        );

        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        // LimelightHelpers.setCameraPose_RobotSpace("limelight-bin",
        //         Units.inchesToMeters(2.5), // Forward offset (meters)
        //         Units.inchesToMeters(0), // Side offset (meters)
        //         Units.inchesToMeters(19), // Height offset (meters)
        //         0.0, // Roll (degrees)
        //         0, // Pitch (degrees)
        //         0 // Yaw (degrees)
        // );



        LimelightHelpers.setPipelineIndex("limelight-dark", 0);
        // LimelightHelpers.setPipelineIndex("limelight-bin", 0);
    }

    private int rejectedVisionUpdateCount =0;


    public void updateRobotPoseTurretSide(SwerveDrivePoseEstimator poseEstimator, ADXRS450_Gyro gyro) {

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
                rejectedVisionUpdateCount++;
                SmartDashboard.putNumber("Rejected Vision Update", rejectedVisionUpdateCount);
            }
         
            if (!doRejectUpdate) {

                SmartDashboard.putString("Updating with Vision ", mt2.pose.toString());
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                poseEstimator.addVisionMeasurement(
                        new Pose2d(mt2.pose.getX() + limeLightDarkAdjustmentTranslationX,
                                mt2.pose.getY() + limeLightDarkAdjustmentTranslationY, mt2.pose.getRotation()),
                        mt2.timestampSeconds);
            }
        }
    };

    public void updateRobotPoseBinSide(SwerveDrivePoseEstimator poseEstimator, ADXRS450_Gyro gyro) {

        // LimelightHelpers.SetRobotOrientation("limelight-bin",
        //         poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-dark");

        // boolean doRejectUpdate = false;

        // // if our angular velocity is greater than 360 degrees per second, ignore vision
        // // updates
        // if (Math.abs(gyro.getRate()) > 360) {
        //     doRejectUpdate = true;
        // }
        // if (mt2 != null) {
        //     if (mt2.tagCount == 0) {
        //         doRejectUpdate = true;
        //     }
        //     if (!doRejectUpdate) {

        //         SmartDashboard.putNumber("Updating with Vision 2", (new Date()).getTime());
        //         poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        //         poseEstimator.addVisionMeasurement(
        //                 mt2.pose,
        //                 mt2.timestampSeconds);
        //     }
        // }
    };

    public void setLimeLightDarkAdjustmentTranslationX(double limeLightDarkAdjustmentTranslationX) {
        this.limeLightDarkAdjustmentTranslationX = limeLightDarkAdjustmentTranslationX;
    }

 

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision Offset X", limeLightDarkAdjustmentTranslationX);
        SmartDashboard.putNumber("Vision Offset Y", limeLightDarkAdjustmentTranslationY);      
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
