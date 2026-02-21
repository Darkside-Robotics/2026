package frc.robot.subsystems;

import java.util.Date;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.library.swerve.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

        private final SwerveModule frontLeft = new SwerveModule(
                        DriveConstants.Motors.Front.Left.Drive.Port,
                        DriveConstants.Motors.Front.Left.Steer.Port,
                        DriveConstants.Motors.Front.Left.Drive.Reversed,
                        DriveConstants.Motors.Front.Left.Steer.Reversed,
                        DriveConstants.Motors.Front.Left.AbsoluteEncoder.Port,
                        DriveConstants.Motors.Front.Left.AbsoluteEncoder.Offset,
                        DriveConstants.Motors.Front.Left.AbsoluteEncoder.Reversed,
                        "Front Left", false);

        private final SwerveModule frontRight = new SwerveModule(
                        DriveConstants.Motors.Front.Right.Drive.Port,
                        DriveConstants.Motors.Front.Right.Steer.Port,
                        DriveConstants.Motors.Front.Right.Drive.Reversed,
                        DriveConstants.Motors.Front.Right.Steer.Reversed,
                        DriveConstants.Motors.Front.Right.AbsoluteEncoder.Port,
                        DriveConstants.Motors.Front.Right.AbsoluteEncoder.Offset,
                        DriveConstants.Motors.Front.Right.AbsoluteEncoder.Reversed,
                        "Front Right", false);

        private final SwerveModule backLeft = new SwerveModule(
                        DriveConstants.Motors.Back.Left.Drive.Port,
                        DriveConstants.Motors.Back.Left.Steer.Port,
                        DriveConstants.Motors.Back.Left.Drive.Reversed,
                        DriveConstants.Motors.Back.Left.Steer.Reversed,
                        DriveConstants.Motors.Back.Left.AbsoluteEncoder.Port,
                        DriveConstants.Motors.Back.Left.AbsoluteEncoder.Offset,
                        DriveConstants.Motors.Back.Left.AbsoluteEncoder.Reversed,
                        "Back Left", false);

        private final SwerveModule backRight = new SwerveModule(
                        DriveConstants.Motors.Back.Right.Drive.Port,
                        DriveConstants.Motors.Back.Right.Steer.Port,
                        DriveConstants.Motors.Back.Right.Drive.Reversed,
                        DriveConstants.Motors.Back.Right.Steer.Reversed,
                        DriveConstants.Motors.Back.Right.AbsoluteEncoder.Port,
                        DriveConstants.Motors.Back.Right.AbsoluteEncoder.Offset,
                        DriveConstants.Motors.Back.Right.AbsoluteEncoder.Reversed,
                        "Back Right", false);

        public static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

        private final Translation2d frontLeftLocation = new Translation2d(0.355, -0.382);
        private final Translation2d frontRightLocation = new Translation2d(0.355, 0.382);
        private final Translation2d backLeftLocation = new Translation2d(-0.355, -0.382);
        private final Translation2d backRightLocation = new Translation2d(-0.355, 0.382);

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        /*
         * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
         * The numbers used
         * below are robot specific, and should be tuned.
         */
        private final SwerveDrivePoseEstimator poseEstimator;

        public SwerveSubsystem() {

                SmartDashboard.putString("Gyro Reset: ", "not touched");

                zeroHeading();

                poseEstimator = new SwerveDrivePoseEstimator(
                                kinematics,
                                getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                backLeft.getPosition(),
                                                backRight.getPosition()
                                },
                                Pose2d.kZero,
                                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(3)),
                                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        }

        public void zeroHeading() {
                gyro.reset();
        }

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xSpeed        Speed of the robot in the x direction (forward).
         * @param ySpeed        Speed of the robot in the y direction (sideways).
         * @param rot           Angular rate of the robot.
         * @param fieldRelative Whether the provided x and y speeds are relative to the
         *                      field.
         */
        public void drive(
                        double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {

                ChassisSpeeds poseFieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rot,
                                poseEstimator.getEstimatedPosition()
                                                .getRotation());
                ChassisSpeeds poseRobotRelative = new ChassisSpeeds(xSpeed, ySpeed, rot);

                ChassisSpeeds oldRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rot, Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360)));

                SmartDashboard.putString("Robot Field Relative Pose", poseFieldRelative.toString());
                SmartDashboard.putString("Robot Relative Pose", poseRobotRelative.toString());
                SmartDashboard.putString("Robot Old Pose", oldRobotRelative.toString());

                SmartDashboard.putBoolean("Use Robot Relative Pose", fieldRelative);

                var swerveModuleStates = kinematics.toSwerveModuleStates(
                                ChassisSpeeds.discretize(
                                                fieldRelative
                                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                                                                xSpeed, ySpeed, rot,
                                                                                poseEstimator.getEstimatedPosition()
                                                                                                .getRotation())
                                                                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                                                /*
                                                 * ChassisSpeeds.fromFieldRelativeSpeeds(
                                                 * xSpeed, ySpeed, rot, getRotation2d()),
                                                 */

                                                periodSeconds));
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                                Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
                frontLeft.setDesiredState(swerveModuleStates[0]);
                frontRight.setDesiredState(swerveModuleStates[1]);
                backLeft.setDesiredState(swerveModuleStates[2]);
                backRight.setDesiredState(swerveModuleStates[3]);
        }

        /** Updates the field relative position of the robot. */
        public void updateOdometry() {
                poseEstimator.update(
                                getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                backLeft.getPosition(),
                                                backRight.getPosition()
                                });

                // Also apply vision measurements. We use 0.3 seconds in the past as an example
                // -- on
                // a real robot, this must be calculated based either on latency or timestamps.
                // poseEstimator.addVisionMeasurement(
                // ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
                // poseEstimator.getEstimatedPosition()),
                // Timer.getTimestamp() - 0.3);
        }

        // ****************************************************************** */

        public double getHeading() {

                SmartDashboard.putNumber("Gyro: ", Math.IEEEremainder(gyro.getAngle(), 360));
                return Math.IEEEremainder(gyro.getAngle(), 360);
        }

        public Rotation2d getRotation2d() {
                // return Rotation2d.fromDegrees(getHeading());
                return gyro.getRotation2d().unaryMinus();
        }

        public Pose2d getPose() {
                return poseEstimator.getEstimatedPosition();
        }

        // public void resetOdometry(Pose2d pose) {
        // SwerveModulePosition[] newModulePositions = {

        // frontLeft.getPosition(),
        // frontRight.getPosition(),
        // backLeft.getPosition(),
        // backRight.getPosition() };

        // odometer.resetPosition(getRotation2d(), newModulePositions, pose);
        // }
        Date last = new Date();
        private static Field2d f = new Field2d();

        @Override
        public void periodic() {

                updateOdometry();

                if ((new Date()).getTime() - last.getTime() > 1000) {
                        last = new Date();

                        f.setRobotPose(getPose());
                        SmartDashboard.putString("Robot Pose", getPose().toString());
                        SmartDashboard.putData("Robot Location", f);
                        SmartDashboard.putString("Robot Location String",
                                        f.getRobotPose().toString());
                        SmartDashboard.putNumber("Last", last.getTime());
                        SmartDashboard.putNumber("Gyro Rot", gyro.getAngle());
                        SmartDashboard.putNumber("Robot Rotation", getRotation2d().getDegrees());
                }

                // SmartDashboard.putString("Robot Location",
                // getPose().getTranslation().toString());

                frontLeft.reportToDashboard();
                backLeft.reportToDashboard();
                frontRight.reportToDashboard();
                backRight.reportToDashboard();
        }

        public void stopModules() {
                frontLeft.stop();
                frontRight.stop();
                backLeft.stop();
                backRight.stop();
        }

        // public void setModuleStates(SwerveModuleState[] desiredStates) {
        // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        // DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // frontLeft.setDesiredState(desiredStates[0], true);
        // frontRight.setDesiredState(desiredStates[1], true);
        // backLeft.setDesiredState(desiredStates[2], true);
        // backRight.setDesiredState(desiredStates[3], true);
        // }

        // public void setModuleStates(SwerveModuleState[] desiredStates, boolean
        // velocityControlled) {
        // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        // DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // frontLeft.setDesiredState(desiredStates[0], velocityControlled);
        // frontRight.setDesiredState(desiredStates[1], velocityControlled);
        // backLeft.setDesiredState(desiredStates[2], velocityControlled);
        // backRight.setDesiredState(desiredStates[3], velocityControlled);
        // }
}
