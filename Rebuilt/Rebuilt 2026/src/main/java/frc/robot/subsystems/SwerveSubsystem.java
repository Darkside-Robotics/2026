package frc.robot.subsystems;

import java.util.Date;
//import java.util.function.BiConsumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

        private boolean pathplannerReady;

        private Rotation2d initialRotationOffset = Rotation2d.kZero;

        public static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

        // Distance between right and left wheels
        public static final double TrackWidth = Units.inchesToMeters(27.0);
        // Distance between front and back wheels
        public static final double TrackLength = Units.inchesToMeters(27.0);

        private final Translation2d frontLeftLocation = new Translation2d((TrackLength / 2.0), (TrackWidth / 2.0));
        private final Translation2d frontRightLocation = new Translation2d((TrackLength / 2.0), -(TrackWidth / 2.0));
        private final Translation2d backLeftLocation = new Translation2d((-TrackLength / 2.0), (TrackWidth / 2.0));
        private final Translation2d backRightLocation = new Translation2d((-TrackLength / 2.0), -(TrackWidth / 2.0));

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        /*
         * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
         * The numbers used
         * below are robot specific, and should be tuned.
         */
        private final SwerveDrivePoseEstimator poseEstimator;
        private final VisionSubsystem visionSubsystem;

        //private final boolean isRed;

        public SwerveSubsystem(VisionSubsystem visionSubsystem) {
                this.visionSubsystem = visionSubsystem;

                // var alliance = DriverStation.getAlliance();
                // if (alliance.isPresent()) {
                //         isRed = alliance.get() == DriverStation.Alliance.Red;
                // } else {
                //         isRed = true;
                // }

                zeroHeading();

                // Pose2d SideRightCornerPose2d;
                // if (isRed) {
                //         SideRightCornerPose2d = new Pose2d(16.063, 7.65, Rotation2d.kZero);
                // } else {
                //         SideRightCornerPose2d = new Pose2d(0.063, 0.35, Rotation2d.k180deg);
                // }

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
                                VecBuilder.fill(0.7, 0.7, 9999999));

                // Load the RobotConfig from the GUI settings. You should probably
                // store this in your Constants file
                RobotConfig config;
                try {
                        config = RobotConfig.fromGUISettings();

                        // Configure AutoBuilder last
                        AutoBuilder.configure(
                                        this::getPose, // Robot pose supplier
                                        this::resetPose, // Method to reset odometry (will be called if your auto has a
                                                         // starting
                                                         // pose)
                                        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                        (speeds, feedforwards) -> driveRelative(speeds, feedforwards), // Method that
                                                                                                       // will drive the
                                                                                                       // robot given
                                                                                                       // ROBOT
                                        // RELATIVE ChassisSpeeds. Also optionally
                                        // outputs individual module feedforwards
                                        new PPHolonomicDriveController( // PPHolonomicController is the built in path
                                                                        // following
                                                                        // controller for holonomic drive trains
                                                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                                                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                                        ),
                                        config, // The robot configuration
                                        () -> {
                                                // Boolean supplier that controls when the path will be mirrored for the
                                                // red
                                                // alliance
                                                // This will flip the path being followed to the red side of the field.
                                                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                                var alliance1 = DriverStation.getAlliance();
                                                if (alliance1.isPresent()) {
                                                        return alliance1.get() == DriverStation.Alliance.Red;
                                                }
                                                return false;
                                        },
                                        this // Reference to this subsystem to set requirements
                        );

                        this.pathplannerReady = true;

                } catch (Exception e) {
                        // Handle exception as needed
                        e.printStackTrace();
                        this.pathplannerReady = false;
                }

        }

        public boolean isPathplannerReady() {
                return this.pathplannerReady;
        }

        public void zeroHeading() {
                /*
                if (this.poseEstimator != null) {

                        Pose2d SideRightCornerPose2d;
                        if (isRed) {
                                SideRightCornerPose2d = new Pose2d(16.063, 7.65, Rotation2d.kZero);
                        } else {
                                SideRightCornerPose2d = new Pose2d(0.063, 0.4, Rotation2d.k180deg);
                        }
                        this.poseEstimator.resetPose(SideRightCornerPose2d);
                }
                */
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

                // ChassisSpeeds poseFieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                // xSpeed, ySpeed, rot,
                // poseEstimator.getEstimatedPosition()
                // .getRotation());
                // ChassisSpeeds poseRobotRelative = new ChassisSpeeds(xSpeed, ySpeed, rot);

                // ChassisSpeeds oldRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                // xSpeed, ySpeed, rot,
                // Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360)));

                // SmartDashboard.putString("Robot Field Relative Pose",
                // poseFieldRelative.toString());
                // SmartDashboard.putString("Robot Relative Pose",
                // poseRobotRelative.toString());
                // SmartDashboard.putString("Robot Old Pose", oldRobotRelative.toString());

                SmartDashboard.putBoolean("Use Robot Relative Pose", fieldRelative);
                SmartDashboard.putNumber("Period", periodSeconds);

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

        public void driveRelative(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedForwards) {

                var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
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
                //visionSubsystem.updateRobotPoseTurretSide(poseEstimator, gyro);
                //visionSubsystem.updateRobotPoseBinSide(poseEstimator, gyro);
        }

        // ****************************************************************** */
        public Rotation2d getRotation2d() {
                
                return gyro.getRotation2d().rotateBy(initialRotationOffset);
        }

        public Rotation2d getInitialRotationOffset(){
                return initialRotationOffset;
        }

        public void setInitialRotationOffset(Rotation2d initialRotationOffset){
                 this.initialRotationOffset=initialRotationOffset;
        }

        public Pose2d getPose() {
                return poseEstimator.getEstimatedPosition();
        }

        /// ****************************************************************** */
        /// NEED TO REVIEW AND TEST THESE METHODS
        public ChassisSpeeds getRobotRelativeSpeeds() {
                return kinematics.toChassisSpeeds(
                                frontLeft.getState(),
                                frontRight.getState(),
                                backLeft.getState(),
                                backRight.getState());
        }

        public void resetPose(Pose2d pose) {  
                
                poseEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[] {
                                frontLeft.getPosition(),
                                frontRight.getPosition(),
                                backLeft.getPosition(),
                                backRight.getPosition()
                }, pose);
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

                if ((new Date()).getTime() - last.getTime() > 100) {
                        last = new Date();

                        f.setRobotPose(getPose());
                        SmartDashboard.putString("Robot Pose", getPose().toString());
                        SmartDashboard.putData("Robot Location", f);
                        SmartDashboard.putString("Robot Location String",
                                        f.getRobotPose().toString());
                        SmartDashboard.putNumber("Last", last.getTime());
                        SmartDashboard.putNumber("Gyro Rot", gyro.getAngle());
                        SmartDashboard.putNumber( "Gyro Offset", initialRotationOffset.getDegrees());
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
