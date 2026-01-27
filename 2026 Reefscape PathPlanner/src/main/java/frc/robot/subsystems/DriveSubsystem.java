package frc.robot.subsystems;

import com.pathplanner.lib.config.PIDConstants;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.library.swerve.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

    

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

    public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    SwerveModulePosition[] modulePositions = {
        new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
        new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
        new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
        new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))};
    
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), modulePositions);

    private Pose2d currentPose2d = null;



  public DriveSubsystem() {
    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    
                zeroHeading();
  }


    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        SwerveModulePosition[] newModulePositions = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))};

        odometer.resetPosition(getRotation2d(), newModulePositions, pose);
    }

    @Override
    public void periodic() {

        SwerveModulePosition[] newModulePositions = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))};

        currentPose2d = odometer.update(getRotation2d(), newModulePositions);

        SmartDashboard.putNumber("Robot Heading", getHeading());

        SmartDashboard.putString("Robot Pose", currentPose2d.toString());
        SmartDashboard.putString("Robot Pose2", getPose().toString());

        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Encoder Front Left", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Back Left", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Front Right", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Back Right", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Front Left Rel", frontLeft.getTurningPosition());
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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], true);
        frontRight.setDesiredState(desiredStates[1], true);
        backLeft.setDesiredState(desiredStates[2], true);
        backRight.setDesiredState(desiredStates[3], true);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean velocityControlled) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], velocityControlled);
        frontRight.setDesiredState(desiredStates[1], velocityControlled);
        backLeft.setDesiredState(desiredStates[2], velocityControlled);
        backRight.setDesiredState(desiredStates[3], velocityControlled);
    }

}