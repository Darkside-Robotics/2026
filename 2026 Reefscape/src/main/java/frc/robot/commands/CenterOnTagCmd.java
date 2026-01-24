package frc.robot.commands;

import java.time.LocalDateTime;
import java.util.Date;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.FiducialAndPose;

public class CenterOnTagCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubSystem;

    private final SlewRateLimiter turningLimiter;
    private final int targetId;
    //private final int targetId2;
    private final int offset;
    private final float rotation;

    public CenterOnTagCmd(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubSystem, int targetId, int offset, float rotation) {

        SmartDashboard.putNumber("Created", (new java.util.Date()).getTime());

        this.rotation = rotation;
        this.offset = offset;
        this.targetId = targetId;
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubSystem = visionSubSystem;
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
        addRequirements(visionSubSystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn("limelight-dark");

    }

    boolean placing = false; //CENTERED START PLACING
    boolean end = false; //REPORTING
    long endMoving = -1; //TIME TO END MOVING


    @Override
    public void execute() {
        FiducialAndPose found = null;
    
            found = visionSubSystem.foundTargetDetailed(this.targetId);
       
        SmartDashboard.putString("Found", found != null ? Integer.toString(found.getFiducial().id)
                : "Didn't find it :" + Integer.toString(LocalDateTime.now().getSecond()));
    

        // PLACING ------------------------------------------------------------
        if (placing) {

            if (endMoving > 0 && endMoving < (new Date()).getTime()) {
                end = true;
                // 4. Construct desired chassis speeds
                end(false);
            } else {
                // 4. Construct desired chassis speeds
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.4, 0, 0);

                // 5. Convert chassis speeds to individual module states
                SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

                // 6. Output each module states to wheels
                swerveSubsystem.setModuleStates(moduleStates, true);
            }
        }

        //int TargetOffset = 7;
        int TargetOffset = this.offset;
        double deadzone = 0.02;
        double rotation_deadzone = 0.5;
        // POSITIONING ------------------------------------------------------------
        if (!placing) {
            if (found == null || 
            ((found.getPose().getX() + deadzone > Units.inchesToMeters(TargetOffset)) &&
            (found.getPose().getX() - deadzone < Units.inchesToMeters(TargetOffset))) &&
            ( 
                 (swerveSubsystem.getRotation2d().getDegrees()+ rotation_deadzone > rotation) &&
                 (swerveSubsystem.getRotation2d().getDegrees()- rotation_deadzone < rotation))) 
                 {


           

                // CENTERED ------------------------------------------------------------
                if (found != null &&  
                ((found.getPose().getX() + deadzone > Units.inchesToMeters(TargetOffset)) &&
                 (found.getPose().getX() - deadzone < Units.inchesToMeters(TargetOffset)))) {

                    //Calculate forward movement
                    placing = true;
                    double distance = found.getFiducial().distToCamera;

                    long seconds = (long) (distance / 0.42);

                    endMoving = ((new Date()).getTime() + (seconds * 1000));

                    // 4. Construct desired chassis speeds
                    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.4, 0, 0);

                    // 5. Convert chassis speeds to individual module states
                    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics
                            .toSwerveModuleStates(chassisSpeeds);

                    // 6. Output each module states to wheels
                    swerveSubsystem.setModuleStates(moduleStates, true);
                }
              
            } else {

                     //test rotation  ------------------------------------------------------------
                     if ( 
                     (swerveSubsystem.getRotation2d().getDegrees()+ rotation_deadzone > rotation) &&
                     (swerveSubsystem.getRotation2d().getDegrees()- rotation_deadzone < rotation)) {
                        
                   
//fix position
                // ALINGING ------------------------------------------------------------
                double yspeed = 0.2 * (found.getPose().getX() > Units.inchesToMeters(TargetOffset) ? 1 : -1);

                // 4. Construct desired chassis speeds
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, yspeed, 0);

                // 5. Convert chassis speeds to individual module states
                SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

                // 6. Output each module states to wheels
                swerveSubsystem.setModuleStates(moduleStates, true);
            }else{
                //fix rotation
                    // ALINGING ------------------------------------------------------------
                    double rotationspeed = 0.2 * (swerveSubsystem.getRotation2d().getDegrees()<rotation ? 1 : -1);

                    // 4. Construct desired chassis speeds
                    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotationspeed);
    
                    // 5. Convert chassis speeds to individual module states
                    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
                    // 6. Output each module states to wheels
                    swerveSubsystem.setModuleStates(moduleStates, true);
            }
        }

                          }
     
        // LOGGING ------------------------------------------------------------
        if (found != null) {
            SmartDashboard.putNumber("distance", found.getFiducial().distToCamera);
            SmartDashboard.putNumber("x", found.getFiducial().txnc);
            SmartDashboard.putNumber("y", found.getFiducial().tync);
            SmartDashboard.putNumber("x pose", found.getPose().getX());
            SmartDashboard.putNumber("angle z", found.getPose().getRotation().getZ());
            SmartDashboard.putNumber("angle y", found.getPose().getRotation().getY());
            SmartDashboard.putNumber("angle x", found.getPose().getRotation().getX());
        }
        SmartDashboard.putBoolean("placing", placing);
        SmartDashboard.putBoolean("end", end);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        LimelightHelpers.setLEDMode_ForceOff("limelight-dark");
        return end;
    }
}
