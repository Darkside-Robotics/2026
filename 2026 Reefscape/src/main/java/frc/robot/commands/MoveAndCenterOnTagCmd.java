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

public class MoveAndCenterOnTagCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubSystem;

    private final SlewRateLimiter turningLimiter;
    private final int targetId;
    //private final int targetId2;

    private final int offset;


    public MoveAndCenterOnTagCmd(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubSystem, int targetId, int offset) {

        this.offset = offset;
        
        SmartDashboard.putNumber("Created", (new java.util.Date()).getTime());

        this.targetId = targetId ;// targetId;
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
    
        

                //int TargetOffsetX = 7;
                int TargetOffsetX = this.offset;
        int TargetOffsetY = -19;
        double deadzone = 0.019;
        // POSITIONING ------------------------------------------------------------
        if (!placing) {
            if (found == null || 
            ((found.getPose().getX() + deadzone > Units.inchesToMeters(TargetOffsetX)) &&
            (found.getPose().getX() - deadzone < Units.inchesToMeters(TargetOffsetX)) && 
            (found.getPose().getY() + deadzone > Units.inchesToMeters(TargetOffsetY)) &&
            (found.getPose().getY() - deadzone < Units.inchesToMeters(TargetOffsetY))
            )
             ) {

                // CENTERED ------------------------------------------------------------
                if (found != null &&  
                (found.getPose().getX() + deadzone > Units.inchesToMeters(TargetOffsetX)) &&
                (found.getPose().getX() - deadzone < Units.inchesToMeters(TargetOffsetX)) && 
                (found.getPose().getY() + deadzone > Units.inchesToMeters(TargetOffsetY)) &&
                (found.getPose().getY() - deadzone < Units.inchesToMeters(TargetOffsetY))) {

                    //Calculate forward movement
                    placing = true;
                    double distance = found.getFiducial().distToCamera;

                    long seconds = (long) (distance / 0.3);

                    endMoving = ((new Date()).getTime() + (seconds * 1500));

                    // 4. Construct desired chassis speeds
                    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.4, 0);

                    // 5. Convert chassis speeds to individual module states
                    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics
                            .toSwerveModuleStates(chassisSpeeds);

                    // 6. Output each module states to wheels
                    swerveSubsystem.setModuleStates(moduleStates, true);
                } else{
                    
                    // 4. Construct desired chassis speeds
                    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

                    // 5. Convert chassis speeds to individual module states
                    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics
                            .toSwerveModuleStates(chassisSpeeds);

                    // 6. Output each module states to wheels
                    swerveSubsystem.setModuleStates(moduleStates, true);
                }
              
            } else {

                double MinSpeedProportion = 0.3;

                // ALINGING ------------------------------------------------------------
                
                //Proportional Speed Control 
                //Start with incches of offset                
                double TargetOffsetXProportionalSpeed = MinSpeedProportion;
                double TargetOffsetXDistance = Math.abs(Units.inchesToMeters(TargetOffsetX) - found.getPose().getX());
                if(TargetOffsetXDistance < .1)
                    TargetOffsetXProportionalSpeed = 0;
                else
                {
                    if(TargetOffsetXDistance > 100) 
                        TargetOffsetXProportionalSpeed = Math.log10(100); //Max 100 INCHES
                    else
                        TargetOffsetXProportionalSpeed = Math.log10(TargetOffsetXDistance);

                    //Scale where 100 INCHES = 100% => / by 100
                    //TargetOffsetXProportionalSpeed = TargetOffsetXProportionalSpeed/100;

                    if(TargetOffsetXProportionalSpeed < MinSpeedProportion)
                        TargetOffsetXProportionalSpeed = MinSpeedProportion;//MINIMUM Proportion

                    
                     if(TargetOffsetXProportionalSpeed > 4)
                        TargetOffsetXProportionalSpeed = 4;//Max Proportion
                }

                double yspeed = (0.6 * (TargetOffsetXProportionalSpeed)) * (found.getPose().getX() > Units.inchesToMeters(TargetOffsetX) ? 1 : -1);


              double TargetOffsetYProportionalSpeed = MinSpeedProportion;
                double TargetOffsetYDistance = Math.abs(Units.inchesToMeters(TargetOffsetY) - found.getPose().getY());

                if(TargetOffsetYDistance < .05)
                    TargetOffsetYProportionalSpeed = 0;
                else
                {
                    if(TargetOffsetYDistance > 100) 
                        TargetOffsetYProportionalSpeed = Math.log10(10*100) ; //Max 100 INCHES
                    else
                        TargetOffsetYProportionalSpeed = Math.log10(10*TargetOffsetYDistance);

                    //Scale where 100 INCHES = 100% => / by 100
                    //TargetOffsetYProportionalSpeed = TargetOffsetYProportionalSpeed/100;

                    if(TargetOffsetYProportionalSpeed < MinSpeedProportion)
                        TargetOffsetYProportionalSpeed = MinSpeedProportion;//MINIMUM Proportion
                    
                        
                    if(TargetOffsetYProportionalSpeed > 4)
                    TargetOffsetYProportionalSpeed = 4;//Max Proportion
                }

                double xspeed = (0.6 * TargetOffsetYProportionalSpeed) * (found.getPose().getY() > Units.inchesToMeters(TargetOffsetY) ? -1 : 1);

                if (found != null) {
                    SmartDashboard.putNumber("xdist", TargetOffsetXDistance);
                    SmartDashboard.putNumber("ydist", TargetOffsetYDistance);
                    SmartDashboard.putNumber("xspeed", xspeed);
                    SmartDashboard.putNumber("yspeed", yspeed);            
                SmartDashboard.putNumber("updated", new Date().getTime());
                }

                
                if(xspeed < .2 && yspeed < .2)
                {
                     // 4. Construct desired chassis speeds
                    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, 0);
                    // 5. Convert chassis speeds to individual module states
                    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                    // 6. Output each module states to wheels
                    swerveSubsystem.setModuleStates(moduleStates, true);

                    end=true;
                    end(false);
                }



                // 4. Construct desired chassis speeds
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, 0);

                // 5. Convert chassis speeds to individual module states
                SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

                // 6. Output each module states to wheels
                swerveSubsystem.setModuleStates(moduleStates, true);
            }
        }

                        
        // LOGGING ------------------------------------------------------------
        if (found != null) {
            SmartDashboard.putNumber("distance", found.getFiducial().distToCamera);
            SmartDashboard.putNumber("x", found.getFiducial().txnc);
            SmartDashboard.putNumber("y", found.getFiducial().tync);
            SmartDashboard.putNumber("x pose", found.getPose().getX());            
            SmartDashboard.putNumber("y pose", found.getPose().getY());
            SmartDashboard.putNumber("angle z", found.getPose().getRotation().getZ());
            SmartDashboard.putNumber("angle y", found.getPose().getRotation().getY());
            SmartDashboard.putNumber("angle x", found.getPose().getRotation().getX());
        }
        SmartDashboard.putBoolean("placing", placing);
        SmartDashboard.putBoolean("end", end);

    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setLEDMode_ForceOff("limelight-dark");
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
