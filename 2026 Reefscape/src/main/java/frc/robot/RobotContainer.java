package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AlgaeDownCmd;
import frc.robot.commands.AlgaeUpCmd;
import frc.robot.commands.BeamBreakDropCoralCmd;
import frc.robot.commands.BeamBreakWaitForCoralCmd;
import frc.robot.commands.ClimbDownCmd;
import frc.robot.commands.ClimbUpCmd;
import frc.robot.commands.GyroResetCmd;
import frc.robot.commands.MoveAndCenterOnTagCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.CenterOnTagCmd;
import frc.robot.subsystems.AlgaeArmSubsystem;
//import frc.robot.subsystems.BackupSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeRampSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.OutfeedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

       // private final LEDSubsystem ledSubsystem = new LEDSubsystem();
        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();
      //  private final AlgaeArmSubsystem algaeSubsystem = new AlgaeArmSubsystem();
        private final IntakeRampSubsystem intakerampSubsystem = new IntakeRampSubsystem();
        //private final BackupSubsystem backupSubsystem = new BackupSubsystem();
    //    private final OutfeedSubsystem outfeedSubsystem = new OutfeedSubsystem(
               //         Constants.OutfeedConstants.OutfeedPWMPort);

        //private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(ledSubsystem, outfeedSubsystem);
     //   private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
        // private final LEDSubsystem ledSubsystem = new LEDSubsystem();

        private final CommandXboxController controller = new CommandXboxController(0);

        public RobotContainer() {                
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -controller.getLeftY(),
                                () -> controller.getLeftX(),
                                () -> controller.getRightX(),
                                () -> !controller.leftTrigger(0.2).getAsBoolean(),
                                () -> !controller.leftStick().getAsBoolean() /*, elevatorSubsystem*/));

                configureButtonBindings();
        }

        private void configureButtonBindings() {

                // LEDPattern pattern = LEDPattern.gradient(GradientType.kContinuous,
                // Color.kOrangeRed);
                // LEDPattern patterntwo = LEDPattern.gradient(GradientType.kContinuous,
                // Color.kYellow);
                // LEDPattern patternthree = LEDPattern.gradient(GradientType.kContinuous,
                // Color.kGreen);

                controller.back().onTrue(new GyroResetCmd(swerveSubsystem));

                // controller.x().whileTrue(new TurnAndLookCmd(swerveSubsystem, visionSubsystem,
                // 21));
                // controller.b().whileTrue(new TurnAndLookCmd(swerveSubsystem, visionSubsystem,
                // 20));

                // controller.y().onTrue(elevatorSubsystem.ElevatorUpCmd());
                // controller.a().onTrue(elevatorSubsystem.ElevatorDownCmd());


                // controller.x().whileTrue(new ClimbUpCmd(climbingSubsystem));
                // controller.b().whileTrue(new ClimbDownCmd(climbingSubsystem));

                // controller.povUp().whileTrue(new ClimbUpCmd(climbingSubsystem));
                // controller.povDown().whileTrue(new ClimbDownCmd(climbingSubsystem));

                controller.leftBumper().onTrue(Commands.runOnce(() -> intakerampSubsystem.openUp()));
                controller.rightBumper().onTrue(Commands.runOnce(() -> intakerampSubsystem.closeDown()));


                //controller.rightTrigger().whileTrue(outfeedSubsystem.OpenOutfeedCmd());
                //controller.rightTrigger().whileFalse(outfeedSubsystem.CloseOutfeedCmd());

                //controller.leftTrigger().whileTrue(new AlgaeDownCmd(algaeSubsystem));
                //controller.leftTrigger().whileFalse(new AlgaeUpCmd(algaeSubsystem));

                // controller.povDownRight().whileTrue(Commands.runOnce(() -> outfeedSubsystem.SetRampAngleZero()));

                // controller.povRight().onTrue(new BeamBreakDropCoralCmd(outfeedSubsystem));
        }

//         public Command getAutonomousCommandLeft(String SelectedColor) {

//                 // 0. Define PID controllers for tracking trajectory
//                 PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0.04);
//                 PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0.04);
//                 ProfiledPIDController thetaController = new ProfiledPIDController(
//                                 AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//                 thetaController.enableContinuousInput(-Math.PI, Math.PI);

//                 // 1. Create trajectory settings
//                 TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//                                 AutoConstants.kMaxSpeedMetersPerSecond,
//                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                                 .setKinematics(DriveConstants.kDriveKinematics);

//                 // ----------------------------------------------------------------------------
//                 // Generate trajectory
//                 Trajectory trajectory0 = TrajectoryGenerator.generateTrajectory(
//                                 new Pose2d(0.0, 0, new Rotation2d(0)),
//                                 List.of(
//                                                 new Translation2d(Units.inchesToMeters(67), Units.inchesToMeters(1)),
//                                                 new Translation2d(Units.inchesToMeters(97), Units.inchesToMeters(26))
//                                                 ),
//                                 new Pose2d(Units.inchesToMeters(106), Units.inchesToMeters(44), Rotation2d.fromDegrees(60)),
//                                 trajectoryConfig);

//                 // Construct command to follow trajectory
//                 SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(
//                                 trajectory0,
//                                 swerveSubsystem::getPose,
//                                 DriveConstants.kDriveKinematics,
//                                 xController,
//                                 yController,
//                                 thetaController,
//                                 swerveSubsystem::setModuleStates,
//                                 swerveSubsystem);

//                 // ----------------------------------------------------------------------------
//                 // Generate trajectory
//                 Trajectory trajectory01 = TrajectoryGenerator.generateTrajectory(
//                         new Pose2d(Units.inchesToMeters(119), Units.inchesToMeters(56), Rotation2d.fromDegrees(60)),
//                                 List.of(
//                                                 new Translation2d(Units.inchesToMeters(118), Units.inchesToMeters(57))                                                ),
//                                 new Pose2d(Units.inchesToMeters(119), Units.inchesToMeters(56), Rotation2d.fromDegrees(59)),
//                                 trajectoryConfig);

//                 // Construct command to follow trajectory
//                 SwerveControllerCommand swerveControllerCommand01 = new SwerveControllerCommand(
//                                 trajectory01,
//                                 swerveSubsystem::getPose,
//                                 DriveConstants.kDriveKinematics,
//                                 xController,
//                                 yController,
//                                 thetaController,
//                                 swerveSubsystem::setModuleStates,
//                                 swerveSubsystem);

                
//                 // ----------------------------------------------------------------------------
//                 // Generate trajectory
//                 Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
//                                 new Pose2d(Units.inchesToMeters(118), Units.inchesToMeters(56), Rotation2d.fromDegrees(60)),
//                                 List.of(new Translation2d(Units.inchesToMeters(159), Units.inchesToMeters(35)),
//                                         new Translation2d(Units.inchesToMeters(164), Units.inchesToMeters(30))),
//                                 new Pose2d(Units.inchesToMeters(169), Units.inchesToMeters(20), Rotation2d.fromDegrees(180)),
//                                 trajectoryConfig);

//                 // Construct command to follow trajectory
//                 SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
//                                 trajectory1,
//                                 swerveSubsystem::getPose,
//                                 DriveConstants.kDriveKinematics,
//                                 xController,
//                                 yController,
//                                 thetaController,
//                                 swerveSubsystem::setModuleStates,
//                                 swerveSubsystem);

//                                 int target =9;
//                                 switch(SelectedColor)
//                                 {
//                                         case Robot.kCustomAutoRed: target = 9; break;
//                                         case Robot.kCustomAutoBlue: target = 22; break;
//                                         case Robot.kCustomAutoTestColor:  target = 6; break;
//                                 }


//                 // Add some init and wrap-up, and return everything
//                 return Commands.sequence(
//                                 new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory0.getInitialPose()))
//                                 ,(new GyroResetCmd(swerveSubsystem))
//                                 ,Commands.waitSeconds(0.5)    
//                                 ,(swerveControllerCommand0)

//                                 ,elevatorSubsystem.ElevatorUpCmd()                                
//                                 ,elevatorSubsystem.ElevatorUpCmd()
//                                 ,elevatorSubsystem.ElevatorUpCmd()     
//                                 , new CenterOnTagCmd(swerveSubsystem, visionSubsystem,  target, 7, 60)    
//                                 // ,elevatorSubsystem.ElevatorUpCmd()                                
//                                 // ,elevatorSubsystem.ElevatorUpCmd()
//                                 // ,elevatorSubsystem.ElevatorUpCmd()                                              
//                                  ,(new BeamBreakDropCoralCmd(outfeedSubsystem))
//                                  ,elevatorSubsystem.ElevatorDownCmd()
//                                  ,elevatorSubsystem.ElevatorDownCmd()
//                                  ,elevatorSubsystem.ElevatorDownCmd()  
//                                  ,(swerveControllerCommand1)
//                                  ,Commands.waitSeconds(1.0)       
//                                  ,(new GyroResetCmd(swerveSubsystem))                                
//                         );

//         }
//         public Command getAutonomousCommandMiddle(String SelectedColor) {
                
//                 // 0. Define PID controllers for tracking trajectory
//                 PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0.04);
//                 PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0.04);
//                 ProfiledPIDController thetaController = new ProfiledPIDController(
//                                 AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//                 thetaController.enableContinuousInput(-Math.PI, Math.PI);

//                 // 1. Create trajectory settings
//                 TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//                                 AutoConstants.kMaxSpeedMetersPerSecond,
//                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                                 .setKinematics(DriveConstants.kDriveKinematics);

//                 // ----------------------------------------------------------------------------
//                 // Generate trajectory
//                 Trajectory trajectory0 = TrajectoryGenerator.generateTrajectory(
//                                 new Pose2d(0.0, 0, new Rotation2d(0)),
//                                 List.of(
//                                                 new Translation2d(Units.inchesToMeters(20), Units.inchesToMeters(4)),
//                                                 new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(1))
//                                                 ),
//                                 new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)),
//                                 trajectoryConfig);

//                 // Construct command to follow trajectory
//                 SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(
//                                 trajectory0,
//                                 swerveSubsystem::getPose,
//                                 DriveConstants.kDriveKinematics,
//                                 xController,
//                                 yController,
//                                 thetaController,
//                                 swerveSubsystem::setModuleStates,
//                                 swerveSubsystem);
                                

//                    // ----------------------------------------------------------------------------
//                 // Generate trajectory
//                 Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
//                         new Pose2d(Units.inchesToMeters(96), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)),
//                                 List.of(new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(1)),
//                                         new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(3))),
//                                 new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(0), Rotation2d.fromDegrees(180)),
//                                 trajectoryConfig);

//                 // Construct command to follow trajectory
//                 SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
//                                 trajectory1,
//                                 swerveSubsystem::getPose,
//                                 DriveConstants.kDriveKinematics,
//                                 xController,
//                                 yController,
//                                 thetaController,
//                                 swerveSubsystem::setModuleStates,
//                                 swerveSubsystem);

//                                 int target =10;
// switch(SelectedColor)
// {
//         case Robot.kCustomAutoRed: target = 10; break;
//         case Robot.kCustomAutoBlue: target = 21; break;
//         case Robot.kCustomAutoTestColor:  target = 6; break;
// }

//                 // Add some init and wrap-up, and return everything
//                 return Commands.sequence(
//                                 new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory0.getInitialPose()))
//                                 ,(new GyroResetCmd(swerveSubsystem))
//                                 ,Commands.waitSeconds(0.5)    
//                                 ,(swerveControllerCommand0)
//                                 ,elevatorSubsystem.ElevatorUpCmd()                                
//                                 ,elevatorSubsystem.ElevatorUpCmd()
//                                 ,elevatorSubsystem.ElevatorUpCmd()     
//                  , new CenterOnTagCmd(swerveSubsystem, visionSubsystem, target, 7, 0)    
//                                 // ,elevatorSubsystem.ElevatorUpCmd()                                
//                                 // ,elevatorSubsystem.ElevatorUpCmd()
//                                 // ,elevatorSubsystem.ElevatorUpCmd()                          
//                                 ,(new BeamBreakDropCoralCmd(outfeedSubsystem))                                                                 
//                                  ,outfeedSubsystem.CloseOutfeedCmd()
//                                  ,elevatorSubsystem.ElevatorDownCmd()
//                                  ,elevatorSubsystem.ElevatorDownCmd()
//                                  ,elevatorSubsystem.ElevatorDownCmd()  
//                                  ,(swerveControllerCommand1)
//                                  ,Commands.waitSeconds(1.0)       
//                                  ,(new GyroResetCmd(swerveSubsystem))                                
//                             );
//         }



//         public Command getAutonomousCommandRight(String SelectedColor) {

//                 // 0. Define PID controllers for tracking trajectory
//                 PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0.04);
//                 PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0.04);
//                 ProfiledPIDController thetaController = new ProfiledPIDController(
//                                 AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//                 thetaController.enableContinuousInput(-Math.PI, Math.PI);

//                 // 1. Create trajectory settings
//                 TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//                                 AutoConstants.kMaxSpeedMetersPerSecond,
//                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                                 .setKinematics(DriveConstants.kDriveKinematics);

//                 // ----------------------------------------------------------------------------
//                 // Generate trajectory
//                 Trajectory trajectory0 = TrajectoryGenerator.generateTrajectory(
//                         new Pose2d(0.0, 0, new Rotation2d(0)),
//                                 List.of(
//                                                 new Translation2d(Units.inchesToMeters(97), Units.inchesToMeters(1))
//                                                 ),
//                                 new Pose2d(Units.inchesToMeters(186), Units.inchesToMeters(-10), Rotation2d.fromDegrees(-119)),
//                                 trajectoryConfig);
              
                

//                 // Construct command to follow trajectory
//                 SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(
//                                 trajectory0,
//                                 swerveSubsystem::getPose,
//                                 DriveConstants.kDriveKinematics,
//                                 xController,
//                                 yController,
//                                 thetaController,
//                                 swerveSubsystem::setModuleStates,
//                                 swerveSubsystem);


//                 // // ----------------------------------------------------------------------------
//                 // // Generate trajectory
//                 // Trajectory trajectory01 = TrajectoryGenerator.generateTrajectory(
//                 //         new Pose2d(Units.inchesToMeters(119), Units.inchesToMeters(-56), Rotation2d.fromDegrees(60)),
//                 //                 List.of(
//                 //                                 new Translation2d(Units.inchesToMeters(118), Units.inchesToMeters(-57))                                                ),
//                 //                 new Pose2d(Units.inchesToMeters(119), Units.inchesToMeters(-56), Rotation2d.fromDegrees(-59)),
//                 //                 trajectoryConfig);

//                 // // Construct command to follow trajectory
//                 // SwerveControllerCommand swerveControllerCommand01 = new SwerveControllerCommand(
//                 //                 trajectory01,
//                 //                 swerveSubsystem::getPose,
//                 //                 DriveConstants.kDriveKinematics,
//                 //                 xController,
//                 //                 yController,
//                 //                 thetaController,
//                 //                 swerveSubsystem::setModuleStates,
//                 //                 swerveSubsystem);


//                 // ----------------------------------------------------------------------------
//                 // Generate trajectory
//                 Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
//                                 new Pose2d(Units.inchesToMeters(167), Units.inchesToMeters(-56), Rotation2d.fromDegrees(-120)),
//                                 List.of(new Translation2d(Units.inchesToMeters(179), Units.inchesToMeters(-35)),
//                                         new Translation2d(Units.inchesToMeters(194), Units.inchesToMeters(-30))),
//                                 new Pose2d(Units.inchesToMeters(230), Units.inchesToMeters(12),
//                                                 Rotation2d.fromDegrees(-123)),
//                                 trajectoryConfig);

//                 // Construct command to follow trajectory
//                 SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
//                                 trajectory1,
//                                 swerveSubsystem::getPose,
//                                 DriveConstants.kDriveKinematics,
//                                 xController,
//                                 yController,
//                                 thetaController,
//                                 swerveSubsystem::setModuleStates,
//                                 swerveSubsystem);


//                                 int target =6;
//                                 switch(SelectedColor)
//                                 {
//                                         case Robot.kCustomAutoRed: target = 6; break;
//                                         case Robot.kCustomAutoBlue: target = 19; break;
//                                         case Robot.kCustomAutoTestColor:  target = 6; break;
//                                 }

//                 target = 7;

//                 // Add some init and wrap-up, and return everything
//                 return Commands.sequence(
//                         new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory0.getInitialPose()))
//                                 ,(new GyroResetCmd(swerveSubsystem))                                                        
//                                 ,Commands.waitSeconds(1)    
//                                 ,(swerveControllerCommand0)                                
//                                 , new MoveAndCenterOnTagCmd(swerveSubsystem, visionSubsystem, target, 7)// target)  
//                                 ,elevatorSubsystem.ElevatorUpCmd()                                
//                                 ,elevatorSubsystem.ElevatorUpCmd()
//                                 ,elevatorSubsystem.ElevatorUpCmd()   
//                                 ,Commands.waitSeconds(1)    
//                                 , new CenterOnTagCmd(swerveSubsystem, visionSubsystem, target, 7, 120)//, target)    
//                                 // ,elevatorSubsystem.ElevatorUpCmd()                                
//                                 // ,elevatorSubsystem.ElevatorUpCmd()
//                                 // ,elevatorSubsystem.ElevatorUpCmd()                          
//                                  ,(new BeamBreakDropCoralCmd(outfeedSubsystem)) 
//                                  //,outfeedSubsystem.OpenOutfeedCmd()                                                  
//                                  //,Commands.waitSeconds(2.5)                                
//                                 //,outfeedSubsystem.CloseOutfeedCmd()                                                                
//                                  ,elevatorSubsystem.ElevatorDownCmd()
//                                  ,elevatorSubsystem.ElevatorDownCmd()
//                                  ,elevatorSubsystem.ElevatorDownCmd()  
//                                  ,(swerveControllerCommand1)
//                                  //NEW BACKUP TO PICKUP COMMAND
//                                  ,new InstantCommand(() -> swerveSubsystem.stopModules()) 
//                                  ,(new BeamBreakWaitForCoralCmd(outfeedSubsystem))                                                                            
//                                 //,Commands.waitSeconds(20)                                  
//                                 , new MoveAndCenterOnTagCmd(swerveSubsystem, visionSubsystem, target, -6)// target) 
//                                 ,elevatorSubsystem.ElevatorUpCmd()                                
//                                 ,elevatorSubsystem.ElevatorUpCmd()
//                                 ,elevatorSubsystem.ElevatorUpCmd()   
//                                 ,Commands.waitSeconds(.5)    
//                                 , new CenterOnTagCmd(swerveSubsystem, visionSubsystem, target, -6, 120)//, target) 
//                                 ,outfeedSubsystem.OpenOutfeedCmd()                                                  
//                                 ,Commands.waitSeconds(2.5)                                
//                                ,outfeedSubsystem.CloseOutfeedCmd()                                                                
//                                 ,elevatorSubsystem.ElevatorDownCmd()
//                                 ,elevatorSubsystem.ElevatorDownCmd()
//                                 ,elevatorSubsystem.ElevatorDownCmd() 
//                                 ,new InstantCommand(() -> swerveSubsystem.stopModules()) 
//                                  //,Commands.waitSeconds(1.0)       
//                                  //,(new GyroResetCmd(swerveSubsystem))                        
//                                  );

//         }
}
