package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GyroResetCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
//import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

        // private final LEDSubsystem ledSubsystem = new LEDSubsystem();
        // private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        private final DriveSubsystem driveSubsystem = new DriveSubsystem();

        private final VisionSubsystem visionSubsystem = new VisionSubsystem();

        private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();

        private final LEDSubsystem ledSubsystem = new LEDSubsystem();

        private final XboxController primaryController = new XboxController(0);
        private final XboxController secondaryController = new XboxController(1);

        public RobotContainer() {
                driveSubsystem.setDefaultCommand(
                                new RunCommand(() -> driveSubsystem.drive(
                                                primaryController.getLeftY(),
                                                primaryController.getLeftX(),
                                                primaryController.getRightX(),
                                                true)
                                                ));
                configureButtonBindings();
        }

        private void configureButtonBindings() {

                controller.back().onTrue(new GyroResetCmd(driveSubsystem));

        }
}
