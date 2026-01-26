package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GyroResetCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

       // private final LEDSubsystem ledSubsystem = new LEDSubsystem();
        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();

        private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
       
        private final LEDSubsystem ledSubsystem = new LEDSubsystem();

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



                controller.back().onTrue(new GyroResetCmd(swerveSubsystem));


        }
}

