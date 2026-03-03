package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GyroResetCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TargetingSwerveJoystickCmd;
//import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

        // private final LEDSubsystem ledSubsystem = new LEDSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();
        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(visionSubsystem);
        //private final TurretSubsystem turretSubsystem = new TurretSubsystem();

        // private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();

        private final LEDSubsystem ledSubsystem = new LEDSubsystem();

        private final CommandXboxController controller = new CommandXboxController(0);
        private final XboxController secondaryController = new XboxController(1);

        
    private final SendableChooser<Command> autoChooser;

        public RobotContainer(TimedRobot robot) {



                swerveSubsystem.setDefaultCommand(new TargetingSwerveJoystickCmd(
                swerveSubsystem,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> -controller.getRightX(),
                () -> !controller.leftTrigger(0.2).getAsBoolean(),
                () -> !controller.leftStick().getAsBoolean(),
                () -> controller.leftTrigger().getAsBoolean(), robot));

                configureButtonBindings();

                
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureButtonBindings() {

                //controller.povUp().onTrue(turretSubsystem.SpinFlywheelUpCmd());
                // controller.povDown().onTrue(turretSubsystem.SpinFlywheelDownCmd());

                // controller.y().onTrue(turretSubsystem.HoodUpCmd());
                // controller.a().onTrue(turretSubsystem.HoodDownCmd());

                controller.back().onTrue(new GyroResetCmd(swerveSubsystem));
                //controller.leftTrigger().whileTrue(null);
        }


            public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }


            

}
