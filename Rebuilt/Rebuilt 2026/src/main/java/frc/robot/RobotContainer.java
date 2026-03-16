package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FireForTimeCmd;
import frc.robot.commands.GyroResetCmd;
import frc.robot.commands.MoveForwardGentlyCmd;
import frc.robot.commands.MoveRightGentlyCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TargetingSwerveJoystickCmd;
import frc.robot.commands.TurnToShootCmd;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

public class RobotContainer {

    // private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(visionSubsystem);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final IndexingSubsystem indexingSubsystem = new IndexingSubsystem();
    private final TargetingSubsystem targetingSubsystem = new TargetingSubsystem(visionSubsystem, swerveSubsystem);
    private final TurretSubsystem turretSubsystem = new TurretSubsystem(indexingSubsystem, targetingSubsystem);
    

    private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();

    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final CommandXboxController controller = new CommandXboxController(0);
    private final XboxController secondaryController = new XboxController(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer(TimedRobot robot) {

        swerveSubsystem.setDefaultCommand(new TargetingSwerveJoystickCmd(
                targetingSubsystem,
                swerveSubsystem,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> -controller.getRightX(),
                () -> !controller.leftTrigger(0.2).getAsBoolean(),
                () -> !controller.leftStick().getAsBoolean(),
                () -> controller.rightBumper().getAsBoolean(), robot));

        configureButtonBindings();
        
          // Register Named Commands
        NamedCommands.registerCommand("TurnToShootCmd", new TurnToShootCmd(targetingSubsystem, swerveSubsystem, robot));
        NamedCommands.registerCommand("FireForTimeCmd", new FireForTimeCmd(turretSubsystem, 2000, robot));
        NamedCommands.registerCommand("StopFiringCmd", turretSubsystem.StopFiringCmd());
        NamedCommands.registerCommand("RightClimbUpCommand", climbingSubsystem.ClimbUpCommand());
        NamedCommands.registerCommand("RightClimbDownCommand", climbingSubsystem.ClimbDownCommand());
        NamedCommands.registerCommand("MoveForwardGentlyCmd", new MoveForwardGentlyCmd(swerveSubsystem, 750, robot));
        NamedCommands.registerCommand("MoveRightGentlyCmd", new MoveRightGentlyCmd(swerveSubsystem, 750, robot));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

      
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private enum TestingTuningEnum
    {
        NONE,
        CLIMBING,
        INTAKE,
        INDEXING,
        TARGETING,
        FLYWHEEL,
        HOOD,
        VISION
    }
    private static TestingTuningEnum tune = TestingTuningEnum.FLYWHEEL;

    private void configureButtonBindings() {

        //******************************************************************
        //CLIMBING
        //******************************************************************
        controller.y().whileTrue(climbingSubsystem.ClimbUpCommand()).whileFalse(climbingSubsystem.StopCommand());
        controller.a().whileTrue(climbingSubsystem.ClimbDownCommand()).whileFalse(climbingSubsystem.StopCommand());
       
        if(tune == TestingTuningEnum.CLIMBING)
        {
            controller.a().onTrue(climbingSubsystem.LeftClimbDownCommand());
            controller.y().onTrue(climbingSubsystem.LeftClimbUpCommand());
            controller.povDown().onTrue(climbingSubsystem.RightClimbDownCommand());
            controller.povUp().onTrue(climbingSubsystem.RightClimbUpCommand());
        }


        //******************************************************************
        //INTAKE
        //******************************************************************
        controller.leftTrigger().whileTrue(intakeSubsystem.IntakeOnCmd());
        controller.leftTrigger().whileFalse(intakeSubsystem.IntakeOffCmd());
        controller.leftBumper().onTrue(intakeSubsystem.IntakeToggleCmd());

        if(tune == TestingTuningEnum.INTAKE)
        {
            controller.povUp().onTrue(intakeSubsystem.SpinIntakeWheelUpCmd());
            controller.povDown().onTrue(intakeSubsystem.SpinIntakeWheelDownCmd());
            controller.povLeft().onTrue(intakeSubsystem.ArmOutIncrementCmd());
            controller.povRight().onTrue(intakeSubsystem.ArmInIncrementCmd());
        }


        //******************************************************************
        //INDEXING
        //******************************************************************
        if(tune == TestingTuningEnum.INDEXING)
        {
            controller.y().onTrue(indexingSubsystem.IndexerUpCmd());
            controller.a().onTrue(indexingSubsystem.IndexerDownCmd());
        }


        //******************************************************************
        //SHOOTING
        //******************************************************************
        controller.rightBumper().whileTrue(turretSubsystem.FireCmd());
        controller.rightBumper().whileFalse(turretSubsystem.StopFiringCmd());
        
        controller.rightTrigger().whileTrue(turretSubsystem.FireCmd());
        controller.rightTrigger().whileFalse(turretSubsystem.StopFiringCmd());

        controller.start().onTrue(turretSubsystem.ResetHoodCmd());

        if(tune == TestingTuningEnum.FLYWHEEL)
        {
            controller.povUp().onTrue(turretSubsystem.FlywheelAdjustmentConstantUpCmd());
            controller.povDown().onTrue(turretSubsystem.FlywheelAdjustmentConstantDownCmd());
        }

        //******************************************************************
        //VISION
        //******************************************************************
        if(tune == TestingTuningEnum.VISION)
        {
            controller.povLeft().onTrue(visionSubsystem.MoveVisionLeftCmd());
            controller.povRight().onTrue(visionSubsystem.MoveVisionRightCmd());
        }



        // controller.y().onTrue(turretSubsystem.HoodUpCmd());
        // controller.a().onTrue(turretSubsystem.HoodDownCmd());

        controller.back().onTrue(new GyroResetCmd(swerveSubsystem));




    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}