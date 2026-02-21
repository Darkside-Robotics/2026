// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


    private Command m_autonomousCommand;
   // private Command m_colorCheckerCommand;

    private RobotContainer m_robotContainer;


    private static final String kCustomAutoLeft = "Left Auto";    
    private static final String kCustomAutoMiddle = "Middle Auto";    
    private static final String kCustomAutoRight = "Right Auto";

      
    public static final String kCustomAutoRed = "Red";    
    public static final String kCustomAutoBlue = "Blue";
    public static final String kCustomAutoTestColor = "Test";

    private String m_autoSelected;
    
    private String m_colorSelected;

    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final SendableChooser<String> m_ColorChooser = new SendableChooser<>();

        public Robot() {
   //   CanBridge.runTCP();
    }
    

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer(this);
       

        m_chooser.addOption("Left Auto", kCustomAutoLeft);
        m_chooser.addOption("Middle Auto", kCustomAutoMiddle);
        m_chooser.setDefaultOption("Right Auto - (Default)", kCustomAutoRight);

        SmartDashboard.putData("Auto choices", m_chooser);

        
        m_ColorChooser.addOption("Testing", kCustomAutoTestColor);
        m_ColorChooser.addOption("Blue", kCustomAutoBlue);
        m_ColorChooser.setDefaultOption("Red (Default)", kCustomAutoRed);

        SmartDashboard.putData("Color choices", m_ColorChooser);



        


    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
       // m_colorCheckerCommand = m_robotContainer.startColorSensor();
       // m_colorCheckerCommand.schedule();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        
    // m_autoSelected = m_chooser.getSelected();     
    // m_colorSelected = m_ColorChooser.getSelected(); 

    // SmartDashboard.putString("Routine Selection", m_autoSelected);
    //     switch (m_autoSelected) {
    //         case kCustomAutoRight:
    //           m_autonomousCommand = m_robotContainer.getAutonomousCommandRight(m_colorSelected);
    //           break;
    //         case kCustomAutoMiddle:
    //           m_autonomousCommand = m_robotContainer.getAutonomousCommandMiddle(m_colorSelected);
    //           break;
    //         case kCustomAutoLeft:            
    //           m_autonomousCommand = m_robotContainer.getAutonomousCommandLeft(m_colorSelected);
    //           break;
    //         default:
    //           m_autonomousCommand = m_robotContainer.getAutonomousCommandRight(m_colorSelected);
    //           break;
    //       }
        

    //     // schedule the autonomous command (example)
    //     if (m_autonomousCommand != null) {
    //         m_autonomousCommand.schedule();
    //     }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAutoRight:
               
              break;
            case kCustomAutoMiddle:
         
              break;
            case kCustomAutoLeft:
            default:
           
              break;
          }      
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}

