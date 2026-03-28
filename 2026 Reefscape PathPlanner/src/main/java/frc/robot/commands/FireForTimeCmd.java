package frc.robot.commands;

import java.util.Date;

//import static edu.wpi.first.units.Units.Rotation;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
//import frc.robot.library.field.FieldColor;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FireForTimeCmd extends Command {

    private final TurretSubsystem turretSubsystem;

    private final TimedRobot robot;
    private final long milliseconds; // TIME TO END MOVING
    private long endMoving; // TIME TO END MOVING
    private boolean finished = false;

    public FireForTimeCmd(TurretSubsystem turretSubsystem, long milliseconds,
            TimedRobot robot) {
        this.robot = robot;
        this.turretSubsystem = turretSubsystem;
        this.milliseconds = milliseconds;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        finished = false;
        this.endMoving = (new Date()).getTime() + milliseconds;
        turretSubsystem.startFiring();
    }

    @Override
    public void execute() {
        if ((new Date()).getTime() > endMoving) {

            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopFiring();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}