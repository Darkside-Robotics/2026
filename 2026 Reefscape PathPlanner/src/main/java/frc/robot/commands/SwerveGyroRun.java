package frc.robot.commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveGyroRun implements Runnable {

    static ADXRS450_Gyro gyro = DriveSubsystem.gyro;
    @Override
    public void run() {
        gyro.reset();
    }
}
