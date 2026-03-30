package frc.robot.commands;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveGyroRun implements Runnable {

    static ADXRS450_Gyro gyro = SwerveSubsystem.gyro;
    @Override
    public void run() {
        gyro.reset();
    }
}
