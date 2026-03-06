package frc.robot.commands;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveGyroRun implements Runnable {

    static AHRS gyro = SwerveSubsystem.gyro;
    @Override
    public void run() {
        gyro.reset();
    }
}
