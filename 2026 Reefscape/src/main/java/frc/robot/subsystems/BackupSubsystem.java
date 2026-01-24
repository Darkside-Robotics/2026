/*
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;//

public class BackupSubsystem extends SubsystemBase {
    private final LaserCan lc;
    private boolean hasValidMeasurement = false;
    private LaserCan.Measurement measurement = null;

    public BackupSubsystem() {
        lc = new LaserCan(Constants.LaserCanPort);

        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public boolean hasBackupDistance() {
        return hasValidMeasurement;
    }
    
    public Measurement getBackupDistance() {
        return measurement;
    }

    @Override
    public void periodic() {
        measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            hasValidMeasurement = true;
        } else {
            hasValidMeasurement = false;
        }
           SmartDashboard.putNumber("Backup distance", measurement.distance_mm);
    }
}
//*/