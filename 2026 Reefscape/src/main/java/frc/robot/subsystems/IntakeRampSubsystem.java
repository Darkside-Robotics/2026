package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRampSubsystem extends SubsystemBase{
    private final PneumaticHub pneumaticHub;
    private final DoubleSolenoid intakeRampDoubleSolenoid;
    


public IntakeRampSubsystem(){

pneumaticHub = new PneumaticHub(Constants.IntakeRampConstants.HubPort);
//pneumaticHub.disableCompressor();
pneumaticHub.enableCompressorDigital();
intakeRampDoubleSolenoid = new DoubleSolenoid(Constants.IntakeRampConstants.HubPort, PneumaticsModuleType.REVPH, Constants.IntakeRampConstants.OpenSolenoidChannel, Constants.IntakeRampConstants.CloseSolenoidChannel);

}

public void openUp()
{
intakeRampDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
}

public void closeDown()
{
    intakeRampDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
}

public void stop() 
{
    intakeRampDoubleSolenoid.set(DoubleSolenoid.Value.kOff);
}
}
