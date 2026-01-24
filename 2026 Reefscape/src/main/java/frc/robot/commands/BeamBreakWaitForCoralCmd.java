package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.IntakeRampSubsystem;
import frc.robot.subsystems.OutfeedSubsystem;

public class BeamBreakWaitForCoralCmd extends Command {

    
    private final OutfeedSubsystem outfeedSubsystem;

    public BeamBreakWaitForCoralCmd (OutfeedSubsystem outfeedSubsystem) {
        this.outfeedSubsystem = outfeedSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    if(outfeedSubsystem.hasCoral())
    {
      end(false);
    }
   }
 
    @Override
    public void end(boolean interrupted) {        
        outfeedSubsystem.closeOutfeed();
    }

    @Override
    public boolean isFinished() {
        return outfeedSubsystem.hasCoral()==true && outfeedSubsystem.isClosed();
    }


    
}
