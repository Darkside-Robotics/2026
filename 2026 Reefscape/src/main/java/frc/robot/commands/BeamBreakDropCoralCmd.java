package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.IntakeRampSubsystem;
import frc.robot.subsystems.OutfeedSubsystem;

public class BeamBreakDropCoralCmd extends Command {

    
    private final OutfeedSubsystem outfeedSubsystem;

    public BeamBreakDropCoralCmd (OutfeedSubsystem outfeedSubsystem) {
        this.outfeedSubsystem = outfeedSubsystem;
    }

    @Override
    public void initialize() {
outfeedSubsystem.OpenOutfeedCmd();
    }

    @Override
    public void execute() {
    if(outfeedSubsystem.hasCoral())
    {
       outfeedSubsystem.openOutFeed();
    } else 
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
        return outfeedSubsystem.hasCoral()==false && outfeedSubsystem.isClosed();
    }


    
}
