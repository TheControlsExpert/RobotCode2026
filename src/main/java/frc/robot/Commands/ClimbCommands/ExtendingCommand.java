package frc.robot.Commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.Climb;

public class ExtendingCommand extends Command{
    Climb climb; 
    public ExtendingCommand(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize(){

    }
}
