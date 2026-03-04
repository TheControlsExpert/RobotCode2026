package frc.robot.Commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Subsystems.Climb.Climb;

public class ClimbUp extends Command{
    Climb climb;

    public ClimbUp (Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }


    @Override
    public void initialize() {
        climb.setClimbGoalUp(true);
        climb.setClimbDutyCycle(0.2);
    }


    public boolean isFinished() {
        return climb.getEncoderValue() > ClimbConstants.upperLimit; //if climb has gone all the way up
    }



    public void end(boolean interrupted) {
        climb.setClimbDutyCycle(0);
    }



    
    
}
