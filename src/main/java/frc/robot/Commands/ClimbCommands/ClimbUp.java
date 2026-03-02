package frc.robot.Commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.Climb;

public class ClimbUp extends Command{
    Climb climb;

    public ClimbUp (Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }


    @Override
    public void initialize() {
        climb.setClimbDutyCycle(0.2);
    }


    public boolean isFinished() {
        boolean limitReached = false;
        if (climb.getEncoderValue() == 50) { //arbitrary constant representing rotations needed to fully extend
            limitReached = true;
        }
        return limitReached;
    }



    public void end() {
        climb.setClimbDutyCycle(0);
    }



    
    
}
