package frc.robot.Commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Subsystems.Climb.Climb;

public class ClimbDown extends Command{
    Climb climb;

    public ClimbDown (Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }


    @Override
    public void initialize() {
        climb.setClimbDutyCycle(-0.2);
    }


    public boolean isFinished() {
        return climb.getEncoderValue() > ClimbConstants.lowerLimit;
    }



    public void end(boolean interrupted) {
        climb.setClimbDutyCycle(0);
    }
}
