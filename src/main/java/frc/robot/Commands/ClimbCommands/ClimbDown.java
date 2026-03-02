package frc.robot.Commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Drive.Drive;

public class ClimbDown extends Command{
    Climb climb;
    Drive drive;

    public ClimbDown (Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }


    @Override
    public void initialize() {
        if (drive.getEstimatedPosition().getTranslation().getDistance(ClimbConstants.RightPoseBlue.getTranslation()) <= 0.2)
        climb.setClimbDutyCycle(-0.2);
    }


    public boolean isFinished() {
        return climb.getEncoderValue() > ClimbConstants.lowerLimit;
    }



    public void end(boolean interrupted) {
        climb.setClimbDutyCycle(0);
    }
}
