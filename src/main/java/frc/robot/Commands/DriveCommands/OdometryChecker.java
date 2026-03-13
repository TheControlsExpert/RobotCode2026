package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;

public class OdometryChecker extends Command{
    Drive drive;

    public OdometryChecker(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.runVelocity(new ChassisSpeeds(0.25, 0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        
    }


    
}
