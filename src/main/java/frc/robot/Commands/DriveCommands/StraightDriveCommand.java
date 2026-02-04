package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;

public class StraightDriveCommand extends Command{ 
    double time;
    Drive drive;
    double iniT;

    public StraightDriveCommand(double time, Drive drive) {
        this.time = time;
        this.drive = drive;
           addRequirements(drive);
    }

    @Override
    public void initialize() {
        iniT = Timer.getFPGATimestamp();
    }


    @Override
    public void execute() {
        drive.runVelocity(new ChassisSpeeds(0.5, 0, 0));
    }


    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - iniT > time;
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
    
}
