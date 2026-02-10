package frc.robot.Commands.DriveCommands.AligningCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;

public class AutoAlignCommand extends Command {
    Double rotationMOE;
    Double translationMOE;
    AutoAlign autoAlign;
    Drive drive;
    Pose2d target;
    
    public AutoAlignCommand(AutoAlign autoAlign, Drive drive, Pose2d target, double rotationMOE, double translationMOE) {
        this.autoAlign = autoAlign;
        this.drive = drive;
        this.rotationMOE = rotationMOE;
        this.translationMOE = translationMOE;
        this.target = target;

        addRequirements(drive);
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(RobotContainer.getInstance().getDrive());
    }

    @Override
    public void execute() {
        
        ChassisSpeeds targetSpeeds = autoAlign.getTargetSpeeds(drive.getEstimatedPosition(), target);
        drive.runVelocity(targetSpeeds);
        
    }
}
