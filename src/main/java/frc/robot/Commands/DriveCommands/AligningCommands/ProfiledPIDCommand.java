package frc.robot.Commands.DriveCommands.AligningCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;

public class ProfiledPIDCommand extends Command {
    Double rotationMOE;
    Double translationMOE;
    AutoAlign autoAlign;
    Drive drive;
    Supplier<Pose2d> target;
    
    public ProfiledPIDCommand(AutoAlign autoAlign, Drive drive, Supplier<Pose2d> target, double rotationMOE, double translationMOE) {
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
        
       boolean isFlipped = DriverStation.getAlliance().get().equals(Alliance.Red);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                autoAlign.getTargetSpeeds(drive.getEstimatedPosition(), target.get()),
                  
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        
    }

    @Override
    public boolean isFinished() {
     double  deltarotation = Math.abs(Units.radiansToDegrees(MathUtil.angleModulus(drive.getEstimatedPosition().getRotation().minus(target.get().getRotation()).getRadians())));
        SmartDashboard.putNumber("is finished aligning", deltarotation );
        return deltarotation < rotationMOE && drive.getEstimatedPosition().getTranslation().minus(target.get().getTranslation()).getNorm() < translationMOE;
    }


}