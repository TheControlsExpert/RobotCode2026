package frc.robot.Commands.DriveCommands.AligningCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;

public class RotationController extends Command{
        Drive drive;
        double kP_rotation;
        double rotationMOE;
        Rotation2d targetRotation;

    public RotationController(Drive drive, double kP_rotation, double rotationMOE, Rotation2d targetRotation) {
        this.drive = drive;
        this.kP_rotation = kP_rotation;
        this.rotationMOE = rotationMOE;
        this.targetRotation = targetRotation;
        addRequirements(drive);
    }

     @Override
     public void execute() {
            double deltaRotation = targetRotation.minus(drive.getEstimatedPosition().getRotation()).getRadians();
            double omega = Math.toDegrees(deltaRotation) * kP_rotation;
    
             boolean isFlipped = DriverStation.getAlliance().get().equals(Alliance.Red);
             drive.runVelocity(new ChassisSpeeds(0,0, omega));
         
     }

     @Override
     public boolean isFinished() {
        double  deltarotation = Math.abs(Units.radiansToDegrees(MathUtil.angleModulus(drive.getEstimatedPosition().getRotation().minus(targetRotation).getRadians())));
        SmartDashboard.putNumber("is finished aligning", deltarotation );
        return deltarotation < rotationMOE;
     }
    
}
