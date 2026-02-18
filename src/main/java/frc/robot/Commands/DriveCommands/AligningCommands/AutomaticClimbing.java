package frc.robot.Commands.DriveCommands.AligningCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Subsystems.Drive.Drive;

public class AutomaticClimbing {
    Drive drive;
    ProfiledPIDCommand profiledPIDCommand;





    public AutomaticClimbing(Drive drive, AutoAlign autoAlign) {
        this.drive = drive;
           

    }

    public Command getClimbingCommand() {
        return new 
    }

    public Pose2d[] getClosestClimbPoses() {
        Pose2d blueRight = ClimbConstants.RightPoseBlue;
        
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            Pose2d blueLeft = FlipVertically_bottom_to_top(blueRight);

            if (drive.getEstimatedPosition().getTranslation().getDistance(blueRight.getTranslation()) < drive.getEstimatedPosition().getTranslation().getDistance(blueLeft.getTranslation())) {
                return new Pose2d[]{blueRight, blueRight.plus(new Transform2d(0,-1.5,Rotation2d.fromDegrees(0)))};
            } else {
                return new Pose2d[]{blueLeft, blueLeft.plus(new Transform2d(0,1.5,Rotation2d.fromDegrees(0)))};
            }
        }

        else {
            Pose2d redLeft = FlipHorizontally_BtoR(blueRight);
            Pose2d redRight = FlipVertically_bottom_to_top(redLeft);

            if (drive.getEstimatedPosition().getTranslation().getDistance(redRight.getTranslation()) < drive.getEstimatedPosition().getTranslation().getDistance(redLeft.getTranslation())) {
                return new Pose2d[]{redRight, redRight.plus(new Transform2d(0,1.5,Rotation2d.fromDegrees(0)))};
            } else {
                return new Pose2d[]{redLeft, redLeft.plus(new Transform2d(0,-1.5,Rotation2d.fromDegrees(0)))};
            }
        }



    }

    //flips translation2d from blue side to red side and vice versa
    public Pose2d FlipHorizontally_BtoR(Pose2d point) {
        return new Pose2d(new Translation2d( 2* (8.219694 - point.getX()) + point.getX(), point.getY()), point.getRotation().plus(Rotation2d.fromDegrees(180))); 
    }
    //flips translation2d from bottom of blue to top of blue
    public Pose2d FlipVertically_bottom_to_top(Pose2d point) {
        return new Pose2d(new Translation2d( point.getX(), 2* (4.021328 - point.getY()) + point.getY()), point.getRotation().plus(Rotation2d.fromDegrees(180))); 
     }


    






    
}
