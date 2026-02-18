package frc.robot.Commands.DriveCommands.AligningCommands;

import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionSubsystem;
import frc.robot.Subsystems.Vision.VisionSubsystem.ServoState;

public class AutomaticClimbing {
    Drive drive;
    ProfiledPIDCommand profiledPIDCommand;
    AutoAlign autoAlign;
    VisionSubsystem vision;

    double translationalMOE = 0.3;
    boolean hasReachedFirstPose = false;
    boolean isClimbingRight = false;

    Timer timer = new Timer();



    public AutomaticClimbing(Drive drive, AutoAlign autoAlign, VisionSubsystem vision) {
        this.drive = drive;
        this.autoAlign = autoAlign;
        this.vision = vision;
           

    }

    public Command getClimbingCommand() {
        Pose2d[] climbPoses = getClosestClimbPoses();
        
        hasReachedFirstPose = false;

        return new InstantCommand(() -> {
            if (isClimbingRight) {
            vision.changeServoState(ServoState.CLIMB_RIGHT);
            } else {
            vision.changeServoState(ServoState.CLIMB_LEFT);
            };}).andThen(
                
            
        new ProfiledPIDCommand(autoAlign, drive,

        () -> {
        if (!hasReachedFirstPose) {
            if (drive.getEstimatedPosition().getTranslation().getDistance(climbPoses[1].getTranslation()) < translationalMOE) {
                hasReachedFirstPose = true;
                timer.restart();
            }
            return climbPoses[1];
        } else {
            return climbPoses[0].minus(climbPoses[1]);
        }
    }));




    }

    public Pose2d[] getClosestClimbPoses() {
        Pose2d blueRight = ClimbConstants.RightPoseBlue;
        
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            Pose2d blueLeft = FlipVertically_bottom_to_top(blueRight);

            if (drive.getEstimatedPosition().getTranslation().getDistance(blueRight.getTranslation()) < drive.getEstimatedPosition().getTranslation().getDistance(blueLeft.getTranslation())) {
                isClimbingRight = true;
                return new Pose2d[]{blueRight, blueRight.plus(new Transform2d(0,-1.5,Rotation2d.fromDegrees(0)))};
            } else {
                isClimbingRight = false;
                return new Pose2d[]{blueLeft, blueLeft.plus(new Transform2d(0,1.5,Rotation2d.fromDegrees(0)))};
            }
        }

        else {
            Pose2d redLeft = FlipHorizontally_BtoR(blueRight);
            Pose2d redRight = FlipVertically_bottom_to_top(redLeft);

            if (drive.getEstimatedPosition().getTranslation().getDistance(redRight.getTranslation()) < drive.getEstimatedPosition().getTranslation().getDistance(redLeft.getTranslation())) {
                isClimbingRight = true;
                return new Pose2d[]{redRight, redRight.plus(new Transform2d(0,1.5,Rotation2d.fromDegrees(0)))};
            } else {
                isClimbingRight = false;
                return new Pose2d[]{redLeft, redLeft.plus(new Transform2d(0,-1.5,Rotation2d.fromDegrees(0)))};
            }
        }



    }

    //flips translation2d from blue side to red side and vice versa
    public Pose2d FlipHorizontally_BtoR(Pose2d point) {
        return new Pose2d(new Translation2d( 2* (8.219694 - point.getX()) + point.getX(), point.getY()), point.getRotation());
    }

    //flips translation2d from bottom of blue to top of blue
    public Pose2d FlipVertically_bottom_to_top(Pose2d point) {
        return new Pose2d(new Translation2d( point.getX(), 2* (4.021328 - point.getY()) + point.getY()), point.getRotation().plus(Rotation2d.fromDegrees(180))); 
     }


    






    
}
