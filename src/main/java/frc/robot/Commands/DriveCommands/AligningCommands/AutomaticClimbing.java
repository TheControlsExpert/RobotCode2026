package frc.robot.Commands.DriveCommands.AligningCommands;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ClimbCommands.ClimbDown;
import frc.robot.Commands.ClimbCommands.ClimbUp;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class AutomaticClimbing {
    Drive drive;
    ProfiledPIDCommand profiledPIDCommand;
    AutoAlign autoAlign;
    VisionSubsystem vision;
    Climb climb;

    double translationalMOE = 0.1;
    boolean hasReachedFirstPose = false;
    boolean isClimbingRight = false;
    boolean isOverridePossible = false; //if the driver can override the climb
    double moving_setpoint_time = 0.15;

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("Target for Climbing", Pose2d.struct).publish(); 

    Timer timer = new Timer();



    public AutomaticClimbing(Drive drive, AutoAlign autoAlign, VisionSubsystem vision, Climb climb) {
        this.drive = drive;
        this.autoAlign = autoAlign;
        this.vision = vision;
        this.climb = climb;
    }

    public Command getClimbingCommand(boolean shouldbeSafe) {
        Pose2d[] climbPoses = getClosestClimbPoses();
        
        hasReachedFirstPose = false;
        
        if (DriverStation.isTeleop()) { //if we're in teleop, only align, climbing will be done manually

            return new ParallelCommandGroup(new ClimbUp(climb), 
            new ProfiledPIDCommand(autoAlign, drive,

            () -> {
                if (!hasReachedFirstPose) {
                    
                    publisher.set(climbPoses[1]);

                    if (drive.getEstimatedPosition().getTranslation().getDistance(climbPoses[1].getTranslation()) < translationalMOE) {
                        hasReachedFirstPose = true;
                        timer.restart();
                    }
                    return climbPoses[1];
                } 
                
                else {
                    publisher.set(climbPoses[2]); 
                    return climbPoses[1].transformBy(climbPoses[0].minus(climbPoses[1]).times(MathUtil.clamp(timer.get() / moving_setpoint_time, 0, 1)));
                }
            
            }));
        }


        else  { //if we're in autonomous
        if (!shouldbeSafe) {

            return new ParallelCommandGroup(new ClimbUp(climb), new ProfiledPIDCommand(autoAlign, drive,

    () -> {
        publisher.set(climbPoses[2]);
        return climbPoses[0];}));
}

    else {
         return new ParallelCommandGroup(new ClimbUp(climb), 
            new ProfiledPIDCommand(autoAlign, drive,

            () -> {
                if (!hasReachedFirstPose) {
                    
                    publisher.set(climbPoses[1]);

                    if (drive.getEstimatedPosition().getTranslation().getDistance(climbPoses[1].getTranslation()) < translationalMOE) {
                        hasReachedFirstPose = true;
                        timer.restart();
                    }
                    return climbPoses[1];
                } 
                
                else {
                    publisher.set(climbPoses[2]); 
                    return climbPoses[1].transformBy(climbPoses[0].minus(climbPoses[1]).times(MathUtil.clamp(timer.get() / moving_setpoint_time, 0, 1)));
                }
            
            }));

    }
        }
}

    public Pose2d[] getClosestClimbPoses() {
        SmartDashboard.putBoolean("isClimbingRight", isClimbingRight);
        Pose2d blueRight = ClimbConstants.RightPoseBlue;
        Pose2d blueRightAdjusted = ClimbConstants.RightPoseBlueAdjusted;

        Pose2d blueLeft = FlipVertically_bottom_to_top(blueRight);
        Pose2d blueLeftAdjusted = FlipVertically_bottom_to_top(blueRightAdjusted);
        
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
           

            if (drive.getEstimatedPosition().getTranslation().getDistance(blueRight.getTranslation()) < drive.getEstimatedPosition().getTranslation().getDistance(blueLeft.getTranslation())) {
                isClimbingRight = true;

                return new Pose2d[]{blueRight, blueRight.plus(new Transform2d(0.0,0.4,Rotation2d.fromDegrees(0))), blueRightAdjusted};
            } else {
                isClimbingRight = false;
                return new Pose2d[]{blueLeft, blueLeft.plus(new Transform2d(0.0,0.4,Rotation2d.fromDegrees(0))), blueLeftAdjusted};
            }
        }

        else {
            
            Pose2d redRight = FlipVertically_bottom_to_top_halfpoint(FlipHorizontally_BtoR(blueRight));
            Pose2d redLeft = FlipVertically_bottom_to_top_halfpoint(FlipHorizontally_BtoR(blueLeft));

            Pose2d redRightAdjusted = FlipVertically_bottom_to_top(blueRightAdjusted);
            Pose2d redLeftAdjusted = FlipVertically_bottom_to_top(blueLeftAdjusted);

            if (drive.getEstimatedPosition().getTranslation().getDistance(redRight.getTranslation()) < drive.getEstimatedPosition().getTranslation().getDistance(redLeft.getTranslation())) {
                isClimbingRight = true;
                return new Pose2d[]{redRight, redRight.plus(new Transform2d(0.0, 0.4,Rotation2d.fromDegrees(0))), redRightAdjusted};
            } else {
                isClimbingRight = false;
                return new Pose2d[]{redLeft, redLeft.plus(new Transform2d(0, 0.4, Rotation2d.fromDegrees(0))), redLeftAdjusted};
            }
        }
    }


    public boolean isOverridePossible() {

        if (drive.getEstimatedPosition().getTranslation().getDistance(getClosestClimbPoses()[2].getTranslation()) < 0.07 && //makes sure the bot is close enough to the tower
         climb.getEncoderValue() > ClimbConstants.upperLimit) //makes sure the climb is actually completely up

        { isOverridePossible = true;}

        return isOverridePossible;
    }




    //flips translation2d from blue side to red side and vice versa
    public Pose2d FlipHorizontally_BtoR(Pose2d point) {
        return new Pose2d(new Translation2d( 2* (8.219694 - point.getX()) + point.getX(), point.getY()), point.getRotation());
    }

    //flips translation2d from bottom of blue to top of blue
    public Pose2d FlipVertically_bottom_to_top(Pose2d point) {
        return new Pose2d(new Translation2d( point.getX()-0.0254 * 2, 2* (3.745611 - point.getY()) + point.getY()), point.getRotation().plus(Rotation2d.fromDegrees(180))); 
     }

    public Pose2d FlipVertically_bottom_to_top_halfpoint(Pose2d point) {
        return new Pose2d(new Translation2d( point.getX(), 2* (4.021328 - point.getY()) + point.getY()), point.getRotation().plus(Rotation2d.fromDegrees(180))); 

     }


    






    
}
