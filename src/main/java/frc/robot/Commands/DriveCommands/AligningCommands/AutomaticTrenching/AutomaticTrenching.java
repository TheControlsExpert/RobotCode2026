package frc.robot.Commands.DriveCommands.AligningCommands.AutomaticTrenching;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drive.Drive;

public class AutomaticTrenching {
    Drive swerve;

    PathConstraints constraints;
    boolean starting_from_middle = false;

    public AutomaticTrenching(Drive swervy, PathConstraints constraints) {
        this.swerve = swervy;
        this.constraints = constraints;

         
    }


    public Pose2d getClosestPathingPose() {
        Pose2d currentPose = swerve.getEstimatedPosition();
        Translation2d pathEnd_blue_bottom;

        if (currentPose.getX() > 4.574794 && currentPose.getX() < 2* (8.219694) - 4.574794) {
            //going from middle to ends

         pathEnd_blue_bottom = new Translation2d(2.148, 0.685);
         starting_from_middle = true;
        }

        else {
            //going from ends to middle
         starting_from_middle = false;
         pathEnd_blue_bottom = new Translation2d(7.466, 0.685);
        }
        

        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                double distance_bottom = currentPose.getTranslation().getDistance(pathEnd_blue_bottom);
                Translation2d pathStart_blue_top = FlipVertically_bottom_to_top(pathEnd_blue_bottom);
                double distance_top = currentPose.getTranslation().getDistance(pathStart_blue_top);
    
                if (distance_bottom < distance_top) {
                    return new Pose2d(pathEnd_blue_bottom, new Rotation2d(Math.PI));
                } else {
                    return new Pose2d(pathStart_blue_top, new Rotation2d(Math.PI));
                }

            // 
        }

        else {
            //assume red
            Translation2d pathStart_red_bottom = FlipHorizontally_BtoR(pathEnd_blue_bottom);
            Translation2d pathStart_red_top = FlipVertically_bottom_to_top(pathStart_red_bottom);


            double distance_bottom = currentPose.getTranslation().getDistance(pathStart_red_bottom);
            double distance_top = currentPose.getTranslation().getDistance(pathStart_red_top);

            if (distance_bottom < distance_top) {
                return new Pose2d(pathStart_red_bottom, new Rotation2d());
            } else {
                return new Pose2d(pathStart_red_top, new Rotation2d());
            }
        }

    } 




    //flips translation2d from blue side to red side
    public Translation2d FlipHorizontally_BtoR(Translation2d point) {
        return new Translation2d( 2* (8.219694 - point.getX()) + point.getX(), point.getY()); 
    }

    //flips translation2d from bottom of blue to top of blue
    public Translation2d FlipVertically_bottom_to_top(Translation2d point) {
        return new Translation2d( point.getX(), 2* (4.021328 - point.getY()) + point.getY()); 
     }

     public Command getPathingCommand() {
        Pose2d closestPose = getClosestPathingPose();
        Command pathfinding = AutoBuilder.pathfindToPose(closestPose, constraints);
        return pathfinding;
     }

     public boolean passedTrench() {
        if (starting_from_middle) {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                //we are on the end
                return swerve.getEstimatedPosition().getX() < 4.574794;
            }
            else {
                //we are on the end
                return swerve.getEstimatedPosition().getX() > 2* (8.219694) - 4.574794;
            }
        }

        else {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                //we are on the end
                return swerve.getEstimatedPosition().getX() > 4.574794;
            }
            else {
                //we are on the end
                return swerve.getEstimatedPosition().getX() < 2* (8.219694) - 4.574794;
            }
        }

     }

    
}
