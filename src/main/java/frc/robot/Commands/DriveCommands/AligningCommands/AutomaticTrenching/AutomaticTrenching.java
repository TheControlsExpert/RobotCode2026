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

    public AutomaticTrenching(Drive swervy, PathConstraints constraints) {
        this.swerve = swervy;
        this.constraints = constraints;

         
    }


    public Pose2d getClosestPathingPose() {
        Pose2d currentPose = swerve.getEstimatedPosition();

        Translation2d pathStart_blue_bottom = new Translation2d(2.148, 0.685);

        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                double distance_bottom = currentPose.getTranslation().getDistance(pathStart_blue_bottom);
                Translation2d pathStart_blue_top = FlipVertically_bottom_to_top(pathStart_blue_bottom);
                double distance_top = currentPose.getTranslation().getDistance(pathStart_blue_top);
    
                if (distance_bottom < distance_top) {
                    return new Pose2d(pathStart_blue_bottom, new Rotation2d(Math.PI));
                } else {
                    return new Pose2d(pathStart_blue_top, new Rotation2d(Math.PI));
                }

            // 
        }

        else {
            //assume red
            Translation2d pathStart_red_bottom = FlipHorizontally_BtoR(pathStart_blue_bottom);
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

    
}
