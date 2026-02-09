package frc.robot.Commands.DriveCommands.AligningCommands;

import java.util.List;

import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drive.Drive;

public class AutomaticTrenching extends SequentialCommandGroup {
    Drive swerve;

    PathConstraints constraints;

    public AutomaticTrenching(Drive swervy) {
        this.swerve = swervy;

        constraints = new PathConstraints(swervy.getMaxLinearSpeedMetersPerSec(), swervy.getMaxLinearSpeedMetersPerSec() / 2, swervy.getMaxAngularSpeedRadPerSec(), swervy.getMaxAngularSpeedRadPerSec() / 2);
        PathPlannerPath trenchPath = PathPlannerPath.fromPathFile("Automatic Pathing Test");

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(swervy.get, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
);

        
        , new InstantCommand(swervy::resetGyro, swervy));

        
    }


    
}
