package frc.robot.Commands.DriveCommands.AligningCommands.AutomaticTrenching;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    //these define the trench x distance, and the distance to the middle of the field in the x direction
    double trench_start_x = 4.57454;
    double half_x_field = 8.219694;
    public AutomaticTrenching(Drive swervy, PathConstraints constraints) {
        this.swerve = swervy;
        this.constraints = constraints;        
    }


    
    //This method finds the closest goal-point out of the 4 on the field: red, blue, top bottom
    public Pose2d[] getClosestPathingPoses() {
        Pose2d currentPose = swerve.getEstimatedPosition(); // =current position of the robot
        Translation2d pathEnd_blue_bottom;
        Translation2d pathWaypoint_blue_bottom;
        
        //if you're in the middle of the field, and more on the blue side:
        if (currentPose.getX() > trench_start_x && currentPose.getX() < 2 * half_x_field - trench_start_x) {
            starting_from_middle = true;
            pathEnd_blue_bottom = new Translation2d(2.838, 0.685); //sets the goal point for blue bottom
            pathWaypoint_blue_bottom = new Translation2d(trench_start_x * 2 - 2.838, 0.685); //sets the waypoint point for blue bottom
         
        }

        // ?????????????????????????????
        else {
            starting_from_middle = false;
            pathEnd_blue_bottom = new Translation2d(trench_start_x * 2 - 2.838, 0.685);
            pathWaypoint_blue_bottom = new Translation2d(2.838, 0.685);

        }
        

        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                double distance_bottom = currentPose.getTranslation().getDistance(pathEnd_blue_bottom);
                Translation2d pathEnd_blue_top = FlipVertically_bottom_to_top(pathEnd_blue_bottom);
                Translation2d pathWaypoint_blue_top = FlipVertically_bottom_to_top(pathWaypoint_blue_bottom);
                double distance_top = currentPose.getTranslation().getDistance(pathEnd_blue_top);
    
                if (distance_bottom < distance_top) {
                    return new Pose2d[]{new Pose2d(pathEnd_blue_bottom, new Rotation2d(Math.PI)), new Pose2d(pathWaypoint_blue_bottom, new Rotation2d(Math.PI))};
                } else {
                    return new Pose2d[]{new Pose2d(pathEnd_blue_top, new Rotation2d(Math.PI)), new Pose2d(pathWaypoint_blue_top, new Rotation2d(Math.PI))};
                }

            // 
        }

        else {
            //assume red
            Translation2d pathEnd_red_bottom = FlipHorizontally_BtoR(pathEnd_blue_bottom);
            Translation2d pathWaypoint_red_bottom = FlipHorizontally_BtoR(pathWaypoint_blue_bottom);
            Translation2d pathEnd_red_top = FlipVertically_bottom_to_top(pathEnd_red_bottom);
            Translation2d pathWaypoint_red_top = FlipVertically_bottom_to_top(pathWaypoint_red_bottom);

            double distance_bottom = currentPose.getTranslation().getDistance(pathEnd_red_bottom);
            double distance_top = currentPose.getTranslation().getDistance(pathEnd_red_top);

            if (distance_bottom < distance_top) {
                return new Pose2d[]{new Pose2d(pathEnd_red_bottom, new Rotation2d()), new Pose2d(pathWaypoint_red_bottom, new Rotation2d())};
            } else {
                return new Pose2d[]{new Pose2d(pathEnd_red_top, new Rotation2d()), new Pose2d(pathWaypoint_red_top, new Rotation2d())};
            }
        }

    } 



    //flips translation2d from blue side to red side and vice versa
    public Translation2d FlipHorizontally_BtoR(Translation2d point) {
        return new Translation2d( 2* (8.219694 - point.getX()) + point.getX(), point.getY()); 
    }
    //flips translation2d from bottom of blue to top of blue
    public Translation2d FlipVertically_bottom_to_top(Translation2d point) {
        return new Translation2d( point.getX(), 2* (4.021328 - point.getY()) + point.getY()); 
     }




    //Determines the optimal path to take based on previously derived point coordinates, and returns a path-making command
     public Command getPathingCommand() {
        Pose2d[] closestPoses = getClosestPathingPoses(); // =creates an array of the two closest goal point and waypoint based on the current robot position
        
        // calculuates the minimum distance required to achieve max velocity in a trapezoidal profile
        double minDistance_trapezoidalprofile = 
            constraints.maxVelocity().baseUnitMagnitude() * constraints.maxVelocity().baseUnitMagnitude() / (constraints.maxAcceleration().baseUnitMagnitude());
        double distance = swerve.getEstimatedPosition().getTranslation().getDistance(closestPoses[0].getTranslation()); // calculates distance between robot and goal point

       
        double totalDistanceX = Math.abs(closestPoses[0].getX() - swerve.getEstimatedPosition().getX()); //finds the absolute x distance between the robot and goal point
        double cos_theta = totalDistanceX/distance; //cos theta = adjacent/hypotenuse |||| x distance/total distance
        double hypot_other = Math.abs(closestPoses[1].getX() - closestPoses[0].getX())/ cos_theta; //goal to waypoint section of hypotenuse
        double hypot_actual = distance - hypot_other; // robot to waypoint distance section of hypotenuse
            
        // if our velocity path can achieve max velocity |||| becomes a trapezoidal profile
        //we know we are far enough to reach cruise velocity
        // we now begin to work in the trapezoidal velocity plane
        if (distance > minDistance_trapezoidalprofile) {

            double point1 = minDistance_trapezoidalprofile/2; //the point at which max velocity is reached
            double point2 = distance - point1; // the point at which the velocity begins decreasing
            
            double vel_at_waypoint;
            if (hypot_actual > point2) { // if our waypoint occurs while the robot is decelerating
                 vel_at_waypoint = //kinematics equations define this velocity as a function of some random ahh variables
                     Math.sqrt(constraints.maxVelocityMPS() * constraints.maxVelocityMPS() - 2*constraints.maxAccelerationMPSSq() * (hypot_actual - point2));  
            }

            
            else if (hypot_actual > point1) { // if our waypoint occurs while in cruise velocity
                vel_at_waypoint = constraints.maxVelocityMPS();
            }
            
            else { // if our waypoint occurs while accelerating
                vel_at_waypoint = Math.sqrt(2*constraints.maxAccelerationMPSSq() * hypot_actual); // kinematics equations
             }

            
            List<Waypoint> firstPath_Waypoints = PathPlannerPath.waypointsFromPoses( //creating a list of the two points specific to the first path
                new Pose2d(swerve.getEstimatedPosition().getTranslation(), 
                           getPathVelocityHeading(swerve.getFieldRelativeSpeeds(), closestPoses[1])), // create a pose of our robot's position & heading
                closestPoses[1] //takes the waypoint from the poses array
            );

            List<Waypoint> secondPath_Waypoints = PathPlannerPath.waypointsFromPoses( // creating a list of the two points specific to the second path
                closestPoses[1], // waypoint
                closestPoses[0] // goal point
            );

            // creates the first PathPlannerPath with the first path waypoints, starting state, and end state
            PathPlannerPath firstPath = new PathPlannerPath(firstPath_Waypoints, constraints, 
            new IdealStartingState(ChassisSpeeds_to_Speed(swerve.getFieldRelativeSpeeds()), swerve.getEstimatedPosition().getRotation()), 
            new GoalEndState(vel_at_waypoint, closestPoses[1].getRotation()));

            // creates the second PathPlannerPat, with the second path waypoings, starting state, and end state
            PathPlannerPath secondPath = new PathPlannerPath(secondPath_Waypoints, constraints,
            new IdealStartingState(vel_at_waypoint, closestPoses[1].getRotation()),
            new GoalEndState(0, closestPoses[0].getRotation()));

            //?????????????????????????????
            firstPath.preventFlipping = true;
            secondPath.preventFlipping = true;

            // most important line, returns the command to follow one path, and then follow the other
            return AutoBuilder.followPath(firstPath).andThen(AutoBuilder.followPath(secondPath));
        }



        // if the velocity forms a triangular profile instead
        // all of the code is nearly exactly the same
        else {
            double halfpoint = minDistance_trapezoidalprofile/2;
            double vel_at_waypoint;

            if (hypot_actual > halfpoint) {
                vel_at_waypoint = Math.sqrt(constraints.maxVelocityMPS() * constraints.maxVelocityMPS() - 2*constraints.maxAccelerationMPSSq() * (hypot_actual - halfpoint));  
            }

            else {
                vel_at_waypoint = Math.sqrt(2*constraints.maxAccelerationMPSSq() * hypot_actual);
             }

            List<Waypoint> firstPath_Waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(swerve.getEstimatedPosition().getTranslation(), getPathVelocityHeading(swerve.getFieldRelativeSpeeds(), closestPoses[1])),
                closestPoses[1]
            );

            List<Waypoint> secondPath_Waypoints = PathPlannerPath.waypointsFromPoses(
                closestPoses[1],
                closestPoses[0]
            );

            PathPlannerPath firstPath = new PathPlannerPath(firstPath_Waypoints, constraints, 
            new IdealStartingState(ChassisSpeeds_to_Speed(swerve.getFieldRelativeSpeeds()), swerve.getEstimatedPosition().getRotation()), 
            new GoalEndState(vel_at_waypoint, closestPoses[1].getRotation()));

            PathPlannerPath secondPath = new PathPlannerPath(secondPath_Waypoints, constraints,
            new IdealStartingState(vel_at_waypoint, closestPoses[1].getRotation()),
            new GoalEndState(0, closestPoses[0].getRotation()));

            firstPath.preventFlipping = true;
            secondPath.preventFlipping = true;

            return AutoBuilder.followPath(firstPath).andThen(AutoBuilder.followPath(secondPath));     

            }
        }


                
        // uses pythagorean theorom and chassis speeds to calculuate the velocity vector the robot is moving in
        public double ChassisSpeeds_to_Speed(ChassisSpeeds cs) {
             return Math.sqrt(Math.pow(cs.vxMetersPerSecond, 2) + Math.pow(cs.vyMetersPerSecond, 2));
        }


    

     public boolean passedTrench() {
        if (starting_from_middle) {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                //we are on the end
                return swerve.getEstimatedPosition().getX() < 3.977894;
            }
            else {
                //we are on the end
                return swerve.getEstimatedPosition().getX() > 2* (8.219694) - 3.977894;
            }
        }

        else {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                //we are on the end
                return swerve.getEstimatedPosition().getX() > 5.171694;
            }
            else {
                //we are on the end
                return swerve.getEstimatedPosition().getX() < 2* (8.219694) - 5.171694;
            }
        }
     }  


     private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if ((cs.vxMetersPerSecond * cs.vxMetersPerSecond + cs.vyMetersPerSecond * cs.vyMetersPerSecond) < 0.25 * 0.25) {
            
            var diff = target.getTranslation().minus(swerve.getEstimatedPosition().getTranslation());
            
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }


        var rotation = new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
        return rotation;
    }
    }
