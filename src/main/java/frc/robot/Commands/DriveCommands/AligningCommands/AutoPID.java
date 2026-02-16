package frc.robot.Commands.DriveCommands.AligningCommands;



import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;







/**
 * Can be used to aim a robot to a selected point. The translation will not be affected.
 */
public class AutoPID  {
 
    double kPtranslation;
    double kProtation;
    
   

    public AutoPID(double kPtranslation, double kProtation){
        this.kPtranslation = kPtranslation;
        this.kProtation = kProtation;
     
        // autoAmpPosePublisher = NetworkTableInstance.getDefault()
        //         .getStructTopic("Vision/autoAmpRobotPose", Pose2d.struct).publish();

        
    }

    /**
     * @param currentPose           the current pose of the robot
     * @param targetTrajectoryState the desired position and speeds of the robot
     * @return the {@code ChassisSpeeds} containing the desired rotation speed to look to the target
     */
    // @Override
    // public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, PathPlannerTrajectory.State targetTrajectoryState) {
    //     return null;
    // }

    /**
     *  converts {@code currentPose} and {@code targetPose} to {@code getTargetSpeeds}
     * @param currentPose the current pose of the robot
     * @param targetPose  the pose to aim towards
     * @return the {@code ChassisSpeeds} containing the desired rotation speed to look to the target
     */
    
    public ChassisSpeeds getTargetSpeeds2( Pose2d currentPose, Pose2d targetPose) {
        double xOffset = targetPose.getX() - currentPose.getX();
        double yOffset = targetPose.getY() - currentPose.getY();
        
        double Xspeed = xOffset * kPtranslation;
        double Yspeed = yOffset * kPtranslation;
    
        //Set deltaRotation in Radians
        double deltaRotation =targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        deltaRotation = MathUtil.angleModulus(deltaRotation);
        //Change back to degrees
        deltaRotation = Math.toDegrees(deltaRotation);
       
        double speedRotation = deltaRotation * kProtation;
        

        return new ChassisSpeeds(Xspeed, Yspeed, speedRotation);
    }
}