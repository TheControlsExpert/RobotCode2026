package frc.robot.Commands.DriveCommands.AligningCommands;


import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
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
public class AutoAlign {
    Transform2d deltaPose;
    double targetPoseX;
    double currentPoseX;
    double deltaX;
    double speedX;
    double targetPoseY;
    double currentPoseY;
    double deltaY;
    double speedY;
    double targetPoseRotation;
    double currentPoseRotation;
    double deltaRotation;
    double speedRotation;
    double kPtranslation;
    double kProtation;
    double translationMarge;
    double rotationMarge;
    private ProfiledPIDController pidDriveToPosX;
    private ProfiledPIDController pidDriveToPosY;
    private TrapezoidProfile.Constraints pidConstants;
    private double lastResetTime = -10;
    double pidPosCorrectieX, pidPosCorrectieY;
    private ChassisSpeeds robotSpeed = new ChassisSpeeds();


    private final double PID_PROFILE_MAX_VELOCITY = 4; // m/s
    private final double PID_PROFILE_MAX_ACCELERATION = 5; // m/s^2


    public AutoAlign(double kPtranslation, double kProtation, double translationMarge, double rotationMarge){
        this.kPtranslation = kPtranslation;
        this.kProtation = kProtation;
        this.translationMarge = translationMarge;
        this.rotationMarge = rotationMarge;

        // autoAmpPosePublisher = NetworkTableInstance.getDefault()
        //         .getStructTopic("Vision/autoAmpRobotPose", Pose2d.struct).publish();

        pidConstants = new TrapezoidProfile.Constraints(PID_PROFILE_MAX_VELOCITY, PID_PROFILE_MAX_ACCELERATION);
        pidDriveToPosX = new ProfiledPIDController(kPtranslation, 0, 0, pidConstants);
        pidDriveToPosY = new ProfiledPIDController(kPtranslation, 0, 0, pidConstants);
       
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
    
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, Pose2d targetPose){ //ChassisSpeeds currentSpeeds) {
        //reset of the pid
        if (1 <= (MathSharedStore.getTimestamp() - lastResetTime)) {
            pidDriveToPosX.reset(currentPose.getX(), robotSpeed.vxMetersPerSecond);
            pidDriveToPosY.reset(currentPose.getY(), robotSpeed.vyMetersPerSecond);
            lastResetTime = MathSharedStore.getTimestamp();

        } else {
            lastResetTime = MathSharedStore.getTimestamp();
        }

        targetPoseX = targetPose.getX();
        currentPoseX = currentPose.getX();
        deltaX = targetPoseX - currentPoseX;

        targetPoseY = targetPose.getY();
        currentPoseY = currentPose.getY();
        deltaY = targetPoseY - currentPoseY;

        // If the Y position of the robot is not in range of the target
        // then we offset the target X position
        // if(Math.abs(deltaX) > TARGET_IN_RANGE_THRESHOLD){
        //     targetPoseX -= TARGET_OUT_OF_RANGE_X_OFFSET;
        // }

        pidDriveToPosX.setGoal(targetPoseX);
        pidPosCorrectieX = pidDriveToPosX.calculate(currentPoseX, targetPoseX);
        double profileVX = pidDriveToPosX.getSetpoint().velocity;
        speedX = pidPosCorrectieX;

        pidDriveToPosY.setGoal(targetPoseY);
        pidPosCorrectieY = pidDriveToPosY.calculate(currentPoseY, targetPoseY);
        double profileVY = pidDriveToPosY.getSetpoint().velocity;
        speedY = pidPosCorrectieY;

       // SmartDashboard.putNumber("off by in x", pidPosCorrectieX);
       // SmartDashboard.putNumber("DT/auto_amping/pidY", pidPosCorrectieY);
        

        targetPoseRotation = targetPose.getRotation().getRadians();
        currentPoseRotation = currentPose.getRotation().getRadians();
        //Set deltaRotation in Radians
        deltaRotation = targetPoseRotation - currentPoseRotation;
        //Calculate the smallest angle, so for example 340 becomes 20
        deltaRotation = MathUtil.angleModulus(deltaRotation);
        //Change back to degrees
        deltaRotation = Math.toDegrees(deltaRotation);
        if (Math.abs(deltaRotation) <= 1) {
            speedRotation = 0;
        } else {
            speedRotation = deltaRotation * kProtation;
        }


        

       // autoAmpPosePublisher.set(targetPose);
        

        //Convert to chassis speed and return
        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            return new ChassisSpeeds(-speedX, -speedY, speedRotation);
        
         }

         else {
             return new ChassisSpeeds(speedX, speedY, speedRotation);
        }
        }

        
      
        
    

   
    public void setRobotSpeed(ChassisSpeeds robotSpeed) {
        this.robotSpeed = robotSpeed;
    }
}