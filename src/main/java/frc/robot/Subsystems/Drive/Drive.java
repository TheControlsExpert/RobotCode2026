package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Robot.ShootingState;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
//import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Vision.VisionSubsystem.VisionMeasurement;

import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
 // TunerConstants doesn't include these constants, so they are declared locally
 static final double ODOMETRY_FREQUENCY = 150;
 //Vector<N3> visionSTDs = VecBuilder.fill(0.1, 0.1, 999999999); 
 // Vector<N2> pose = VecBuilder.fill(0, 0);

 Timer gyroResetTimer = new Timer();
 public Rotation2d simRotation = new Rotation2d();
 public Pose2d estimatedPose = new Pose2d(0, 0, new Rotation2d());
 public Pose2d odometryPose = new Pose2d();
 public Pose2d lastodometrypose = new Pose2d();
 ReentrantLock visionLock = new ReentrantLock();
public final double translationkP = 2.5;
public final double rotationkP = 0.10;
public PathConstraints constraints_auto = new PathConstraints(3, 3, 13, 26);
public PathConstraints constraints_pathfinding = new PathConstraints(3, 3, 500, 500);

private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
Transform2d simulatedLL = new Transform2d(new Translation2d(SwerveConstants.wheelBase / 2, -SwerveConstants.trackWidth / 2), Rotation2d.fromDegrees(-160));



private Twist2d twist = new Twist2d();

public double maxSpeed = 5;


public boolean resettingLocalization = false;
public double tester = 0.1;






SwerveModuleState[] mods = new SwerveModuleState[] {
 new SwerveModuleState(),
 new SwerveModuleState(),
 new SwerveModuleState(),
 new SwerveModuleState()
};


 

 



 

 

 // PathPlanner config constants
 private static final double ROBOT_MASS_KG = 74.088; 
private final Field2d m_field = new Field2d();
 //private ReentrantLock poseLock = new ReentrantLock();
// Do this in either robot or subsystem init

 private static final double ROBOT_MOI = 6.883;
 private static final double WHEEL_COF = 1.2;


 static final Lock odometryLock = new ReentrantLock();



 
 private final GyroIO gyroIO;
 private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
 private final Module[] modules = new Module[4]; // FL, FR, BL, BR

 private double lastRotation = 0.0;
 //private final SysIdRoutine sysId;
 private final Alert gyroDisconnectedAlert =
 new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

 private SwerveDriveKinematics kinematics = SwerveConstants.swerveKinematics;
 private Rotation2d rawGyroRotation = new Rotation2d();
 public SwerveModulePosition[] lastModulePositions = // For delta tracking
 new SwerveModulePosition[] {
 new SwerveModulePosition(),
 new SwerveModulePosition(),
 new SwerveModulePosition(),
 new SwerveModulePosition()
 };
 private SwerveDrivePoseEstimator SwervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d(0, 0, new Rotation2d()), VecBuilder.fill(0.005,0.005, Radians.convertFrom(5, Degrees)), VecBuilder.fill(0.05, 0.05, 999999999) );
 private boolean wasGyroDisconnected = false;
 SysIdRoutine routine;
 
 
 
 
 private int numTimes = 0;
 private double lastgyro = 0.0;
 
 
 public Drive(
 GyroIO gyroIO,
 ModuleIO flModuleIO,
 ModuleIO frModuleIO,
 ModuleIO blModuleIO,
 ModuleIO brModuleIO) {
 //this.initTime = Timer.getFPGATimestamp();
 this.gyroIO = gyroIO;
 
 modules[0] = new Module(flModuleIO,0, SwerveConstants.Mod0.constants);
 modules[1] = new Module(frModuleIO, 1, SwerveConstants.Mod1.constants);
 modules[2] = new Module(blModuleIO, 2, SwerveConstants.Mod2.constants);
 modules[3] = new Module(brModuleIO, 3,SwerveConstants.Mod3.constants);
 
 // Usage reporting for swerve template
 //HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
 SmartDashboard.putData("Field", m_field);
 // Start odometry thread
 PhoenixOdometryThread.getInstance().start();

 RobotConfig config = null;
 try{
 config = RobotConfig.fromGUISettings();
 } catch (Exception e) {
 // Handle exception as needed
 e.printStackTrace();
 }

 gyroResetTimer.start();

// // Configure AutoBuilder last
 AutoBuilder.configure(
 this::getEstimatedPosition, // Robot pose supplier
 this::resetPosition, // Method to reset odometry (will be called if your auto has a starting pose)
 this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
 (speeds, feedforwards) -> runVelocity(speeds),
 // Method that will drive the robot gn ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
 new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
 new PIDConstants(translationkP, 0.0, 0.8), // Translation PID constants
 new PIDConstants(2.5, 0.0, 0.0) // Rotation PID constants
 ),
 
 config, // The robot configuration
 () -> {return false;},
 this // Reference to this subsystem to set requirements
 );

 routine =
 new SysIdRoutine(
 new SysIdRoutine.Config(
 null,
 null,
 null,
 (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
 new SysIdRoutine.Mechanism(
 (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
 }
 
 @Override
 public void periodic() {
    SmartDashboard.putNumber("gyro", SwervePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    m_field.setRobotPose(SwervePoseEstimator.getEstimatedPosition()); 
    SmartDashboard.putNumber("velocity of chassis", Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond));
    SmartDashboard.putNumber("distance to center", SwervePoseEstimator.getEstimatedPosition().getTranslation().getDistance(calculateShootingPosition(0)));

if (DriverStation.isDisabled()) {
 if (!gyroInputs.connected && !wasGyroDisconnected) {
    Robot.reportDisconnection("Gyro");
    wasGyroDisconnected = true;
 }
 if (wasGyroDisconnected && gyroInputs.connected) {
    Robot.removeDisconnection("Gyro");
    wasGyroDisconnected = false;
 }
}
 
//  if (DriverStation.isAutonomous() && PathPlannerAuto.currentPathName != null) {
//  PathPlannerLogging.setLogActivePathCallback((poses) -> {
//  // Do whatever you want with the poses here
//  m_field.getObject("path").setPoses(poses);
//  });
//  PathPlannerLogging.setLogTargetPoseCallback((pose) -> {SmartDashboard.putNumber("error x", pose.getX() - getEstimatedPosition().getX());
//  SmartDashboard.putNumber("error y", pose.getY() - getEstimatedPosition().getY());
//  SmartDashboard.putNumber("error rotation", pose.getRotation().getDegrees());
//  });

 
//  }

 

 
 odometryLock.lock(); // Prevents odometry updates while reading data
 gyroIO.updateInputs(gyroInputs);
 
 //Logger.processInputs("Drive/Gyro", gyroInputs);
 for (var module : modules) {
 module.periodic();
 }
 odometryLock.unlock();
 
 
 // Stop moving when disabled
 if (DriverStation.isDisabled() && RobotBase.isReal()) {
 for (var module : modules) {
 module.stop();
 }
 }

 // Log empty setpoint states when disabled

 if (RobotBase.isReal()) {
 
 // Update odometry
 double[] sampleTimestamps =
 modules[0].getOdometryTimestamps(); // All signals are sampled together
 int sampleCount = sampleTimestamps.length;
 
 
 for (int i = 0; i < sampleCount; i++) {
 // Read wheel positions and deltas from each module
 // double vx = 0;
 // double vy = 0;
 modulePositions = new SwerveModulePosition[4];
 moduleDeltas = new SwerveModulePosition[4];
 for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
 modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
 moduleDeltas[moduleIndex] =
 new SwerveModulePosition(
 modulePositions[moduleIndex].distanceMeters
 - lastModulePositions[moduleIndex].distanceMeters,
 modulePositions[moduleIndex].angle);
 lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
 }
 
 twist = kinematics.toTwist2d(moduleDeltas);
 
 if ( gyroInputs.connected) {
 // Use the real gyro angle
 rawGyroRotation = gyroInputs.odometryYawPositions[i];
 } else {
 // Use the angle delta from the kinematics and module deltas
 Twist2d twist = kinematics.toTwist2d(moduleDeltas);
 rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
 }
 
 

 
 //visionLock.lock();
 SwervePoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
// visionLock.unlock();
 }
 }

 else {
 double timestamp = Timer.getFPGATimestamp();
 modulePositions = new SwerveModulePosition[4];
 for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
 modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[0];

 
 moduleDeltas[moduleIndex] =
 new SwerveModulePosition(
 modulePositions[moduleIndex].distanceMeters 
 - lastModulePositions[moduleIndex].distanceMeters,
 modulePositions[moduleIndex].angle);
 lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
 }
 
 twist = kinematics.toTwist2d(moduleDeltas);
 simRotation = simRotation.plus(new Rotation2d(twist.dtheta));
 // if ( PathPlannerAuto.currentPathName != null) {
 // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {simRotation = new Rotation2d(pose.getRotation().getRadians());});

 // }
 


 SwervePoseEstimator.updateWithTime(timestamp, simRotation , modulePositions);

 //prevTime = Timer.getFPGATimestamp();
 


 }
 
 // Update gyro alert
 gyroDisconnectedAlert.set(!gyroInputs.connected);
LimelightHelpers.SetRobotOrientation("limelight-four", SwervePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
LimelightHelpers.SetRobotOrientation("limelight-threegf", SwervePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
LimelightHelpers.SetRobotOrientation("limelight-threegs", SwervePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

 
 }


 public double[] getWheelRadiusCharacterizationPosition() {
 return Arrays.stream(modules).mapToDouble(Module::getPositionRadians).toArray();
 }


 // public SwerveModuleState[] getModuleStates() {
 // SwerveModuleState[] states = new SwerveModuleState[4];
 // for (int i = 0; i < 4; i++) {
 // states[i] = modules[i].getState();
 // }
 // return states;
 // }



 public void setPose(Pose2d pose) {
 odometryLock.lock();
 SwervePoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose); 

 odometryLock.unlock();
 }
 
 
 // poseLock.lock();
 // poseBuffer.clear();
 // odometryPose = pose;
 // lastodometrypose = pose;
 // estimatedPose = pose;
 // stdX = 0.2;
 // stdY = 0.2;

 // stdX_odom = 0.2;
 // stdY_odom = 0.2;

 // poseLock.unlock();
 // }
 
 // gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
 
 
 /**
 * Runs the drive at the desired velocity.
 *
 * @param speeds Speeds in meters/sec
 */
 public void runVelocity(ChassisSpeeds speeds) {
 // Calculate module setpoints
 ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

//  if (RobotContainer.isShooting) {

//  double x = discreteSpeeds.vxMetersPerSecond;
//  double y = discreteSpeeds.vyMetersPerSecond;

// //  if (x > 1.15) {

// //     discreteSpeeds = discreteSpeeds.times(1.15/x);
// //  }

// //  if (y > 1.55) {
// //     discreteSpeeds = discreteSpeeds.times(1.55/x);
// //  }
// }
 

 // ChassisSpeeds heightLimit = getNewTargetVelocity(discreteSpeeds);
 SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

 SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());


 //SmartDashboard.putNumber("accel", Math.abs(VecBuilder.fill(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond).minus(VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)).norm()));
 
 // Log unoptimized setpoints and setpoint speeds
 //Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
 //Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
 
 // Send setpoints to modules
 for (int i = 0; i < 4; i++) {
 modules[i].runSetpoint(setpointStates[i]);
 }
 
 // Log optimized setpoints (runSetpoint mutates each state)
 //Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
 }
 
 /** Runs the drive in a straight line with the specified drive output. */
 public void runCharacterization(double output) {
 for (int i = 0; i < 4; i++) {
 modules[i].runCharacterization(output);
 }
 }


 // public ChassisSpeeds getNewTargetVelocity(ChassisSpeeds vel) {
 // Vector<N2> accel = VecBuilder.fill(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond).minus(VecBuilder.fill(vel.vxMetersPerSecond, vel.vyMetersPerSecond));
 // //ChassisSpeeds newvel = vel;
 // Vector<N2> velFixed = VecBuilder.fill(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond);
 // //double maxAccel = (-0.08558 * Superstructure.encoderElevator + 4.426);
 // //SmartDashboard.putNumber("max Accel", maxAccel);
 // if (accel.norm() > maxAccel) {
 // accel = accel.times( maxAccel/ accel.norm());
 // velFixed = velFixed.plus(accel);
 // SmartDashboard.putNumber("resulting velocity", velFixed.norm());
 // SmartDashboard.putNumber("wanted velocity", Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond));
 // return new ChassisSpeeds(velFixed.get(0), velFixed.get(1), vel.omegaRadiansPerSecond);
 // }
 // return vel;
 // }
 
 /** Stops the drive. */
 public void stop() {
 runVelocity(new ChassisSpeeds());
 }

 public void setTraj() {
 PathPlannerLogging.setLogActivePathCallback((poses) -> {
 // Do whatever you want with the poses here
 m_field.getObject("path").setPoses(poses);
 });
 }

 public double getAngularSpeed() {
 return getRobotRelativeSpeeds().omegaRadiansPerSecond;
 }

 public Translation2d calculateShootingPosition(double time) {
        if (Robot.shootingState.equals(ShootingState.SHOOTING)) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                return new Translation2d(4.626, 4.034);
            }

            else {
                return FlipHorizontally_BtoR(new Translation2d(4.626, 4.034));
            }
        }

        else {
            Translation2d bottomBlue = new Translation2d(1.15, 1.5);
            double period = 12/10;
            double angle = time/period * 2 * Math.PI;

            Translation2d circleRandomness = new Translation2d(Math.cos(angle) * 0.25, Math.sin(angle) * 0.7);

           if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                Translation2d topBlue = FlipVertically_bottom_to_top(bottomBlue);

                double distanceBottom = bottomBlue.getDistance(getEstimatedPosition().getTranslation());
                double distanceTop = topBlue.getDistance(getEstimatedPosition().getTranslation());
                if (distanceBottom < distanceTop) {
                    return bottomBlue.plus(circleRandomness);
                } else {
                    return topBlue.plus(circleRandomness);
                }
            }

            else {
                Translation2d bottomRed = FlipHorizontally_BtoR(new Translation2d(1.15, 1.5));
                Translation2d topRed = FlipVertically_bottom_to_top(bottomRed);

                double distanceBottom = bottomRed.getDistance(getEstimatedPosition().getTranslation());
                double distanceTop = topRed.getDistance(getEstimatedPosition().getTranslation());
                if (distanceBottom < distanceTop) {
                    return bottomRed.plus(circleRandomness);
                } else {
                    return topRed.plus(circleRandomness);
            }
           
        }
    }
}


 
    public static Translation2d FlipHorizontally_BtoR(Translation2d point) {
        return new Translation2d( 2* (8.219694 - point.getX()) + point.getX(), point.getY()); 
    }

     public Translation2d FlipVertically_bottom_to_top(Translation2d point) {
        return new Translation2d( point.getX(), 2* (4.021328 - point.getY()) + point.getY()); 
     }


 /**
 * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
 * return to their normal orientations the next time a nonzero velocity is requested.
 */
 // public void stopWithX() {
 // Rotation2d[] headings = new Rotation2d[4];
 // for (int i = 0; i < 4; i++) {
 // headings[i] = getModuleTranslations()[i].getAngle();
 // }
 // kinematics.resetHeadings(headings);
 // stop();
 // }
 
 /** Returns a command to run a quasistatic test in the specified direction. */
 // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
 // return run(() -> runCharacterization(0.0))
 // .withTimeout(1.0)
 // .andThen(sysId.quasistatic(direction));
 // }
 
 /** Returns a command to run a dynamic test in the specified direction. */
 // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
 // return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
 // }
 
 /** Returns the module states (turn angles and drive velocities) for all of the modules. */
 //@AutoLogOutput(key = "SwerveStates/Measured")
 private SwerveModuleState[] getModuleStates() {
 SwerveModuleState[] states = new SwerveModuleState[4];
 double totalSpeed = 0;
 for (int i = 0; i < 4; i++) {
 states[i] = modules[i].getState();
 totalSpeed += states[i].speedMetersPerSecond;
 }
 //SmartDashboard.putNumber("avg motor speed", totalSpeed / 4);

 
 return states;
 }


 public ChassisSpeeds getRobotRelativeSpeeds() {
 return kinematics.toChassisSpeeds(getModuleStates());
 }

 private static Translation2d convertSwerveStateToVelocityVector(SwerveModuleState swerveModuleState) {
 return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
 }

 public static double getSkiddingRatio(SwerveModuleState[] swerveStatesMeasured, SwerveDriveKinematics swerveDriveKinematics) {
 final double angularVelocityOmegaMeasured = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
 final SwerveModuleState[] swerveStatesRotationalPart = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));
 final double[] swerveStatesTranslationalPartMagnitudes = new double[swerveStatesMeasured.length];

 for (int i =0; i < swerveStatesMeasured.length; i++) {
 final Translation2d swerveStateMeasuredAsVector = convertSwerveStateToVelocityVector(swerveStatesMeasured[i]),
 swerveStatesRotationalPartAsVector = convertSwerveStateToVelocityVector(swerveStatesRotationalPart[i]),
 swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
 swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
 }

 double maximumTranslationalSpeed = 0, minimumTranslationalSpeed = Double.POSITIVE_INFINITY;
 for (double translationalSpeed:swerveStatesTranslationalPartMagnitudes) {
 maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
 minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
 }

 return maximumTranslationalSpeed / minimumTranslationalSpeed;
 }
 
 
 // public Pose2d getPose() {
 // return SwervePoseEstimator.getEstimatedPosition();
 // }
 
 
 // public void resetGyro() {
 // gyroIO.resetGyro();
 // }
 
 /** Returns the module positions (turn angles and drive positions) for all of the modules. */
 private SwerveModulePosition[] getModulePositions() {
 SwerveModulePosition[] states = new SwerveModulePosition[4];
 for (int i = 0; i < 4; i++) {
 states[i] = modules[i].getPosition();
 }
 return states;
 }
 
 /** Returns the measured chassis speeds of the robot. */
 //@AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
 public ChassisSpeeds getChassisSpeeds() {
 return kinematics.toChassisSpeeds(getModuleStates());
 }

 public ChassisSpeeds getFieldRelativeSpeeds() {
 
 return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(),getEstimatedPosition().getRotation());
 
 
 }
 
 /** Returns the position of each module in radians. */
 public double[] getWheelRadiusCharacterizationPositions() {
 double[] values = new double[4];
 for (int i = 0; i < 4; i++) {
 values[i] = modules[i].getWheelRadiusCharacterizationPosition();
 }
 return values;
 }
 

 public void addVision(VisionMeasurement measurement) {
 Vector<N3> stds = VecBuilder.fill(measurement.std()[0], measurement.std()[1], 9999999);
 //SmartDashboard.putBoolean("adding vision", true);
 //visionLock.lock();
    SmartDashboard.putBoolean("angle conditions", Math.abs(gyroInputs.rollDegrees) < 1 || Math.abs(gyroInputs.pitchDegrees) < 1);
 if (Math.abs(gyroInputs.rollDegrees) < 5 && Math.abs(gyroInputs.pitchDegrees) < 5 && getGyroSpeed() < 180 && getTranslationalSpeed() < 5) {

 SwervePoseEstimator.addVisionMeasurement(new Pose2d(measurement.pose().getTranslation(), getRotation()), measurement.timestamp(), stds);

 if (gyroResetTimer.hasElapsed(15) && getGyroSpeed() < 1 && getTranslationalSpeed() < 0.1 && measurement.numTags() >= 2 && measurement.avgDistance() < 2.35) {
 SwervePoseEstimator.resetRotation(Rotation2d.fromDegrees(measurement.rotationDegreees()));
 //SmartDashboard.putBoolean("gyro reset", true);
 gyroResetTimer.restart();
 }
 /// SmartDashboard.putBoolean("gyro reset", false);

 

 }
 
 //visionLock.unlock();
 }

 public Pose2d getEstimatedPosition() {
 return SwervePoseEstimator.getEstimatedPosition();
 }


 public void resetPosition(Pose2d pose) {
 visionLock.lock();
 
 SwervePoseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
 visionLock.unlock();
 //poseLock.lock();
 // = pose;
 //gyroIO.resetGyro(pose.getRotation());


 //stdX = 0.1;
 //stdY = 0.1;
 //poseLock.unlock();
 }
 
// 
 /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
 public double getFFCharacterizationVelocity() {
 double output = 0.0;
 for (int i = 0; i < 4; i++) {
 output += modules[i].getFFCharacterizationVelocity() / 4.0;
 }
 return output;
 }

 public double getFFCharacterizationAcceleration() {
 double output = 0.0;
 for (int i = 0; i < 4; i++) {
 output += modules[i].getFFCharacterizationAcceleration() / 4.0;
 }
 return output;
 
 }

 /** Returns the current odometry pose. */
 // @AutoLogOutput(key = "Odometry/Robot")
 // public Pose2d getPose() {
 // return poseEstimator.getEstimatedPosition();
 // }

 /** Returns the current odometry rotation. */

 public Rotation2d getRotation() {
 return SwervePoseEstimator.getEstimatedPosition().getRotation();
 }

 public double getTranslationalSpeed() {
    return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
 }

 /** Resets the current odometry pose. */
 // public void setPose(Pose2d pose) {
 // poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
 // }

 /** Adds a new timestamped vision measurement. */
 // public void addVisionMeasurement(
 // Pose2d visionRobotPoseMeters,
 // double timestampSeconds,
 // Matrix<N3, N1> visionMeasurementStdDevs) {
 // poseEstimator.addVisionMeasurement(
 // visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
 // }

 /** Returns the maximum linear speed in meters per sec. */
 public double getMaxLinearSpeedMetersPerSec() {
 if (DriverStation.isAutonomous()) {
 return 6;
 }

 else if (RobotContainer.isShooting) {
return 1.65;
 }
 else {
 return 5;
 }
 }

 /** Returns the maximum angular speed in radians per sec. */
 public double getMaxAngularSpeedRadPerSec() {
 return getMaxLinearSpeedMetersPerSec() * 1 / SwerveConstants.DRIVE_BASE_RADIUS;
 }

 public boolean isGyroConnected() {
 return gyroInputs.connected;
 }


 public Command getPathFollowingCommand(String pathName) {
 try {
 PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
 return AutoBuilder.followPath(path);
 }

 catch (Exception e) {
 DriverStation.reportError("oops, couldnt find path:" + e.getMessage(), e.getStackTrace());
 return Commands.none();
 }
 
 }


 public double getGyroSpeed() {
 return Math.abs(Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec));
 }
}
