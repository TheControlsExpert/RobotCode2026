package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import frc.robot.Constants.Mode;
import frc.robot.Constants.SwerveConstants;
//import frc.robot.Subsystems.Superstructure.Superstructure;

import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = 250;
 //   Vector<N2> pose = VecBuilder.fill(0, 0);
  

  public LinearFilter filter = LinearFilter.movingAverage(10);
  public Pose2d estimatedPose = new Pose2d(0, 0, new Rotation2d());
  public Pose2d odometryPose = new Pose2d();
  public Pose2d lastodometrypose = new Pose2d();



  
  

  private TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(2);
  private TimeInterpolatableBuffer<Double> SDBufferX = TimeInterpolatableBuffer.createDoubleBuffer(2);
  private TimeInterpolatableBuffer<Double> SDBufferY = TimeInterpolatableBuffer.createDoubleBuffer(2);

private double[] sampleTimestamps;
private int sampleCount;
private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

private double accelX = 0;
private double accely = 0;
private double stdX_addition = 0.0;
private double stdY_addition = 0.0;
private Twist2d twist = new Twist2d();


private double accelX_fieldO = 0;
private double accelY_fieldO = 0;

private double ratio = 1;

private Pose2d odom_pose_at_time = new Pose2d();

private Transform2d backwards_twist = new Transform2d();

private Pose2d pose_at_time = new Pose2d();

private double xval = 0;
private double yval = 0;
private Transform2d forwardTransform = new Transform2d();
private double std_valx = 0;
private double std_valy = 0;
private double diff_x = 0;
private double diff_y = 0;

public double stdX = 100;
public double stdY = 100;

public double stdX_odom = 100;
public double stdY_odom = 100;

public double maxSpeed = 6;


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
  @AutoLogOutput
  double prevaccelX = 0;

  @AutoLogOutput
  double prevaccelY = 0;

  @AutoLogOutput
  double prevTime = 0;

  static final Lock odometryLock = new ReentrantLock();
  
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
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
          HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
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

//     // Configure AutoBuilder last
     AutoBuilder.configure(
             this::getEstimatedPosition, // Robot pose supplier
            this::resetPosition, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> runVelocity(speeds),
             // Method that will drive the robot gn ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
             new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(2, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.1, 0.0, 0.0) // Rotation PID constants
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
    // }
 


 





        

          
          
    
          
      
          // Configure AutoBuilder for PathPlanner
     
          // Configure SysId
          // sysId =
          //     new SysIdRoutine(
          //         new SysIdRoutine.Config(
          //             null,
          //             null,
          //             null,
          //             (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
          //         new SysIdRoutine.Mechanism(
          //             (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        
      
        @Override
        public void periodic() {
          if (stdX < 0.01 && DriverStation.isDisabled()) {
            stdX = 0.01;
          }

          if (stdY < 0.01 && DriverStation.isDisabled()) {
            stdY = 0.01;
          }
         m_field.setRobotPose(getEstimatedPosition());
          tester += 0.1;
          SmartDashboard.putNumber("tester", tester);
          //SmartDashboard.putNumber("bop bop", numTimes);
          odometryLock.lock(); // Prevents odometry updates while reading data
          gyroIO.updateInputs(gyroInputs);
          Logger.processInputs("Drive/Gyro", gyroInputs);
          for (var module : modules) {
            module.periodic();
          }
          odometryLock.unlock();
    
      
          // Stop moving when disabled
          if (DriverStation.isDisabled()) {
            for (var module : modules) {
              module.stop();
            }
          }

          // Log empty setpoint states when disabled
          if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
          }
      
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
    
            if (gyroInputs.connected) {
              // Use the real gyro angle
              rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
              // Use the angle delta from the kinematics and module deltas
              Twist2d twist = kinematics.toTwist2d(moduleDeltas);
              rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }
      

         
   
           SwervePoseEstimator.updateWithTime(sampleTimestamps[i],  rawGyroRotation, modulePositions);

      }
  
      // Update gyro alert
      gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getPositionRadians).toArray();
  }


  // public SwerveModuleState[] getModuleStates() {
  //   SwerveModuleState[] states = new SwerveModuleState[4];
  //   for (int i = 0; i < 4; i++) {
  //     states[i] = modules[i].getState();
  //   }
  //   return states;
  // }

  public void resetGyro() {
    int addOn = DriverStation.getAlliance().get().equals(Alliance.Red) ? 180 : 0;
    gyroIO.resetGyro(Rotation2d.fromDegrees(addOn));
  }

  public void resetGyro(double degrees) {
    gyroIO.resetGyro(Rotation2d.fromDegrees(degrees));
  }



  public void setPose(Pose2d pose) {
  odometryLock.lock();
  SwervePoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);  

  odometryLock.unlock();
  }
  
    
  //   poseLock.lock();
  //   poseBuffer.clear();
  //   odometryPose = pose;
  //   lastodometrypose = pose;
  //   estimatedPose = pose;
  //   stdX = 0.2;
  //   stdY = 0.2;

  //   stdX_odom = 0.2;
  //   stdY_odom = 0.2;

  //   poseLock.unlock();
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
     // ChassisSpeeds heightLimit = getNewTargetVelocity(discreteSpeeds);
      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec() );


      SmartDashboard.putNumber("accel", Math.abs(VecBuilder.fill(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond).minus(VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)).norm()));
      
      // Log unoptimized setpoints and setpoint speeds
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
  
      // Send setpoints to modules
      for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
      }
  
      // Log optimized setpoints (runSetpoint mutates each state)
      Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }
  
    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
      for (int i = 0; i < 4; i++) {
        modules[i].runCharacterization(output);
      }
    }


    // public ChassisSpeeds getNewTargetVelocity(ChassisSpeeds vel) {
    //  Vector<N2> accel =  VecBuilder.fill(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond).minus(VecBuilder.fill(vel.vxMetersPerSecond, vel.vyMetersPerSecond));
    //  //ChassisSpeeds newvel = vel;
    //  Vector<N2> velFixed = VecBuilder.fill(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond);
    //  //double maxAccel = (-0.08558 * Superstructure.encoderElevator + 4.426);
    //  //SmartDashboard.putNumber("max Accel", maxAccel);
    //  if (accel.norm() > maxAccel) {
    //    accel = accel.times( maxAccel/ accel.norm());
    //    velFixed = velFixed.plus(accel);
    //    SmartDashboard.putNumber("resulting velocity", velFixed.norm());
    //    SmartDashboard.putNumber("wanted velocity", Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond));
    //    return new ChassisSpeeds(velFixed.get(0), velFixed.get(1), vel.omegaRadiansPerSecond);
    //  }
    //   return vel;
    // }
  
    /** Stops the drive. */
    public void stop() {
      runVelocity(new ChassisSpeeds());
    }
  
    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
  //   public void stopWithX() {
  //     Rotation2d[] headings = new Rotation2d[4];
  //     for (int i = 0; i < 4; i++) {
  //       headings[i] = getModuleTranslations()[i].getAngle();
  //     }
  //     kinematics.resetHeadings(headings);
  //     stop();
  //   }
  
    /** Returns a command to run a quasistatic test in the specified direction. */
  //   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //     return run(() -> runCharacterization(0.0))
  //         .withTimeout(1.0)
  //         .andThen(sysId.quasistatic(direction));
  //   }
  
    /** Returns a command to run a dynamic test in the specified direction. */
  //   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //     return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  //   }
  
    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getState();
      }
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
    //   return SwervePoseEstimator.getEstimatedPosition();
    // }
  
  
    // public void resetGyro() {
    //   gyroIO.resetGyro();
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
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
      return kinematics.toChassisSpeeds(getModuleStates());
    }
  
    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
      double[] values = new double[4];
      for (int i = 0; i < 4; i++) {
        values[i] = modules[i].getWheelRadiusCharacterizationPosition();
      }
      return values;
    }
  

    public void addVision(Pose2d pose, double timestamp, double[] visionstds) {
      SmartDashboard.putBoolean("what the sigma", true);

      //add smthn to make sure timestamp is in correct interval
      //  odom_pose_at_time = poseBuffer.getSample(timestamp).get();
      //  //m_field.setRobotPose(odom_pose_at_time);
       
      //  //double[] stds_at_time_odom = {SDBufferX.getSample(timestamp).get(), SDBufferY.getSample(timestamp).get()};
      // // SmartDashboard.putNumber("stds at time", stds_at_time_odom[0]);

      //  backwards_twist = odom_pose_at_time.minus(odometryPose);
      // // SmartDashboard.putNumber("length of transform", backwards_twist.getY());

      //  //double[] backward_std = {stds_at_time_odom[0] - stdX_odom, stds_at_time_odom[1] - stdY_odom};

      // pose_at_time = new Pose2d(estimatedPose.getX() + backwards_twist.getX(), estimatedPose.getY() + backwards_twist.getY(), estimatedPose.getRotation());
       
      //  //estimatedPose.transformBy(backwards_twist);
      
      //  //SmartDashboard.putNumber("stds XXXXXXX :(", visionstds[0]);


      //  //double[] stds_at_time = {stdX - backward_std[0], stdY - backward_std[1]};
      //  //SmartDashboard.putNumber("stds at time????", stds_at_time[0]);

      // //Vector<N2> posVector = VecBuilder.fill(pose_at_time.getX(), pose_at_time.getY());

      //  xval = (1/(1/Math.pow(stdX, 2) + 1/Math.pow(visionstds[0], 2))) * (1/Math.pow(stdX, 2) * pose_at_time.getX() + 1/Math.pow(visionstds[0],2) * pose.getX()); 
      //  yval = (1/(1/Math.pow(stdY, 2) + 1/Math.pow(visionstds[1], 2))) * (1/Math.pow(stdY, 2) * pose_at_time.getY() + 1/Math.pow(visionstds[1],2) * pose.getY()); 

      //  forwardTransform = new Pose2d(xval, yval, pose_at_time.getRotation()).minus(pose_at_time);
      // estimatedPose = estimatedPose.plus(forwardTransform);


      //  stdX = 1 / Math.sqrt(1/(visionstds[0] * visionstds[0]) + 1/(stdX * stdX));
      

      //  stdY = 1 / Math.sqrt(1/(visionstds[1] * visionstds[1]) + 1/(stdY * stdY));



      //  diff_x = std_valx - stds_at_time[0];
      //  diff_y = std_valy - stds_at_time[1];

      // stdX = diff_x + stdX;
      // stdY = diff_y + stdY;

      Vector<N3> stds = VecBuilder.fill(visionstds[0], visionstds[1], 9999999);


      SwervePoseEstimator.addVisionMeasurement(new Pose2d(pose.getTranslation(), SwervePoseEstimator.getEstimatedPosition().getRotation()), timestamp, stds);
      //Vector<N2> estimation = ()
    }

    public Pose2d getEstimatedPosition() {
      return new Pose2d(SwervePoseEstimator.getEstimatedPosition().getTranslation(), gyroInputs.yawPosition);
    }


    public void resetPosition(Pose2d pose) {
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

  /** Returns the current odometry pose. */
  // @AutoLogOutput(key = "Odometry/Robot")
  // public Pose2d getPose() {
  //   return poseEstimator.getEstimatedPosition();
  // }

  /** Returns the current odometry rotation. */

  public Rotation2d getRotation() {
    return SwervePoseEstimator.getEstimatedPosition().getRotation();
  }

  /** Resets the current odometry pose. */
  // public void setPose(Pose2d pose) {
  //   poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  // }

  /** Adds a new timestamped vision measurement. */
  // public void addVisionMeasurement(
  //     Pose2d visionRobotPoseMeters,
  //     double timestampSeconds,
  //     Matrix<N3, N1> visionMeasurementStdDevs) {
  //   poseEstimator.addVisionMeasurement(
  //       visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  // }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    if (DriverStation.isAutonomous()) {
      return 5.5;
    }
    else {
      return maxSpeed;
    }
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() * 1  / SwerveConstants.DRIVE_BASE_RADIUS;
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
    return gyroIO.getRate();
  }


  // public Vector<N2> rotatebytheta(double theta, Vector<N2> vector) {
  //   Matrix rotationMatrix = 
  // }

  /** Returns an array of module translations. */
  
}