package frc.robot.Subsystems.Vision;

import java.lang.reflect.Field;
import java.util.ArrayList;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;
import edu.wpi.first.cameraserver.CameraServer;

public class VisionSubsystem extends SubsystemBase {
   //Field2d field = new Field2d();



    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
        private Drive drive;

        boolean disable_all_cameras = false;
   
        
    double lastUsedTimestamp = -1000;
    private double minTranslation = 10000.0;   
    double velocity_of_servo = 0.0;
    public boolean ruin = false;
    public record VisionMeasurement(Pose2d pose, double rotationDegreees, double timestamp, double[] std, int numTags, double avgDistance) {}
    ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();


    boolean wasDisconnected_LL4 = false;
    boolean wasDisconnected_LL3GS = false;
    boolean wasDisconnected_LL3GF = false;
    boolean disable_other_cameras = false;
 

            
    public VisionSubsystem(VisionIO io, Drive drive) {
                    this.io = io;
                    this.drive = drive;
                   //SmartDashboard.putData("field", field);
                   getWebcamFeed(); //begins sending webcam video footage to smart dashboard
                    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", new int[]{1,2,3,4,5,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32});
                    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-threegs", new int[]{1,2,3,4,5,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32});
                     LimelightHelpers.SetFiducialIDFiltersOverride("limelight-threegf", new int[]{1,2,3,4,5,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32});
    }


    public void getWebcamFeed() { //puts the webcam connected to dev port 0 onto the dashboard
      //  CameraServer.startAutomaticCapture(0);
    }
    
    

    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
       
        
        if (!wasDisconnected_LL4 &&!inputs.isConnected_LL4) {
            Robot.reportDisconnection("Limelight 4");
            wasDisconnected_LL4 = true;
        }

        if (!wasDisconnected_LL3GS && !inputs.isConnected_LL3GS) {
            Robot.reportDisconnection("Limelight 3GS");
            wasDisconnected_LL3GS = true;
    
        }

        if (!wasDisconnected_LL3GF && !inputs.isConnected_LL3GF) {
            Robot.reportDisconnection("Limelight 3GF");
            wasDisconnected_LL3GF = true;
        }

        if (wasDisconnected_LL3GF && inputs.isConnected_LL3GF) {
            Robot.removeDisconnection("Limelight 3GF");
            wasDisconnected_LL3GF = false;
        }

        if (wasDisconnected_LL3GS && inputs.isConnected_LL3GS) {
            Robot.removeDisconnection("Limelight 3GS");
            wasDisconnected_LL3GS = false;
        }

        if (wasDisconnected_LL4 && inputs.isConnected_LL4) {
            Robot.removeDisconnection("Limelight 4");
            wasDisconnected_LL4 = false;
        }


        if (inputs.isNew_LL4 && inputs.isConnected_LL4 && inputs.tagCount_LL4 > 0 && Robot.useLimelightFour) {
            SmartDashboard.putBoolean("sensor receiving data", true);
            double std_LL4 = (inputs.avgDistance_LL4 * 0.02 ) / inputs.tagCount_LL4;
            SmartDashboard.putNumber("std_LL4", std_LL4);
            SmartDashboard.putNumber("LL4 std", std_LL4);
            SmartDashboard.putBoolean("LL4 accepted", std_LL4 < 0.1);
            SmartDashboard.putNumber("pose diff", 
            drive.getEstimatedPosition().getTranslation()
                .getDistance(inputs.MT2pose_LL4.getTranslation()));
            double[] stds_LL4 = {std_LL4, std_LL4};
            if (std_LL4 < 0.1) {
               visionMeasurements.add(new VisionMeasurement(inputs.MT2pose_LL4, inputs.rotation_LL4, inputs.time_LL4, stds_LL4, inputs.tagCount_LL4, inputs.avgDistance_LL4));
            }
        }

        // else {
        //     SmartDashboard.putBoolean("sensor receiving data", false);

        // }
        if (DriverStation.isDisabled()) {}
 
         if (inputs.isNew_LL3GS && inputs.isConnected_LL3GS && inputs.tagCount_LL3GS > 0 && !disable_other_cameras && Robot.useLimelightThreeGS) {
            double std_LL3GS = (inputs.avgDistance_LL3GS * 0.02 ) / inputs.tagCount_LL3GS;
            double[] stds_LL3GS = {std_LL3GS, std_LL3GS};
            if (std_LL3GS < 0.1) {
               visionMeasurements.add(new VisionMeasurement(inputs.MT2pose_LL3GS, inputs.rotation_LL3GS, inputs.time_LL3GS, stds_LL3GS, inputs.tagCount_LL3GS, inputs.avgDistance_LL3GS));
            }
        }

        if (inputs.isNew_LL3GF && inputs.isConnected_LL3GF && inputs.tagCount_LL3GF > 0 && !disable_other_cameras && Robot.useLimelightThreeGF) {
                double std_LL3GF = (inputs.avgDistance_LL3GF * 0.02 ) / inputs.tagCount_LL3GF;
                double[] stds_LL3GF = {std_LL3GF, std_LL3GF};
                if (std_LL3GF < 0.1) {
                visionMeasurements.add(new VisionMeasurement(inputs.MT2pose_LL3GF, inputs.rotation_LL3GF, inputs.time_LL3GF, stds_LL3GF, inputs.tagCount_LL3GF, inputs.avgDistance_LL3GF));
                }
        }

        VisionMeasurement bestmeasurement = new VisionMeasurement(new Pose2d(), 0, 0, new double[]{0,0}, 0, 0);

        if (!visionMeasurements.isEmpty()) {
            for (int i = 0; i < visionMeasurements.size(); i++) {
                if (bestmeasurement.pose().equals(new Pose2d())) {
                    bestmeasurement = visionMeasurements.get(i);
                }

                else {
                    if (bestmeasurement.std[0] > visionMeasurements.get(i).std[0]) {
                        //standard deviations are lower for this measurement, so that is the new best
                        bestmeasurement = visionMeasurements.get(i);
                    }
                }
            }
        }

        visionMeasurements.clear();

        if (!bestmeasurement.pose.equals(new Pose2d())) {
            addVisionMeasurement(bestmeasurement);
        } 
         
        }
    
        public double[] times(double multiplier, double[] list) {
            for (int i = 0; i < list.length; i++) {
                list[i] = list[i] * multiplier;
            }
            return list;
        }
    
    

        public void addVisionMeasurement(VisionMeasurement measurement) {
          //  SmartDashboard.putBoolean("ruin", ruin);

            // if (ruin) {
            // drive.addVision(new VisionMeasurement(measurement.pose().plus(new Transform2d(0.0,0.5, new Rotation2d())), measurement.rotationDegreees, measurement.timestamp, measurement.std, measurement.numTags, measurement.avgDistance));
            // }

            if (!disable_all_cameras) {
            drive.addVision(measurement);
            
            }
            


            
    }

    public void enableVision() {
        disable_all_cameras = false;

    }

    public void disableVision() {
        disable_all_cameras = true;
    }

    public void ShootingMode(boolean isShooting) {
        if (isShooting) {
            disable_other_cameras = true;
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", new int[]{21,24, 25, 26, 27, 18, 19, 20});
            }
            
            else {
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", new int[]{9, 10, 11, 2, 8, 5, 3, 4});
            }
        }

        else {
            disable_other_cameras = false;
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", new int[]{1,2,3,4,5,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32});
        }
    }
    }
    