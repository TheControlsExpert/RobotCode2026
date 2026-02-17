package frc.robot.Subsystems.Vision;

import java.lang.reflect.Field;
import java.util.ArrayList;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
   //Field2d field = new Field2d();


    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private Drive drive;

        
    double lastUsedTimestamp = -1000;
    private double minTranslation = 10000.0; 
    public record VisionMeasurement(Pose2d pose, Rotation2d rotation, double timestamp, double[] std) {}
    ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();

    public Servo servy; //makes a servo motor
    public ServoState currentServoState; // rotational state of the servo
 


      
    public VisionSubsystem(VisionIO io, Drive drive) { 
        this.io = io;
        this.drive = drive;
        //SmartDashboard.putData("field", field);
    }

    




    public enum ServoState { //creates three possible rotational states for the servo motor
        forward(LimelightConstants.climbLeftAngle),
        backward(LimelightConstants.climbRightAngle),
        sideways(LimelightConstants.normalAngle);

        public double position;

        private ServoState(double position) {
            this.position = position;
        }
    }


    public void changeServoState(ServoState goalServoState) { //changes the current servo rotational state to the inputted one from the parameter
        currentServoState = goalServoState;
    }






    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (inputs.isNew_LL4 && inputs.isConnected_LL4) {
             double std_LL4 = (inputs.avgDistance_LL4 * 0.02 ) / inputs.tagCount_LL4;
             double[] stds_LL4 = {std_LL4, std_LL4};
            if (std_LL4 < 0.1) {
               visionMeasurements.add(inputs.MT2pose_LL4, inputs);
            }
        }

        servy.setAngle(currentServoState.position);    
    }
    



    
        public double[] times(double multiplier, double[] list) {
            for (int i = 0; i < list.length; i++) {
                list[i] = list[i] * multiplier;
            }
            return list;  
        }
    
    

        public void addVisionMeasurement(Pose2d pose, double timestamp, double[] std) {
            SmartDashboard.putBoolean("adding vision", true);
            drive.addVision(pose, timestamp, std);

    }
}
    