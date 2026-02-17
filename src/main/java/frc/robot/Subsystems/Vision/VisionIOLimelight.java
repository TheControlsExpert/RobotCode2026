package frc.robot.Subsystems.Vision;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionIOLimelight implements VisionIO {

    Pose2d oldposeLL4 = new Pose2d();
    Pose2d oldposeLL3GS = new Pose2d();
    Pose2d oldposeLL3GF = new Pose2d();

    //private Field2d field = new Field2d();


    DoubleArraySubscriber Limelight_4 = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber ll4_rotation = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    
    DoubleArraySubscriber Limelight_3GS = NetworkTableInstance.getDefault().getTable("limelight-threegs").getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber ll3gs_rotation = NetworkTableInstance.getDefault().getTable("limelight-threegs").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));

    
  
    
    public VisionIOLimelight() {
     //   SmartDashboard.putData("Field", field);
    }

 

    public void updateInputs(VisionIOInputs inputs) {
   
        TimestampedDoubleArray data_LL4 = Limelight_4.getAtomic();
        TimestampedDoubleArray rotation_LL4 = ll4_rotation.getAtomic();
    
        double timestamp_LL4 = data_LL4.serverTime/1000000.0 - data_LL4.value[6]/1000.0;
        inputs.MT2pose_LL4 = new Pose2d(new Translation2d(data_LL4.value[0], data_LL4.value[1]), Rotation2d.fromDegrees(data_LL4.value[5]));
        inputs.avgDistance_LL4 = data_LL4.value[9];
    
        inputs.isNew_LL4 = !inputs.MT2pose_LL4.getTranslation().equals(oldposeLL4.getTranslation()) && data_LL4.value[7] > 0;
        oldposeLL4 = inputs.MT2pose_LL4;
        inputs.time_LL4 = timestamp_LL4;
        inputs.tagCount_LL4 = data_LL4.value[7];
        inputs.rotation_LL4 = rotation_LL4.value[5];

        
        TimestampedDoubleArray data_LL3GS = Limelight_3GS.getAtomic();
        TimestampedDoubleArray rotation_LL3GS = ll3gs_rotation.getAtomic();
    
        double timestamp_LL3GS = data_LL3GS.serverTime/1000000.0 - data_LL3GS.value[6]/1000.0;
        inputs.MT2pose_LL3GS = new Pose2d(new Translation2d(data_LL3GS.value[0], data_LL3GS.value[1]), Rotation2d.fromDegrees(data_LL3GS.value[5]));
        inputs.avgDistance_LL3GS = data_LL3GS.value[9];
    
        inputs.isNew_LL3GS = !inputs.MT2pose_LL3GS.getTranslation().equals(oldposeLL3GS.getTranslation()) && data_LL3GS.value[7] > 0;
        oldposeLL3GS = inputs.MT2pose_LL3GS;
        inputs.time_LL3GS = timestamp_LL3GS;
        inputs.tagCount_LL3GS = data_LL3GS.value[7];
        inputs.rotation_LL3GS = rotation_LL3GS.value[5];
       
    }   
}