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
    DoubleArraySubscriber offsets = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[6], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber Limelight_4_stds = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleArrayTopic("stddevs").subscribe(new double[12], PubSubOption.keepDuplicates(true));
    
    DoubleArraySubscriber Limelight_3GS = NetworkTableInstance.getDefault().getTable("limelight-threegs").getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber Limelight_3GS_stds = NetworkTableInstance.getDefault().getTable("limelight-threegs").getDoubleArrayTopic("stddevs").subscribe(new double[12], PubSubOption.keepDuplicates(true));

    DoubleArraySubscriber Limelight_3GF = NetworkTableInstance.getDefault().getTable("limelight-threegf").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber Limelight_3GF_stds = NetworkTableInstance.getDefault().getTable("limelight-threegf").getDoubleArrayTopic("stddevs").subscribe(new double[12], PubSubOption.keepDuplicates(true));
    
  
    
    public VisionIOLimelight() {
     //   SmartDashboard.putData("Field", field);
    }

 

    public void updateInputs(VisionIOInputs inputs) {
        //DoubleArrayTopic Limelight_4_pose = NetworkTableInstance.getDefault().getTable("limelight-3gf").getDoubleArrayTopic("botpose_orb_wpiblue");
      
       
       // inputs.isConnected_LL4 = true;
        TimestampedDoubleArray data_LL4 = Limelight_4.getAtomic();
        TimestampedDoubleArray std_LL4 = Limelight_4_stds.getAtomic();
        TimestampedDoubleArray rotation_LL4 = ll4_rotation.getAtomic();
        SmartDashboard.putNumber("std_LL4.server", std_LL4.serverTime/1000000.0);
        SmartDashboard.putNumber("value 6 - latency", std_LL4.value[6]);

        double timestamp_LL4 = data_LL4.serverTime/1000000.0 - data_LL4.value[6]/1000.0;
      //  double[] data_general = NetworkTableInstance.getDefault().getTable("limelight-threegf").getEntry("hw").getDoubleArray(new double[5]);
        inputs.MT2pose_LL4 = new Pose2d(new Translation2d(data_LL4.value[0], data_LL4.value[1]), Rotation2d.fromDegrees(data_LL4.value[5]));
        inputs.avgArea_LL4 = data_LL4.value[10];
        inputs.avgDistance_LL4 = data_LL4.value[9];
    
        //inputs.fps_LL4 = data_general[0];
        inputs.isNew_LL4 = !inputs.MT2pose_LL4.getTranslation().equals(oldposeLL4.getTranslation()) && data_LL4.value[7] > 0;
        oldposeLL4 = inputs.MT2pose_LL4;
        inputs.time_LL4 = timestamp_LL4;
        inputs.tagCount_LL4 = data_LL4.value[7];
        inputs.rotation_LL4 = rotation_LL4.value[5];
        double[] std_LL4_doublearray = {std_LL4.value[6], std_LL4.value[7], 999999999};
        SmartDashboard.putNumber("std vision x", std_LL4_doublearray[0]);
        SmartDashboard.putNumber("std vision Y", std_LL4_doublearray[1]);

        inputs.visionSTDs_LL4 = std_LL4_doublearray;
        inputs.xOffset = offsets.getAtomic().value[2];
        inputs.yOffset = offsets.getAtomic().value[0];

        TimestampedDoubleArray LL3GS = Limelight_3GS.getAtomic();
        TimestampedDoubleArray std_LL3GS = Limelight_3GS_stds.getAtomic();
        SmartDashboard.putNumber("LL3GS.server", LL3GS.serverTime/1000000.0);
        SmartDashboard.putNumber("value 6 - latency", LL3GS.value[6]);


        //double timestamp_LL3GS = LL3GS.serverTime/1000000.0 - LL3GS.value[6]/1000.0;
      //  double[] data_general = NetworkTableInstance.getDefault().getTable("limelight-threegf").getEntry("hw").getDoubleArray(new double[5]);
    //     inputs.MT2pose_LL3GS = new Pose2d(new Translation2d(LL3GS.value[0], LL3GS.value[1]), Rotation2d.fromDegrees(LL3GS.value[5]));
    //     inputs.avgArea_LL3GS = LL3GS.value[10];
    //     inputs.avgDistance_LL3GS = LL3GS.value[9];
    //     //inputs.fps_LL4 = data_general[0];
    //     inputs.isNew_LL3GS = !inputs.MT2pose_LL3GS.getTranslation().equals(oldposeLL3GS.getTranslation());
    //     oldposeLL3GS = inputs.MT2pose_LL3GS;
    //     inputs.time_LL3GS = timestamp_LL3GS;
    //     inputs.tagCount_LL3GS = LL3GS.value[7];
    //     double[] std_LL3GS_array = {std_LL3GS.value[6], std_LL3GS.value[7], 999999999};
    //    // SmartDashboard.putNumber("std vision x", std_LL3GS[0]);
    //    // SmartDashboard.putNumber("std vision Y", std_LL3GS[1]);

    //     inputs.visionSTDs_LL3GS = std_LL3GS_array;















        
    //     TimestampedDoubleArray LL3GF = Limelight_3GF.getAtomic();
    //     TimestampedDoubleArray std_LL3GF = Limelight_3GF_stds.getAtomic();
    //     SmartDashboard.putNumber("LL3GF.server", LL3GF.serverTime/1000000.0);
    //     SmartDashboard.putNumber("value 6 - latency", LL3GF.value[6]);


    //     double timestamp_LL3GF = LL3GF.serverTime/1000000.0 - LL3GF.value[6]/1000.0;
    //   //  double[] data_general = NetworkTableInstance.getDefault().getTable("limelight-threegf").getEntry("hw").getDoubleArray(new double[5]);
    //     inputs.MT2pose_LL3GF = new Pose2d(new Translation2d(LL3GF.value[0], LL3GF.value[1]), Rotation2d.fromDegrees(LL3GF.value[5]));
    //     inputs.avgArea_LL3GF = LL3GF.value[10];
    //     inputs.avgDistance_LL3GF = LL3GF.value[9];
    //     //inputs.fps_LL4 = data_general[0];
    //     inputs.isNew_LL3GF = !inputs.MT2pose_LL3GF.getTranslation().equals(oldposeLL3GF.getTranslation());
    //     oldposeLL3GF = inputs.MT2pose_LL3GF;
    //     inputs.time_LL3GF = timestamp_LL3GF;
    //     inputs.tagCount_LL3GF = LL3GF.value[7];
    //     double[] std_LL3GF_array = {std_LL3GF.value[6], std_LL3GF.value[7], 999999999};
    //     //SmartDashboard.putNumber("std vision x", std_LL3GF[0]);
    //     //SmartDashboard.putNumber("std vision Y", std_LL3GF[1]);

    //     inputs.visionSTDs_LL3GF = std_LL3GF_array;





    }


    
}