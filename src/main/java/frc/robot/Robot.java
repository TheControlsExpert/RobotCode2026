
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Brings in the different enum states necessary for auto


import java.util.ArrayList;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ActivePeriodTracker.ShiftInfo;
import frc.robot.PositionEnums;
import frc.robot.Subsystems.Drive.GyroIOPigeon2;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;

//import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  public static boolean winner_selection_done = false;

  //private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final RobotContainer m_robotContainer;
  public static ShootingState shootingState = ShootingState.SHOOTING;
  public static double combinedTimeLeft = 0;
  public static boolean isActive = true;
  public static LocalizationState localizationState = LocalizationState.OPERATIONAL;

  public static AutoWinner autoWinner = AutoWinner.US;
  public static ArrayList<String> DisconnectedMotorNames = new ArrayList<String>();


  //creates the choosers that will hold possible enum states for each choice
  // public static SendableChooser<AutoEnums.LoaderEnums> LoaderChooser = new SendableChooser<>();
  // public static SendableChooser<AutoEnums.ClimbEnums> climbChooser = new SendableChooser<>();
  // public static SendableChooser<AutoEnums.MiddleEnums> middleChooser = new SendableChooser<>();
  public static SendableChooser<PositionEnums> positionChooser = new SendableChooser<>();

  //our two auto paths
  PathPlannerPath firstMiddlePath;
  Command firstMiddleAuto;

  PathPlannerPath secondMiddlePath;
  Command secondMiddleAuto;




    public Robot() {
     m_robotContainer = new RobotContainer();

     
    }

    @Override
    public void robotInit() {
      // if (isReal()) {
      // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      // Logger.addDataReceiver(new NT4Publisher());
      // }
      // else {
      // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      // }

      // Logger.start();
      //sets the states for initial autos as part of the chooser options
      // LoaderChooser.setDefaultOption("Zero Loaders", AutoEnums.LoaderEnums.ZERO_LOADERS);
      // LoaderChooser.addOption("One Loader", AutoEnums.LoaderEnums.ONE_LOADER);
      // LoaderChooser.addOption("Two Loaders", AutoEnums.LoaderEnums.TWO_LOADERS);


   
      //where da bot at?
      positionChooser.setDefaultOption("Outpost", PositionEnums.OUTPOST);
      positionChooser.addOption("Depot", PositionEnums.DEPOT);
      positionChooser.addOption("Hub", PositionEnums.HUB);

      SmartDashboard.putData("Initial Position", positionChooser);
    
             

    }
  



    @Override
    public void disabledInit() {}

    public void robotPeriodic() {

      SmartDashboard.putBoolean("is shooting", RobotContainer.isShooting);
      SmartDashboard.putBoolean("ruin", m_robotContainer.vision.ruin);
        CommandScheduler.getInstance().run();
       // SmartDashboard.putNumber("timer for shooting", m_robotContainer.timeout_shuffle.get());

        //  String fullList_disconnections = "";
        // for (String motorName : DisconnectedMotorNames) {
        //     fullList_disconnections += motorName + ", " + "\n";
        // }
        
        // SmartDashboard.putString("Disconnected Motors", fullList_disconnections);
    }

  


  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    try { //creates the autonnoumous paths

      if (positionChooser.getSelected().equals(PositionEnums.OUTPOST)) {

        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
          firstMiddlePath = PathPlannerPath.fromChoreoTrajectory("FirstBumpOutpost");
          secondMiddlePath = PathPlannerPath.fromChoreoTrajectory("SecondBumpOutpost");

        } else {
          firstMiddlePath = PathPlannerPath.fromChoreoTrajectory("FirstBumpOutpost").flipPath();
          secondMiddlePath = PathPlannerPath.fromChoreoTrajectory("SecondBumpOutpost").flipPath();
        }
      }

      else {

        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
          firstMiddlePath = PathPlannerPath.fromChoreoTrajectory("FirstBumpOutpost").mirrorPath();
          secondMiddlePath = PathPlannerPath.fromChoreoTrajectory("SecondBumpOutpost").mirrorPath();

        } else {
          firstMiddlePath = PathPlannerPath.fromChoreoTrajectory("First Bump Outpost").mirrorPath().flipPath();
          secondMiddlePath = PathPlannerPath.fromChoreoTrajectory("SecondBumpOutpost").flipPath().mirrorPath();
        }
      }

    } catch (Exception e) { 
      SmartDashboard.putBoolean("Errer", true);
    }

    
      


  
    //passes in all the currently selected states to construct an auto program
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(firstMiddlePath, secondMiddlePath);



    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    ActivePeriodTracker.initialize();

  
  }




  @Override
  public void autonomousPeriodic() {
        ShiftInfo shiftInfo = ActivePeriodTracker.getOfficialShiftInfo();
        combinedTimeLeft = shiftInfo.remainingTimeCombined(); 
        isActive = shiftInfo.active();
        SmartDashboard.putString("Current Shift", shiftInfo.currentShift().name() + "\n" + String.format("%.1f", shiftInfo.remainingTime()));
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
   
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    ActivePeriodTracker.initialize();

  }



  @Override
  public void teleopPeriodic() {
   
    ShiftInfo shiftInfo = ActivePeriodTracker.getOfficialShiftInfo();
    combinedTimeLeft = shiftInfo.remainingTimeCombined(); 
    isActive = shiftInfo.active();
    SmartDashboard.putBoolean("has chosen", winner_selection_done);
    SmartDashboard.putString("Current Shift", (shiftInfo.active() ? "ACTIVE: " : "INACTIVE:")  + "\n" + shiftInfo.currentShift().name() + "\n" + String.format("%.1f", shiftInfo.remainingTime()));
    SmartDashboard.putString("Shooting State", shootingState.toString());
    SmartDashboard.putString("Localization State", localizationState.toString());
    String autoWinnerText = "";

if (!winner_selection_done) {
  autoWinnerText = "Winner not selected yet";
} else {
  autoWinnerText = autoWinner.toString();
}


    SmartDashboard.putString("Auto Winner", autoWinnerText);
  }
  
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
  
    ActivePeriodTracker.getOfficialShiftInfo();
    CommandScheduler.getInstance().cancelAll();

    

  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static void reportDisconnection(String motorName) {
    DisconnectedMotorNames.add(motorName);
  }

  public static void removeDisconnection(String motorName) {
    DisconnectedMotorNames.remove(motorName);
  }

  

  public enum ShootingState {
    PASSING,
    SHOOTING
  }

  public enum LocalizationState {
    OPERATIONAL,
    DISABLED
  }

  public enum AutoWinner {
    ENEMY,
    US
  }

  

    
  
}
