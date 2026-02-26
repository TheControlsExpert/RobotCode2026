
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Brings in the different enum states necessary for auto
import frc.robot.Enums.AutoEnums;
import frc.robot.Enums.ClimbEnums;
import frc.robot.Enums.PositionEnums;
import frc.robot.Enums.MiddleEnums;

import java.util.ArrayList;
import org.littletonrobotics.junction.LoggedRobot;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.GyroIOPigeon2;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;

//import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  //private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final RobotContainer m_robotContainer;
  public static ShootingState shootingState = ShootingState.SHOOTING;
  public static AutoWinner autoWinner = AutoWinner.US;
  public static ArrayList<String> DisconnectedMotorNames = new ArrayList<String>();


  //creates the choosers for possible auto enum states and inital position
  public static SendableChooser<AutoEnums> autoChooser = new SendableChooser<>();
  public static SendableChooser<ClimbEnums> climbChooser = new SendableChooser<>();
  public static SendableChooser<PositionEnums> positionChooser = new SendableChooser<>();
  public static SendableChooser<MiddleEnums> middleChooser = new SendableChooser<>();



    public Robot() {
     m_robotContainer = new RobotContainer();
    }


    @Override
    public void robotInit() {
      Pathfinding.setPathfinder(new LocalADStar());

      //sets the states for initial autos as part of the chooser options
      autoChooser.setDefaultOption("Zero Loaders", AutoEnums.ZERO_LOADERS);
      autoChooser.addOption("One Loader", AutoEnums.ONE_LOADER);
      autoChooser.addOption("Two loaders", AutoEnums.TWO_LOADERS);

      //sets the states for initial climb autos as part of the chooser options
      climbChooser.setDefaultOption("No Climb", ClimbEnums.NO_CLIMB);
      climbChooser.addOption("Yes climb", ClimbEnums.YES_CLIMB);

      //sets the state for going into the middle of the field or not
      middleChooser.setDefaultOption("No middle", MiddleEnums.NO_MIDDLE);
      middleChooser.addOption("Yes middle", MiddleEnums.YES_MIDDLE);
   

      //sets the inital field position
      positionChooser.setDefaultOption("Hub", PositionEnums.HUB);
      positionChooser.addOption("Depot", PositionEnums.DEPOT); //hub position not needed for auto logic
      positionChooser.addOption("Outpost", PositionEnums.OUTPOST);
        
    }
  



    @Override
    public void disabledInit() {}

    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

         String fullList_disconnections = "";
    for (String motorName : DisconnectedMotorNames) {
        fullList_disconnections += motorName + ", " + "\n";
    }
    SmartDashboard.putString("Disconnected Motors", fullList_disconnections);
    }

  


  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //passes in all the currently selected states for auto, represented by different enums and selected by the drive team
  m_autonomousCommand = m_robotContainer.getAutonomousCommand(positionChooser.getSelected(), autoChooser.getSelected(), climbChooser.getSelected(), middleChooser.getSelected());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    ActivePeriodTracker.initialize();

    //shows the user the possible climb and auto states on smart dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Climb chooser", climbChooser);
    //alow the driver to decide whether to go into the middle of the field or not
    SmartDashboard.putData("MiddleChooser", middleChooser);
    //allows the driver to select color
    SmartDashboard.putData("Field Position", positionChooser);





  }

  @Override
  public void autonomousPeriodic() {
        ShiftInfo shiftInfo = ActivePeriodTracker.getOfficialShiftInfo();
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
      SmartDashboard.putString("Current Shift", (shiftInfo.active() ? "ACTIVE: " : "INACTIVE:")  + "\n" + shiftInfo.currentShift().name() + "\n" + String.format("%.1f", shiftInfo.remainingTime()));
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

  public enum AutoWinner {
    ENEMY,
    US
  }

  

    
  
}
