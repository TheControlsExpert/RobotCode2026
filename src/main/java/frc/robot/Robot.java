
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Brings in the different enum states necessary for auto

import frc.robot.AutoEnums;

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
import frc.robot.Subsystems.Drive.GyroIOPigeon2;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;

//import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  //private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final RobotContainer m_robotContainer;
  public static ShootingState shootingState = ShootingState.SHOOTING;
  public static LocalizationState localizationState = LocalizationState.OPERATIONAL;

  public static AutoWinner autoWinner = AutoWinner.US;
  public static ArrayList<String> DisconnectedMotorNames = new ArrayList<String>();


  //creates the choosers that will hold possible enum states for each choice
  public static SendableChooser<AutoEnums.LoaderEnums> LoaderChooser = new SendableChooser<>();
  public static SendableChooser<AutoEnums.ClimbEnums> climbChooser = new SendableChooser<>();
  public static SendableChooser<AutoEnums.MiddleEnums> middleChooser = new SendableChooser<>();
  public static SendableChooser<AutoEnums.PositionEnums> positionChooser = new SendableChooser<>();



    public Robot() {
     m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
      Pathfinding.setPathfinder(new LocalADStar());

      //sets the states for initial autos as part of the chooser options
      LoaderChooser.setDefaultOption("Zero Loaders", AutoEnums.LoaderEnums.ZERO_LOADERS);
      LoaderChooser.addOption("One Loader", AutoEnums.LoaderEnums.ONE_LOADER);

      //sets the states for initial climb autos as part of the chooser options
      climbChooser.setDefaultOption("No Climb", AutoEnums.ClimbEnums.FALSE);
      climbChooser.addOption("Yes climb", AutoEnums.ClimbEnums.TRUE);

      //sets the state for going into the middle of the field or not
      middleChooser.setDefaultOption("No middle", AutoEnums.MiddleEnums.FALSE);
      middleChooser.addOption("Yes middle", AutoEnums.MiddleEnums.TRUE);
   

      //sets the inital field position
      positionChooser.setDefaultOption("Depot", AutoEnums.PositionEnums.DEPOT);
      positionChooser.addOption("Outpost", AutoEnums.PositionEnums.OUTPOST);

      //shows the driver all the choosers on smart dashboard
      SmartDashboard.putData("How many loaders?", LoaderChooser);
      SmartDashboard.putData("Climb or not?", climbChooser);
      //alow the driver to decide whether to go into the middle of the field or not
      SmartDashboard.putData("Middle or not?", middleChooser);
      //allows the driver to select position on the field
      SmartDashboard.putData("Iniitial Position", positionChooser);
        
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

    //reads the states the driver chose for this specific auto
    AutoEnums.LoaderEnums chosenLoader = LoaderChooser.getSelected();
    AutoEnums.ClimbEnums chosenClimb = climbChooser.getSelected();
    AutoEnums.MiddleEnums chosenMiddle = middleChooser.getSelected();
    AutoEnums.PositionEnums chosenPosition = positionChooser.getSelected();

    //passes in all the currently selected states to construct an auto program
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(chosenLoader, chosenClimb, chosenMiddle, chosenPosition);


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    ActivePeriodTracker.initialize();

    





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

  public enum LocalizationState {
    OPERATIONAL,
    DISABLED
  }

  public enum AutoWinner {
    ENEMY,
    US
  }

  

    
  
}
