// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants.Mod0;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;

import frc.robot.Commands.DriveCommands.DriveCommand;
import frc.robot.Commands.DriveCommands.FeedforwardCharacterization;
import frc.robot.Commands.DriveCommands.IntakeCommand;
import frc.robot.Commands.DriveCommands.StraightDriveCommand;
import frc.robot.Commands.DriveCommands.WheelRadiusCharacterization;
import frc.robot.Commands.DriveCommands.kACharacterization;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIOPigeon2;
import frc.robot.Subsystems.Drive.ModuleIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;

// import frc.robot.Subsystems.Superstructure.ElevatorIOKrakens;
// import frc.robot.Subsystems.Superstructure.Superstructure;
// import frc.robot.Subsystems.Superstructure.WristIOKrakens;
// import frc.robot.Subsystems.Superstructure.Superstructure.ManualMode;


import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;

  
 //private final PathConstraints constraints;
  private Command pathfindingCommand;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  //public static Spark leds = new Spark(0);

       
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final XboxController xbox = new XboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

 

// private boolean isFlipped =
// DriverStation.getAlliance().isPresent()
// && DriverStation.getAlliance().get() == Alliance.Red;
  
  
  
    private GyroIOPigeon2 gyro;
    Intake intake = new Intake();
    
    //    private VisionSubsystem vision;


            
          
            // Dashboard inputs
            // final LoggedDashboardChooser<Command> autoChooser;
          
            /** The container for the robot. Contains subsystems, OI devices, and commands. */
            public RobotContainer() {
             
            //  SmartDashboard.putData("Auto Chooser", autoChooser);
    
            // this.intake = new Intake();
              this.gyro = new GyroIOPigeon2();
            
                // Real robot, instantiate hardware IO implementations
                drive =
                    new Drive(
                        gyro,
                        new ModuleIOTalonFX(Mod0.constants, 0),
                        new ModuleIOTalonFX(Mod1.constants, 1),
                        new ModuleIOTalonFX(Mod2.constants, 2),
                        new ModuleIOTalonFX(Mod3.constants, 3));
        
                
        //       superstructure = new Superstructure(new WristIOKrakens(), new ElevatorIOKrakens());        
               
        
         //        vision = new VisionSubsystem(new VisionIO_Limelight(), drive);
    
        //     constraints = new PathConstraints(
       //       2.0, 4.0,
          //    Units.degreesToRadians(400), Units.degreesToRadians(720));
      //
      // Since AutoBuilder is configured, we can use it to build pathfinding commands
    
      
        // Set up SysId routines
        //autoChooser.addOption(
        //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        // autoChooser.addOption(
        //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Forward)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Reverse)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    
        // Configure the button bindings
        configureButtonBindings();

       
       // autoChooser.addOption("Left Path", Autos.getLeftAuto(drive, superstructure, vision));
       // autoChooser.addOption("Right Path ", Autos.getRightAuto(drive, superstructure, vision));
      //  autoChooser.addOption("Center Path", Autos.getCenterCommand(drive, superstructure, vision));
      //  autoChooser.addOption("Nothing", Commands.none());
       // autoChooser.addOption("Drive Forward", new StraightDriveCommand(3, drive));
      //  SmartDashboard.putData(autoChooser);

      }
    
     
    
      /**
       * Use this method to define your button->command mappings. Buttons can be created by
       * instantiating a {@link GenericHID} or one of its subclasses ({@link
       * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
       * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
       */
      private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            new DriveCommand(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                drive,
                controller));

        controller.x().whileTrue(FeedforwardCharacterization.feedforwardCommand(drive, xbox));
        
       // controller.().whileTrue(new IntakeCommand(superstructure));
       //controller.rightTrigger().whileTrue(new EjectCommand(superstructure, drive, vision));
      //  controller.rightTrigger().whileTrue(new EjectCommand(superstructure, drive, vision).andThen(new ThirdPartAutoAlign(drive, vision, superstructure, controller)));
       //  controller.leftTrigger().whileTrue(new CoralPrepCommand(superstructure, controller));
       //  controller.x().whileTrue(new AutoAlignReef(drive, vision, superstructure, controller));
       // // controller.y().onTrue(new OH_SHIT(superstructure));
      //   controller.leftBumper().whileTrue(new IntakeCommand(superstructure));
       //  controller.button(8).whileTrue(new ClimbUpCommand(climb));
       //  controller.button(7).whileTrue(new ClimbDownCommand(climb));
         //controller.button(7).onTrue(new InstantCommand(() -> {if (isInClimbMode) { isInClimbMode = false;} else { isInClimbMode = true;};}));
 
      //  new JoystickButton(LevelsController, 5).onTrue(Commands.runOnce((() -> {superstructure.hasCoral = true;})));
      //  new JoystickButton(LevelsController, 7).onTrue(new InstantCommand(() -> {drive.resettingLocalization = true;})).onFalse(new InstantCommand(() -> {drive.resettingLocalization = false;}));
      //  new JoystickButton(LevelsController, 6).onTrue(Commands.runOnce(() -> {superstructure.shouldFlip += 32;}));
        
     //    controller.a().onTrue(new ResetWristCommand(superstructure));
      //   controller.b().onTrue(new ResetElevatorCommand(superstructure));
      //   controller.y().whileTrue(new AlgaeIntakeCommand(superstructure, drive, controller));

       //  controller.povUp().onTrue(new InstantCommand(( ) -> {drive.maxSpeed += 0.5;}));
      //   controller.povDown().onTrue(new InstantCommand(( ) -> {drive.maxSpeed -= 0.5;}));



         //controller.b().whileTrue(new FirstPartAutoAlignSource(superstructure, drive).andThen(new SecondPartAutoAlignSource(drive, superstructure)).andThen(new ThirdPartAutoAlignSource(drive,  superstructure)));
         //controller.y().whileTrue(new FirstPartAutoAlignSource(superstructure, drive));

         //controller.rightBumper().whileTrue(new AutoAlignerProcessor(superstructure, controller));
        // controller.y().whileTrue(new WheelRadiusCharacterization(drive));
         
       // controller.rightBumper().whileTrue(new FirstPartAutoAlignSource(superstructure, drive).andThen(new );

        
        // controller.leftStick().     
        
       // controller.leftBumper().whileTrue(Commands.either(Commands.startEnd(() -> {superstructure.setIntakeManual(0.2);}, () -> {superstructure.setIntakeManual(0);}, superstructure.intake),
        //                                                   new IntakeCommand(superstructure), 
        //                                                  () -> (superstructure.getManualMode().equals(ManualMode.MANUAL))));
        // controller.leftTrigger().and(() -> (superstructure.hasCoral)).whileTrue(new PrepCommand(superstructure, controller));
        // controller.rightTrigger().and(() -> (superstructure.hasCoral)).whileTrue(new EjectCommand(superstructure));
        // controller.x().whileTrue(Commands.run(() -> 
      
        //   drive.runVelocity(
        //       ChassisSpeeds.fromFieldRelativeSpeeds(
        //         driver.getTargetSpeeds(drive.getEstimatedPosition(), Robot.reefMode.equals(ReefMode.CORAL) ? RobotState.getInstance().getScoringPose() : RobotState.getInstance().getAlgaePose()),
                  
        //         isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
        //               : drive.getRotation())), drive));


        //controller.rightBumper().onTrue(Commands.runOnce(() -> { boolean whichSwitch = superstructure.getManualMode().equals(ManualMode.AUTOMATIC); if (whichSwitch) {superstructure.setManualMode(ManualMode.MANUAL); superstructure.setIntakeManual(0);} else {superstructure.setManualMode(ManualMode.AUTOMATIC); superstructure.setDesiredState(SuperstructureState.HOME_UP); superstructure.setIntakeManual(0);}}, superstructure));
        //controller.y().and(() -> superstructure.getManualMode().equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setPivotManual(0.1 * 12);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setPivotManual(0);}, superstructure));
        //controller.a().and(() -> superstructure.getManualMode().equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setPivotManual(-0.1 * 12);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setPivotManual(0);}, superstructure));
        

       // controller.button(7).onTrue(Commands.runOnce(() -> {drive.resetGyro();}, drive));

        
        

        
    

                                                          

        
    
       

        
   // controller.x().whileTrue(Commands.runEnd(() -> {intake.setState(Intake_states.Bofore_First);}, () -> {intake.setState(Intake_states.Ready);}, intake));
    
    
    //controller.leftBumper().whileTrue(new AutoSourcingCommand(drive, superstructure, intake, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));
    //controller.leftTrigger().whileTrue(Commands.either(new AutoScoreAimCommand( superstructure, controller, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()), 
                                //      Commands.runEnd(() -> {superstructure.setIntakeManual(0.2);}, () -> {superstructure.setIntakeManual(0);}, superstructure), 
                                //      () -> (!superstructure.DesiredManualMode.equals(ManualMode.MANUAL));
    
    
    //controller.x().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setIntakeManual(-0.2);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setIntakeManual(0);}, superstructure));
    //controller.x().onTrue(Commands.runOnce(() -> {drive.setPose(new Pose2d());}, drive));
    //MANUAL MODES
    
    //controller.povUp().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setElevatorManual(0.35);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setElevatorManual(0);}, superstructure));
    //controller.povDown().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setElevatorManual(-0.25);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setElevatorManual(0);}, superstructure));
    //controller.povRight().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setWristManual(0.3);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setWristManual(0);}, superstructure));
   // controller.povLeft().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setWristManual(-0.3);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setWristManual(0);}, superstructure));
    
    //controller.y().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setPivotManual(0.1 * 12);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setPivotManual(0);}, superstructure));
    //controller.a().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setPivotManual(-0.1 * 12);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setPivotManual(0);}, superstructure));
        
    
    //controller.rightBumper().onTrue(Commands.runOnce(() -> { boolean whichSwitch = superstructure.getManualMode().equals(ManualMode.AUTOMATIC); if (whichSwitch) {superstructure.setDesiredManualMode(ManualMode.MANUAL);} else {superstructure.setDesiredManualMode(ManualMode.AUTOMATIC); superstructure.setDesiredState(SuperstructureState.HOME_UP)}}, superstructure));
    
    //controller.b().onTrue(Commands.runOnce(() -> {drive.resetGyro();}, drive));

   // controller.b().whileTrue(pathfindingCommand)

   //JoystickButton seventeen = new JoystickButton(LevelsController, 6);
   controller.rightBumper().onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d(drive.getEstimatedPosition().getTranslation(), DriverStation.getAlliance().get().equals(Alliance.Blue) ? Rotation2d.kZero : Rotation2d.fromDegrees(180))), drive)
                .ignoringDisable(true));

    controller.a().whileTrue(new IntakeCommand(intake))         ;   

   //seventeen.onTrue(Commands.runOnce(() -> {drive.resetGyro();}, drive).ignoringDisable(true));
      }
      
  
// controller.y().whileTrue(


//       new SelectCommand<>(
//           // Maps selector values to commands
//           Map.ofEntries(
//               Map.entry(CommandSelector.ONE, new PrintCommand("Command one was selected!")),
//               Map.entry(CommandSelector.TWO, new PrintCommand("Command two was selected!")),
//               Map.entry(CommandSelector.THREE, new PrintCommand("Command three was selected!"))),
//           this::selectPathCommand));




  


    
    
    
   
   // controller.y().onTrue(Commands.runOnce(() -> gyro.resetGyro()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *                                                                                                                                                                                                                                                                                                                                                                                                                                          
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // return new FullRun(drive, superstructure, vision);
   //return new StraightDriveCommand(1.4, drive);
   //return Commands.none();
   
   return new PathPlannerAuto("3 Piece Top");
  
   
   
   //return new FirstPartAutoAlign(drive,  superstructure, ScoringPosition.J).andThen(new SecondPartAutoAlign(drive, vision, superstructure)).andThen(new EjectCommand(superstructure, drive, vision)).andThen(new InstantCommand(() -> {drive.runVelocity(new ChassisSpeeds(-0.5, -2.5, 0));})).andThen(new WaitCommand(0.5)).andThen(new FirstPartAutoAlignSource(superstructure, drive)).andThen(new SecondPartAutoAlignSource(drive, superstructure)).andThen(new ThirdPartAutoAlignSource(drive, superstructure)).andThen(new FirstPartAutoAlign(drive, superstructure, ScoringPosition.K)).andThen(new SecondPartAutoAlign(drive, vision, superstructure)).andThen(new EjectCommand(superstructure, drive ,vision)).andThen(new InstantCommand(() -> {drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0));})).andThen(new WaitUntilCommand(() -> (superstructure.current_state.equals(SuperstructureState.HOME_UP)))).andThen(new FirstPartAutoAlignSource(superstructure, drive)).andThen(new SecondPartAutoAlignSource(drive, superstructure));


 
}









public enum ScoringPosition {
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L
}


// public enum ScoringCommand {
//   OFFSET_STRAIGHT,
//   CURVING
// }

// public static Pose2d getScoringPose_w_offset() {
//   return PositionGetter.get()


// }


// public static Pose2d getScoringPose() {

// }


// public ScoringCommand selectPathCommand() {

//   //check if we are in zone for collision

//   double[] positions = PositionGetter_offset.get(currentScoringCommand);
//   double[] xyposition_goal = new double[2];
//   double m;
//   double b;

//   if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
//     xyposition_goal[0] = positions[0];
//     xyposition_goal[1] = positions[1];

//   }

//   else {
//     xyposition_goal[0] = positions[2];
//     xyposition_goal[1] = positions[3];
//   }

//   m = (drive.getEstimatedPosition().getY() - xyposition_goal[1]) / (drive.getEstimatedPosition().getX() - xyposition_goal[0]);
//   b = xyposition_goal[1] + m * xyposition_goal[0];












// }
}