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

import frc.robot.AutoEnums;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.SwerveConstants.Mod0;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;
import frc.robot.Robot.LocalizationState;
import frc.robot.Robot.ShootingState;
import frc.robot.Commands.ClimbCommands.ClimbDown;
import frc.robot.Commands.ClimbCommands.ClimbUp;
import frc.robot.Commands.DriveCommands.DriveCommand;
import frc.robot.Commands.DriveCommands.FeedforwardCharacterization;
import frc.robot.Commands.DriveCommands.StraightDriveCommand;
import frc.robot.Commands.DriveCommands.WheelRadiusCharacterization;
import frc.robot.Commands.DriveCommands.kACharacterization;
import frc.robot.Commands.DriveCommands.AligningCommands.AutoAlign;
import frc.robot.Commands.DriveCommands.AligningCommands.AutoBumping;
import frc.robot.Commands.DriveCommands.AligningCommands.AutomaticClimbing;
import frc.robot.Commands.DriveCommands.AligningCommands.AutomaticTrenching;
import frc.robot.Commands.IntakeCommands.IntakeCommand;
import frc.robot.Commands.IntakeCommands.Jam;
import frc.robot.Commands.IntakeCommands.ShuffleCommand;
import frc.robot.Commands.ShootingCommands.ResetHood;
import frc.robot.Commands.ShootingCommands.Revv;
import frc.robot.Commands.ShootingCommands.RevvAuto;
import frc.robot.Commands.ShootingCommands.Shooting;
import frc.robot.Commands.ShootingCommands.ShootingAuto;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbIO;
import frc.robot.Subsystems.Drive.Drive;

import frc.robot.Subsystems.Drive.GyroIOPigeon2;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Drive.ModuleIOTalonFX;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIO;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Vision.VisionIOLimelight;
import frc.robot.Subsystems.Vision.VisionSubsystem;

// import frc.robot.Subsystems.Superstructure.ElevatorIOKrakens;
// import frc.robot.Subsystems.Superstructure.Superstructure;
// import frc.robot.Subsystems.Superstructure.WristIOKrakens;
// import frc.robot.Subsystems.Superstructure.Superstructure.ManualMode;


import java.util.HashMap;
import java.util.Map;
import java.util.Set;

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
 // public final DriveSim drivesim;

  
 //private final PathConstraints constraints;
  private Command pathfindingCommand;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  //public static Spark leds = new Spark(0);

       
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final XboxController co4Controller = new XboxController(0);
  //private final XboxController xbox = new XboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  AutomaticTrenching autoTrenching;
  AutomaticClimbing autoClimbing;

 

// private boolean isFlipped =
// DriverStation.getAlliance().isPresent()
// && DriverStation.getAlliance().get() == Alliance.Red;
  
  
  
    private GyroIOPigeon2 gyro;
   
        private VisionSubsystem vision;

       IntakeSubsystem intake = new IntakeSubsystem(new IntakeIO());
        Shooter shooter = new Shooter(new ShooterIO());
        Indexer indexer = new Indexer(new IndexerIO());
      //  Climb climb = new Climb(new ClimbIO());



 

            
          
            // Dashboard inputs
            // final LoggedDashboardChooser<Command> autoChooser;
          
            /** The container for the robot. Contains subsystems, OI devices, and commands. */
            public RobotContainer() {
             
            //  SmartDashboard.putData("Auto Chooser", autoChooser);
    
            // this.intake = new Intake();
            
              this.gyro = new GyroIOPigeon2();

   
            
                // Real robot, instantiate hardware IO implementations

                // drive =
                //     new Drive(
                //         gyro,
                //         new ModuleIOTalonFX(Mod0.constants, 0),
                //         new ModuleIOTalonFX(Mod1.constants, 1),
                //         new ModuleIOTalonFX(Mod2.constants, 2),
                //         new ModuleIOTalonFX(Mod3.constants, 3));
                if (Robot.isReal()) {
                           drive =
                    new Drive(
                        gyro,
                        new ModuleIOTalonFX(Mod0.constants, 0),
                        new ModuleIOTalonFX(Mod1.constants, 1),
                        new ModuleIOTalonFX(Mod2.constants, 2),
                        new ModuleIOTalonFX(Mod3.constants, 3));
                }
                
                else {
                    drive =
                    new Drive(
                        gyro,
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                }

                 // drivesim = new DriveSim(new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
             //   vision = new VisionSubsystem(new VisionIOLimelight(), drive);
              //  autoTrenching = new AutomaticTrenching(drive, drive.constraints_auto, () -> -controller.getLeftY(), () -> -controller.getLeftX(), 0.08, controller);     
             //   autoClimbing = new AutomaticClimbing(drive, new AutoAlign(2.5, drive.rotationkP, 0.01, 1), vision, climb);
      
                
        //       superstructure = new Superstructure(new WristIOKrakens(), new ElevatorIOKrakens());        
               
                

                vision = new VisionSubsystem(new VisionIOLimelight(), drive);
    
        //     constraints = new PathConstraints(
       //       2.0, 4.0,
          //    Units.degreesToRadians(400), Units.degreesToRadians(720));
      //
      // Since AutoBuilder is configured, we can use it to build pathfinding commands
    //controller.x().whileTrue(FeedforwardCharacterization.feedforwardCommand(drive, co4Controller));
    controller.x().whileTrue(kACharacterization.feedforwardCommand(drive, co4Controller));
      
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


         controller.leftBumper().whileTrue(new InstantCommand(() -> {intake.is_busy = true; }).andThen(new IntakeCommand(intake))).onFalse(new InstantCommand(() -> {intake.is_busy = false;}));
         controller.rightBumper().onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d(drive.getEstimatedPosition().getTranslation(), DriverStation.getAlliance().get().equals(Alliance.Blue) ? Rotation2d.kZero : Rotation2d.fromDegrees(180))), drive)
                 .ignoringDisable(true));


       // Timer timeout_shuffle = new Timer();
       // Trigger timeoutshuffle_trigger = new Trigger(() -> (shooter.isShooting)).onTrue(new InstantCommand(() -> {timeout_shuffle.restart();}));
      //  Trigger shuffle_trigger = new Trigger(() -> (shooter.isShooting  && !intake.is_busy)).onTrue(new WaitUntilCommand(() -> (intake.isReadyToClose() || timeout_shuffle.hasElapsed(2.0))).andThen(new InstantCommand(() -> {intake.Shuffle();}, intake)));
         // .andThen(new WaitUntilCommand(() -> {return !intake.is_busy
        
       // && (intake.isReadyToClose() || timeout_shuffle.hasElapsed(2.0)))).onTrue(new InstantCommand(() -> {intake.Shuffle();}));
         // .andThen(new WaitUntilCommand(() -> {return !intake.is_busy && (intake.isReadyToClose() || timeout_shuffle.hasElapsed(2.0));}))
         // .andThen(new InstantCommand(() -> {intake.Shuffle(); shooter.needsShuffling = false;}, intake)));
        
      
          
        
      
         // controller.leftTrigger().whileTrue(new Revv(shooter, drive, controller));
        controller.rightTrigger().
        onTrue(new InstantCommand(() -> {shooter.isShooting = true;}))
        .whileTrue(new Shooting(shooter, drive, indexer, intake, controller, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX(), drive.rotationkP, vision))
        .onFalse(new InstantCommand(() -> {intake.Extend(); shooter.isShooting = false;}, intake));

        controller.leftTrigger().whileTrue(new Revv(shooter, drive, controller));
        //                                                              new WaitUntilCommand(() -> {return intake.isReadyToClose() && !intake.is_busy;}).andThen(new ShuffleCommand(intake).getShuffleCommand())))
        //                          .onFalse(new Jam(indexer, shooter, 1.0));                                 
        

        //automatic climbing and climb up in teleop, with potential for overridng
        // controller.x().whileTrue(Commands.defer(() -> autoClimbing.getClimbingCommand(true).until( //stops the command when:
        //   () -> autoClimbing.isOverridePossible() && //overriding is possible
        //   (Math.abs(controller.getLeftY()) > 0.1 || Math.abs(controller.getLeftX()) > 0.1)), //driver moves the controller enough
        //   Set.of(climb, drive))); 



        //shouldn't use any localization, so only encoder values
        // controller.button(7).onTrue(Commands.defer(() -> {
        //   if (!climb.isClimbGoalUp()) {
        //     return new ClimbUp(climb);
        //   } else {
        //     return new ClimbDown(climb, drive);
        //   }
        // }, Set.of( climb)
        // ));

     //   controller.a().whileTrue(new AutoBumping(drive, intake, () -> -controller.getLeftY(), () -> -controller.getLeftX(), 0.08, controller));
         
       
      //  controller.x().whileTrue(autoTrenching.andThen(
        
      // Commands.defer(() -> autoTrenching.getPathingCommand().until(
        
      //  () -> (autoTrenching.passedTrench() && 
      //  (Math.abs(controller.getLeftY()) > 0.1 || Math.abs(controller.getLeftX()) > 0.1 || Math.abs(controller.getRightX()) > 0.1))), Set.of(drive))));

      //COPILOT

      //intake overrides/fixes
       controller2.leftTrigger().or(controller.x()).whileTrue(new StartEndCommand(() -> {intake.Shuffle(); intake.is_busy = true;}, () -> {intake.Extend(); intake.is_busy = false;}, intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

      // controller2.rightTrigger().whileTrue(new Jam(indexer, shooter));
         // controller.rightTrigger().whileTrue(
      // Commands.defer(() -> { 
      //   if (intake.isShuffling) {
      // return Commands.none();}

      //   else {
      //     return new InstantCommand(() -> {
      //       intake.isShuffling = true;
      //       intake.setIntakeDutyCycle(0.3);}, intake)
      //  .andThen((new InstantCommand(() -> {intake.Retract();}, intake)
      //          .andThen(new WaitCommand(0.7))
      //          .andThen(new InstantCommand(() -> {intake.Extend();}, intake))
      //          .andThen(new WaitCommand(0.5))).repeatedly()).
               
      //   handleInterrupt(() -> {intake.setIntakeDutyCycle(0);
      //                          intake.Extend();
      //                          intake.isShuffling = false;
      //                           });
      //   }
      
      // }, Set.of(intake)));


      

       //state changes
      //  controller2.leftBumper().onTrue(new InstantCommand(() -> {
      //   if (Robot.shootingState.equals(ShootingState.PASSING)) {
      //     Robot.shootingState = ShootingState.SHOOTING;
      //   }
      //   else {
      //     Robot.shootingState = ShootingState.PASSING;
      //   }
      //  }));

      //  controller2.rightBumper().onTrue(new InstantCommand(() -> {
      //   if (Robot.localizationState.equals(LocalizationState.OPERATIONAL)) {
      //     Robot.localizationState = LocalizationState.DISABLED;
      //   }
      //   else {
      //     Robot.localizationState = LocalizationState.OPERATIONAL;
      //   }
      //  }));

      //  //decide auto winner
      //  controller2.y().onTrue(new InstantCommand(() -> {Robot.autoWinner = Robot.AutoWinner.US; Robot.winner_selection_done = true;}));
      //  controller2.a().onTrue(new InstantCommand(() -> {Robot.autoWinner = Robot.AutoWinner.ENEMY; Robot.winner_selection_done = true;}));

       //resets of encoders
      // controller2.b().onTrue(new InstantCommand(() -> {intake.resetPivotPosition();}));
      // controller2.x().onTrue(new ResetHood(shooter));

      //  Timer when_to_signal_disconnectedFMS = new Timer();
      //  RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {when_to_signal_disconnectedFMS.restart();}));
      //  RobotModeTriggers.teleop().and(() -> {return when_to_signal_disconnectedFMS.hasElapsed(2);}).and(() -> (!Robot.winner_selection_done)).whileTrue(Commands.startEnd(
      //           () -> {
      //             SmartDashboard.putBoolean("controller was told to rumble", true);
      //             controller2.setRumble(RumbleType.kBothRumble, 1);
      //           },
      //           () -> {
      //             SmartDashboard.putBoolean("controller was told to rumble", false);
      //             controller2.setRumble(RumbleType.kBothRumble, 0);
      //           }));
        
      }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *                                                                                                                                                                                                                                                                                                                                                                                                                                          
   * @return the command to run in autonomous
   */






 
  public Command getAutonomousCommand(AutoEnums.LoaderEnums chosenLoader, AutoEnums.ClimbEnums chosenClimb, AutoEnums.MiddleEnums chosenMiddle, AutoEnums.PositionEnums chosenPosition) {
     
    // //initializes all paths and auto commands
    // //everything here should be put in the if statemnet
    // PathPlannerPath goMiddleAutoPath;
    // Command goMiddleAuto;
    // PathPlannerPath leaveMiddleAutoPath;
    // Command leaveMiddleAuto;
    // PathPlannerPath goLoaderAutoPath;
    // Command goLoaderAuto;
    // PathPlannerPath shootFromLoaderAutoPath;
    // Command shootFromLoaderAuto;
    // PathPlannerPath returnFromLoaderAutoPath;
    // Command returnFromLoaderAuto;

     

      


      
    //   if (chosenPosition.equals(AutoEnums.PositionEnums.OUTPOST) && AutoEnums.LoaderEnums.TWO_LOADERS != chosenLoader) {
    //     double trenchTimeout = 2.3;

    //     //create all paths for outpost starting location
    //     try {

    //       if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
    //         goMiddleAutoPath = PathPlannerPath.fromChoreoTrajectory("FirstBumpOutpost").flipPath();
    //         leaveMiddleAutoPath = PathPlannerPath.fromChoreoTrajectory("SecondBumpOutpost").flipPath();
    //       //  goLoaderAutoPath = PathPlannerPath.fromPathFile("Collect Outpost").flipPath();
    //        // shootFromLoaderAutoPath = PathPlannerPath.fromPathFile("Outpost to Climb").flipPath();
    //       }
    //       else {
    //         goMiddleAutoPath = PathPlannerPath.fromChoreoTrajectory("FirstBumpOutpost");
    //         leaveMiddleAutoPath = PathPlannerPath.fromChoreoTrajectory("SecondBumpOutpost");
    //       //  goLoaderAutoPath = PathPlannerPath.fromPathFile("Collect Outpost");
    //         //shootFromLoaderAutoPath = PathPlannerPath.fromPathFile("Outpost to Climb");
    //       } 
    //     }

    //    catch (Exception e) {
    //     return Commands.none();
    //    }

    //     //creates the autos based on outpost paths
    //     goMiddleAuto = AutoBuilder.followPath(goMiddleAutoPath);
    //     leaveMiddleAuto = AutoBuilder.followPath(leaveMiddleAutoPath);  
    //    // goLoaderAuto = AutoBuilder.followPath(goLoaderAutoPath); 
    //     //shootFromLoaderAuto = AutoBuilder.followPath(shootFromLoaderAutoPath); 





    //    //logic
    //     if (chosenLoader.equals(AutoEnums.LoaderEnums.ZERO_LOADERS)) {
    //       if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) {
            
    //         PathPlannerPath basicPath;
    //           Command basicAuto;
    //           try {
    //             if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
    //               basicPath = PathPlannerPath.fromPathFile("Basic Starting Path");
    //             } else {
    //               basicPath = PathPlannerPath.fromPathFile("Basic Starting Path").flipPath();
    //             }
    //           } catch (Exception e) {return Commands.none();}
    //           basicAuto = AutoBuilder.followPath(basicPath);

    //         if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) { //just shoot the pre-loaded eight balls

    //           return new InstantCommand(() -> {drive.resetPosition(basicPath.getStartingHolonomicPose().get());}).
    //           andThen(new ParallelRaceGroup(basicAuto, new RevvAuto(shooter, drive, 0))).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, basicPath.getPathPoses().get(basicPath.getPathPoses().size() - 1).getTranslation(), vision));              
    //         }


    //         else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) { //shoot the preloaded balls and then climb
              
    //           return new InstantCommand(() -> {drive.resetPosition(basicPath.getStartingHolonomicPose().get());}).
    //           andThen(new ParallelRaceGroup(basicAuto, new RevvAuto(shooter, drive, 0))).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, basicPath.getPathPoses().get(basicPath.getPathPoses().size() - 1).getTranslation(), vision)).   
    //           andThen(Commands.defer(() -> autoClimbing.getClimbingCommand(true), Set.of(drive)));
    //         }
    //       }


    //       else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
    //         if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) { //go to middle, come back and shoot
              
    //           return new InstantCommand(() -> {drive.resetPosition(goMiddleAutoPath.getStartingHolonomicPose().get());}).
    //           andThen(new ParallelRaceGroup(new WaitCommand(2.7).andThen(new RevvAuto(shooter, drive, 0)), new IntakeCommand(intake), goMiddleAuto)).
    //           //andThen(new ParallelRaceGroup(new RevvAuto(shooter, drive, 0), leaveMiddleAuto)).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 5, goMiddleAutoPath.getPathPoses().get(goMiddleAutoPath.getPathPoses().size() - 1).getTranslation(), vision)). 
    //           andThen(new ParallelRaceGroup(new WaitCommand(4.5).andThen(new RevvAuto(shooter, drive, 0)), new IntakeCommand(intake), leaveMiddleAuto)).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 4, leaveMiddleAutoPath.getPathPoses().get(leaveMiddleAutoPath.getPathPoses().size() - 1).getTranslation(), vision));



    //       }

    //         else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) { //go to middle, come back and shoot, then climb
              
    //           return new InstantCommand(() -> {drive.resetPosition(goMiddleAutoPath.getStartingHolonomicPose().get());}).
    //           andThen(new ParallelRaceGroup(new IntakeCommand(intake), goMiddleAuto)).
    //           andThen(new ParallelRaceGroup(new RevvAuto(shooter, drive, trenchTimeout), leaveMiddleAuto)).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, leaveMiddleAutoPath.getPathPoses().get(leaveMiddleAutoPath.getPathPoses().size() - 1).getTranslation(), vision)).              
    //           andThen(Commands.defer(() -> autoClimbing.getClimbingCommand(true), Set.of(drive)));
    //         } 
    //       }
    //     }


    //     else if (chosenLoader.equals(AutoEnums.LoaderEnums.ONE_LOADER)) {
    //       if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) { //this assumes that we are starting from the hub position


    //         //new paths and autos have to be created to go from the hub to the outpost
    //         PathPlannerPath HubToOutpostPath;
    //         Command HubToOutpostAuto;
    //         PathPlannerPath HubLeaveOutpostPath;
    //         Command HubLeaveOutpostAuto;

    //         try {
    //           if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
    //             HubToOutpostPath = PathPlannerPath.fromPathFile("Hub to (Collect Outpost)");
    //             HubLeaveOutpostPath = PathPlannerPath.fromPathFile("Hub to (Return Outpost)");
    //           } else {
    //             HubToOutpostPath = PathPlannerPath.fromPathFile("Hub to (Collect Outpost)").flipPath();
    //             HubLeaveOutpostPath = PathPlannerPath.fromPathFile("Hub to (Return Outpost)").flipPath();
    //           }
    //         } catch (Exception e) {
    //           return Commands.none();
    //         }

    //         HubToOutpostAuto = AutoBuilder.followPath(HubToOutpostPath);
    //         HubLeaveOutpostAuto = AutoBuilder.followPath(HubLeaveOutpostPath);
            



    //         if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) { //go to the outpost and shoot
              
    //           return new InstantCommand(() -> {drive.resetPosition(HubToOutpostPath.getStartingHolonomicPose().get());}).
    //           andThen(HubToOutpostAuto).andThen(new ParallelRaceGroup(new WaitCommand(2), new RevvAuto(shooter, drive, 0))). //goes to loader, waits and begins revving the shooter
    //           andThen(new ParallelRaceGroup(HubLeaveOutpostAuto, new RevvAuto(shooter, drive, 0))).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, HubLeaveOutpostPath.getPathPoses().get(HubLeaveOutpostPath.getPathPoses().size() - 1).getTranslation(), vision));
    //         }


    //         else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) { //go to the outpost, shoot, then climb

    //           return new InstantCommand(() -> {drive.resetPosition(HubToOutpostPath.getStartingHolonomicPose().get());}).
    //           andThen(HubToOutpostAuto).andThen(new ParallelRaceGroup(new WaitCommand(2), new RevvAuto(shooter, drive, 0))). //goes to loader, waits and begins revving the shooter
    //           andThen(new ParallelRaceGroup(HubLeaveOutpostAuto, new RevvAuto(shooter, drive, 0))).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, HubLeaveOutpostPath.getPathPoses().get(HubLeaveOutpostPath.getPathPoses().size() - 1).getTranslation(), vision)).
    //           andThen(Commands.defer(() -> autoClimbing.getClimbingCommand(true), Set.of(drive)));
    //         }
                   
    //       }


    //       else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
    //         if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) { //go to middle, come back, shoot, go to outpost, shoot
              
    //           return new InstantCommand(() -> {drive.resetPosition(goMiddleAutoPath.getStartingHolonomicPose().get());}).
    //           andThen(new ParallelRaceGroup(new IntakeCommand(intake), goMiddleAuto)).
    //           andThen(new ParallelRaceGroup(new RevvAuto(shooter, drive, trenchTimeout), leaveMiddleAuto));
    //          // andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, goLoaderAutoPath.getStartingHolonomicPose().get().getTranslation(), vision)).
    //          // andThen(goLoaderAuto).andThen(new ParallelRaceGroup(new WaitCommand(2), new RevvAuto(shooter, drive, 0))).
    //          // andThen(new ParallelRaceGroup(shootFromLoaderAuto, new RevvAuto(shooter, drive, 0))).
    //          // andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, shootFromLoaderAutoPath.getPathPoses().get(shootFromLoaderAutoPath.getPathPoses().size() - 1).getTranslation(), vision));
    //         }


    //         else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) { //go to middle, come back while shooting, go to outpost, shoot, climb
              
    //           return new InstantCommand(() -> {drive.resetPosition(goMiddleAutoPath.getStartingHolonomicPose().get());}).
    //           andThen(new ParallelRaceGroup(new IntakeCommand(intake), goMiddleAuto)).
    //           andThen(new ParallelRaceGroup(new RevvAuto(shooter, drive, trenchTimeout), leaveMiddleAuto)).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, leaveMiddleAutoPath.getPathPoses().get(leaveMiddleAutoPath.getPathPoses().size() - 1).getTranslation(), vision));
    //           // andThen(new ParallelCommandGroup(goLoaderAuto, new Jam(indexer, shooter, 0.5))).
    //           // andThen(new ParallelRaceGroup(new WaitCommand(2), new RevvAuto(shooter, drive, 0))).
    //           // andThen(new ParallelRaceGroup(shootFromLoaderAuto, new RevvAuto(shooter, drive, 0))).
    //           // andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, shootFromLoaderAutoPath.getPathPoses().get(shootFromLoaderAutoPath.getPathPoses().size() - 1).getTranslation(), vision)).
    //           // andThen(new ParallelCommandGroup(Commands.defer(() -> autoClimbing.getClimbingCommand(false), Set.of(drive)), new Jam(indexer, shooter, 0.5)));
    //         }      
    //       }
    //     }    



    //     else if (chosenLoader.equals(AutoEnums.LoaderEnums.TWO_LOADERS)) { //go to outpost, shoot, go to depot, shoot
      
    //     }
    //   }







    //   else if (chosenPosition.equals(AutoEnums.PositionEnums.DEPOT) && AutoEnums.LoaderEnums.TWO_LOADERS != chosenLoader) {
    //       double trenchTimeout = 2.3;


    //     //create all paths for outpost starting location
    //     try {

    //       if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
    //         goMiddleAutoPath = PathPlannerPath.fromPathFile("Human Player Center Approach").flipPath().mirrorPath();
    //         leaveMiddleAutoPath = PathPlannerPath.fromPathFile("Depot Wayback").flipPath();
    //         goLoaderAutoPath = PathPlannerPath.fromPathFile("Collect Depot").flipPath();
    //         returnFromLoaderAutoPath = PathPlannerPath.fromPathFile("Return Depot").flipPath();
    //       }
    //       else {
    //         goMiddleAutoPath = PathPlannerPath.fromPathFile("Human Player Center Approach").mirrorPath();
    //         leaveMiddleAutoPath = PathPlannerPath.fromPathFile("Depot Wayback");
    //         goLoaderAutoPath = PathPlannerPath.fromPathFile("Collect Depot");
    //         returnFromLoaderAutoPath = PathPlannerPath.fromPathFile("Return Depot");
    //       }
    //       } 
        

    //    catch (Exception e) {
    //     return Commands.none();
    //    }

    //     //creates the autos based on outpost paths
    //     goMiddleAuto = AutoBuilder.followPath(goMiddleAutoPath);
    //     leaveMiddleAuto = AutoBuilder.followPath(leaveMiddleAutoPath);  
    //     goLoaderAuto = AutoBuilder.followPath(goLoaderAutoPath); 
    //     returnFromLoaderAuto = AutoBuilder.followPath(returnFromLoaderAutoPath);





    //     if (chosenLoader.equals(AutoEnums.LoaderEnums.ZERO_LOADERS)) {
    //       if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) {
            
    //         PathPlannerPath basicPath;
    //           Command basicAuto;
    //           try {
    //             if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
    //               basicPath = PathPlannerPath.fromPathFile("Basic Starting Path");
    //             } else {
    //               basicPath = PathPlannerPath.fromPathFile("Basic Starting Path").flipPath();
    //             }
    //           } catch (Exception e) {return Commands.none();}
    //           basicAuto = AutoBuilder.followPath(basicPath);

    //         if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) { //just shoot the pre-loaded eight balls

    //           return new InstantCommand(() -> {drive.resetPosition(basicPath.getStartingHolonomicPose().get());}).
    //           andThen(new ParallelRaceGroup(basicAuto, new RevvAuto(shooter, drive, 0))).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, basicPath.getPathPoses().get(basicPath.getPathPoses().size() - 1).getTranslation(), vision));              
    //         }


    //         else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) { //shoot the preloaded balls and then climb
              
    //           return new InstantCommand(() -> {drive.resetPosition(basicPath.getStartingHolonomicPose().get());}).
    //           andThen(new ParallelRaceGroup(basicAuto, new RevvAuto(shooter, drive, 0))).
    //           andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, basicPath.getPathPoses().get(basicPath.getPathPoses().size() - 1).getTranslation(), vision)).   
    //           andThen(Commands.defer(() -> autoClimbing.getClimbingCommand(true), Set.of(drive)));
    //         }
    //       }


    //           else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
    //             if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) { //go to middle, come back and shoot
                  
    //               return new InstantCommand(() -> {drive.resetPosition(goMiddleAutoPath.getStartingHolonomicPose().get());}).
    //               andThen(new ParallelRaceGroup(new IntakeCommand(intake), goMiddleAuto)).
    //               andThen(new ParallelRaceGroup(new RevvAuto(shooter, drive, trenchTimeout), leaveMiddleAuto)).
    //               andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, leaveMiddleAutoPath.getPathPoses().get(leaveMiddleAutoPath.getPathPoses().size() - 1).getTranslation(), vision));
    //             }


    //             else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) { //go to middle, come back and shoot, then climb
                  
    //               return new InstantCommand(() -> {drive.resetPosition(goMiddleAutoPath.getStartingHolonomicPose().get());}).
    //               andThen(new ParallelRaceGroup(new IntakeCommand(intake), goMiddleAuto)).
    //               andThen(new ParallelRaceGroup(new RevvAuto(shooter, drive, trenchTimeout), leaveMiddleAuto)).
    //               andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, leaveMiddleAutoPath.getPathPoses().get(leaveMiddleAutoPath.getPathPoses().size() - 1).getTranslation(), vision)).
    //               andThen(Commands.defer(() -> autoClimbing.getClimbingCommand(true), Set.of(drive)));
    //             } 
    //           }
    //         }



    //         else if (chosenLoader.equals(AutoEnums.LoaderEnums.ONE_LOADER)) {
    //           if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) { //this assumes we're starting at the hub and going to the DEPOT

    //             PathPlannerPath hubToDepotPath;
    //             Command hubToDepotAuto;
    //             PathPlannerPath hubLeaveDepotPath;
    //             Command hubLeaveDepotAuto;

    //             try {
    //               if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
    //                 hubToDepotPath = PathPlannerPath.fromPathFile("Hub to Depot Entrance");
    //                 hubLeaveDepotPath = PathPlannerPath.fromPathFile("Return Depot");
    //               } else {
    //                 hubToDepotPath = PathPlannerPath.fromPathFile("Hub to Depot Entrance").flipPath();
    //                 hubLeaveDepotPath = PathPlannerPath.fromPathFile("Return Depot").flipPath();
    //               }
    //             } catch (Exception e) {
    //               return Commands.none();
    //             }

    //             hubToDepotAuto = AutoBuilder.followPath(hubToDepotPath);
    //             hubLeaveDepotAuto = AutoBuilder.followPath(hubLeaveDepotPath);





    //             if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) { //go to the depot and shoot
                  
    //               return new InstantCommand(() -> {drive.resetPosition(hubToDepotPath.getStartingHolonomicPose().get());}).
    //               andThen(new ParallelRaceGroup(hubToDepotAuto, new WaitCommand(1).andThen(new IntakeCommand(intake))), new WaitCommand(1.5).andThen(new RevvAuto(shooter, drive, 0))). //TRIPLE PARALLELRACEGROUP!!!!!!!! AAAAAHHHH
    //               andThen(new ParallelRaceGroup(hubLeaveDepotAuto, new RevvAuto(shooter, drive, 0))).
    //               andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, hubLeaveDepotPath.getPathPoses().get(hubLeaveDepotPath.getPathPoses().size() - 1).getTranslation(), vision));
    //             }


    //             else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) { //go to the depot, shoot, then climb
                  
    //               return new InstantCommand(() -> {drive.resetPosition(hubToDepotPath.getStartingHolonomicPose().get());}).
    //               andThen(new ParallelRaceGroup(hubToDepotAuto, new WaitCommand(1).andThen(new IntakeCommand(intake)), new WaitCommand(1.5).andThen(new RevvAuto(shooter, drive, 0)))). //TRIPLE PARALLELRACEGROUP!!!!!!!! AAAAAHHHH
    //               andThen(new ParallelRaceGroup(hubLeaveDepotAuto, new RevvAuto(shooter, drive, 0))).
    //               andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, hubLeaveDepotPath.getPathPoses().get(hubLeaveDepotPath.getPathPoses().size() - 1).getTranslation(), vision)).
    //               andThen(Commands.defer(() -> autoClimbing.getClimbingCommand(true), Set.of(drive)));
    //             }
    //           }


    //           else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
    //             if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) { //go to middle, come back, shoot, go to depot, shoot
                  
    //               return new InstantCommand(() -> {drive.resetPosition(goMiddleAutoPath.getStartingHolonomicPose().get());}).
    //               andThen(new ParallelRaceGroup(new IntakeCommand(intake), goMiddleAuto)).
    //               andThen(new ParallelRaceGroup(new RevvAuto(shooter, drive, trenchTimeout), leaveMiddleAuto)).
    //               andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, goLoaderAutoPath.getStartingHolonomicPose().get().getTranslation(), vision)).
    //               andThen(new ParallelRaceGroup(new IntakeCommand(intake), goLoaderAuto.andThen(returnFromLoaderAuto), new RevvAuto(shooter, drive, 0))).
    //               andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, returnFromLoaderAutoPath.getPathPoses().get(returnFromLoaderAutoPath.getPathPoses().size() - 1).getTranslation(), vision));
    //             }


    //             else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) { //go to middle, come back while shooting, go to depot, shoot, climb
                  
    //               return new InstantCommand(() -> {drive.resetPosition(goMiddleAutoPath.getStartingHolonomicPose().get());}).
    //               andThen(new ParallelRaceGroup(new IntakeCommand(intake), goMiddleAuto)).
    //               andThen(new ParallelRaceGroup(new RevvAuto(shooter, drive, trenchTimeout), leaveMiddleAuto)).
    //               andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, goLoaderAutoPath.getStartingHolonomicPose().get().getTranslation(), vision)).
    //               andThen(new ParallelRaceGroup(new IntakeCommand(intake), goLoaderAuto.andThen(returnFromLoaderAuto), new RevvAuto(shooter, drive, 0))).
    //               andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, returnFromLoaderAutoPath.getPathPoses().get(returnFromLoaderAutoPath.getPathPoses().size() - 1).getTranslation(), vision)).
    //               andThen(Commands.defer(() -> autoClimbing.getClimbingCommand(true), Set.of(drive)));
    //             }      
    //           }
    //         }   
    //   }



    //   else if (chosenLoader.equals(AutoEnums.LoaderEnums.TWO_LOADERS)) { 

    //     //new paths and autos have to be created to go from the hub to the outpost
    //         PathPlannerPath HubToOutpostPath;
    //         Command HubToOutpostAuto;

    //         PathPlannerPath OutpostToDepotPath;
    //         Command OutpostToDepotAuto;

    //         PathPlannerPath LeaveDepotPath;
    //         Command LeaveDepotAuto;

    //         try {
    //           if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
    //             HubToOutpostPath = PathPlannerPath.fromPathFile("Hub to (Collect Outpost)");
    //             OutpostToDepotPath = PathPlannerPath.fromPathFile("OUTPOST TO DEPOT RAAHH");
    //             LeaveDepotPath = PathPlannerPath.fromPathFile("Return Depot");

    //           } else {
    //             HubToOutpostPath = PathPlannerPath.fromPathFile("Hub to (Collect Outpost)").flipPath();
    //             OutpostToDepotPath = PathPlannerPath.fromPathFile("OUTPOST TO DEPOT RAAHH").flipPath();
    //             LeaveDepotPath = PathPlannerPath.fromPathFile("Return Depot").flipPath();

    //           }
    //         } catch (Exception e) {
    //           return Commands.none();
    //         }

    //       HubToOutpostAuto = AutoBuilder.followPath(HubToOutpostPath);
    //       OutpostToDepotAuto = AutoBuilder.followPath(OutpostToDepotPath);
    //       LeaveDepotAuto = AutoBuilder.followPath(LeaveDepotPath);


    //       return new InstantCommand(() -> {drive.resetPosition(HubToOutpostPath.getStartingHolonomicPose().get());}).
    //       andThen(HubToOutpostAuto).andThen(new WaitCommand(2)).
    //       andThen(new ParallelRaceGroup(OutpostToDepotAuto, new WaitCommand(2).andThen(new IntakeCommand(intake)), new WaitCommand(3).andThen(new RevvAuto(shooter, drive, 0)))).
    //       andThen(new ParallelRaceGroup(LeaveDepotAuto, new RevvAuto(shooter, drive, 0))).
    //       andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), 3, LeaveDepotPath.getPathPoses().get(LeaveDepotPath.getPathPoses().size() - 1).getTranslation(), vision));

    //   }

      
      return Commands.none();
  
    } 
  }        