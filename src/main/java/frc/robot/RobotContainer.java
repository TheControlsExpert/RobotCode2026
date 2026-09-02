// Copyright 2021-2025 FRC 6328 blah blah blah
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
import frc.robot.AutoEnums.PositionEnums;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Robot.ElmoState;
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
import frc.robot.Commands.DriveCommands.AligningCommands.AutomaticPushingP1;
import frc.robot.Commands.DriveCommands.AligningCommands.AutomaticPushingP2;
import frc.robot.Commands.DriveCommands.AligningCommands.AutomaticTrenching;
import frc.robot.Commands.DriveCommands.AligningCommands.ProfiledPIDCommand;
import frc.robot.Commands.IntakeCommands.IntakeCommand;
import frc.robot.Commands.IntakeCommands.Jam;
import frc.robot.Commands.IntakeCommands.ShuffleCommand;
import frc.robot.Commands.ShootingCommands.ResetHood;
import frc.robot.Commands.ShootingCommands.Revv;
import frc.robot.Commands.ShootingCommands.RevvJam;
import frc.robot.Commands.ShootingCommands.RevvTest;
import frc.robot.Commands.ShootingCommands.Shooting;
import frc.robot.Commands.ShootingCommands.ShootingTest;
import frc.robot.Commands.ShootingCommands.shootingPathsAuto;
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
import java.util.function.Supplier;

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
  public static boolean isShooting = false;


  //public static Spark leds = new Spark(0);

       
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final XboxController co4Controller = new XboxController(0);
  private final CommandXboxController controller3 = new CommandXboxController(2);
  //private final XboxController xbox = new XboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);
  public Timer timeout_shuffle;
  AutomaticTrenching autoTrenching;
  AutomaticClimbing autoClimbing;
  Command autoCommand = Commands.none();

  PathPlannerPath firstMiddlePath;
  PathPlannerPath secondMiddlePath;


 

// private boolean isFlipped =
// DriverStation.getAlliance().isPresent()
// && DriverStation.getAlliance().get() == Alliance.Red;
  
  
  
    private GyroIOPigeon2 gyro;
   
    public VisionSubsystem vision;

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
                autoTrenching = new AutomaticTrenching(drive, drive.constraints_auto, () -> -controller.getLeftY(), () -> -controller.getLeftX(), 0.15, controller);     
             //   autoClimbing = new AutomaticClimbing(drive, new AutoAlign(2.5, drive.rotationkP, 0.01, 1), vision, climb);
      
                
        //       superstructure = new Superstructure(new WristIOKrakens(), new ElevatorIOKrakens());        
               
                

                vision = new VisionSubsystem(new VisionIOLimelight(), drive);
    
        //     constraints = new PathConstraints(
       //       2.0, 4.0,
          //    Units.degreesToRadians(400), Units.degreesToRadians(720));
      //
      // Since AutoBuilder is configured, we can use it to build pathfinding commands
    //controller.x().whileTrue(FeedforwardCharacterization.feedforwardCommand(drive, co4Controller));
  //  controller.x().whileTrue(kACharacterization.feedforwardCommand(drive, co4Controller));
      
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

      FollowPathCommand.warmupCommand();

      try {
         PathPlannerPath firstPath = PathPlannerPath.fromChoreoTrajectory("ShortSurf1").flipPath();
         PathPlannerPath secondPath = PathPlannerPath.fromChoreoTrajectory("ShortSurf2").flipPath();
         PathPlannerPath thirdPath = PathPlannerPath.fromChoreoTrajectory("ShortSurf3").flipPath();
         PathPlannerPath fourthPath = PathPlannerPath.fromChoreoTrajectory("ShortSurf4").flipPath();

         
        Command firstPathCommand = AutoBuilder.followPath(firstPath);
        Command secondPathCommand = AutoBuilder.followPath(secondPath);
        Command thirdPathCommand = AutoBuilder.followPath(thirdPath);
        Command fourthPathCommand = AutoBuilder.followPath(fourthPath);
         autoCommand = new ParallelCommandGroup(new ParallelRaceGroup(firstPathCommand.andThen(secondPathCommand), new IntakeCommand(intake, 6).andThen(new RevvJam(shooter, drive, indexer, controller, vision))), new WaitCommand(1).andThen(new InstantCommand(() -> {vision.enableVision();}))).



                      andThen(new ParallelRaceGroup(new shootingPathsAuto(shooter, drive, indexer, thirdPath, vision), new WaitCommand(1).andThen(  
                      (new InstantCommand(() -> {intake.Shuffle(); intake.setIntakeDutyCycle(0.4);}, intake).
                      andThen(new WaitCommand(0.5)).
                      andThen(new InstantCommand(() -> {intake.Extend();}, intake)).
                      andThen(new WaitCommand(0.3))).repeatedly()

                      ))).
                      andThen(new Jam(indexer, shooter, 0.4)).
                     
                      andThen(new ParallelRaceGroup(thirdPathCommand.andThen(fourthPathCommand), new IntakeCommand(intake, 5.5).andThen(new RevvJam(shooter, drive, indexer, controller, vision)))).
                      andThen(new ParallelCommandGroup(new Shooting(shooter, drive, indexer, intake, controller, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX(), drive.rotationkP, vision), new WaitCommand(1).andThen((
                      (new InstantCommand(() -> {intake.Shuffle(); intake.setIntakeDutyCycle(0.4);}, intake).
                      andThen(new WaitCommand(0.5)).
                      andThen(new InstantCommand(() -> {intake.Extend();}, intake)).
                      andThen(new WaitCommand(0.3)))).repeatedly())));
    
      }

      catch (Exception e) {
        System.out.println("Failed to load auto paths, defaulting to nothing");
        autoCommand = Commands.none();
       // autoChooser.setDefaultOption("Nothing", Commands.none());

      }
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


         controller.leftBumper().and(controller::isConnected).whileTrue(new InstantCommand(() -> {intake.is_busy = true; }).andThen(new IntakeCommand(intake)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)).onFalse(new InstantCommand(() -> {intake.is_busy = false;}));
         controller.button(8).onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d(drive.getEstimatedPosition().getTranslation(), DriverStation.getAlliance().get().equals(Alliance.Blue) ? Rotation2d.kZero : Rotation2d.fromDegrees(180))), drive)
                 .ignoringDisable(true));


       controller2.povDown().and(controller2::isConnected).whileTrue(new InstantCommand(() -> {intake.setIntakeDutyCycle(-0.5); intake.is_busy = true;}, intake)).onFalse(new InstantCommand(() -> {intake.setIntakeDutyCycle(0); intake.is_busy = false;}, intake));
    
      
          
        
      Shooting shooting = new Shooting(shooter, drive, indexer, intake, controller, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX(), drive.rotationkP, vision);
        controller.leftTrigger().whileTrue(new Revv(shooter, drive, controller, vision));
        controller.rightTrigger().and(controller::isConnected)
        .whileTrue(shooting)
        .onFalse(
        new InstantCommand(() -> {intake.Extend(); shooter.isShooting = false; RobotContainer.isShooting = false; 
          intake.setIntakeDutyCycle(0.0);}, intake));

        // controller.rightTrigger().and(controller::isConnected).and(() -> Robot.elmoState.equals(ElmoState.ManualControl))
       
        // .whileTrue(shooting)
        // .onFalse(
        // new InstantCommand(() -> {intake.Extend(); shooter.isShooting = false; RobotContainer.isShooting = false; 
        //   intake.setIntakeDutyCycle(0.0);}, intake));


        // controller.rightTrigger().and(()->(!shooting.readyToShoot || !ActivePeriodTracker.getShiftedShiftInfo().active())).whileTrue(new InstantCommand(()-> {controller.setRumble(RumbleType.kBothRumble, 0.35);}))
        // .onFalse(new InstantCommand(() -> {controller.setRumble(RumbleType.kBothRumble, 0);}));
       
        


        //(controller.leftTrigger().and(controller::isConnected)).or(controller2.leftTrigger().and(controller2::isConnected)).whileTrue(new RevvJam(shooter, drive, indexer, controller, vision));
        // Y/A in ManualControl mode: adjust ShootingManualHoodPosition (base for shootManual)
        controller.a().and(controller::isConnected)
            .and(() -> Robot.elmoState.equals(ElmoState.ManualControl))
            .whileTrue(Commands.run(() ->
                { shooter.ShootingManualHoodPosition = Math.min(shooter.ShootingManualHoodPosition + 0.5, 17.5); }));

        controller.y().and(controller::isConnected)
            .and(() -> Robot.elmoState.equals(ElmoState.ManualControl))
            .whileTrue(Commands.run(() ->
                { shooter.ShootingManualHoodPosition = Math.max(shooter.ShootingManualHoodPosition - 0.5, -17.5); }));

        

        

       
       controller.rightBumper().and(controller::isConnected).whileTrue(autoTrenching);
         
         //.andThen(
        
      // Commands.defer(() -> autoTrenching.getPathingCommand().until(
        
      //  () -> (autoTrenching.passedTrench() && 
      //  (Math.abs(controller.getLeftY()) > 0.1 || Math.abs(controller.getLeftX()) > 0.1 || Math.abs(controller.getRightX()) > 0.1))), Set.of(drive))));

      //COPILOT

      //intake overrides/fixes
     // controller.x().whileTrue(kACharacterization.feedforwardCommand(drive, co4Controller));
      controller2.b().and(controller2::isConnected).and(() -> (shooter.isShooting)).whileTrue((new InstantCommand(() -> {intake.Shuffle(); intake.setIntakeDutyCycle(0.6);}, intake).
       andThen(new WaitCommand(0.5)).
       andThen(new InstantCommand(() -> {intake.Extend();}, intake)).
       andThen(new WaitCommand(0.3))).repeatedly());


      controller.x().and(controller::isConnected).whileTrue(new StartEndCommand(() -> {intake.Retract(); intake.is_busy = true; intake.setIntakeDutyCycle(0.4);}, () -> {intake.Extend(); intake.is_busy = false; intake.setIntakeDutyCycle(0);}, intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
      //controller2.b().onTrue(new InstantCommand(() -> {intake.Retract(); intake.is_busy = true;}, intake));

      controller3.x().whileTrue(new StartEndCommand(() -> {vision.ruin = true;}, () -> {vision.ruin = false;}).ignoringDisable(true));

      // controller2.rightTrigger().whileTrue(new Jam(indexer, shooter));
         // controller.rightTrigger().whileTrue(
      // Commands.defer(() -> { 
      //   if (intake.isShuffling) {
      // return Commands.none();}vis

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
       controller2.leftBumper().onTrue(new InstantCommand(() -> {
        if (Robot.shootingState.equals(ShootingState.PASSING)) {
          Robot.shootingState = ShootingState.SHOOTING;
        }
        else {
          Robot.shootingState = ShootingState.PASSING;
        }
       }));

       controller.b().onTrue(new InstantCommand(() -> {
        if (Robot.elmoState.equals(ElmoState.ManualControl)) {
          Robot.elmoState = ElmoState.InterpolatonShooting;
        }
        else {
          Robot.elmoState = ElmoState.ManualControl;
        }
       }));

      //  //decide auto winner
       controller2.y().onTrue(new InstantCommand(() -> {Robot.autoWinner = Robot.AutoWinner.ENEMY; Robot.winner_selection_done = true;}));
       controller2.a().onTrue(new InstantCommand(() -> {Robot.autoWinner = Robot.AutoWinner.US; Robot.winner_selection_done = true;}));










       //resets of encoders
      // controller2.b().onTrue(new InstantCommand(() -> {intake.resetPivotPosition();}));
      // controller2.x().onTrue(new ResetHood(shooter));

       Timer when_to_signal_disconnectedFMS = new Timer();
       RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {when_to_signal_disconnectedFMS.restart();}));
       RobotModeTriggers.teleop().and(() -> {return when_to_signal_disconnectedFMS.hasElapsed(3);}).and(() -> (!Robot.winner_selection_done)).whileTrue(Commands.startEnd(
                () -> {
                  SmartDashboard.putBoolean("controller was told to rumble", true);
                  controller2.setRumble(RumbleType.kBothRumble, 1);
                },
                () -> {
                  SmartDashboard.putBoolean("controller was told to rumble", false);
                  controller2.setRumble(RumbleType.kBothRumble, 0);
                }));
        
      }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *                                                                                                                                                                                                                                                                                                                                                                                                                                          
   * @return the command to run in autonomous
   */






  public Command getAutonomousCommand() {
    vision.disableVision();
    drive.resetPosition(firstMiddlePath.getStartingHolonomicPose().get());


   return autoCommand;
   
  }
     







    public Translation2d FlipHorizontally_BtoR(Translation2d point) {
        return new Translation2d( 2* (8.219694 - point.getX()) + point.getX(), point.getY()); 
    }
    //flips translation2d from bottom of blue to top of blue
    public Translation2d FlipVertically_bottom_to_top(Translation2d point) {
        return new Translation2d( point.getX(), 2* (4.021328 - point.getY()) + point.getY()); 
     }

  }             