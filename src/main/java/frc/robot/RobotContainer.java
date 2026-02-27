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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.SwerveConstants.Mod0;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;
import frc.robot.Robot.ShootingState;
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
import frc.robot.Commands.IntakeCommands.ShuffleCommand;
import frc.robot.Commands.ShootingCommands.Revv;
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
  private final XboxController xbox = new XboxController(0);
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
        Climb climb = new Climb(new ClimbIO());



 

            
          
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
                vision = new VisionSubsystem(new VisionIOLimelight(), drive);
                autoTrenching = new AutomaticTrenching(drive, drive.constraints_auto, () -> -controller.getLeftY(), () -> -controller.getLeftX(), 0.08, controller);     
                autoClimbing = new AutomaticClimbing(drive, new AutoAlign(2.5, drive.rotationkP, 0.01, 1), vision);
      
                
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
         controller.leftBumper().whileTrue(new IntakeCommand(intake));

         

         controller.rightTrigger().whileTrue(new InstantCommand(() -> {intake.setIntakeDutyCycle(0.3);}, intake)
       .andThen((new InstantCommand(() -> {intake.Retract();}, intake)
               .andThen(new WaitCommand(0.7))
               .andThen(new InstantCommand(() -> {intake.Extend();}, intake))
               .andThen(new WaitCommand(0.5))).repeatedly()).
               
        handleInterrupt(() -> {intake.setIntakeDutyCycle(0);}));

        controller.leftTrigger().whileTrue(new StartEndCommand(() -> {intake.Retract();}, () -> {intake.Extend();}, intake));
        controller.leftBumper().whileTrue(new StartEndCommand(() -> {shooter.setOutputShooter(0.3); shooter.setFeederVelocity(0.3);}, 
                                                              () -> {shooter.setOutputShooter(0); shooter.setFeederVelocity(0);}, shooter));
         

        
      }           



      //  controller.x().whileTrue(autoTrenching.andThen(
        
      // Commands.defer(() -> autoTrenching.getPathingCommand().until(
        
      //  () -> (autoTrenching.passedTrench() && 
      //  (Math.abs(controller.getLeftY()) > 0.1 || Math.abs(controller.getLeftX()) > 0.1 || Math.abs(controller.getRightX()) > 0.1))), Set.of(drive))));

      
      //  controller.a().whileTrue(new AutoBumping(drive, intake, () -> -controller.getLeftY(), () -> -controller.getLeftX(), 0.08, controller));
      //  controller.y().whileTrue(Commands.defer(() -> autoClimbing.getClimbingCommand(), Set.of(drive)));
      //  controller.leftBumper().whileTrue(new IntakeCommand(intake));
      //  controller.rightBumper().onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d(drive.getEstimatedPosition().getTranslation(), DriverStation.getAlliance().get().equals(Alliance.Blue) ? Rotation2d.kZero : Rotation2d.fromDegrees(180))), drive)
      //           .ignoringDisable(true));
       
      //  controller2.rightTrigger().whileTrue(new InstantCommand(() -> {intake.setIntakeDutyCycle(0.3);}, intake)
      //  .andThen((new InstantCommand(() -> {intake.Retract();}, intake)
      //          .andThen(new WaitCommand(0.7))
      //          .andThen(new InstantCommand(() -> {intake.Extend();}, intake))
      //          .andThen(new WaitCommand(0.5))).repeatedly()).
               
      //   handleInterrupt(() -> {intake.setIntakeDutyCycle(0);}));
       
      //  controller2.leftTrigger().whileTrue(new StartEndCommand(() -> {intake.Retract();}, () -> {intake.Extend();}, intake));
      //  controller2.rightBumper().whileTrue(new Jam(indexer, shooter, intake));
      //  controller2.leftBumper().onTrue(new InstantCommand(() -> {
      //   if (Robot.shootingState.equals(ShootingState.PASSING)) {
      //     Robot.shootingState = ShootingState.SHOOTING;
      //   }
      //   else {
      //     Robot.shootingState = ShootingState.PASSING;
      //   }
      //  }));

      //  controller2.y().onTrue(new InstantCommand(() -> {Robot.autoWinner = Robot.AutoWinner.US;}));
      //  controller2.a().onTrue(new InstantCommand(() -> {Robot.autoWinner = Robot.AutoWinner.ENEMY;}));
      //  controller2.b().onTrue(new InstantCommand(() -> {intake.resetPivotPosition();}));
       //put controller command for reseting pivot of shooter
       
       

    
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *                                                                                                                                                                                                                                                                                                                                                                                                                                          
   * @return the command to run in autonomous
   */







 //builds an auto command based on initialized auto states chosen by 
  public Command getAutonomousCommand(AutoEnums.LoaderEnums chosenLoader, AutoEnums.ClimbEnums chosenClimb, AutoEnums.MiddleEnums chosenMiddle, AutoEnums.PositionEnums chosenPosition) {
      Command middleAuto = Commands.none();
    Command returnPath = Commands.none();
    Command DepotPath = Commands.none();

 try{
    PathPlannerPath middleAutoPath = PathPlannerPath.fromPathFile("Human Player Center Approach");
     middleAuto = AutoBuilder.followPath(middleAutoPath);
     returnPath = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Human Player Wayback"));
     DepotPath = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Collect Outpost"));

     return new InstantCommand(() -> {drive.resetPosition(middleAutoPath.getStartingHolonomicPose().get());}).
     andThen(new ParallelRaceGroup(new IntakeCommand(intake), middleAuto)).
     andThen(new ParallelRaceGroup(new Revv(shooter, drive, controller), returnPath)).
     andThen(new Shooting(shooter, drive, indexer, intake, controller,  () -> -controller.getLeftY(), () -> -controller.getLeftX(), drive.rotationkP, new ShuffleCommand(intake))).
     andThen(DepotPath).andThen(new WaitCommand(2)).
     andThen(new ShootingAuto(shooter, drive, indexer, intake, drive.rotationkP, new ShuffleCommand(intake), new Translation2d(1.326,2.3), 0.1)).
     andThen(Commands.defer(() -> autoClimbing.getClimbingCommand(), Set.of(drive)));
 }

    catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

  

    if (chosenPosition.equals(AutoEnums.PositionEnums.DEPOT)) {
      if (chosenLoader.equals(AutoEnums.LoaderEnums.ZERO_LOADERS)) {
        if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //basic auto and do nothing
          }


          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //only climb
          }
        }


        else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //go to middle, come back and shoot
          }


          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //go to middle, come back and shoot, then climb
          } 
        }
      }



      else if (chosenLoader.equals(AutoEnums.LoaderEnums.ONE_LOADER)) {
        if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //go to the depot and shoot
          }


          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //go to the depot, shoot, then climb
          }
        }


        else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //go to middle, come back while shooting, go to depot, shoot
          }


          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //go to middle, come back while shooting, go to depot, shoot, climb
          }      
        }
      }



      else if (chosenLoader.equals(AutoEnums.LoaderEnums.TWO_LOADERS)) {
        if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //go to the depot, shoot while going to outpost, shoot
          }

          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //go to the depot, shoot while going to outpost, shoot, then climb
          }
        }


        else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //just go to the middle and shoot while coming back

  //         }

          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //just go to the middle, shoot while coming back, and climb
          }         
        }
      }
    }







    if (chosenPosition.equals(AutoEnums.PositionEnums.OUTPOST)) {
      if (chosenLoader.equals(AutoEnums.LoaderEnums.ZERO_LOADERS)) {
        if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //basic auto and do nothing
          }


          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //only climb
          }
        }


        else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //go to middle, come back and shoot
          }


          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //go to middle, come back and shoot, then climb
          } 
        }
      }



      else if (chosenLoader.equals(AutoEnums.LoaderEnums.ONE_LOADER)) {
        if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //go to the outpost and shoot
          }


          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //go to the outpost, shoot, then climb
          }
        }


        else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //go to middle, come back while shooting, go to outpost, shoot
          }


          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //go to middle, come back while shooting, go to outpost, shoot, climb
          }      
        }
      }



      else if (chosenLoader.equals(AutoEnums.LoaderEnums.TWO_LOADERS)) {
        if (chosenMiddle.equals(AutoEnums.MiddleEnums.FALSE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //go to the outpost, shoot while going to depot, shoot
          }

          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //go to the outpost, shoot while going to depot, shoot, then climb
          }
        }


        else if (chosenMiddle.equals(AutoEnums.MiddleEnums.TRUE)) {
          if (chosenClimb.equals(AutoEnums.ClimbEnums.FALSE)) {
            //just go to the middle and shoot while coming back

  //         }

          else if (chosenClimb.equals(AutoEnums.ClimbEnums.TRUE)) {
            //just go to the middle, shoot while coming back, and climb
          }         
        }
      }
    }
  





    







    
  //   //decides which initial states exist, which will affect the auto paths
  //   //1. middle or not
  //  //1. loaders
  //  //2. clkimb no climb

  //  //finish enum, then if statements, delete colors





   
  //  return new PathPlannerAuto("3 Piece Top");
  
   
   
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