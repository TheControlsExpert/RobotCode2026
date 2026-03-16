package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot.LocalizationState;
import frc.robot.Robot.ShootingState;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class Revv extends Command {

    private final Shooter shooter;
    private final Drive drive;
    private final CommandXboxController controller;
    private double when_to_start = -1000;
    VisionSubsystem vision;


    public Revv(Shooter shooter, Drive drive, CommandXboxController controller, VisionSubsystem vision) {
        this.shooter = shooter;
        this.drive = drive;
        this.controller = controller;
        this.vision = vision;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        vision.ShootingMode(true);
      //  shooter.setShooterVelocity(3450/60);
      //  shooter.setPositionPivot(4);
    }
  
    @Override
    public void execute() {
        
        shooter.LookupTable_Shooting(drive);
        // if (Robot.localizationState.equals(LocalizationState.OPERATIONAL)) {
        //     double distance = drive.getEstimatedPosition().getTranslation().getDistance(drive.calculateShootingPosition());
        //     double[] shootingValues = shooter.LookupTable_Shooting(drive);
        //     shooter.setShooterVelocity(shootingValues[0]);
        // }
        // else {
        //     if (Robot.shootingState.equals(ShootingState.SHOOTING)) {
        //         shooter.setShooterVelocity(ShooterConstants.HUB_SHOOTING_VELOCITY);
        //     }
        //      else if (Robot.shootingState.equals(ShootingState.PASSING)) {
        //         shooter.setShooterVelocity(ShooterConstants.BASIC_PASSING_VELOCITY);
        //     }
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if (!controller.rightTrigger().getAsBoolean() && DriverStation.isTeleop()) {
            shooter.setShooterVelocity(0);
            shooter.setPositionPivot(ShooterConstants.Pivot_HOME);
            vision.ShootingMode(false);
        }
}
   
}
