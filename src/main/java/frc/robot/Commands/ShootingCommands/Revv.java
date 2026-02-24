package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot.ShootingState;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.Shooter;

public class Revv extends Command {

    private final Shooter shooter;
    private final Drive drive;
    private final CommandXboxController controller;


    public Revv(Shooter shooter, Drive drive, CommandXboxController controller) {
        this.shooter = shooter;
        this.drive = drive;
        this.controller = controller;

        addRequirements(shooter, drive);
    }

  
    @Override
    public void execute() {
        double distance = drive.getEstimatedPosition().getTranslation().getDistance(drive.calculateShootingPosition());

        shooter.LookupTable_Shooting(drive);
        SmartDashboard.putBoolean("Shooter is at Velocity", shooter.isAtShootingVelocity(distance));
    }

    @Override
    public void end(boolean interrupted) {
        if (!controller.rightTrigger().getAsBoolean()) {
            shooter.setShooterVelocity(0);
            shooter.setPositionPivot(ShooterConstants.Pivot_HOME);      
        }
}


   


    
    
    
}
