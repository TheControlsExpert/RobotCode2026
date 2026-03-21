package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Shooter.Shooter;

public class shootingPathsAuto extends Command{
    Shooter shooter;
    Drive drive;
    Indexer indexer;
    public shootingPathsAuto(Shooter shooter, Drive drive, Indexer indexer) {
        this.drive =drive;
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, drive, indexer);
    }
    public void initialize() {
        
    }

}
