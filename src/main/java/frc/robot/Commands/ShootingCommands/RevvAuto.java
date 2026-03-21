// package frc.robot.Commands.ShootingCommands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.Shooter.Shooter;
// import frc.robot.Subsystems.Drive.Drive;

// public class RevvAuto extends Command{
//     private final Shooter shooter;
//     private final Drive drive;
//     private final double timeout;
//     private final Timer timer = new Timer();

//     public RevvAuto(Shooter shooter, Drive drive, double timeout) {
//         this.shooter = shooter;
//         this.drive = drive;
//         this.timeout = timeout;
//         addRequirements(shooter);
//     }

//     @Override
//     public void initialize() {
//         timer.restart();
//     }

//     @Override
//     public void execute() {
//         double distance = drive.getEstimatedPosition().getTranslation().getDistance(drive.calculateShootingPosition());
//         double[] shootingValues = shooter.LookupTable_Shooting(drive);
//         shooter.setShooterVelocity(shootingValues[0]);

//         if (timer.hasElapsed(timeout)) {
//             shooter.setPositionPivot(shootingValues[1]);
//         }
        
//     }

    
// }
