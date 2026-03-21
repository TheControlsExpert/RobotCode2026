package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.Shooter;

public class RevvTest extends Command {
    
private final Shooter shooter;
private final CommandXboxController controller;


public RevvTest(Shooter shooter, CommandXboxController controller) {
    this.shooter = shooter;
    this.controller = controller;

    addRequirements(shooter);
}

@Override
public void initialize() {
    shooter.setShooterVelocity(5000/60);
    shooter.setPositionPivot(0.5);
}

@Override
public void execute() {
}

@Override
public void end(boolean interrupted) {
    if (!controller.rightTrigger().getAsBoolean()) {
        shooter.setShooterVelocity(0);
        shooter.setPositionPivot(ShooterConstants.Pivot_HOME);
    }

}

@Override
public boolean isFinished() {
    return false;
}
}