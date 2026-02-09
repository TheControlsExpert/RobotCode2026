package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class IntakeCommand extends Command{
    Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setSpeed(0.35);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }
    
}
