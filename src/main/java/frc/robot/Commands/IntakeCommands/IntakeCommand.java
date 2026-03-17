package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IntakeCommand extends Command{
    IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    //    intake.setIntakeDutyCycle(1);
        intake.Extend();
    }

    @Override
    public void end(boolean interrupted) {
   //     intake.setIntakeDutyCycle(0.0);
    }   
}
