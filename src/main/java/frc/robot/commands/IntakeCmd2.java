package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class IntakeCmd2 extends Command {
    Intake intake;
    Feeder feeder;
    boolean noteWasDetected = false;
    boolean shouldEnd = false;

    public IntakeCmd2(
            Intake intake, Feeder feeder) {
        this.intake = intake;
        this.feeder = feeder;
        addRequirements(intake, feeder);
    }

    @Override
    public void initialize() {
     
    }

    @Override
    public void execute() {
        this.intake.intake();
        this.feeder.feed();
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.intakeStop();
        this.feeder.stopAll();
        
    }

    @Override
    public boolean isFinished() {
        return this.feeder.isNoteDetected();
    }

}
