package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Fulcrum;
import frc.robot.subsystems.Intake;

public class IntakeCmd2 extends Command {
    Intake intake;
    Feeder feeder;
    Fulcrum fulcrum;
    boolean noteWasDetected = false;
    boolean shouldEnd = false;

    public IntakeCmd2(
            Intake intake, Feeder feeder, Fulcrum fulcrum) {
        this.intake = intake;
        this.feeder = feeder;
        this.fulcrum = fulcrum;
        addRequirements(intake, feeder, fulcrum);
    }

    @Override
    public void initialize() {
        this.fulcrum.resetController();
        this.fulcrum.setSetPoint(9);
     
    }

    @Override
    public void execute() {
        this.fulcrum.closedLoopFulcrum();
        if (this.fulcrum.isAtPoint()) {
            this.intake.intake();
            this.feeder.feed();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.intakeStop();
        this.feeder.stopAll();
        this.fulcrum.closedLoopFulcrum();        
    }

    @Override
    public boolean isFinished() {
        return this.feeder.isNoteDetected();
    }

}
