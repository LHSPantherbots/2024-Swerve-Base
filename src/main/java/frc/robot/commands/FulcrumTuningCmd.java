package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Fulcrum;

public class FulcrumTuningCmd extends Command {
    Fulcrum fulcrum;
    boolean finishes;

    public FulcrumTuningCmd(Fulcrum fulcrum) {
        this.fulcrum = fulcrum;
        this.finishes = true;
        addRequirements(fulcrum);
    }

    public FulcrumTuningCmd(Fulcrum fulcrum, boolean finishes) {
        this.fulcrum = fulcrum;
        this.finishes = finishes;
        addRequirements(fulcrum);
    }

    @Override
    public void initialize() {
        this.fulcrum.resetController();
        this.fulcrum.setSetPoint(this.fulcrum.getSbAngle());
    }

    @Override
    public void execute() {
        this.fulcrum.closedLoopFulcrum();
    }

    @Override
    public void end(boolean interrupted) {
        this.fulcrum.closedLoopFulcrum();
    }

    @Override
    public boolean isFinished() {
        if (finishes) {
            return this.fulcrum.isAtPoint();
        } else {
            return false;
        }
    }
}
