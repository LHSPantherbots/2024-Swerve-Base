package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Fulcrum;
import frc.utils.Position;

public class FulcrumCmd extends Command {
    Position position;
    Fulcrum fulcrum;
    boolean finishes;

    public FulcrumCmd(Position position, Fulcrum fulcrum) {
        this.position = position;
        this.fulcrum = fulcrum;
        this.finishes = true;
        addRequirements(fulcrum);
    }

    public FulcrumCmd(Position position, Fulcrum fulcrum, boolean finishes) {
        this.position = position;
        this.fulcrum = fulcrum;
        this.finishes = finishes;
        addRequirements(fulcrum);
    }

    @Override
    public void initialize() {
        this.fulcrum.resetController();
        switch (this.position) {
            case STOW:
                this.fulcrum.setSetPoint(10);
                break;
            case INTAKE:
                this.fulcrum.setSetPoint(15);
                break;
            case AMP:
                this.fulcrum.setSetPoint(90);
                break;
            case SPEAKER:
                this.fulcrum.setSetPoint(20);
                break;
        }
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
