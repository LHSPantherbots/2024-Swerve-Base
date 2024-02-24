package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Fulcrum;

public class FulcrumAimCmd extends Command {
    Fulcrum fulcrum;

    public FulcrumAimCmd(Fulcrum fulcrum) {
        this.fulcrum = fulcrum;
        addRequirements(fulcrum);
    }

    @Override
    public void initialize() {
        this.fulcrum.resetController();
        this.fulcrum.setSetPoint(this.fulcrum.getAutoFulcrumAngle());
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
        return this.fulcrum.isAtPoint();
    }
}
