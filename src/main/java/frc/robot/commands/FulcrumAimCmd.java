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
    }

    @Override
    public void execute() {
        this.fulcrum.autoAim();
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
