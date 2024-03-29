// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeedHoldCmd extends Command {
  /** Creates a new FeedHoldCmd. */
  private final Feeder m_feeder;
  public FeedHoldCmd(Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feeder = feeder;
    addRequirements(m_feeder);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder.closedLoopFeeder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.closedLoopFeeder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
