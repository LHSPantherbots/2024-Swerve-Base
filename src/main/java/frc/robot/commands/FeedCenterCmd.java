// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeedCenterCmd extends Command {
  /** Creates a new FeedCenterCmd. */

  Feeder feeder;
  double timerSet = 10;
  double countdown = timerSet;
  double countdownIncrament = 1;


  public FeedCenterCmd(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);


    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print("Countdown: ");
    System.out.println(countdown);
    if(countdown>=0){
      feeder.setFeed(.25);
      countdown = countdown-countdownIncrament;
    }else if(countdown<0 && countdown>-timerSet){
      feeder.setFeed(-.25);
      countdown = countdown-countdownIncrament;
    }else{
      countdown = timerSet;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
