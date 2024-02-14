// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class FeedbackSubsystem extends SubsystemBase {
  /** Creates a new FeedbackSubsystem. */

  double driveBuzz = 0;
  double oppBuzz = 0;
  //private CommandXboxController driverController;
  private XboxController opperatorController;
  private XboxController driverController;


  public FeedbackSubsystem(XboxController driverController, XboxController opperatorController) {
    this.driverController = driverController;
    this.opperatorController = opperatorController;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void buzzDriver(){
    if (this.driveBuzz>0){
      this.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
      driveBuzz = driveBuzz - 10;
    }
  }

  public void buzzOperator(){
    if (this.oppBuzz>0){
      this.opperatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
      oppBuzz = oppBuzz - 10;
    }
  }

  public void addDriverBuzzTime(double time){
    this.driveBuzz = time;
  }

  public void addOppBuzztime(double time){
    this.oppBuzz = time;
  }
}
