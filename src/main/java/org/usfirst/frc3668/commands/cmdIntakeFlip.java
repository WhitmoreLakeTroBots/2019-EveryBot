/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3668.commands;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.usfirst.frc3668.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class cmdIntakeFlip extends Command {
  public cmdIntakeFlip() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.pneumatics);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.pneumatics.doubleFlip(DoubleSolenoid.Value.kForward);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.pneumatics.doubleFlip(DoubleSolenoid.Value.kReverse);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  Robot.pneumatics.doubleFlip(DoubleSolenoid.Value.kOff);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  end();
  }
}
