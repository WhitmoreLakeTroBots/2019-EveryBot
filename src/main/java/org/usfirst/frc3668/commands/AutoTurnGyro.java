/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class AutoTurnGyro extends Command {
  public AutoTurnGyro() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.Chassis.resetBothEncoders();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.err.println("TICS: "+  Robot.Chassis.getRightEncoderDist() +" ,Right " + Robot.Chassis.getLeftEncoderDist() + "  NAVX: " + Robot.Chassis.navx.getAngle());
    if(Robot.Chassis.navx.getAngle() < 90){
      Robot.Chassis.runLeftMotor(0.2);
      Robot.Chassis.runRightMotor(0.2);
    }
    else if (Robot.Chassis.navx.getAngle() >= 84){
      end();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.Chassis.runMotors(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
