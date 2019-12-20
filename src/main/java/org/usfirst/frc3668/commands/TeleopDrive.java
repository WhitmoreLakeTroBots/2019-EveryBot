package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class TeleopDrive extends Command {

    public TeleopDrive() {
        requires(Robot.Chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.Chassis.DriveStick(Robot.oi.joyDrive);
    
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.Chassis.DriveMan(0, 0);
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.Chassis.DriveMan(0, 0);
    }
}