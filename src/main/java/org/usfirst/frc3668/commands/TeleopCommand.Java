package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;


import edu.wpi.first.wpilibj.command.Command;

public class teleopCommand extends Command {

    public cmdTeleopDrive() {
        requires(Robot.subChassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.subChassis.Drive(Robot.oi.joyDrive);
        //System.err.println("Front Lift Encoder: " + Robot.subLift.getLiftEncoderTics() + " Lift Encoder Meters: " + Robot.subLift.getLiftEncoderMeters() + " Tail Angle: " + Robot.subTail.getTailAngle());
        //System.err.println(String.format("Heading: %1$.3f\t llAngle: %2$.3f", Robot.subChassis.getNormaliziedNavxAngle(), Robot.ltx.getDouble(Settings.ltDefaultAngle)));
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.subChassis.DriveMan(0, 0);
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.subChassis.DriveMan(0, 0);
    }
}