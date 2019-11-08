


package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;
import org.usfirst.frc3668.subsystems.Chassis;

import edu.wpi.first.wpilibj.command.Command;

public class AutonomousCommand extends Command {
    double tthrottle = .4;

    public AutonomousCommand() {
        requires(Robot.Chassis);
    }
    
   
    @Override
    protected void initialize() {
        Robot.Chassis.resetBothEncoders();
    }

  
    @Override
    protected void execute() {
        System.err.println("TICS: "+  Robot.Chassis.getRightEncoderDist() +" ,Right " + Robot.Chassis.getLeftEncoderDist() + "  NAVX: " + Robot.Chassis.navx.getAngle());
      
    }

   
    @Override
    protected boolean isFinished() {
        return false;
    }

   
    @Override
    protected void end() {
        isFinished();
    }

   
    @Override
    protected void interrupted() {
    }
}
