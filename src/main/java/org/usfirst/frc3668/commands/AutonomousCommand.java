


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
        System.err.println("TICS: "+  Robot.Chassis.getRightEncoderTics() +" ,Right " + Robot.Chassis.getLeftEncoderTics());
        
        if (Robot.Chassis.rightDrive1.getSelectedSensorPosition() < 113 * 100){
            Robot.Chassis.runMotors(tthrottle);
        }
        else if (Robot.Chassis.rightDrive1.getSelectedSensorPosition() >= 113 * 100){
            Robot.Chassis.runMotors(0);
        }
    
        
    }

   
    @Override
    protected boolean isFinished() {
        return false;
    }

   
    @Override
    protected void end() {
    }

   
    @Override
    protected void interrupted() {
    }
}
