
package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;

import edu.wpi.first.wpilibj.command.Command;
public class CargoIn extends Command {
    public CargoIn (){
        requires(Robot.Intake);
    }
    @Override
    protected void initialize (){

    }
    @Override 
    protected void execute (){
    Robot.Intake.setIntakepMotor(1);
    }
    @Override 
    protected boolean isFinished (){
        return false;
    }
    @Override
    protected void end(){
        Robot.Intake.setIntakeMotor(0);
    }
    @Override
    protected void interrupted (){
        Robot.Intake.setIntakeMotor(0);
    }
}
