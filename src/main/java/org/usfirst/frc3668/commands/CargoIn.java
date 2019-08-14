
package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;

import edu.wpi.first.wpilibj.command.Command;
public class CargoIn extends Command {
    public CargoIn (){
        requires(Robot.intake);
    }
    @Override
    protected void initialize (){

    }
    @Override 
    protected void execute (){
    Robot.intake.setIntakeMotor(1);
    }
    @Override 
    protected boolean isFinished (){
        return false;
    }
    @Override
    protected void end(){
        Robot.intake.setIntakeMotor(0);
    }
    @Override
    protected void interrupted (){
        Robot.intake.setIntakeMotor(0);
    }
}
