package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;

import edu.wpi.first.wpilibj.command.Command;
public class CargoUp extends Command {
    public CargoUp (){
        requires(Robot.cargoFLip);
    }
    @Override
    protected void initialize (){

    }
    @Override 
    protected void execute (){
    Robot.cargoFLip.setFlipMotor(0.3);
    }
    @Override 
    protected boolean isFinished (){
        return false;
    }
    @Override
    protected void end(){
        Robot.cargoFLip.setFlipMotor(0);
    }
    @Override
    protected void interrupted (){
        Robot.cargoFLip.setFlipMotor(0);
    }
}
