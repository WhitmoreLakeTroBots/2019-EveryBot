package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;

import edu.wpi.first.wpilibj.command.Command;
public class CargoUp extends Command {
    public CargoUp (){
        requires(Robot.CargoFlip);
    }
    @Override
    protected void initialize (){

    }
    @Override 
    protected void execute (){
    Robot.CargoFlip.setFlipMotor(0.3);
    }
    @Override 
    protected boolean isFinished (){
        return false;
    }
    @Override
    protected void end(){
        Robot.CargoFlip.setFlipMotor(0);
    }
    @Override
    protected void interrupted (){
        Robot.CargoFlip.setFlipMotor(0);
    }
}
