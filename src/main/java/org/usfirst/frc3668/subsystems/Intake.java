


package org.usfirst.frc3668.subsystems;


import org.usfirst.frc3668.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;




public class Intake extends Subsystem {

    
    public WPI_TalonSRX intake;



    public Intake() {
        
        intake = new WPI_TalonSRX(5);
        
        
        

    
    }

    @Override
    public void initDefaultCommand() {
        
    }

    @Override
    public void periodic() {
        

    }

    



}

