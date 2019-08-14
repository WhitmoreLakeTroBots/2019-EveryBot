


package org.usfirst.frc3668.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {

    public TalonSRX intake;
   
	public static Encoder intakeEncoder;


    public Intake() {
        
        intake = new TalonSRX(5);
        //intake = new TalonSRX(intakeCanID);
    intake.setNeutralMode(NeutralMode.Brake);
    intake.configPeakCurrentLimit(Chassis.chassisDriveMaxCurrentLimit, Chassis.talonTimeOut);
    intake.configPeakCurrentDuration(Chassis.chassisDriveMaxCurrentTimeout,  Chassis.talonTimeOut);
   

        
        

    
    }
    public void setIntakeMotor(double Throttle){
        intake.set(ControlMode.PercentOutput, Throttle);
    } 

    @Override
    public void initDefaultCommand() {
        
    }

    @Override
    public void periodic() {
        

    }

    



}

