


package org.usfirst.frc3668.subsystems;


import org.usfirst.frc3668.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.Encoder;


public class Intake extends Subsystem {



    public TalonSRX intake ;
   
	public static Encoder intake;


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

