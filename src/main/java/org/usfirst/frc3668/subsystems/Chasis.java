


package org.usfirst.frc3668.subsystems;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Joystick;
import org.usfirst.frc3668.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

import edu.wpi.first.wpilibj.command.Subsystem;



public class Chasis extends Subsystem {

    
    public static PowerDistributionPanel powerDistributionPanel1;
    public static WPI_TalonSRX leftDrive1;
    public static WPI_TalonSRX rightDrive1;
    public static WPI_TalonSRX leftDrive2;
    public static WPI_TalonSRX rightDrive2;
    public static final int chassisDriveMaxCurrentLimit = 55;
    public static final int talonTimeOut = 10;
    public static final int chassisDriveMaxCurrentTimeout = 500;
    public static final int rightDrive1CanID = 1;
	public static final int rightDrive2CanID = 2;
	public static final int leftDrive1CanID = 3;
    public static final int leftDrive2CanID = 4;
    public static boolean isDriveInverted = false;
    public final double joyDriveDeadband = 0.05;
    public final double chassisRightSideScalar = 1;
	public final double chassisLeftSideScalar = 1;
	public final boolean chassisSquareJoyInput = true;
	public final double chassisBeltReduction = 1.0/1.0;
    

    public Chasis() {
        
        powerDistributionPanel1 = new PowerDistributionPanel(0);
        addChild("PowerDistributionPanel 1",powerDistributionPanel1);
        
        
        leftDrive1 = new WPI_TalonSRX(leftDrive1CanID);
        leftDrive1.setNeutralMode(NeutralMode.Brake);
		leftDrive1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Chasis.talonTimeOut);
		leftDrive1.configPeakCurrentLimit(Chasis.chassisDriveMaxCurrentLimit, Chasis.talonTimeOut);
		leftDrive1.configPeakCurrentDuration(Chasis.chassisDriveMaxCurrentTimeout, Chasis.talonTimeOut);
        
        
        rightDrive1 = new WPI_TalonSRX(rightDrive1CanID);
        rightDrive1.setNeutralMode(NeutralMode.Brake);
		rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Chasis.talonTimeOut);
		rightDrive1.configPeakCurrentLimit(Chasis.chassisDriveMaxCurrentLimit, Chasis.talonTimeOut);
		rightDrive1.configPeakCurrentDuration(Chasis.chassisDriveMaxCurrentTimeout, Chasis.talonTimeOut);
        
        
        
        leftDrive2 = new WPI_TalonSRX(leftDrive2CanID);
        leftDrive2.setNeutralMode(NeutralMode.Brake);
		leftDrive2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Chasis.talonTimeOut);
		leftDrive2.configPeakCurrentLimit(Chasis.chassisDriveMaxCurrentLimit, Chasis.talonTimeOut);
		leftDrive2.configPeakCurrentDuration(Chasis.chassisDriveMaxCurrentTimeout, Chasis.talonTimeOut);
        
        
        rightDrive2 = new WPI_TalonSRX(rightDrive2CanID);
        rightDrive2.setNeutralMode(NeutralMode.Brake);
		rightDrive2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Chasis.talonTimeOut);
		rightDrive2.configPeakCurrentLimit(Chasis.chassisDriveMaxCurrentLimit, Chasis.talonTimeOut);
		rightDrive2.configPeakCurrentDuration(Chasis.chassisDriveMaxCurrentTimeout, Chasis.talonTimeOut);
        
    }
        

        public void Drive(Joystick stick) {
            double joyX = stick.getX();
            double joyY = stick.getY();
    
            if (Math.abs(joyX) < joyDriveDeadband) {
                joyX = 0;
            } else if (Math.abs(joyY) < joyDriveDeadband) {
                joyY = 0;
            }
    
            double rightMotorThrottle;
            double leftMotorThrottle;
            if (isDriveInverted) {
                rightMotorThrottle = (joyX + joyY) * chassisRightSideScalar;
                leftMotorThrottle = (joyX - joyY) *  chassisLeftSideScalar;
                if (chassisSquareJoyInput) {
                    double rightSignum = Math.signum(rightMotorThrottle);
                    double leftSignum = Math.signum(leftMotorThrottle);
                    rightMotorThrottle = Math.pow(rightMotorThrottle, 2) * rightSignum;
                    leftMotorThrottle = Math.pow(leftMotorThrottle, 2) * leftSignum;
                }
                
                setLeftMotors(leftMotorThrottle);
                setRightMotors(rightMotorThrottle);
    
            } else {
                rightMotorThrottle = (joyX - joyY) * chassisRightSideScalar;
                leftMotorThrottle = (joyX + joyY) * chassisLeftSideScalar;
                if (chassisSquareJoyInput) {
                    double rightSignum = Math.signum(rightMotorThrottle);
                    double leftSignum = Math.signum(leftMotorThrottle);
                    rightMotorThrottle = Math.pow(rightMotorThrottle, 2) * rightSignum;
                    leftMotorThrottle = Math.pow(leftMotorThrottle, 2) * leftSignum;
                }
                
                setLeftMotors(leftMotorThrottle);
                setRightMotors(rightMotorThrottle);
            }
        }
    
        public void DriveMan(double leftThrottle, double rightThrottle) {
            if(isDriveInverted){
                setLeftMotors(leftThrottle);
                setRightMotors(-rightThrottle);
            } else {
                setLeftMotors(-leftThrottle);
                setRightMotors(rightThrottle);
            }
        }
    
        public void setRightMotors(double throttle) {
            rightDrive1.set(ControlMode.PercentOutput, throttle);
            rightDrive2.set(ControlMode.PercentOutput, throttle);
        }
    
        public void setLeftMotors(double throttle) {
            leftDrive1.set(ControlMode.PercentOutput, throttle);
            leftDrive2.set(ControlMode.PercentOutput, throttle);
        }
    
        public void Drive(double move, double rotate) {
            double rightMotorThrottle;
            double leftMotorThrottle;
            if (!isDriveInverted) {
                rightMotorThrottle = (rotate + move) * chassisRightSideScalar;
                leftMotorThrottle = (rotate - move) * chassisLeftSideScalar;
                setLeftMotors(leftMotorThrottle);	
                setRightMotors(rightMotorThrottle);
            } else {
                rightMotorThrottle = (-rotate - move) * chassisRightSideScalar;
                leftMotorThrottle = (-rotate + move) * chassisLeftSideScalar;
                setLeftMotors(leftMotorThrottle);
                setRightMotors(rightMotorThrottle);
            }
    
        }
    

    @Override
    public void initDefaultCommand() {
       
    }

    @Override
    public void periodic() {
       

    }

}

