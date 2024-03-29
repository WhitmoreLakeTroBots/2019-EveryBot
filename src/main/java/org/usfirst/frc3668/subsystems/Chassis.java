


package org.usfirst.frc3668.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc3668.Robot;
import org.usfirst.frc3668.Settings;
import org.usfirst.frc3668.commands.TeleopDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Chassis extends Subsystem {


    public static PowerDistributionPanel PDP;
    public static TalonSRX leftDrive1;
    public static TalonSRX rightDrive1;
    public static TalonSRX leftDrive2;
    public static TalonSRX rightDrive2;
    public static final int chassisDriveMaxCurrentLimit = 55;
    public static final int talonTimeOut = 10;
    public static final int chassisDriveMaxCurrentTimeout = 500;
    public static final int rightDrive1CanID = 1;
    public static final int rightDrive2CanID = 3;//5
    public static final int leftDrive1CanID = 2;
	public static final int leftDrive2CanID = 4;
    public static boolean isDriveInverted = false;
    public final double joyDriveDeadband = 0.05;
    public final double chassisRightSideScalar = 1;
	public final double chassisLeftSideScalar = 0.92;
	public final boolean chassisSquareJoyInput = true;
	public final double chassisBeltReduction = 1.0/1.0;
	public static AHRS navx;

	public static CANSparkMax sparkOne;
	public static CANSparkMax sparkTwo;
	public static int sparkTwoCanID = 7;
	public static int sparkOneCanID = 6;
	
	
    

    public Chassis() {
        
		PDP = new PowerDistributionPanel(0);
		navx = new AHRS(SPI.Port.kMXP);
		
        leftDrive1 = new TalonSRX(leftDrive1CanID);
		leftDrive1.setNeutralMode(NeutralMode.Brake);
		leftDrive1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Chassis.talonTimeOut);
		leftDrive1.configPeakCurrentLimit(Chassis.chassisDriveMaxCurrentLimit, Chassis.talonTimeOut);
		leftDrive1.configPeakCurrentDuration(Chassis.chassisDriveMaxCurrentTimeout, Chassis.talonTimeOut);
        
        
        rightDrive1 = new TalonSRX(rightDrive1CanID);
		rightDrive1.setNeutralMode(NeutralMode.Brake);
		rightDrive1.setInverted(true);
		rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Chassis.talonTimeOut);
		rightDrive1.configPeakCurrentLimit(Chassis.chassisDriveMaxCurrentLimit, Chassis.talonTimeOut);
		rightDrive1.configPeakCurrentDuration(Chassis.chassisDriveMaxCurrentTimeout, Chassis.talonTimeOut);
        
        
        
		leftDrive2 = new TalonSRX(leftDrive2CanID);
        leftDrive2.setNeutralMode(NeutralMode.Brake);
		leftDrive2.configPeakCurrentLimit(Chassis.chassisDriveMaxCurrentLimit, Chassis.talonTimeOut);
		leftDrive2.configPeakCurrentDuration(Chassis.chassisDriveMaxCurrentTimeout, Chassis.talonTimeOut);
        
        
        rightDrive2 = new TalonSRX(rightDrive2CanID);
		rightDrive2.setNeutralMode(NeutralMode.Brake);
		rightDrive2.setInverted(true);
		rightDrive2.configPeakCurrentLimit(Chassis.chassisDriveMaxCurrentLimit, Chassis.talonTimeOut);
		rightDrive2.configPeakCurrentDuration(Chassis.chassisDriveMaxCurrentTimeout, Chassis.talonTimeOut);
		
		sparkOne = new CANSparkMax(sparkOneCanID, MotorType.kBrushless);
		sparkOne.setIdleMode(IdleMode.kBrake);

    }
		
	public void resetLeftEncoder(){
		leftDrive1.setSelectedSensorPosition(0,0,0);
	}
	public void resetRightEncoder(){
		rightDrive1.setSelectedSensorPosition(0,0,0);
	}
	public void resetBothEncoders(){
		resetLeftEncoder();
		resetRightEncoder();
		
	}
	public void resetNavX(){
		navx.reset();
		navx.zeroYaw();
	}
	public double getEncoderAvgDistInch() {
		double retVal = 0;
		double leftDistance = getLeftEncoderDist();
		double rightDistance = getRightEncoderDist();
		if (Math.abs(leftDistance) < 0.5) {
			retVal = rightDistance;
		} else if (Math.abs(rightDistance) < 0.5) {
			retVal = leftDistance;
		} else {
			retVal = (leftDistance + rightDistance) / 2;
		}
		return retVal;
	}
	public double getNavxAngleRaw(){
		return navx.getAngle();
	}

	public double getNormaliziedNavxAngle(){
		return gyroNormalize(getNavxAngleRaw());
	}

	public void runMotors(double throttle){
		setLeftMotors(throttle);
		setRightMotors(throttle);
	}
	public int getRightEncoderDist(){
		return (int)(rightDrive1.getSelectedSensorPosition(0) * Settings.chassisEncoderDistancePerPulse);
	}

	public int getLeftEncoderDist(){
		return (int)(rightDrive1.getSelectedSensorPosition(0)* Settings.chassisEncoderDistancePerPulse);
	}
	public void runLeftMotor(double throttle){
		setLeftMotors(throttle);
	}
	public void runRightMotor(double throttle){
		setRightMotors(throttle);
	}
	public void runLeftMotorRev(double throttle){
		setLeftMotors(-throttle);
	}
	public void runRightMotorRev(double throttle){
		setRightMotors(-throttle);
	}
	public void runLeftNeo(double throttle){
		sparkOne.set(throttle);
	}
	public void runRightNeo(double throttle){
		sparkTwo.set(throttle);
	}
	


	

    public void DriveStick(Joystick stick) {
        double joyX = stick.getX();
        double joyY = stick.getY();

		if (Math.abs(joyX) < joyDriveDeadband) {
			joyX = 0;
		} else if (Math.abs(joyY) < joyDriveDeadband) {
			joyY = 0;
		}

		double rightMotorThrottle;
		double leftMotorThrottle;
		if (Robot.isDriveInverted) {
			rightMotorThrottle = (joyX + -joyY) * chassisRightSideScalar ;
			leftMotorThrottle = (joyX - -joyY) *  chassisLeftSideScalar ;
			
			if (chassisSquareJoyInput) {
				double rightSignum = Math.signum(rightMotorThrottle);
				double leftSignum = Math.signum(leftMotorThrottle);
				rightMotorThrottle = Math.pow(rightMotorThrottle, 2) * rightSignum ;
				leftMotorThrottle = Math.pow(leftMotorThrottle, 2) * leftSignum  ;
				
			}
            
            setLeftMotors(leftMotorThrottle);
			setRightMotors(-rightMotorThrottle);
			runLeftNeo(leftMotorThrottle);
			runRightNeo(-rightMotorThrottle);
			
			

		} else {
			rightMotorThrottle = (joyX - -joyY) * chassisRightSideScalar ;
			leftMotorThrottle = (joyX + -joyY) * chassisLeftSideScalar ;
			if (chassisSquareJoyInput) {
				double rightSignum = Math.signum(rightMotorThrottle);
				double leftSignum = Math.signum(leftMotorThrottle);
				rightMotorThrottle = Math.pow(rightMotorThrottle, 2) * rightSignum ;
				leftMotorThrottle = Math.pow(leftMotorThrottle, 2) * leftSignum;
			}
			
			setLeftMotors(leftMotorThrottle);
			setRightMotors(-rightMotorThrottle);
			runLeftNeo(leftMotorThrottle);
			runRightNeo(-rightMotorThrottle);
			
		}
	}

	public void DriveMan(double leftThrottle, double rightThrottle) {
		if(Robot.isDriveInverted){
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
		//if (!Robot.isDriveInverted) {
		//	rightMotorThrottle = (rotate + move) * chassisRightSideScalar;
		//	leftMotorThrottle = (rotate - move) * chassisLeftSideScalar;
		//	setLeftMotors(leftMotorThrottle);	
		//	setRightMotors(rightMotorThrottle);
			rightMotorThrottle = (-rotate - move) * chassisRightSideScalar;
			leftMotorThrottle = (-rotate + move) * chassisLeftSideScalar;
			setLeftMotors(leftMotorThrottle);
			setRightMotors(rightMotorThrottle);
		

	}
	public double gyroNormalize(double heading){
		double degrees = heading % 360;

		if (degrees > 180){
			degrees = degrees -360;

		}
		if (degrees < -179){
			degrees = degrees +360;

		}
		return degrees;
	}
    

    @Override
    public void initDefaultCommand() {
       setDefaultCommand(new TeleopDrive());
    }

    @Override
    public void periodic() {
       

    }

}

