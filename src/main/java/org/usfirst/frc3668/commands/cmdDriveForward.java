/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3668.commands;

import org.usfirst.frc3668.Robot;
import org.usfirst.frc3668.RobotMath;
import org.usfirst.frc3668.Settings;
import org.usfirst.frc3668.motionProfile.MotionProfiler;
import org.usfirst.frc3668.PID;

import edu.wpi.first.wpilibj.command.Command;

public class cmdDriveForward extends Command {
  public final double ticsToIn = 100;
  public final double throttle = 0.32;
  private double _distance;
  private MotionProfiler mp;
  private double _absDistance;
  private double _cruiseSpeed;
  private double _abortTime;
  private double _endTime;
  private double _startTime;
  private boolean _isFinished = false;
  private double _distanceSignum;
  private double _requestedHeading = 0;
  private PID pid = new PID(Settings.driveKp, Settings.driveKi, Settings.driveKd);
  public cmdDriveForward(double requestedHeading, double cruiseSpeed, double distance) {    
    System.err.println("AutoDriveProfileGyro");
		requires(Robot.Chassis);
		_distance = distance;
		_absDistance = Math.abs(distance);
		_distanceSignum = Math.signum(distance);
		_cruiseSpeed = cruiseSpeed;
		_requestedHeading = requestedHeading;
    
  }
  public cmdDriveForward(double cruiseSpeed, double distance) {
		requires(Robot.Chassis);
		_distance = distance;
		_absDistance = Math.abs(distance);
		_distanceSignum = Math.signum(distance);
    _cruiseSpeed = cruiseSpeed;
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.err.println("Initializing");
		mp = new MotionProfiler(24, 0.0, 145,
				55);
		Robot.Chassis.resetBothEncoders();
		_abortTime = 24 / 145;
		_endTime = mp._stopTime * 1.3;
	//	System.err.println(String.format(
		//		"Projected Accelration Time: %1$.3f \tProjected Cruise Time: %2$.3f \t Projected Deccelration Time: %3$.3f \t Projected Length of Drive: %4$.3f \t Given Distance: %5$.3f \t Abort: %6$.3f",
			//	mp._accelTime, mp._cruiseTime, mp._deccelTime, mp._stopTime, 12/*distance*/, _abortTime));
		_startTime = RobotMath.getTime();
		_isFinished = false;
  }
  

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double encoderVal = Robot.Chassis.getEncoderAvgDistInch();
		double deltaTime = RobotMath.getTime() - _startTime;
		double profileDist = mp.getTotalDistanceTraveled(deltaTime);
		double currentHeading = Robot.Chassis.getNormaliziedNavxAngle();
		double turnValue = calcTurnRate(currentHeading);
		double profileVelocity = mp.getProfileCurrVelocity(deltaTime);
		double throttlePos = (profileVelocity / Settings.chassisMaxInchesPerSecond);
		double pidVal = pid.calcPID(profileDist, encoderVal);
    double finalThrottle = throttlePos + pidVal;
    

		String msg = String.format(
				"CurrVel: %1$.3f \t throttle: %2$.3f \t Time: %3$.3f \t ProfileX: %4$.3f \t Encoder: %5$.3f \t PID Value: %10$.3f \t P: %14$.3f \t I: %13$.3f \t D: %11$.3f \t Final Throttle: %12$.3f \t Gyro: %15$.3f",
				profileVelocity, throttlePos, deltaTime, mp.getTotalDistanceTraveled(deltaTime), encoderVal,
				Robot.Chassis.getLeftEncoderDist(), Robot.Chassis.getRightEncoderDist(), currentHeading,
				turnValue, pidVal, pid.getDError(), finalThrottle, pid.getIError(), pid.getPError(), currentHeading);
		// FULL LOG MESSAGE: CurrVel: %1$.3f \t throttle: %2$.3f \t deltaTime: %3$.3f \t
		// Disantce Travelled: %4$.3f \t AvgEncoder: %5$.3f \t Left Encoder: %6$.3f \t
		// Right Encoder: %7$.3f \t Gyro Raw Heading: %8$.3f \t Turn Value: %9$.3f \t
		// PID Value: %10$.3f \t P Value: %11$.3f \t Final Throttle: %12$.3f
		System.err.println(msg);
		// log.makeEntry(msg);
		// SmartDashboard.putNumber("Drive Left Encoder:",
		// Robot.Chassis.getLeftEncoderDist());
		// SmartDashboard.putNumber("Drive Right Encoder: ",
		// Robot.Chassis.getRightEncoderDist());

		Robot.Chassis.Drive((finalThrottle * _distanceSignum), turnValue);

		// if (deltaTime > _abortTime && Robot.Chassis.getEncoderAvgDistInch() == 0)
		// {
		// System.out.println("Pasted Abort Time, Dead Encoders");
		// _isFinished = true;
		// Robot.Chassis._isSafeToMove = false;
		// }
		if (encoderVal < _absDistance + Settings.profileMovementThreshold
				&& encoderVal > _absDistance - Settings.profileMovementThreshold) {
			System.err.println("At Distance");
			_isFinished = true;
		}

		if (deltaTime > _endTime) {
			_isFinished = true;
		}
    
  }
  protected double calcTurnRate(double currentHeading) {
		double turnRate = RobotMath.calcTurnRate(currentHeading, _requestedHeading,
				Settings.chassisDriveStraightGyroKp);
		// if(currentHeading > _requestedHeading) {
		// turnRate = turnRate * -1;
		// }
		return turnRate;
	}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.Chassis.Drive(0,0);
		Robot.Chassis.resetBothEncoders();
		System.out.println("AutoDriveProfileGyro is Finished Left Encoder: " + Robot.Chassis.getLeftEncoderDist()
				+ " Right Encoder: " + Robot.Chassis.getRightEncoderDist());
		System.err.println(String.format(
				"Projected Accelration Time: %1$.3f \tProjected Cruise Time: %2$.3f \t Projected Deccelration Time: %3$.3f \t Projected Length of Drive: %4$.3f \t Given Distance: %5$.3f",
				mp._accelTime, mp._cruiseTime, mp._deccelTime, mp._stopTime, _distance));

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
