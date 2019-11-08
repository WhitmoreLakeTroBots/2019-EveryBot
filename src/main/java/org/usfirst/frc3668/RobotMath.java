/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3668;

/**
 * Add your docs here.
 */
public class RobotMath {
    public static double getTime() {
        return (System.nanoTime() / Math.pow(10, 9));

    }
    public static double headingDelta(double currentHeading, double targetHeading) {
		double headingDelta = 0;
		double invertedHeadingDelta = 0;
		
		 //Positive value
        if (currentHeading >= 0 && targetHeading >= 0) {
            headingDelta = targetHeading - currentHeading;
        }
        // one of each
        else if (currentHeading >= 0 && targetHeading <= 0) {
            //headingDelta =  (targetHeading + currentHeading);
        	headingDelta = Math.abs(targetHeading) + Math.abs(currentHeading);
        	invertedHeadingDelta = Math.abs(360 + targetHeading) - Math.abs(currentHeading);
        	headingDelta = Math.min(Math.abs(headingDelta), Math.abs(invertedHeadingDelta));
        	if(invertedHeadingDelta != headingDelta) {
        		headingDelta = headingDelta * -1;
        	}
        }
        //one of each again
        else if (currentHeading <= 0 && targetHeading >= 0) {
            //headingDelta = -1 * (targetHeading + currentHeading);
        	headingDelta = Math.abs(targetHeading) + Math.abs(currentHeading);
        	invertedHeadingDelta = Math.abs(360 - targetHeading) - Math.abs(currentHeading);
        	headingDelta = Math.min(Math.abs(headingDelta), Math.abs(invertedHeadingDelta));
        	if(invertedHeadingDelta == headingDelta) {
        		headingDelta = headingDelta * -1;
        	}
        }
        // both negative
        else if (currentHeading <= 0 && targetHeading <= 0) {
            headingDelta = targetHeading - currentHeading;
        }
        return headingDelta;
	}
    public static double calcTurnRate( double currentHeading, double targetHeading, double proportion) {

        double headingDelta = headingDelta(currentHeading, targetHeading);
        
       
		double commandedTurnRate = headingDelta * proportion;
		return commandedTurnRate; //IS ALWAYS POSITIVE!
	}

}
