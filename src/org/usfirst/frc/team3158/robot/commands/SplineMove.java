package org.usfirst.frc.team3158.robot.commands;

import org.usfirst.frc.team3158.robot.Robot;
import org.usfirst.frc.team3158.robot.RobotMap;
import org.usfirst.frc.team3158.robot.splines.CubicSpline;
import org.usfirst.frc.team3158.robot.splines.Derivate;
import org.usfirst.frc.team3158.robot.splines.SplineGenerator;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SplineMove extends Command {
// ALL UNITS ARE IN CENTIMETERS
	
		float targetPower;
		float wheelRadius = RobotMap.wheelRadius;
		float robotRadius = RobotMap.robotRadius;
		//The power to set both talons
	    float leftPower, rightPower;
	    //The power aplied to both talons last frame
	    float lastLeftPower = 1f , lastRightPower = 1f;
	    //The distance advanced since last frame
	    float deltaDistanceLeft = .001f, deltaDistanceRight = .001f;
	    //The diference of distance that wheels will travel in this frame, positive, left travels more, negative right does
	    float deltaDistance;
	    //The angle and encoder count in last frame
	    float lastAngle = 0, lastLeftCount = 0, lastRightCount = 0;
	    //The distance that the robot need to do a 360° turn
	    float robotCircumference;
	    //The degrees that the robot has to turn in this frame
	    float deltaAngle;
	    //The angle that the robot has to go
	    float nextAngle;
	    //The distance that covered last frame at power 1
	    float leftSpeedPerFrameAtPower1 = .005f, rightSpeedPerFrameAtPower1 = .005f;
	    //The x position of the robot
	    float xPos;
	    //If robot has completed the trajectory
	    boolean hasFinished;
	
	  //Puntos por los que pasará el robot, x0 y z0 son la posicion inicial
	    float x1, x2;
	    float z1, z2;
	    float x0 = 0, z0 = 0;
	  //Trayectoria del robot en ambos intervalos 
	   CubicSpline spline0, spline1;
	  //Derivada de ambos splines
	   Derivate derivada0, derivada1;
	    
    public SplineMove(float g_x1, float g_x2, float g_z1, float g_z2, float g_targetPower) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.chassis);
    	
    	x1 = g_x1;
    	x2 = g_x2;
    	z1 = g_z1;
    	z2 = g_z2;
    	targetPower = g_targetPower;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	robotCircumference = (float) (2 * robotRadius * Math.PI);
    	lastAngle = (float) Robot.navx.getYaw();
    	spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	lastAngle = (float) Robot.navx.getYaw();
    	xPos = Robot.xPos;
    	
    	 if(xPos >= x2)
         {
             leftPower = 0;
             rightPower = 0;
             hasFinished = true;
         }
    	 
    	  if (deltaDistanceLeft != 0 && lastLeftPower != 0)
          {
              leftSpeedPerFrameAtPower1 = deltaDistanceLeft / lastLeftPower;
          }
          if (deltaDistanceRight != 0 && lastRightCount != 0)
          {
              rightSpeedPerFrameAtPower1 = deltaDistanceRight / lastRightPower;
          }
    	
          if(xPos <= x1)
          {
              nextAngle = AnguloAPartirDeDerivada(derivada0, xPos);
          }else if( xPos <= x2 && xPos >= x1)
          {
              nextAngle = AnguloAPartirDeDerivada(derivada1, xPos);
          }
          deltaAngle = (float) (nextAngle - Robot.navx.getYaw());
          
          if (deltaAngle < -180 || deltaAngle > 180)
          {
              deltaAngle = (float) (nextAngle - Robot.navx.getYaw() + 360);
          }

          deltaDistance =Math.abs( (deltaAngle * robotCircumference) / 360);
          
          //if turning right
          if (deltaAngle < 0 && !hasFinished)
          {
              leftPower = targetPower;
              rightPower = ((leftSpeedPerFrameAtPower1 * targetPower) - deltaDistance) / rightSpeedPerFrameAtPower1;
          }else if(deltaAngle > 0 && !hasFinished)
          {
              //if turning left 
              rightPower = targetPower;
              leftPower = ((rightSpeedPerFrameAtPower1 * targetPower) - deltaDistance) / leftSpeedPerFrameAtPower1;
          }else if(deltaAngle == 0 && !hasFinished)
          {
              rightPower = targetPower;
              leftPower = targetPower;
          }
          
          if(hasFinished) {
        	  rightPower = 0;
        	  leftPower = 0;
          }
          
    	
          Robot.chassis.SetLeft(leftPower);
          Robot.chassis.SetRight(rightPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return hasFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    
    
    
    float AnguloAPartirDeDerivada(Derivate derivate, float xPos)
    {
        float pendienteActual = (derivate.squareX * xPos * xPos) + (derivate.x * xPos) + derivate.indep;
        return (float) (Math.atan(pendienteActual) * 360 / (2*Math.PI));
    }
}
