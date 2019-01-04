package org.usfirst.frc.team3158.robot.splines;

public class CubicSpline {

	 public float cubicX;
     public float squareX;
     public float x;
     public float indep;

     public CubicSpline(float cubic, float square, float lineal, float term)
     {
         cubicX = cubic;
         squareX = square;
         x = lineal;
         indep = term;
     }
	
}
