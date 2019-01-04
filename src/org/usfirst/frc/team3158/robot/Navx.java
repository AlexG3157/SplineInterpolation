package org.usfirst.frc.team3158.robot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
public class Navx {
private double yaw;
private double rate;
private double pitch;
private double acceleration;

private double lastAngle = 0;
private double totalDegAngle = 0;

private AHRS ahrs ;
private double last_world_linear_accel_x;
private double last_world_linear_accel_y;
private double curr_world_linear_accel_x;
private double curr_world_linear_accel_y;
private double currentJerkX;
private double currentJerkY;
private double kCollisionThreshold_DeltaG = 1.5f;

public Navx(){
try 
{
ahrs = new AHRS(SPI.Port.kMXP); 
DriverStation.reportWarning("Navx running", true);
}
catch (RuntimeException ex ) 
{
DriverStation.reportError("Error stantiating navX-MXP: " + ex.getMessage(), true); 
}
last_world_linear_accel_x = 0.0f;
last_world_linear_accel_y = 0.0f;
}
public void run(){
yaw=ahrs.getYaw();
rate=ahrs.getRawMagY();
pitch=ahrs.getPitch();
acceleration=ahrs.getRawAccelY();
}
public double getYaw(){
return yaw;
}
public double getYaw360Deg() {
lastAngle = yaw;
if(lastAngle < 0) {
lastAngle=-lastAngle;
}
double deltaAngle = lastAngle-yaw;
totalDegAngle += deltaAngle;
return 0;
}
public double getRate(){
return rate;
}
public double getAcceleration(){
return acceleration;
}
public void reset(){
ahrs.reset();
}
public double getPitch(){
return pitch;
}
public float getWorldLinearAccelY(){
return ahrs.getWorldLinearAccelY();
}
private float getWorldLinearAccelX(){
return ahrs.getWorldLinearAccelX();
}
public boolean getCollision(){
boolean collisionDetected = false; 
curr_world_linear_accel_x = getWorldLinearAccelX();
currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
last_world_linear_accel_x = curr_world_linear_accel_x;
curr_world_linear_accel_y = getWorldLinearAccelY();
currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
last_world_linear_accel_y = curr_world_linear_accel_y;
if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
collisionDetected = true;
System.out.println("Collision detected");
}
return collisionDetected;
}
public double getCollisionY(){
curr_world_linear_accel_y = getWorldLinearAccelY();
currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
last_world_linear_accel_y = curr_world_linear_accel_y;
return currentJerkY;
}
public double getCollisionX(){
curr_world_linear_accel_x = getWorldLinearAccelX();
currentJerkY = curr_world_linear_accel_x - last_world_linear_accel_x;
last_world_linear_accel_x = curr_world_linear_accel_x;
return currentJerkX;
}
}