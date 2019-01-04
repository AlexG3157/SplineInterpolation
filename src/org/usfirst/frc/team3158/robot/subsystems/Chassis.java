package org.usfirst.frc.team3158.robot.subsystems;

import org.usfirst.frc.team3158.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Chassis extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	TalonSRX left1 = new TalonSRX(RobotMap.left1);
	TalonSRX left2 = new TalonSRX(RobotMap.left2);
	TalonSRX right1 = new TalonSRX(RobotMap.right1);
	TalonSRX right2 = new TalonSRX(RobotMap.right2);

	public void SetRight(float value) {
	right1.set(ControlMode.PercentOutput, value);
	right2.set(ControlMode.PercentOutput, value);
	}
	public void SetLeft(float value) {
		left1.set(ControlMode.PercentOutput, value);
		left2.set(ControlMode.PercentOutput, value);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

