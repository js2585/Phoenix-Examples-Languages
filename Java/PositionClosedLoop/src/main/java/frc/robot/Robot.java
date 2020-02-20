/**
 * Phoenix Software License Agreement
 * <p>
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * <p>
 * Cross The Road Electronics (CTRE) licenses to you the right to
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * <p>
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL,
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 * <p>
 * Description:
 * The PositionClosedLoop example demonstrates the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * <p>
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon is driving
 * forward (Green LED on Talon/Victor) when the position sensor is moving in the postive
 * direction. If this is not the case, flip the boolean input in setSensorPhase().
 * <p>
 * Controls:
 * Button 1: When pressed, start and run Position Closed Loop on Talon/Victor
 * Button 2: When held, start and run Percent Output
 * Left Joytick Y-Axis:
 * + Position Closed Loop: Servo Talon forward and reverse [-10, 10] rotations
 * + Percent Ouput: Throttle Talon forward and reverse
 * <p>
 * Gains for Position Closed Loop may need to be adjusted in Constants.java
 * <p>
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */

/**
 * Description:
 * The PositionClosedLoop example demonstrates the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 *
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon/Victor) when the position sensor is moving in the postive 
 * direction. If this is not the case, flip the boolean input in setSensorPhase().
 *
 * Controls:
 * Button 1: When pressed, start and run Position Closed Loop on Talon/Victor
 * Button 2: When held, start and run Percent Output
 * Left Joytick Y-Axis:
 * 	+ Position Closed Loop: Servo Talon forward and reverse [-10, 10] rotations
 * 	+ Percent Ouput: Throttle Talon forward and reverse
 *
 * Gains for Position Closed Loop may need to be adjusted in Constants.java
 *
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
package frc.robot;

import edu.wpi.first.wpilibj.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	public static final int SETPOINT_PERIOD = 15;
	public static final int MAX_DEGREE_HOOD = 18;
	/** Hardware */
	TalonSRX hoodMotor = new TalonSRX(9);
//	TalonSRX hoodMotor = new TalonSRX(9);

	Joystick _joy = new Joystick(0);
	Joystick _joy2 = new Joystick(1);
	DigitalInput leftSwitch = new DigitalInput(0);
	DigitalInput rightSwitch = new DigitalInput(1);
	/** Used to create string thoughout loop */
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	//	public static final double ZMOTOR_PULSES_PER_DEGREE = (5772f) / 45; // (pulses per degree)
	public static final double HOOD_PULSES_PER_DEGREE = (187 + 857) / 24.2; // (pulses per degree)


	/** Track button state for single press event */
	boolean _lastButton1 = false;

	/** Save the target position to servo to */
	double targetPositionRotations;
	double joystickP;
	Timer setpointTimer;
	private double currentSetpointValue;

	public void robotInit() {
		/* Factory Default all hardware to prevent unexpected behaviour */
		hoodMotor.configFactoryDefault();
		setpointTimer = new Timer();
		setpointTimer.start();
		currentSetpointValue = 10;
		/* Config the sensor used for Primary PID and sensor direction */
		hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
			Constants.kPIDLoopIdx,
			Constants.kTimeoutMs);

		/* Ensure sensor is positive when output is positive */
		hoodMotor.setSensorPhase(true);
		hoodMotor.setSelectedSensorPosition(0, 0, 0);
		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */
		hoodMotor.setInverted(Constants.kMotorInvert);

		/* Config the peak and nominal outputs, 12V means full */
		hoodMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		hoodMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		hoodMotor.configPeakOutputForward(Constants.kGains_Hood.kPeakOutput, Constants.kTimeoutMs);
		hoodMotor.configPeakOutputReverse(-Constants.kGains_Hood.kPeakOutput, Constants.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		hoodMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		hoodMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Hood.kF, Constants.kTimeoutMs);
//		hoodMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Hood.kP, Constants.kTimeoutMs);
		hoodMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Hood.kI, Constants.kTimeoutMs);
		hoodMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Hood.kD, Constants.kTimeoutMs);
		joystickP = ((_joy2.getZ() + 1) / 2);
		hoodMotor.config_kP(Constants.kPIDLoopIdx, joystickP, Constants.kTimeoutMs);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
//		int absolutePosition = hoodMotor.getSensorCollection().getPulseWidthPosition();
//
//		/* Mask out overflows, keep bottom 12 bits */
//		absolutePosition &= 0xFFF;
//		if (Constants.kSensorPhase) {
//			absolutePosition *= -1;
//		}
//		if (Constants.kMotorInvert) {
//			absolutePosition *= -1;
//		}
//
//		/* Set the quadrature (relative) sensor to match absolute */
//		hoodMotor.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	void commonLoop() {
		/* Gamepad processing */
		double leftZ = _joy.getZ();
		double leftZDegrees = ((leftZ + 1) / 2) * MAX_DEGREE_HOOD;
		boolean button5 = _joy.getRawButton(5);    // X-Button
		boolean button4 = _joy.getRawButton(4);    // A-Button

		/* Get Talon/Victor's current output percentage */


		/**
		 * When button 1 is pressed, perform Position Closed Loop to selected position,
		 * indicated by Joystick position x10, [-10, 10] rotations
		 */
		if (setpointTimer.get() > SETPOINT_PERIOD) {
			currentSetpointValue *= -1;
			setpointTimer.reset();
		}

//		if (!leftSwitch.get()) {
//			leftZ = (leftZ + 1) / 2;
//		}
//		if (!rightSwitch.get()) {
//			leftZ = -(leftZ + 1) / 2;
//		}
		if (button5) {
			/* Position Closed Loop */

			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = leftZDegrees * HOOD_PULSES_PER_DEGREE;

			hoodMotor.set(ControlMode.Position, targetPositionRotations);
//			hoodMotor.set(ControlMode.Position, currentSetpointValue * HOOD_PULSES_PER_DEGREE);
		}

		/* When button 2 is held, just straight drive */
		else if (button4) {
			/* Percent Output */

			hoodMotor.set(ControlMode.PercentOutput, leftZ);
		} else {
			hoodMotor.set(ControlMode.PercentOutput, 0);
		}

		SmartDashboard.putNumber("Joystick Z Axis", leftZ);
		SmartDashboard.putNumber("Joystick Setpoint (Degrees)", leftZDegrees);
		SmartDashboard.putNumber("Current Position (Degrees)", hoodMotor.getSelectedSensorPosition() / HOOD_PULSES_PER_DEGREE);
		SmartDashboard.putNumber("Closed Loop Error", hoodMotor.getClosedLoopError() / HOOD_PULSES_PER_DEGREE);
		SmartDashboard.putNumber("Joystick P", joystickP);
		SmartDashboard.putNumber("Encoder Distance", hoodMotor.getSelectedSensorPosition());
		SmartDashboard.putBoolean("LeftLimitSwitch", !leftSwitch.get());
		SmartDashboard.putBoolean("RightLimitSwitch", !rightSwitch.get());
		SmartDashboard.putNumber("Setpoint", currentSetpointValue);


		/* If Talon is in position closed-loop, print some more info */
//		if (hoodMotor.getControlMode() == ControlMode.Position) {
//			/* ppend more signals to print when in speed mode. */
//			_sb.append("\terr:");
//			_sb.append(hoodMotor.getClosedLoopError(0));
//			_sb.append("u");	// Native Units
//
//			_sb.append("\ttrg:");
//			_sb.append(targetPositionRotations);
//			_sb.append("u");	/// Native Units
//		}
//
//		/**
//		 * Print every ten loops, printing too much too fast is generally bad
//		 * for performance.
//		 */
//		if (++_loops >= 10) {
//			_loops = 0;
//			System.out.println(_sb.toString());
//		}
//
//		/* Reset built string for next loop */
//		_sb.setLength(0);
//
//		/* Save button state for on press detect */
//		_lastButton1 = button5;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		commonLoop();
	}

	public void disabledInit() {

	}

	public void disabledPeriodic() {

	}
}
