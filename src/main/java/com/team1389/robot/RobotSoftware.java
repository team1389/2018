package com.team1389.robot;

import com.team1389.configuration.PIDConstants;
import com.team1389.control.SynchronousPIDController;
import com.team1389.hardware.controls.ControlBoard;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.outputs.software.RangeOut;
import com.team1389.hardware.value_types.Percent;
import com.team1389.hardware.value_types.Position;
import com.team1389.operation.TeleopMain;
import com.team1389.systems.Arm;

public class RobotSoftware extends RobotHardware
{
	private static RobotSoftware INSTANCE = new RobotSoftware();
	SynchronousPIDController<Percent, Position> pid;
	PIDConstants constants;
	ControlBoard control;
	RangeIn<Position> pos;
	RangeOut<Percent> motor;
	Arm arm;
	TeleopMain main;
	

	public static RobotSoftware getInstance()
	{
		return INSTANCE;
	}

	public RobotSoftware()
	{
		constants = new PIDConstants(0.0001, 0, 0);
		pid = new SynchronousPIDController<Percent, Position>(constants, armCan.getSensorPositionStream(), armCan.getVoltageController());
	}

}
