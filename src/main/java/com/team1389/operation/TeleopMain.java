package com.team1389.operation;

import com.team1389.control.SynchronousPIDController;
import com.team1389.hardware.controls.ControlBoard;
import com.team1389.hardware.value_types.Percent;
import com.team1389.hardware.value_types.Position;
import com.team1389.robot.RobotSoftware;
import com.team1389.system.Subsystem;
import com.team1389.system.SystemManager;
import com.team1389.systems.Arm;

public class TeleopMain
{
	SynchronousPIDController<Percent, Position> pid;
	SystemManager manager;
	ControlBoard controls;
	RobotSoftware robot;
	Arm arm;

	public TeleopMain(RobotSoftware robot)
	{
		this.robot = robot;
	}

	public void init()
	{
		controls = ControlBoard.getInstance();
		Subsystem Arm = setArm();
		manager = new SystemManager(Arm);
		manager.init();

	}
	public void periodic()
	{
		manager.update();
	}
	public Subsystem setArm()
	{
		Arm arm = new Arm(pid, controls.rightStickYAxis(), robot.armCan.getVoltageController());
		
		return arm;
	}
}
