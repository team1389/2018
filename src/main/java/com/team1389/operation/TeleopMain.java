package com.team1389.operation;

import com.team1389.hardware.controls.ControlBoard;
import com.team1389.robot.RobotSoftware;
import com.team1389.system.Subsystem;
import com.team1389.system.SystemManager;
import com.team1389.system.drive.CurvatureDriveSystem;
import com.team1389.watch.Watcher;

public class TeleopMain
{
	SystemManager manager;
	ControlBoard controls;
	RobotSoftware robot;
	Watcher watcher;
	boolean vision;

	public TeleopMain(RobotSoftware robot)
	{
		this.robot = robot;
	}

	public void init()
	{
		controls = ControlBoard.getInstance();
		Subsystem driveSystem = setUpDriveSystem();
		manager = new SystemManager(driveSystem);
		manager.init();
		watcher.watch(driveSystem);
		vision = false;

	}

	public Subsystem setUpDriveSystem()
	{
		return new CurvatureDriveSystem(robot.drive, controls.xDriveY(), controls.xDriveX(), controls.rightBumper());

	}

	public void periodic()
	{
		if (!vision)
		{
			manager.update();
		}
	}
}
