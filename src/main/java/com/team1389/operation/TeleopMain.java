package com.team1389.operation;

import com.team1389.hardware.controls.ControlBoard;
import com.team1389.robot.RobotSoftware;
import com.team1389.system.Subsystem;
import com.team1389.system.SystemManager;
import com.team1389.system.drive.CurvatureDriveStraightSystem;
import com.team1389.watch.Watcher;

public class TeleopMain
{
	SystemManager manager;
	ControlBoard controls;
	RobotSoftware robot;
	Watcher watcher;

	public TeleopMain(RobotSoftware robot)
	{
		this.robot = robot;
	}

	public void init()
	{
		watcher = new Watcher();
		controls = ControlBoard.getInstance();
		Subsystem driveSystem = setUpDriveSystem();
		manager = new SystemManager(driveSystem);
		manager.init();
		watcher.watch(driveSystem);

	}

	public Subsystem setUpDriveSystem()
	{
		return new CurvatureDriveStraightSystem(robot.drive.getAsTank(), controls.xDriveY(), controls.xDriveX(),
				controls.rightBumper(), robot.angle, .05, controls.leftBumper());
	}

	public void periodic()
	{
		manager.update();
	}
}
