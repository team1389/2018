package com.team1389.autonomous.commands;

import com.team1389.command_framework.command_base.Command;
import com.team1389.configuration.PIDConstants;
import com.team1389.control.SynchronousPIDController;
import com.team1389.hardware.inputs.software.AngleIn;
import com.team1389.hardware.value_types.Position;
import com.team1389.robot.RobotSoftware;
import com.team1389.util.Timer;
import com.team1389.watch.Watcher;

import jaci.pathfinder.Pathfinder;

public class WheelBase extends Command
{
	RobotSoftware robot;
	AngleIn<Position> angle;
	Watcher watch;
	Timer timer;
	SynchronousPIDController pidL;
	SynchronousPIDController pidR;

	public WheelBase(RobotSoftware robot)
	{
		timer = new Timer();
		this.robot = robot;
		pidL = new SynchronousPIDController<>(new PIDConstants(0.1, 0, 0),
				robot.leftDriveT.getSensorPositionStream().mapToRange(0, 1).mapToRange(0, (.127 * Math.PI)),
				robot.leftDriveT.getVoltageController());
		pidR = new SynchronousPIDController<>(new PIDConstants(0.1, 0, 0),
				robot.rightDriveT.getSensorPositionStream().mapToRange(0, 1).mapToRange(0, (.127 * Math.PI)),
				robot.rightDriveT.getVoltageController());
		this.angle = robot.angle;
		pidL.setSetpoint(-5);
		pidR.setSetpoint(.5);
	}

	@Override
	protected boolean execute()
	{
		pidL.update();
		pidR.update();
		return (pidL.onTarget(.05) && pidR.onTarget(0.05));
	}

	@Override
	protected void done()
	{
		System.out.println("angle " + angle.get());
		System.out.println("angle in radians " + Pathfinder.d2r(angle.get()));
		System.out.println("wheel base in meters " + 1 / Pathfinder.d2r(angle.get()));
	}

}
