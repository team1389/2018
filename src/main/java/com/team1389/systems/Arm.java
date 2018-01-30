package com.team1389.systems;

import com.team1389.control.SynchronousPIDController;
import com.team1389.hardware.controls.ControlBoard;
import com.team1389.hardware.inputs.software.DigitalIn;
import com.team1389.hardware.inputs.software.PercentIn;
import com.team1389.hardware.outputs.software.PercentOut;
import com.team1389.hardware.value_types.Percent;
import com.team1389.hardware.value_types.Position;
import com.team1389.robot.RobotSoftware;
import com.team1389.system.Subsystem;
import com.team1389.util.list.AddList;
import com.team1389.watch.Watchable;
import com.team1389.watch.Watcher;
import com.team1389.watch.info.BooleanInfo;

public class Arm extends Subsystem {
	PercentOut percent;
	SynchronousPIDController<Percent, Position> pid;
	ControlBoard controls;
	DigitalIn thumb;
	RobotSoftware robot;
	PercentIn rightStickY;
	Watcher watch;
	
	final double POS_A = 50;
	final double POS_B = 75;
	final double POS_C = 100;
	
	public Arm(SynchronousPIDController<Percent, Position> pid, PercentIn rightStickY, PercentOut VoltageControl)
	{
		this.pid = pid;
		this.rightStickY = rightStickY;
		percent = VoltageControl;
	}
	public void position()
	{
		percent.set(controls.rightStickYAxis().get());
	}
	
	/*
	public Arm(SynchronousPIDController<Percent, Position> pid, DigitalIn a, DigitalIn b, DigitalIn c)
	{
		this.pid = pid; 
		this.a = a;
		this.b = b;
		this.c = c;
	}
	
	public void positionA()
	{
		pid.setSetpoint(POS_A);	
		pid.update();
	}
	
	public void positionB()
	{
		pid.setSetpoint(POS_B);
		pid.update();
	}
	public void positionC()
	{
		pid.setSetpoint(POS_C);
		pid.update();
	}
	public void reset()
	{
		pid.setSetpoint(0);
		pid.update();
	}
	*/
	@Override
	public AddList<Watchable> getSubWatchables(AddList<Watchable> arg0) {
		// TODO Auto-generated method stub
		
		return arg0.put(new BooleanInfo("object in arm!", ()-> robot.switchHardware.getSwitchInput().get()));	
		}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void init() {
		//reset();
	}
	
	public void update()
	{
		position();
	}
	/*
	@Override
	public void update()
	{
		if (a.get())
		{
			positionA();
		}
		if (b.get())
		{
			positionB();
		}
		if (c.get())
		{
			positionC();
		}
	}
	*/
}
