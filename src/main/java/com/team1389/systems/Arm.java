package com.team1389.systems;

import com.team1389.configuration.PIDConstants;
import com.team1389.control.SynchronousPIDController;
import com.team1389.hardware.inputs.software.DigitalIn;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.outputs.software.RangeOut;
import com.team1389.hardware.value_types.Position;
import com.team1389.hardware.value_types.Speed;
import com.team1389.system.Subsystem;
import com.team1389.util.list.AddList;
import com.team1389.watch.Watchable;

public class Arm extends Subsystem
{
	RangeIn<Position> position;
	RangeOut intakeVoltage;
	RangeOut armVoltage;
	SynchronousPIDController pid;
	DigitalIn zero;
	DigitalIn ninty;
	DigitalIn oneEighty;
	State currentState;

	public Arm(RangeIn<Position> position, RangeOut intakeVoltage, RangeOut armVoltage, DigitalIn zero, DigitalIn ninty,
			DigitalIn oneEighty)
	{
		super();
		this.position = position;
		this.intakeVoltage = intakeVoltage;
		this.armVoltage = armVoltage;
		this.zero = zero;
		this.ninty = ninty;
		this.oneEighty = oneEighty;
	}

	@Override
	public AddList<Watchable> getSubWatchables(AddList<Watchable> arg0)
	{
		return null;
	}

	@Override
	public String getName()
	{
		return null;
	}

	@Override
	public void init()
	{
		pid = new SynchronousPIDController<>(new PIDConstants(0.1, 0, 0), position, armVoltage);
		position.setRange(0, 360);
		zero();
		setState(State.Down);

	}

	@Override
	public void update()
	{

	}

	public void zero()
	{
		while (!zero.get())
		{
			armVoltage.set(-.25);
		}
		armVoltage.set(0);
		position.offset(-position.get());
	}

	enum State
	{
		IntakingZero, IntakingOneEighty, Lifting, Down
	}

	private void setState(State desired)
	{
		currentState = desired;
	}

}
