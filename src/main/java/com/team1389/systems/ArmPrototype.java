package com.team1389.systems;

import com.team1389.configuration.PIDConstants;
import com.team1389.control.PIDController;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.outputs.software.RangeOut;
import com.team1389.hardware.value_types.Position;
import com.team1389.system.Subsystem;
import com.team1389.util.list.AddList;
import com.team1389.watch.Watchable;

public class ArmPrototype extends Subsystem
{

	RangeIn<Position> armPos;
	RangeOut armVoltage;
	PIDController pid;

	public ArmPrototype(RangeIn<Position> armPos, RangeOut armVoltage)
	{
		this.armPos = armPos;
		this.armVoltage = armVoltage;
		pid = new PIDController<>(new PIDConstants(.001, 0, 0), armPos, armVoltage);
	}

	@Override
	public AddList<Watchable> getSubWatchables(AddList<Watchable> stem)
	{
		return stem.put(armPos.getWatchable("Arm Pos"));
	}

	@Override
	public String getName()
	{
		return "Arm Prototype";
	}

	@Override
	public void init()
	{
		pid.enable();

	}

	@Override
	public void update()
	{
		pid.setSetpoint(100);
	}

}
