package com.team1389.systems;

import com.team1389.hardware.inputs.software.DigitalIn;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.outputs.software.RangeOut;
import com.team1389.hardware.value_types.Percent;
import com.team1389.hardware.value_types.Position;
import com.team1389.hardware.value_types.Speed;
import com.team1389.hardware.value_types.Value;

/**
 * do we need to call scheduler.update()?
 * @author Quunii
 *
 */
public class TeleopElevator extends Elevator {

	//can have max bound to start, decided that low and high switch can just be high switch
	DigitalIn zeroBtn, switchBtn, scaleLowBtn, scaleHighBtn, maxBtn;
	DigitalIn manualBtn;
	RangeIn<Value> ctrlAxis;
	boolean manual;
	
	public TeleopElevator(DigitalIn zero, RangeIn<Position> elevPos, RangeIn<Speed> elevVel, RangeOut<Percent> elevVolt,
			DigitalIn zeroBtn, DigitalIn switchBtn, DigitalIn scaleLowBtn, DigitalIn scaleHighBtn, DigitalIn maxBtn,DigitalIn manualBtn,
			RangeIn<Value> ctrlAxis) {
		super(zero, elevPos, elevVel, elevVolt);
		this.zeroBtn = zeroBtn;
		this.switchBtn = switchBtn;
		this.scaleLowBtn = scaleLowBtn;
		this.scaleHighBtn = scaleHighBtn;
		this.maxBtn = maxBtn;
		this.ctrlAxis = ctrlAxis;
		this.manualBtn = manualBtn;
	}
	

	@Override
	public void update() {
		manual = manual^manualBtn.get();
		if(manual)
		{
			updateManual();
		}
		else
		{
			updateDefault();
		}
		
	}
	private void updateDefault()
	{
		if(zeroBtn.get())
		{
			goToZero();
		}
		else if(switchBtn.get())
		{
			goToSwitchHigh(true);
		}
		else if(scaleLowBtn.get())
		{
			goToScaleLow(true);
		}
		else if(scaleHighBtn.get())
		{
			goToScaleHigh(true);
		}
		else if(maxBtn.get())
		{
			goToMax();
		}
		super.update();
	}
	private void updateManual()
	{
		elevVolt.set(ctrlAxis.get());
	}

	
}
