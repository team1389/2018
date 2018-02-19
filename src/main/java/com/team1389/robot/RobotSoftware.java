package com.team1389.robot;

import com.team1389.hardware.inputs.software.AngleIn;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.outputs.software.RangeOut;
import com.team1389.hardware.value_types.Percent;
import com.team1389.hardware.value_types.Position;
import com.team1389.system.drive.FourDriveOut;

import edu.wpi.first.wpilibj.Preferences;

public class RobotSoftware extends RobotHardware
{
	private static RobotSoftware INSTANCE = new RobotSoftware();

	public final RangeOut<Percent> right = rightDriveT.getVoltageController();
	public final RangeOut<Percent> left = leftDriveT.getVoltageController();
	public final FourDriveOut<Percent> drive = new FourDriveOut<Percent>(leftDriveT.getVoltageController(),
			rightDriveT.getVoltageController(), leftDriveV.getVoltageController(), rightDriveV.getVoltageController());
	public final AngleIn<Position> angle = gyro.getAngleInput();
	public final RangeIn<Position> leftPos = leftDriveT.getSensorPositionStream();
	public final RangeIn<Position> rightPos = rightDriveT.getSensorPositionStream();
	public Preferences prefs;
	public static RobotSoftware getInstance()
	{
		return INSTANCE;
	}
	

	public RobotSoftware()
	{
		prefs = Preferences.getInstance();
		//PositionEncoderIn.setGlobalWheelDiameter(RobotConstants.WheelDiameter);
	}

}
