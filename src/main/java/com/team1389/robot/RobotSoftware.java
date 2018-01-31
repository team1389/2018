package com.team1389.robot;

import com.team1389.hardware.inputs.software.AngleIn;
import com.team1389.hardware.inputs.software.PositionEncoderIn;
import com.team1389.hardware.outputs.software.RangeOut;
import com.team1389.hardware.value_types.Percent;
import com.team1389.hardware.value_types.Position;
import com.team1389.system.drive.DriveOut;

public class RobotSoftware extends RobotHardware
{
	private static RobotSoftware INSTANCE = new RobotSoftware();

	public final RangeOut<Percent> right = rightDriveT.getVoltageController()
			.addFollowers(rightDriveV.getVoltageOutput());
	public final RangeOut<Percent> left = leftDriveT.getVoltageController().addFollowers(leftDriveV.getVoltageOutput());
	public final DriveOut<Percent> drive = new DriveOut<Percent>(leftDriveT.getVoltageController(),
			rightDriveT.getVoltageController());
	public final AngleIn<Position> pos = gyro.getAngleInput();

	public static RobotSoftware getInstance()
	{
		return INSTANCE;
	}

	public RobotSoftware()
	{
		PositionEncoderIn.setGlobalWheelDiameter(RobotConstants.WheelDiameter);
	}

}
