
package com.team1389.robot;

import com.team1389.auto.AutoModeExecuter;
import com.team1389.hardware.registry.Registry;
import com.team1389.operation.TeleopMain;
import com.team1389.watchers.DashboardInput;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
	RobotSoftware robot;
	TeleopMain teleOperator;
	AutoModeExecuter autoModeExecuter;
	Registry registry;

	EncoderFollower left;
	EncoderFollower right;

	Trajectory trajectory;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		registry = new Registry();
		robot = RobotSoftware.getInstance();
		teleOperator = new TeleopMain(robot);
		autoModeExecuter = new AutoModeExecuter();
		DashboardInput.getInstance().init();

	}

	@Override
	public void autonomousInit()
	{
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
		Waypoint[] points = new Waypoint[] { new Waypoint(0, 1, 0), new Waypoint(0, 2, 0), new Waypoint(0, 0, 0) };

		trajectory = Pathfinder.generate(points, config);
		System.out.println("Trajectory length: " + trajectory.length());

		TankModifier modifier = new TankModifier(trajectory).modify(0.67945);

		;
		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());

		left.configureEncoder((int) robot.leftDriveT.getSensorPositionStream().get(), 1024, 5);
		right.configureEncoder((int) robot.rightDriveT.getSensorPositionStream().get(), 1024, 5);
		left.configurePIDVA(1, 0.0, 0.0, (1 / RobotConstants.MaxVelocity), 0);
		right.configurePIDVA(1, 0.0, 0.0, (1 / RobotConstants.MaxVelocity), 0);

		left.setTrajectory(trajectory);
		right.setTrajectory(trajectory);

	}

	@Override
	public void teleopInit()
	{
		autoModeExecuter.stop();

		teleOperator.init();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic()
	{
		SmartDashboard.putNumber("Right Talon", robot.rightDriveT.getSensorPositionStream().get());
		SmartDashboard.putNumber("Left Talon", robot.leftDriveT.getSensorPositionStream().get());

		teleOperator.periodic();
	}

	@Override
	public void autonomousPeriodic()
	{
		/*
		 * double l = left.calculate((int)
		 * robot.leftDriveT.getSensorPositionStream().get()); double r =
		 * right.calculate((int)
		 * robot.rightDriveT.getSensorPositionStream().get());
		 * 
		 * double gyro_heading = robot.pos.get(); // Assuming the gyro is giving
		 * a // value in degrees double desired_heading =
		 * Pathfinder.r2d(left.getHeading()); // Should // also be // in //
		 * degrees
		 * 
		 * double angleDifference = Pathfinder.boundHalfDegrees(desired_heading
		 * - gyro_heading); double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
		 */

		robot.leftDriveT.getVoltageController()
				.set(left.calculate((int) robot.leftDriveT.getSensorPositionStream().get()));
		robot.rightDriveT.getVoltageController()
				.set(right.calculate((int) robot.rightDriveT.getSensorPositionStream().get()));

		for (int i = 0; i < trajectory.length(); i++)
		{
			Trajectory.Segment seg = trajectory.get(i);

			System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y, seg.position, seg.velocity,
					seg.acceleration, seg.jerk, seg.heading);
		}

	}

	@Override
	public void disabledInit()
	{
	}
}
