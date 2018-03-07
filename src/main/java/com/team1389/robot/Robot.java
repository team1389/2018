
package com.team1389.robot;

import com.team1389.auto.AutoModeExecuter;
import com.team1389.hardware.inputs.software.AngleIn;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.registry.Registry;
import com.team1389.hardware.value_types.Position;
import com.team1389.hardware.value_types.Speed;
import com.team1389.operation.TeleopMain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Robot extends IterativeRobot
{
	RobotSoftware robot;
	TeleopMain teleOperator;
	AutoModeExecuter autoModeExecuter;
	Registry registry;
	RangeIn<Position> lPos, rPos;
	RangeIn<Speed> lVel, rVel;
	DistanceFollower left;
	DistanceFollower right;

	Trajectory trajectory;

	AngleIn gyro;

	@Override
	public void robotInit()
	{
		robot = RobotSoftware.getInstance();
		teleOperator = new TeleopMain(robot);
		autoModeExecuter = new AutoModeExecuter();
		lVel = robot.leftDriveT.getVelocityStream().scale(10).scale(1 / 1024).scale(.127 * Math.PI);
		rVel = robot.rightDriveT.getVelocityStream().scale(10).scale(1 / 1024).scale(.127 * Math.PI);

		gyro = robot.angle;

	}

	@Override
	public void autonomousInit()
	{

		lPos = robot.leftPos.offset(-robot.leftPos.get());
		rPos = robot.rightPos.offset(-robot.rightPos.get());
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, robot.prefs.getDouble("MaxVel", 0.0),
				robot.prefs.getDouble("MaxAccel", 0.0), robot.prefs.getDouble("MaxJerk", 0.0));
		// Waypoint[] points = new Waypoint[] { new Waypoint(0, .9144, 0), new
		// Waypoint(7.62, 2.1336, 0)};
		Waypoint[] points = new Waypoint[] { new Waypoint(0, .9144, 0), new Waypoint(2, .9144, 0) };
		trajectory = Pathfinder.generate(points, config);
		System.out.println("Trajectory length: " + trajectory.length());

		TankModifier modifier = new TankModifier(trajectory).modify(0.656);

		left = new DistanceFollower(modifier.getLeftTrajectory());
		right = new DistanceFollower(modifier.getRightTrajectory());
		left.reset();
		right.reset();
		gyro = (AngleIn) gyro.offset(-gyro.get());
		left.configurePIDVA(robot.prefs.getDouble("PathP", 0.0), robot.prefs.getDouble("PathI", 0.0),
				robot.prefs.getDouble("PathD", 0.0), (1 / robot.prefs.getDouble("MaxVel", 0.0)),
				robot.prefs.getDouble("PathF", 0.0));
		right.configurePIDVA(robot.prefs.getDouble("PathP", 0.0), robot.prefs.getDouble("PathI", 0.0),
				robot.prefs.getDouble("PathD", 0.0), (1 / robot.prefs.getDouble("MaxVel", 0.0)),
				robot.prefs.getDouble("PathF", 0.0));

		SmartDashboard.putNumber("MAX VEL", robot.prefs.getDouble("MaxVel", 0));
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
		robot.drive.getAsTank().set(0.25, 0.25);

	}

	@Override
	public void autonomousPeriodic()
	{
		SmartDashboard.putNumber("l pos(ticks)", lPos.get());
		SmartDashboard.putNumber("r pos", rPos.get());
		double gyroHeading = gyro.get();
		System.out.println("pos is " + left.isFinished());
//		 double leftDistance = (lPos.get()) / 1024 * .127 * Math.PI;
//		 double rightDistance = (rPos.get()) / 1024 * .127 * Math.PI;
		 double leftDistance = lPos.get()/1024;
		 double rightDistance = rPos.get()/1024;
		double l = left.calculate((int) leftDistance);
		double r = right.calculate((int) rightDistance);
		SmartDashboard.putNumber("left distance", leftDistance);
		SmartDashboard.putNumber("right distance", rightDistance);

		double desired_heading = Pathfinder.r2d(left.getHeading()); // Should
																	// also be
																	// in
																	// degrees

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyroHeading);
		double angleError = (-1.0 / 80.0) * angleDifference;
		double turn = robot.prefs.getDouble("GyroP", 0.0) * angleError;
		SmartDashboard.putNumber("angleError", angleError);
		// robot.drive.getAsTank().set(l+turn, r-turn);
		robot.drive.getAsTank().set(l, r);
		SmartDashboard.putNumber("Angle ", robot.angle.get());
		if (!left.isFinished())
		{
			double error = left.getSegment().position - leftDistance;
			SmartDashboard.putNumber("left expected vel", left.getSegment().velocity);
			SmartDashboard.putNumber("left actual vel", lVel.get());

			SmartDashboard.putNumber("angle error",
					((robot.gyro.getAngleInput().get() - Pathfinder.r2d(left.getHeading()))
							+ (robot.gyro.getAngleInput().get() - Pathfinder.r2d(right.getHeading()))) / 2);
			SmartDashboard.putNumber("Desired heading", (left.getHeading() + right.getHeading()) / 2);
			SmartDashboard.putNumber("left expected pos", left.getSegment().position);

		}

		if (!right.isFinished())
		{
			double error = right.getSegment().position - rightDistance;
			SmartDashboard.putNumber("errorR", error);
			SmartDashboard.putNumber("right expected vel", right.getSegment().velocity);
			SmartDashboard.putNumber("right actual vel", rVel.get());
			SmartDashboard.putNumber("right expected pos", right.getSegment().position);

		}
	}

	@Override
	public void disabledInit()
	{
	}

	@Override
	public void disabledPeriodic()
	{

	}
}
