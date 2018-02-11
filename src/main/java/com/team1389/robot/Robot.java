
package com.team1389.robot;

import com.team1389.auto.AutoModeExecuter;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.registry.Registry;
import com.team1389.hardware.value_types.Position;
import com.team1389.operation.TeleopMain;
import com.team1389.watch.LogFile;
import com.team1389.watch.Watcher;
import com.team1389.watch.LogFile.LogType;
import com.team1389.watch.info.NumberInfo;
import com.team1389.watchers.DashboardInput;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
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
	RangeIn<Position> lPos, rPos;
	EncoderFollower left;
	EncoderFollower right;
	Watcher watcher;

	Trajectory trajectory;

	double maxSpeed;
	double maxAccel;
	double maxJerk;
	boolean driving;
	Timer timer;
	boolean first = false;
	double speed = 0;
	double accel = 0;
	double jerk = 0;
	double prevTime = 0;
	double prevSpeed = 0;
	double prevAccel;

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
		lPos = robot.leftPos.offset(-robot.leftPos.get());
		rPos = robot.rightPos.offset(-robot.rightPos.get());
		timer = new Timer();

		watcher = new Watcher();
		// LogFile log = new LogFile("roborio-1389-frc.local/src/LogFile",
		// LogType.CSV);
		// watcher.setLogLocation(log);

	}

	@Override
	public void autonomousInit()
	{
		lPos = robot.leftPos.offset(-robot.leftPos.get());
		rPos = robot.rightPos.offset(-robot.rightPos.get());
		/**
		 * Pathfinder uses the following convention: X+ -> Robot forward (across
		 * the field from your alliance to the other) Y+ -> Left hand side of X+
		 * Angle -> Positive going from X+ to Y+.
		 */
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, RobotConstants.MaxVelocity, RobotConstants.MaxAcceleration,
				RobotConstants.MaxJerk);

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5.486, 0, Pathfinder.d2r(90)),
				new Waypoint(5.486, 4.877, 0) };
		trajectory = Pathfinder.generate(points, config);
		System.out.println("Trajectory length: " + trajectory.length());

		TankModifier modifier = new TankModifier(trajectory).modify(0.67945);

		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());

		left.configureEncoder((int) lPos.get(), 1024, .127);
		right.configureEncoder((int) rPos.get(), 1024, .127);
		left.configurePIDVA(1, 0.0, 0.0, (1 / RobotConstants.MaxVelocity), 0);
		right.configurePIDVA(1, 0.0, 0.0, (1 / RobotConstants.MaxVelocity), 0);
		left.setTrajectory(modifier.getLeftTrajectory());
		right.setTrajectory(modifier.getRightTrajectory());

		for (int i = 0; i < trajectory.length(); i++)
		{
			Trajectory.Segment seg = trajectory.get(i);
			SmartDashboard.putNumber("x init", seg.x);
			SmartDashboard.putNumber("y init", seg.y);
			SmartDashboard.putNumber("dt init", seg.dt);

		}
	}

	@Override
	public void teleopInit()
	{
		autoModeExecuter.stop();

		// timer.start();

		/*
		 * watcher.watch( new NumberInfo("Speed", () ->
		 * (((robot.leftDriveT.getVelocityStream().get()) +
		 * (robot.rightDriveT.getVelocityStream().get())) / 2)), new
		 * NumberInfo("Time", () -> timer.get()));
		 */

		teleOperator.init();
		maxAccel = 0;
		maxJerk = 0;
		maxSpeed = 0; // watcher.outputToLog();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic()
	{
		// SmartDashboard.putNumber("average linear dist", (lPos.get() +
		// rPos.get()) / 2);
		double timePer = timer.get() - prevTime;
		prevTime = timer.get();
		// everything in ticks per second
		SmartDashboard.putNumber("time per", timePer);
		SmartDashboard.putNumber("time", timer.get());
		speed = (robot.leftDriveT.getVelocityStream().get()) * 10
				+ ((robot.rightDriveT.getVelocityStream().get()) * 10) / 2;
		speed = speed / 1024;
		speed = speed * .127 * Math.PI;
		SmartDashboard.putNumber("Speed", speed);

		accel = (speed - prevSpeed) / timePer;

		jerk = (accel - prevAccel) / timePer;

		SmartDashboard.putNumber("Accel", accel);

		SmartDashboard.putNumber("Jerk", jerk);

		teleOperator.periodic();

		if (robot.leftDriveT.getVelocityStream().get() > .5)
		{
			driving = true;
			first = true;
		}
		if (driving && first)
		{
			timer.start();
			first = false;
		}
		if (speed > maxSpeed)
		{
			maxSpeed = speed;
		}
		if (accel > maxAccel)
		{
			maxAccel = accel;
		}
		if (jerk > maxJerk)
		{
			maxJerk = jerk;
		}
		prevSpeed = speed;
		prevAccel = accel;

		System.out.println("Max accel" + maxAccel);
		System.out.println("Max Speed" + maxSpeed);
		System.out.println("Max Jerk" + maxJerk);

		Watcher.update();
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
		if (!left.isFinished())
		{
			robot.leftDriveT.getVoltageController().set(left.calculate((int) lPos.get()));// scaled4pracbot
			robot.rightDriveT.getVoltageController().set(right.calculate((int) rPos.get()));
		} else
		{
			robot.leftDriveT.getVoltageController().set(0);// scaled4pracbot
			robot.rightDriveT.getVoltageController().set(0);
		}
		SmartDashboard.putNumber("Right Talon", rPos.get());
		SmartDashboard.putNumber("Left Talon", lPos.get());
		System.out.println("Are we therrre yettt? " + left.isFinished());
		System.out.println("are we there yet right ed. " + right.isFinished());
		speed = (robot.leftDriveT.getVelocityStream().get()) * 10
				+ ((robot.rightDriveT.getVelocityStream().get()) * 10) / 2;
		speed = speed / 1024;
		speed = speed * .127 * Math.PI;
		SmartDashboard.putNumber("Speed", speed);

		SmartDashboard.putBoolean("Right Finished", right.isFinished());
		SmartDashboard.putBoolean("Left Finished", left.isFinished());
		SmartDashboard.putNumber("Right pos", rPos.get() / 1024 * Math.PI * .127);
		SmartDashboard.putNumber("Left pos", lPos.get() / 1024 * Math.PI * .127);

	}

	@Override
	public void disabledInit()
	{
	}

	@Override
	public void disabledPeriodic()
	{
		// TODO Auto-generated method stub
		SmartDashboard.putNumber("Right Talon", rPos.get());
		SmartDashboard.putNumber("Left Talon", lPos.get());
	}
}
