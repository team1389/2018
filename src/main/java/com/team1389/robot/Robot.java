
package com.team1389.robot;

import com.team1389.auto.AutoModeExecuter;
import com.team1389.hardware.inputs.software.AngleIn;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.registry.Registry;
import com.team1389.hardware.value_types.Position;
import com.team1389.hardware.value_types.Speed;
import com.team1389.operation.TeleopMain;
import com.team1389.system.drive.FourWheelSignal;
import com.team1389.watch.Watcher;
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
	RangeIn<Speed> lVel, rVel;
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
	double error = 0;

	AngleIn gyro;

	double gyroheading;

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
		lVel = robot.leftDriveT.getVelocityStream().scale(10).scale(1/1024).scale(.127 * Math.PI);
		rVel = robot.rightDriveT.getVelocityStream().scale(10).scale(1/1024).scale(.127 * Math.PI);
		watcher = new Watcher();
		first = false;

		gyro = robot.angle;
		SmartDashboard.putNumber("error", error);

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
				Trajectory.Config.SAMPLES_HIGH, 0.05, robot.prefs.getDouble("MaxVel", 0.0),
				robot.prefs.getDouble("MaxAccel", 0.0), robot.prefs.getDouble("MaxJerk", 0.0));

		// Waypoint[] points = new Waypoint[] { new Waypoint(-4, -1,
		// Pathfinder.d2r(-45)), new Waypoint(-2, -2, 0),
		// new Waypoint(0, 0, 0) };

		//Waypoint[] points = new Waypoint[] { new Waypoint(.508, 6.731, 0.0), new Waypoint(3.048, 7.62, -0.349),
		//		new Waypoint(4.191, 6.477, -1.2) };//, new Waypoint(4.2, 6.6, 0) };
		
		Waypoint[] points = new Waypoint[] { new Waypoint(0, .9144, 0), new Waypoint(7.62, 2.1336, 0), new Waypoint(5.334, 2.286, 0)};

		trajectory = Pathfinder.generate(points, config);
		System.out.println("Trajectory length: " + trajectory.length());

		TankModifier modifier = new TankModifier(trajectory).modify(0.656); // 0.67945

		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());
		left.reset();
		right.reset();
		lPos.offset(-lPos.get());
		rPos.offset(-rPos.get());
		gyro.offset(-gyro.get());
		left.configureEncoder((int) lPos.get(), 1024, .127);
		right.configureEncoder((int) rPos.get(), 1024, .127);
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

		teleOperator.periodic();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic()
	{
		robot.drive.set(new FourWheelSignal(0.25, 0.25, 0.25, 0.25));
	}

	@Override
	public void autonomousPeriodic()
	{
		SmartDashboard.putNumber("l pos", lPos.get());
		SmartDashboard.putNumber("r pos", rPos.get());
		gyroheading = gyro.get();
		System.out.println("pos is " + left.isFinished());

		double l = left.calculate((int) lPos.get());
		double r = right.calculate((int) rPos.get());
		double distance_covered = ((double) (lPos.get() / 1024) * .127 * Math.PI);

		double desired_heading = Pathfinder.r2d(left.getHeading()); // Should
																	// also be
																	// in
																	// degrees

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyroheading);
		double turn = robot.prefs.getDouble("GyroP", 0.0) * (-1.0 / 80.0) * angleDifference;

		robot.leftDriveT.getVoltageController().set(l + turn);
		robot.rightDriveT.getVoltageController().set(r - turn);
SmartDashboard.putBoolean("left is finished", left.isFinished());
		SmartDashboard.putNumber("Speed", robot.leftDriveT.getVelocityStream().get() * 10);
		SmartDashboard.putNumber("Angle" + "++" + "", robot.angle.get());
		if (!left.isFinished())
		{
			distance_covered = ((double) (lPos.get()) / 1024) * .127 * Math.PI;
			error = left.getSegment().position - distance_covered;
			SmartDashboard.putNumber("error", error);
			SmartDashboard.putNumber("left expected vel", left.getSegment().velocity);
			SmartDashboard.putNumber("left actual vel", lVel.get());
			
			SmartDashboard.putNumber("Error angle does",
					((robot.gyro.getAngleInput().get() - Pathfinder.r2d(left.getHeading()))
							+ (robot.gyro.getAngleInput().get() - Pathfinder.r2d(right.getHeading()))) / 2);
			SmartDashboard.putNumber("Desired heading", (left.getHeading() + right.getHeading()) / 2);

		}
		if (!right.isFinished())
		{
			SmartDashboard.putNumber("right expected vel", right.getSegment().velocity);
			SmartDashboard.putNumber("right actual vel", rVel.get());
		}

	}

	@Override
	public void disabledInit()
	{
	}

	@Override
	public void disabledPeriodic()
	{
		// TODO Auto-generated method stub
		SmartDashboard.putNumber("angle", robot.gyro.getAngleInput().get());

		SmartDashboard.putNumber("r pos", rPos.get());
		SmartDashboard.putNumber("l pos", lPos.get());

		SmartDashboard.putNumber("Speed", robot.leftDriveT.getVelocityStream().get() * 10);

		SmartDashboard.putNumber("Error angle does", 0);

		SmartDashboard.putNumber("Desired heading", 0);

	}
}
