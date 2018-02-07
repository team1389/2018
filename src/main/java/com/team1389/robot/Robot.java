
package com.team1389.robot;

import com.team1389.auto.AutoModeExecuter;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.registry.Registry;
import com.team1389.hardware.value_types.Position;
import com.team1389.operation.TeleopMain;
import com.team1389.system.drive.FourWheelSignal;
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
	RangeIn<Position> lPos, rPos;
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
		lPos = robot.leftPos.offset(-robot.leftPos.get());
		rPos = robot.rightPos.offset(-robot.rightPos.get());

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

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(2, 0, 0), };
		trajectory = Pathfinder.generate(points, config);
		System.out.println("Trajectory length: " + trajectory.length());

		TankModifier modifier = new TankModifier(trajectory).modify(0.67945);

		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());

		left.configureEncoder((int) lPos.get(), 1024, .127);
		right.configureEncoder((int) rPos.get(), 1024, .127);
		left.configurePIDVA(.33, 0.0, 0.0, (1 / RobotConstants.MaxVelocity), 0);
		right.configurePIDVA(.33, 0.0, 0.0, (1 / RobotConstants.MaxVelocity), 0);
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

		teleOperator.init();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic()
	{
		SmartDashboard.putNumber("average linear dist", (lPos.get() + rPos.get())/2);
		// SmartDashboard.putNumber("Gyro pos", robot.pos.get());
		robot.leftDriveV.getVoltageOutput().set(1);
		//robot.left.set(1);
		//robot.left
		//robot.rightDriveV.getVoltageOutput().set(-1);
		//robot.drive.set(new FourWheelSignal(1, 1 , 1, 1));
		//robot.right.set(1);
		robot.left.set(1);
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

		robot.leftDriveT.getVoltageController().set(left.calculate((int) lPos.get()));// scaled4pracbot
		robot.rightDriveT.getVoltageController().set(right.calculate((int) rPos.get()));
		SmartDashboard.putNumber("Right Talon", rPos.get());
		SmartDashboard.putNumber("Left Talon", lPos.get());
		System.out.println("Are we therrre yettt? " + left.isFinished());
		System.out.println("are we there yet right ed. " + right.isFinished());

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
