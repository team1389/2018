
package com.team1389.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.team1389.auto.AutoModeBase;
import com.team1389.auto.AutoModeExecuter;
import com.team1389.hardware.inputs.hardware.SwitchHardware;
import com.team1389.hardware.inputs.software.PercentIn;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.outputs.hardware.CANTalonHardware;
import com.team1389.hardware.outputs.hardware.VictorHardware;
import com.team1389.hardware.outputs.software.RangeOut;
import com.team1389.hardware.registry.Registry;
import com.team1389.hardware.registry.port_types.CAN;
import com.team1389.hardware.registry.port_types.PWM;
import com.team1389.hardware.value_types.Position;
import com.team1389.hardware.value_types.Speed;
import com.team1389.hardware.value_types.Value;
import com.team1389.operation.TeleopMain;
import com.team1389.trajectory.PathFollowingSystem;
import com.team1389.trajectory.PathFollowingSystem.Constants;
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
	CANTalonHardware masterTalon;
	CANTalonHardware followerTalon;
	Registry registry;

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
		Pathfinder path = new Pathfinder();
		Constants constants = new Constants(RobotConstants.MaxJerk, RobotConstants.MaxAcceleration,
				RobotConstants.MaxVelocity, 1, 0, 0, robot.pos.get(), 2);

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
		Waypoint[] points = new Waypoint[] { new Waypoint(-4, -1, Pathfinder.d2r(-45)), new Waypoint(-2, -2, 0),
				new Waypoint(0, 0, 0) };

		Trajectory trajectory = Pathfinder.generate(points, config);

		// Wheelbase Width = 0.762m
		TankModifier modifier = new TankModifier(trajectory).modify(0.762);
		
		EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
		EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());
		
		left.configureEncoder((int) robot.leftDriveT.getSensorPositionStream().get(), 4096, 5);
		right.configureEncoder((int) robot.rightDriveT.getSensorPositionStream().get(), 4096, 5);
		
		left.configurePIDVA(1.0, 0.0, 0.0, 1 / RobotConstants.MaxVelocity, 0);
		right.configurePIDVA(1.0, 0.0, 0.0, 1 / RobotConstants.MaxVelocity, 0);

		// path.generate(points, config);

		// Do something with the new Trajectories...
		// Trajectory left = modifier.getLeftTrajectory();
		// Trajectory right = modifier.getRightTrajectory();

		/*
		 * autoModeExecuter.stop(); AutoModeBase selectedAutonMode =
		 * DashboardInput.getInstance().getSelectedAutonMode();
		 * autoModeExecuter.setAutoMode(selectedAutonMode);
		 */

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

		teleOperator.periodic();
	}

	@Override
	public void disabledInit()
	{
	}
}
