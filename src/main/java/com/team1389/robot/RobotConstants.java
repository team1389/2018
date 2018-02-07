package com.team1389.robot;

public class RobotConstants {
	public static final int MaxConcurrentThreads = 20;

	/**
	 * constants for odometry calculations
	 */
	public static final double WheelDiameter = 5; // in
	public static final double TrackWidth = 22; // in
	public static final double TrackLength = 23;
	public static final double TrackScrub = 1;

	/**
	 * constants for motion profiling
	 */
	public static final double MaxVelocity = 22; // m/s
	public static final double MaxAcceleration = 22; // m/s^2
	public static final double MaxDeceleration = 22; // m/s^2
	public static final double MaxJerk = 22; // no idea
	
	/*
	 * constants for elevator motion profiling
	 * in inches because we have elevator pos in inches
	 */
	public static final double ElevMaxVelocity = 0; // inches/s
	public static final double ElevMaxAcceleration = 0;// inches/s/s
	public static final double ElevMaxDeceleration = 0; // inches/s/s
	public static final double ElevMaxJerk = 0; // inches/s/s/s


}
