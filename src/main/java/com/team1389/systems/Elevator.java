package com.team1389.systems;

import com.team1389.command_framework.CommandUtil;
import com.team1389.command_framework.command_base.Command;
import com.team1389.control.MotionProfileController;
import com.team1389.hardware.inputs.software.DigitalIn;
import com.team1389.hardware.inputs.software.RangeIn;
import com.team1389.hardware.outputs.software.RangeOut;
import com.team1389.hardware.value_types.Percent;
import com.team1389.hardware.value_types.Position;
import com.team1389.hardware.value_types.Speed;
import com.team1389.motion_profile.MotionProfile;
import com.team1389.motion_profile.ProfileUtil;
import com.team1389.robot.RobotConstants;
import com.team1389.system.Subsystem;
import com.team1389.util.list.AddList;
import com.team1389.watch.Watchable;

/**
 * note that one must zero elevator before going to any pos (since we scale
 * encoder val by 3 for up, it gets super inaccurate on the way down, as i'm not
 * sure about how cascading elevators work on the way down 
 * Think we have to use commands if we want to zero before going to a pos
 * 
 * @author Quunii
 *
 */
public class Elevator extends Subsystem {
	DigitalIn zero;
	RangeIn<Position> elevPos;
	RangeIn<Speed> elevVel;
	RangeOut<Percent> elevVolt;
	State currentState;
	MotionProfileController profileController;

	/**
	 * 
	 * @param zero
	 * @param elevPos
	 *            must be mapped using circ on pulley, scaled by 3 to represent
	 *            cascading
	 * @param elevVolt
	 */
	public Elevator(DigitalIn zero, RangeIn<Position> elevPos, RangeIn<Speed> elevVel, RangeOut<Percent> elevVolt) {
		this.zero = zero;
		this.elevPos = elevPos;
		this.elevVolt = elevVolt;
		this.elevVel = elevVel;
		profileController = new MotionProfileController(0.1, 0, 0, 0, elevPos, elevVel, elevVolt);
	}

	@Override
	public AddList<Watchable> getSubWatchables(AddList<Watchable> arg0) {
		return arg0;
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void init() {
		currentState = State.ZERO;

	}

	@Override
	public void update() {
		if (zero.get()) {
			elevPos.offset(-elevPos.get());
		}
		scheduler.update();
	}
	// public static MotionProfile trapezoidal(double dx, double Vo, double
	// maxAccel, double maxDecel, double maxSpeed) {

	/**
	 * 
	 * @param desired
	 * @return motion profile that is being followed
	 */
	public Command goTo(State desired) {
		setState(desired);
		MotionProfile profile = calculateProfile(desired);
		return profileController.followProfileCommand(profile);

	}

	public void goToZero() {
		scheduler.schedule(goTo(State.ZERO));
		// (simultaneously) put arm at zero as well or keep it in current state
	}

	/**
	 * 
	 * @param front
	 *            set true if arm should be in front, false if not
	 */
	public void goToSwitchLow(boolean front) {
		goToZero();
		scheduler.schedule(goTo(State.SWITCH_LOW));
		if (front) {
			//  put arm at -90 (front)
		} else {
			// put arm at 90 (back)
		}
	}
	//run armcommand simultaneously with elev command
	public void goToSwitchHigh(boolean front) {
		goToZero();
		scheduler.schedule(goTo(State.SWITCH_HIGH));
		if (front) {
			// put arm at -90 (front)
		} else {
			// put arm at 90 (back)
		}
	}

	/**
	 * note: would always have to use SCALE_LOW for elevDuration 
	 * could totally use the arm enum for front and back instead of boolean
	 *  have to figure out how to select front or back on armCommand
	 * @param front
	 *            set true if arm should be in front, false if not
	 */
	public void goToScaleHigh(boolean front) {
		goToZero();
		// idea is that we do armDuration - elevDuration, if its < 1 we wait for
		// (armDuration-elevDuration), which should allow the arm mp to finish before we
		// hit scale
		double elevDuration = calculateProfile(State.SCALE_LOW).getDuration();
		// armDuration is expected duration of arm profile
		double armDuration = 0;
		double bufferTime = armDuration - elevDuration;
		Command armCommand; //= ((bufferTime<1)? waitTimeCommand(bufferTime) + armToZeroCommand(): armToZeroCommand()
		scheduler.schedule(CommandUtil.combineSimultaneous(goTo(State.SCALE_HIGH), armCommand));	

	}
	public void goToScaleLow(boolean front) {
		goToZero();
		// idea is that we do armDuration - elevDuration, if its < 1 we wait for
		// (armDuration-elevDuration), which should allow the arm mp to finish before we
		// hit scale
		double elevDuration = calculateProfile(State.SCALE_LOW).getDuration();
		// armDuration is expected duration of arm profile
		double armDuration = 0;
		double bufferTime = armDuration - elevDuration;
		Command armCommand; //= ((bufferTime<1)? waitTimeCommand(bufferTime) + armToZeroCommand(): armToZeroCommand()
		scheduler.schedule(CommandUtil.combineSimultaneous(goTo(State.SCALE_LOW), armCommand));	

	}
	/**
	 * check if we have max hall effect, if so we dont have to zero beforehand
	 */
	public void goToMax()
	{
		scheduler.schedule(goTo(State.MAX));
	}
	

	public enum State {
		ZERO(0), SWITCH_LOW(0), SWITCH_HIGH(0), SCALE_LOW(0), SCALE_HIGH(0), MAX(0);
		private final double pos;

		private State(double pos) {
			this.pos = pos;
		}
	}

	private void setState(State toSet) {
		currentState = toSet;
	}
	private MotionProfile calculateProfile(State desired)
	{
		return ProfileUtil.trapezoidal(desired.pos, elevVel.get(),
				RobotConstants.ElevMaxAcceleration, RobotConstants.ElevMaxDeceleration, RobotConstants.ElevMaxVelocity);
	}

}
