package org.usfirst.frc.team5348.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

@SuppressWarnings("unused")
public class Robot extends IterativeRobot {

	public ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	NetworkTable netTable;
	NetworkTable sd;
	boolean resgyro = false;
	double shoott = 0.78;
	double x = 0;
	boolean up = false;
	boolean track = false;
	boolean doneangle = false;
	double distance = 40;
	double gyroangle;
	int step = 0;
	int baselinedist = -1500;
	AnalogInput distancesens = new AnalogInput(1);
	boolean timerr = false;
	Timer timer = new Timer();
	int time = 0;
	Victor left = new Victor(1);
	String auto;
	Victor right = new Victor(0);
	VictorSP Intake = new VictorSP(2);
	VictorSP Winch = new VictorSP(5);
	Spark Shoot = new Spark(3);
	Spark Stir = new Spark(4);

	DoubleSolenoid Mouth = new DoubleSolenoid(0, 1);
	DoubleSolenoid Move = new DoubleSolenoid(2, 3);
	DoubleSolenoid Shift = new DoubleSolenoid(4, 5);
	DoubleSolenoid Door = new DoubleSolenoid(6, 7);
	Encoder encRight = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder encLeft = new Encoder(2, 3, false, Encoder.EncodingType.k4X);

	Compressor c = new Compressor(0);

	boolean invert = true;

	RobotDrive myRobot = new RobotDrive(left, right);
	Joystick stick = new Joystick(0);
	Joystick stick2 = new Joystick(1);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		CameraServer.getInstance().startAutomaticCapture();
		NetworkTable.setIPAddress("10.53.48.109");
		netTable = NetworkTable.getTable("datatable");
		encLeft.reset();
		encRight.reset();
		up = false;
		Shift.set(DoubleSolenoid.Value.kReverse);
		sd = NetworkTable.getTable("SmartDashboard");
		sd.putString("Auto", netTable.getString("Auto", "Error"));
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		sd.putString("Auto", netTable.getString("Auto", "Error"));
		timer.reset();
		timer.start();
		step = 0;
		encLeft.reset();
		encRight.reset();
		c.setClosedLoopControl(true);
		gyro.reset();
		timerr = false;
		switch (netTable.getString("Auto", "Error")) {
		case "Mid":
			auto = "Mid";
			break;
		case "Left":
			auto = "Left";
			break;
		case "Right":
			auto = "Right";
			break;
		}
		System.out.println(auto);

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@SuppressWarnings("static-access")
	@Override
	public void autonomousPeriodic() {

		gyroangle = gyro.getAngle() + 15;

		netTable.putNumber("EncL", encLeft.get());
		netTable.putNumber("EncR", encRight.get());

		sd.putNumber("EncL", encLeft.get());
		sd.putNumber("EncR", encRight.get());
		sd.putNumber("Gyro Angle", gyro.getAngle());
		sd.putNumber("Distance", ((distancesens.getAverageVoltage() * 1000) / 0.977) / 10);
		sd.putNumber("POV", stick.getPOV());
		sd.putString("Auto", netTable.getString("Auto", "Error"));
		double encAv = (encLeft.get() + encRight.get()) / 2;

		switch (auto) {
		case "Mid":

			switch (step) {
			case 0:
				if (encAv > baselinedist) {
					myRobot.tankDrive(0.975, 1);
				} else {
					timer.delay(2);
					step++;
				}
				break;
			case 1:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(0.75, 0);
				if (timer.get() >= 0.5) {
					step++;
					timerr = false;
				}

				break;
			case 2:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(0, 0.75);
				if (timer.get() >= 0.5) {
					step++;
					timerr = false;
				}

				break;
			case 3:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				Move.set(DoubleSolenoid.Value.kReverse);
				if (timer.get() >= 1) {
					step++;
					timerr = false;
				}
				break;
			case 4:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(-0.5, -0.5);
				if (timer.get() >= 3.2) {
					step++;
					timerr = false;
				}
				break;

			case 5:

				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				Move.set(DoubleSolenoid.Value.kForward);
				if (timer.get() >= 1.5) {
					timerr = false;
				}
				break;

			}
			break;

		case "Left":

			switch (step) {
			case 0:
				if (encAv > baselinedist + 850) {
					myRobot.tankDrive(0.975, 1);
				} else {
					timer.delay(2);
					step++;
				}
				break;
			case 1:
				if (!((gyroangle >= 68) && (gyroangle <= 70))) {
					myRobot.tankDrive(-0.5, 0.5);
					System.out.println("Case1");
				} else {
					step++;

				}
				break;
			case 2:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(0.75, 0.75);
				if (timer.get() >= .75) {
					timer.delay(2);

					step++;
					timerr = false;
				}

				break;
			case 3:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(0.75, 0);
				if (timer.get() >= 0.5) {
					step++;
					timerr = false;
				}

				break;
			case 4:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(0, 0.75);
				if (timer.get() >= 0.5) {
					step++;
					timerr = false;
				}

				break;
			case 5:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				Move.set(DoubleSolenoid.Value.kReverse);
				if (timer.get() >= 1) {
					step++;
					timerr = false;
				}
				break;
			case 6:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(-0.5, -0.5);
				if (timer.get() >= 3.2) {
					step++;
					timerr = false;
				}
				break;

			case 7:
				System.out.println("step2");
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				Move.set(DoubleSolenoid.Value.kForward);
				if (timer.get() >= 1.5) {// step++;
					timerr = false;
				}
				break;
			}
			break;

		case "Right":
			switch (step) {
			case 0:
				if (encAv > baselinedist + 600) {
					myRobot.tankDrive(0.975, 1);
				} else {
					timer.delay(2);
					step++;
				}
				break;
			case 1:
				if (!((gyroangle <= -33) && (gyroangle >= -35))) {
					myRobot.tankDrive(0.5, -0.5);
					System.out.println("Case1");
				} else {
					step++;

				}
				break;
			case 2:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(0.75, 0.75);
				if (timer.get() >= .75) {
					timer.delay(2);

					step++;
					timerr = false;
				}

				break;
			case 3:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(0.75, 0);
				if (timer.get() >= 0.5) {
					step++;
					timerr = false;
				}

				break;
			case 4:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(0, 0.75);
				if (timer.get() >= 0.5) {
					step++;
					timerr = false;
				}

				break;
			case 5:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				Move.set(DoubleSolenoid.Value.kReverse);
				if (timer.get() >= 1) {
					step++;
					timerr = false;
				}
				break;
			case 6:
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				myRobot.tankDrive(-0.5, -0.5);
				if (timer.get() >= 3.2) {
					step++;
					timerr = false;
				}
				break;

			case 7:
				System.out.println("step2");
				if (timerr == false) {
					timer.reset();
					timer.start();
					timerr = true;
				}
				Move.set(DoubleSolenoid.Value.kForward);
				if (timer.get() >= 1.5) {// step++;
					timerr = false;
				}
				break;
			}

			break;
		case "Error":

			break;
		default:

			break;
		}

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {

		c.setClosedLoopControl(true);
		timer.reset();
		gyro.reset();
		sd.putNumber("Shoot", shoott);
		track = false;
	}

	/**
	 * This function is called periodically during operator control
	 */

	@SuppressWarnings("deprecation")
	@Override
	public void teleopPeriodic() {
		sd.putString("Auto", netTable.getString("Auto", "Error"));
		netTable.putNumber("EncL", encLeft.get());
		netTable.putNumber("EncR", encRight.get());
		sd.putNumber("EncL", encLeft.get());
		sd.putNumber("EncR", encRight.get());
		sd.putNumber("Gyro Angle", gyro.getAngle());
		sd.putNumber("Distance", ((distancesens.getAverageVoltage() * 1000) / 0.977) / 10);
		sd.putNumber("POV", stick.getPOV());

		// Driving
		if (invert == false) {
			if (up == true) {
				x = 0.65 * stick.getX();
			} else {
				x = 0.75 * stick.getX();
			}
			myRobot.arcadeDrive(stick.getY(), x);
		} else {
			if (up == true) {
				x = 0.65 * stick.getX();
			} else {
				x = 0.75 * stick.getX();
			}

			myRobot.arcadeDrive(-stick.getY(), x);
		}

		// Turns on Stir and Shoot Motors
		if (stick.getRawButton(3) || (stick2.getRawButton(12))) {

			Stir.set(1);
			Door.set(DoubleSolenoid.Value.kReverse);

		}
		// Turns off stir and shooter motors
		else {
			Stir.set(0);
			Door.set(DoubleSolenoid.Value.kForward);
		}
		if ((stick.getRawButton(4) || (stick2.getRawButton(1)))) {

			Shoot.set(sd.getNumber("Shoot", 0.78));

		}
		// Turns off stir and shooter motors
		else {
			Shoot.set(0);

		}
		// Moves Winch Down
		if (stick.getRawButton(14)) {

			Winch.set(-1);
			encLeft.reset();
			encRight.reset();
		}
		// Moves Winch up
		else if (stick.getRawButton(13)) {

			encLeft.reset();
			encRight.reset();
		}
		// Turns Winch off
		else {
			Winch.set(0);
		}
		// Opens Chunnel
		if (stick.getRawButton(5)) {

			Mouth.set(DoubleSolenoid.Value.kForward);
		}
		// Moves gear down
		if (stick.getRawButton(6)) {

			Move.set(DoubleSolenoid.Value.kForward);
		}
		// Closes Chunnel
		if (stick.getRawButton(7)) {

			Mouth.set(DoubleSolenoid.Value.kReverse);
		}
		// Move Gear Up
		if (stick.getRawButton(8)) {

			Move.set(DoubleSolenoid.Value.kReverse);
		}
		// Reverts Drive Controls
		if (stick.getRawButton(9) == true) {
			invert = false;
		}
		// Inverts Drive Controls
		if (stick.getRawButton(10) == true) {
			invert = true;
		}

		// Turn Intake on
		if (stick.getRawButton(12) || (stick2.getRawButton(11))) {

			Intake.set(1);
		}
		// Turn Intake off
		else {
			Intake.set(0);
		}
		// Shift Up
		if (stick.getRawButton(1)) {

			up = true;
			Shift.set(DoubleSolenoid.Value.kForward);
		}
		// Shift Down
		if (stick.getRawButton(2)) {
			System.out.println("Down");
			up = false;
			Shift.set(DoubleSolenoid.Value.kReverse);
		}
		// Pi Tracking
		if (stick.getRawButton(11)) {
			// Pi tracking

			encLeft.reset();
			encRight.reset();
			// track=true;

		}

	}

	/**
	 * This function is called periodically during test mode
	 */

	@SuppressWarnings("deprecation")
	@Override
	public void testPeriodic() {

	}
}
