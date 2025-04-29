package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Snowball_default (Java)")
public class Snowball_default extends LinearOpMode {
	// DriveTrain
	private DcMotor leftMotor;
	private DcMotor rightMotor;
	private static final double LEFT_SPEED_COEFFICIENT = 0.75; // NOTE: run at 3/4 power to compensate for resistance in right motor
	private static final double RIGHT_SPEED_COEFFICIENT = 1.0;
	private double drive_power;
	private double drive_turn;

	// Pickup
	private DcMotor pickupMotor;
	private static final double PICKUP_MOTOR_POWER = 0.5;
	private boolean pickup_motor_on;
	private ElapsedTime pickup_elapsed_time; // TODO: remove

	/* Shooting */
	// Servo: TODO: determine if setAngle() is an option
	private Servo armingServo;
	private boolean shooterArmed;
	private static final int SERVO_BLOCKING_ANGLE = 0;
	private static final int SERVO_OPEN_ANGLE = 45;

	// Main Shooting Motor
	private static final double SHOOTER_POWER_DELTA = 0.05;
	private static final double ACCEL_DELAY = 100;
	private ElapsedTime shooter_elapsed_time; 
	private DcMotor shootingMotor;
	private void driveMotors(double left_power, double right_power) {
		leftMotor.setPower(-left_power * LEFT_SPEED_COEFFICIENT); // Reverse direction
		rightMotor.setPower(right_motor * RIGHT_SPEED_COEFFICIENT);
	}

	@Override
	public void runOpMode() {
		// Init
		leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
		rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
		pickupMotor = hardwareMap.get(DcMotor.class, "pickupMotor");
		shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
		armingServo = hardwareMap.get(Servo.class, "armingServo");

		waitForStart();
		shooterArmed = false;
		pickup_motor_on = false;
		armingServo.setAngle(SERVO_BLOCKING_ANGLE);
		shooter_elapsed_time = new ElapsedTime();
		pickup_elapsed_time = new ElapsedTime();
		if (opModeIsActive()) {
			while (opModeIsActive()) {
				// Pickup System
				if (gamepad1.y) {
					pickup_motor_on = !pickup_motor_on;
				}
				if (pickup_motor_on && gamepad1.left_stick_y > 0.5 && pickup_elapsed_time.milliseconds() >= 500) {
					pickupMotor.setPower(pickupMotor.getPower() + 0.1);
					pickup_elapsed_time.reset();
				} else if (!pickup_motor_on) {
					pickupMotor.setPower(0);
				}
				// TODO: uncomment once fixed speed found
				// pickupMotor.setPower((pickup_motor_on) ? PICKUP_MOTOR_POWER : 0);

				// Driving
				drive_power = gamepad1.right_stick_y;
				drive_turn = gamepad1.right_stick_x;
				if (gamepad1.dpad_left) {
					driveMotors(-1, 1);
				} else if (gamepad1.dpad_right) {
					driveMotors(1, -1);
				} else {
					driveMotors(drive_power - drive_turn, drive_power + drive_turn);
				}

				// Arming Servo
				// TODO: determine if getAngle() and setAngle() work
				if (gamepad1.b) {
					if (armingServo.getAngle() == SERVO_BLOCKING_ANGLE) {
						armingServo.setAngle(SERVO_OPEN_ANGLE);
					} else {
						armingServo.setAngle(SERVO_BLOCKING_ANGLE);
					}
				}

				// Shooting
 				// NOTE: Motor can NEVER be run counterclockwise
				shootingMotor.setPower(0.01); // TODO: try this to ensure going in proper direction

				// if (gamepad1.a && (shooter_elapsed_time.milliseconds() >= ACCEL_DELAY && gamepad1.dpad_up > 0.5 && shootingMotor.getPower() < 1)) {
				// 	shootingMotor.setPower(shootingMotor.getPower() + SHOOTER_POWER_DELTA);
				// 	shooter_elapsed_time.reset();
				// }

				telemetry.addData("Pickup Motor Speed", pickupMotor.getPower());
				telemetry.update();
			}
		}
	}
}
