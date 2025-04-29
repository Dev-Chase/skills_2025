package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Snowball_default (Java)")
public class SnowballDefault extends LinearOpMode {
    // DriveTrain
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private static final double LEFT_SPEED_COEFFICIENT = 1.0;
    private static final double RIGHT_SPEED_COEFFICIENT = 1.0;
    private static final double TURN_COEFFICIENT = 0.7;
    private static final double SPIN_COEFFICIENT = 0.5;
    private double drive_power;
    private double drive_turn;
    private double drive_spin;

    // Pickup
    private DcMotor pickupMotor;
    private static final double PICKUP_MOTOR_POWER = -0.5;
    private static final double PICKUP_DURATION = 700; // millis
    private boolean pickup_motor_on;
    private double pickup_power = 0;
    private ElapsedTime pickup_elapsed_time;
    private final double PICKUP_TUNING_DELTA = 0.05;

    // Shooting Servo
    private Servo armingServo;
    private boolean shooterArmed;
    private static final double SERVO_BLOCKING_POS = 0.35;
    private static final double SERVO_OPEN_POS = 0.6;

    // Shooting Physics
    private static final double PROJ_PWR1 = 0.62; // seconds
    private static final double PROJ_SPEED_Y_FIRST = 1.66; // m/s
    private static final double PROJ_PWR2 = 0.69; // seconds
    private static final double PROJ_SPEED_Y_LAST = 1.96; // m/s
    private static final double POWER_SPEED_SLOPE = (PROJ_SPEED_Y_LAST-PROJ_SPEED_Y_FIRST) / (PROJ_PWR2-PROJ_PWR1);
    private static final double PROJ_RELATION_INIT_VAL = PROJ_SPEED_Y_FIRST - (POWER_SPEED_SLOPE * PROJ_PWR1);
    private static final double PROJ_THETA = 0.3; // rads (use degs?)
    private static final double GRAVITY = -9.8;
    private static final double ABS_GRAVITY = 9.8;
    private static final double HALF_GRAVITY = -4.9;

    // Shooting Estimation
    private static final double SHOOTER_MAX_POWER = 1.0;
    private static final double SHOOTER_MIN_POWER = 0.0;
    private static final double SHOOTER_DEF_POWER = 0.6;
    private static final double SHOOTER_TOLERANCE = 0.005;
    private double shooter_power = 0.0;
    private double shooter_target_power = 0.0;
    private double shooter_target_power_swapped = SHOOTER_MAX_POWER;
    private boolean shooter_enabled = false;
    private static final double SHOOTER_HEIGHT = 0.21; // meters
    private static final double SHOOTER_TUNE_DELTA = 0.01;
    private static final double SHOOTER_TUNE_DELAY = 200;

    // Shooter Motor
    private static final double SHOOTER_ACCEL = 0.01;
    private static final double SHOOTER_ACCEL_DELAY = 100;
    private ElapsedTime shooter_tune_time;
    private ElapsedTime shooter_accel_time;
    private DcMotor shooterMotor;

    private double distanceFromPower(double power) {
        double vy = POWER_SPEED_SLOPE*power + PROJ_RELATION_INIT_VAL;
        double vx = vy / Math.tan(PROJ_THETA);
        double time = (vy + Math.sqrt((vy*vy) + 2*ABS_GRAVITY*SHOOTER_HEIGHT)) / ABS_GRAVITY;
        return vx * time;
    }

    private void driveMotors(double left_power, double right_power) {
        leftMotor.setPower(-left_power * LEFT_SPEED_COEFFICIENT); // Reverse direction
        rightMotor.setPower(right_power * RIGHT_SPEED_COEFFICIENT);
    }

    // Input Trackers
    private boolean right_bumper = false;
    private boolean left_trigger = false;
    private boolean right_trigger = false;
    private boolean y_button = false;
    private boolean a_button = false;
    private boolean b_button = false;
    private boolean x_button = false;

    private boolean last_right_bumper = false;
    private boolean last_left_trigger = false;
    private boolean last_right_trigger = false;
    private boolean last_y_button = false;
    private boolean last_a_button = false;
    private boolean last_b_button = false;
    private boolean last_x_button = false;

    @Override
    public void runOpMode() {
        // Init
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        pickupMotor = hardwareMap.get(DcMotor.class, "pickupMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        armingServo = hardwareMap.get(Servo.class, "armingServo");

        waitForStart();
        // Shooting/Launching
        shooter_power = SHOOTER_MIN_POWER;
        shooterArmed = false;
        shooter_target_power = SHOOTER_DEF_POWER;
        shooter_target_power_swapped = SHOOTER_MAX_POWER;
        shooter_enabled = false;
        armingServo.setPosition(SERVO_BLOCKING_POS);
        shooter_tune_time = new ElapsedTime();
        shooter_accel_time = new ElapsedTime();
        
        // Pickup
        // TODO: switch to dpad controls
        // TODO: add ability to remove balls from mechanism
        pickup_power = PICKUP_MOTOR_POWER;
        pickup_motor_on = false;
        pickup_elapsed_time = new ElapsedTime();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                right_bumper = gamepad1.right_bumper;
                right_trigger = gamepad1.right_trigger > 0.5;
                left_trigger = gamepad1.left_trigger > 0.5;
                y_button = gamepad1.y;
                a_button = gamepad1.a;
                x_button = gamepad1.x;
                b_button = gamepad1.b;

                // Pickup System Input
                if (!left_trigger && last_left_trigger) {
                    pickup_motor_on = !pickup_motor_on;
                    pickup_elapsed_time.reset();
                }

                // Pickup System TODO: make speed more fixed and get rid of variable power
                if (!pickup_motor_on) {
                    pickupMotor.setPower(0.0);
                    if (pickup_elapsed_time.milliseconds() >= 1000) {
                        if (gamepad1.dpad_down && pickup_power + PICKUP_TUNING_DELTA <= 1.0) {
                            pickup_power += PICKUP_TUNING_DELTA;
                            pickup_elapsed_time.reset();
                        } else if (gamepad1.dpad_up && pickup_power - PICKUP_TUNING_DELTA >= -1.0) {
                            pickup_power -= PICKUP_TUNING_DELTA;
                            pickup_elapsed_time.reset();
                        }
                    }
                } else if (pickup_elapsed_time.milliseconds() <= PICKUP_DURATION) {
                    pickupMotor.setPower(pickup_power);
                } else {
                    pickupMotor.setPower(0.0);
                    pickup_motor_on = false;
                }

                drive_power = gamepad1.right_stick_y;
                drive_turn = gamepad1.right_stick_x * TURN_COEFFICIENT;
                if (gamepad1.left_bumper) {
                    drive_spin = drive_turn / TURN_COEFFICIENT * SPIN_COEFFICIENT;
                    driveMotors(-drive_spin, drive_spin);
                } else {
                    driveMotors(drive_power - drive_turn, drive_power + drive_turn);
                }

                // Shooter Choosing Power
                if (shooter_tune_time.milliseconds() >= SHOOTER_TUNE_DELAY && !(shooter_target_power == SHOOTER_MAX_POWER && shooter_target_power_swapped != shooter_target_power)) {
                    if (y_button && !last_y_button && shooter_target_power + SHOOTER_TUNE_DELTA <= SHOOTER_MAX_POWER) {
                        shooter_target_power += SHOOTER_TUNE_DELTA;
                        shooter_tune_time.reset();
                    }

                    if (a_button && !last_a_button && shooter_target_power - SHOOTER_TUNE_DELTA >= SHOOTER_MIN_POWER) {
                        shooter_target_power -= SHOOTER_TUNE_DELTA;
                        shooter_tune_time.reset();
                    }
                }

                // Enabling Shooting Mechanism
                if (right_trigger && !last_right_trigger) {
                    shooter_enabled = !shooter_enabled;
                    shooter_accel_time.reset();
                    shooter_power = SHOOTER_OFF_POWER;
                    if (!shooter_enabled) {
                        shooterArmed = false;
                    }
                }

                // Swap with max value in case it gets stuck
                if (b_button && !last_b_button) {
                    double tmp = shooter_target_power_swapped;
                    shooter_target_power_swapped = shooter_target_power;
                    shooter_target_power = tmp;
                }

                // && shooter_power != shooter_target_power
                // Shooting
                // NOTE: Motor can NEVER be run counterclockwise (enforced by SHOOTER_MIN_POWER)
                if (shooter_enabled && shooter_accel_time.milliseconds() >= SHOOTER_ACCEL_DELAY && Math.abs(shooter_target_power - shooter_power) > SHOOTER_TOLERANCE) {
                    if (shooter_power > shooter_target_power && shooter_power - SHOOTER_ACCEL >= SHOOTER_MIN_POWER && shooter_target_power >= SHOOTER_MIN_POWER) {
                        shooter_power -= SHOOTER_ACCEL;
                        shooter_accel_time.reset();
                    } else if (shooter_power < shooter_target_power && shooter_power + SHOOTER_ACCEL <= SHOOTER_MAX_POWER && shooter_target_power <= SHOOTER_MAX_POWER) {
                        shooter_power += SHOOTER_ACCEL;
                        shooter_accel_time.reset();
                    }
                }
                shooterMotor.setPower(shooter_power);
                
                // Arming Servo Engaging
                if (shooter_enabled && !right_bumper && last_right_bumper) {
                    shooterArmed = !shooterArmed;
                }

                // Arming Servo Handling
                if (shooterArmed) {
                    armingServo.setPosition(SERVO_OPEN_POS);
                } else {
                    armingServo.setPosition(SERVO_BLOCKING_POS);
                }

                // Telemetry
                telemetry.addData("Pickup Motor Power", pickupMotor.getPower());
                telemetry.addData("Pickup Motor Target Power", pickup_power);
                telemetry.addData("Shooter Motor Power", shooter_power);
                telemetry.addData("Shooter Motor Target Power", shooter_target_power);
                telemetry.addData("Shooter Motor Estimated Distance", distanceFromPower(shooter_target_power));
                telemetry.addData("Shooter Armed", shooterArmed);
                telemetry.update();

                // Keeping Track of Inputs
                last_right_bumper = right_bumper;
                last_right_trigger = right_trigger;
                last_left_trigger = left_trigger;
                last_y_button = y_button;
                last_a_button = a_button;
                last_x_button = x_button;
                last_b_button = b_button;
            }
        }
    }
}
