package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Snowblock_default (Java)")
public class SnowblockDefault extends LinearOpMode {
  // DriveTrain
  private DcMotor leftMotor;
  private DcMotor rightMotor;
  private static final double LEFT_SPEED = 1.0;
  private static final double RIGHT_SPEED = 1.0;
  private static final double TURN_COEFFICIENT = 0.7;
  private static final double SPIN_COEFFICIENT = 0.5;
  private double drive_power;
  private double drive_turn;
  private double drive_spin;

  private void driveMotors(double left_power, double right_power) {
    leftMotor.setPower(-left_power * LEFT_SPEED); // Reverse direction
    rightMotor.setPower(right_power * RIGHT_SPEED);
  }

  // Arm
  private DcMotorEx armMotor;
  private double arm_target_pos; // in ticks
  private double arm_height;
  private static final double TICKS_PER_REV = 1464; // For Studica Maverick
  private static final double ARM_GEAR_RATIO = 1 / 4;
  private static final double TICKS_PER_DEGREE = TICK_PER_REV / 360 / ARM_GEAR_RATIO;
  private static final double DEGREES_PER_TICK = 360 / TICKS_PER_REV * ARM_GEAR_RATIO;
  private static final double START_ANGLE = 62.5;
  private static final double ARM_PIVOT_HEIGHT = 149; // mm
  private static final double ARM_LENGTH = 205; // mm
  private static final double ARM_BOTTOM_OFFSET = 114.5;
  private static final double ARM_SPEED = 0.7;
  private static final double ARM_TOP_SPEED = (100 / 60) * TICKS_PER_REV; // 100 RPM -> Ticks Per Second

  private double armAngleToPos(double deg) {
    return -((deg - START_ANGLE) * TICKS_PER_DEGREE);
  }

  private double getArmAngle() {
    return -(armMotor.getCurrentPosition() * DEGREES_PER_TICK) + START_ANGLE;
  }

  private double getArmHeight() {
    return Math.sin(Math.to_radians(getArmAngle())) * ARM_LENGTH + ARM_PIVOT_HEIGHT - ARM_BOTTOM_OFFSET;
  }

  private double setArmPos(double pos) {
    armMotor.setTargetPosition(pos);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private static final double ARM_HEIGHT_INTERVAL = 2.5 * 25.4; // inches -> mm
  private static final double ARM_DEG_INTERVAL = Math.asin(ARM_HEIGHT_INTERVAL/ARM_LENGTH);
  private static final double ARM_TICK_INTERVAL = ARM_DEG_INTERVAL * TICKS_PER_DEGREE;

  // Input Trackers
  private boolean left_up;
  private boolean left_down;
  private boolean last_left_up;
  private boolean last_left_down;

  @Override
  public void runOpMode() {
    // Init
    leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
    rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
    armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

    // Input Trackers
    left_up = false;
    left_down = false;
    last_left_up = false;
    last_left_down = false;

    // Encoder Reset
    // NOTE: make sure arm starts at correct angle
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_target_pos = 0;

    waitForStart();
    setArmPos(arm_target_pos);
    armMotor.setVelocity(TOP_ARM_VEL * ARM_SPEED);
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Input Updating
        left_up = gamepad1.left_stick_y < -0.5;
        left_down = gamepad1.left_stick_y > 0.5;

        // Driving
        drive_power = gamepad1.right_stick_y;
        drive_turn = gamepad1.right_stick_x * TURN_COEFFICIENT;
        if (gamepad1.left_bumper) {
          drive_spin = drive_turn / TURN_COEFFICIENT * SPIN_COEFFICIENT;
          driveMotors(-drive_spin, drive_spin);
        } else {
          driveMotors(drive_power - drive_turn, drive_power + drive_turn);
        }

        // TODO: add max speed?
        // TODO: add limits?
				if (left_up && !last_left_up) {
				  arm_target_pos += ARM_TICK_INTERVAL;
				  setArmPos(arm_target_pos);
				} else if (left_down && ! last_left_down) {
				  arm_target_pos -= ARM_TICK_INTEVAL;
				  setArmPos(arm_target_pos);
				}

				if (gamepad1.right_bumper) {
          armMotor.setVelocity(TOP_ARM_VEL * ARM_SPEED);
				  setArmPos(0);
				}

        // Telemetry
        telemetry.addData("Arm Height", getArmHeight());
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        telemetry.addData("Arm Angle", getArmAngle());
        telemetry.addData("Arm Target Position", arm_target_pos);
        telemetry.addData("Arm At Target", !armMotor.isBusy());
        telemetry.update();

        last_left_up = left_up;
        last_left_down = left_down;
      }
    }
  }
}
