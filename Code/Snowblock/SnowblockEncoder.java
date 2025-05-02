package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Snowblock_encoder (Java)")
public class SnowblockEncoder extends LinearOpMode {
  // DriveTrain
  private DcMotor leftMotor;
  private DcMotor rightMotor;
  private static final double LEFT_SPEED = 0.7;
  private static final double RIGHT_SPEED = 0.7;
  private static final double TURN_COEFFICIENT = 0.6;
  private static final double SPIN_COEFFICIENT = 0.5;
  private double drive_power;
  private double drive_turn;
  private double drive_spin;

  private void driveMotors(double left_power, double right_power) {
    leftMotor.setPower(left_power * LEFT_SPEED); // Reverse direction
    rightMotor.setPower(-right_power * RIGHT_SPEED);
  }

  // Arm
  private DcMotorEx armMotor;
  private int arm_target_pos; // in ticks
  private double arm_height;
  private static final double TICKS_PER_REV = 1464; // For Studica Maverick
  private static final double ARM_GEAR_RATIO = 4 / 1;
  private static final double ARM_TICKS_PER_REV = TICKS_PER_REV * ARM_GEAR_RATIO;
  private static final double TICKS_PER_DEGREE = ARM_TICKS_PER_REV / 360;
  private static final double DEGREES_PER_TICK = 360 / ARM_TICKS_PER_REV;
  private static final double START_ANGLE = 62.5;
  private static final double ARM_PIVOT_HEIGHT = 149; // mm
  private static final double ARM_LENGTH = 205; // mm
  private static final double ARM_BOTTOM_OFFSET = 114.5;
  private static final double ARM_TOP_VEL = (100 / 60) * TICKS_PER_REV; // 100 RPM -> Ticks Per Second
  private static final double ARM_SPEED = 0.7;

  private double armAngleToPos(double deg) {
    return -((deg - START_ANGLE) * TICKS_PER_DEGREE);
  }

  private double getArmAngle() {
    return -(armMotor.getCurrentPosition() * DEGREES_PER_TICK) + START_ANGLE;
  }

  private double getArmHeight() {
    return Math.sin(Math.toRadians(getArmAngle())) * ARM_LENGTH + ARM_PIVOT_HEIGHT - ARM_BOTTOM_OFFSET;
  }

  private void setArmPos() {
    armMotor.setTargetPosition(arm_target_pos);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setVelocity(ARM_TOP_VEL * ARM_SPEED);
  }

  private static final double ARM_HEIGHT_INTERVAL = 2.5 * 25.4; // inches -> mm
  private static final double ARM_DEG_INTERVAL = Math.toDegrees(Math.asin(ARM_HEIGHT_INTERVAL/ARM_LENGTH));
  private static final int ARM_TICK_INTERVAL = (int)(ARM_DEG_INTERVAL * TICKS_PER_DEGREE);
  private static final int ARM_MAX_POS = 985; // Closest to the ground
  private static final int ARM_MIN_POS = -1400; // Closest to the ground
  private static final int ARM_START_POS = (int)(ARM_MAX_POS - (int)((double)ARM_MAX_POS / ARM_TICK_INTERVAL) * ARM_TICK_INTERVAL);

  // Input Trackers
  private boolean dpad_up;
  private boolean dpad_down;
  private boolean last_dpad_up;
  private boolean last_dpad_down;
  
  private boolean target_in_bounds(int target) {
    return target >= ARM_MIN_POS && target <= ARM_MAX_POS;
  }

  @Override
  public void runOpMode() {
    // Init
    leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
    rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
    armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
    armMotor.setDirection(DcMotor.Direction.REVERSE);

    // Input Trackers
    dpad_up = false;
    dpad_down = false;
    last_dpad_up = false;
    last_dpad_down = false;

    // Encoder Reset
    // NOTE: make sure arm starts at correct angle
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_target_pos = ARM_START_POS;
    //TODO: remove
    while (!opModeIsActive()) {
      telemetry.addData("Arm Height", getArmHeight());
      telemetry.addData("Arm Position", armMotor.getCurrentPosition());
      telemetry.addData("Arm Angle", getArmAngle());
      telemetry.addData("Arm Target Position", arm_target_pos);
      telemetry.addData("Arm Deg Interval", ARM_DEG_INTERVAL);
      telemetry.addData("Ticks per degree", TICKS_PER_DEGREE);
      telemetry.addData("Arm Tick Interval", ARM_TICK_INTERVAL);
      telemetry.update();
    }
    waitForStart();
    setArmPos();
    armMotor.setVelocity(ARM_TOP_VEL * ARM_SPEED);
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Input Updating
        dpad_up = gamepad1.dpad_up;
        dpad_down = gamepad1.dpad_down;

        // Driving
        drive_power = gamepad1.right_stick_y;
        drive_turn = gamepad1.right_stick_x * TURN_COEFFICIENT;
        if (gamepad1.left_bumper) {
          drive_spin = drive_turn / TURN_COEFFICIENT * SPIN_COEFFICIENT;
          driveMotors(-drive_spin, drive_spin);
        } else {
          driveMotors(drive_power - drive_turn, drive_power + drive_turn);
        }

        if (dpad_up && !last_dpad_up && target_in_bounds(arm_target_pos - ARM_TICK_INTERVAL)) {
          arm_target_pos -= ARM_TICK_INTERVAL;
          setArmPos();
        } else if (dpad_down && ! last_dpad_down && target_in_bounds(arm_target_pos + ARM_TICK_INTERVAL)) {
          arm_target_pos += ARM_TICK_INTERVAL;
          setArmPos();
        }

        if (gamepad1.right_bumper) {
          arm_target_pos = 0;
          setArmPos();
        } else if (gamepad1.a) {
          arm_target_pos = ARM_MAX_POS;
        	setArmPos();
        }

        // Telemetry
        telemetry.addData("Arm Height", getArmHeight());
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        telemetry.addData("Arm Angle", getArmAngle());
        telemetry.addData("Arm Target Position", arm_target_pos);
        telemetry.addData("Arm Tick Interval", ARM_TICK_INTERVAL);
        telemetry.addData("Arm At Target", !armMotor.isBusy());
        telemetry.update();

        last_dpad_up = dpad_up;
        last_dpad_down = dpad_down;
      }
    }
  }
}
