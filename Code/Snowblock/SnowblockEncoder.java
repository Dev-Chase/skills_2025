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
  private static final double ARM_SPEED = 0.4;

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
  private static final int ARM_MAX_POS = 960; // Closest to the ground
  private static final int ARM_MIN_POS = -1400; // Closest to the ground
  private static final int ARM_START_POS = (int)(ARM_MAX_POS - (int)((double)ARM_MAX_POS / ARM_TICK_INTERVAL) * ARM_TICK_INTERVAL);

  // Arm Position Tuning
  private static final int ARM_TUNE_DELTA = (int)(TICKS_PER_DEGREE * 3.0f);
  private static final double ARM_TUNE_DELAY = 100;
  private static final double ARM_TUNE_THRESHOLD = 0.5;
  private ElapsedTime arm_timer;

  // Claw
  private Servo leftClawServo;
  private Servo rightClawServo;
  private static final double LEFT_MAX_POS = 0.87;
  private static final double LEFT_MIN_POS = 0.65; // Open Position
  private static final double RIGHT_MAX_POS = 0.57; // Open Position
  private static final double RIGHT_MIN_POS = 0.35;
  private static final double LEFT_OPEN_POS = LEFT_MIN_POS;
  private static final double RIGHT_OPEN_POS = RIGHT_MAX_POS;
  private static final double LEFT_CLOSED_POS = LEFT_MAX_POS;
  private static final double RIGHT_CLOSED_POS = RIGHT_MIN_POS;
  private static final double COEFFICIENT_3 = 0.35; // 35% closed
  private static final double COEFFICIENT_6 = 0.7; // 70% closed
  private static final double LEFT_3_POS = LEFT_MIN_POS + COEFFICIENT_3 * (LEFT_MAX_POS - LEFT_MIN_POS);
  private static final double RIGHT_3_POS = RIGHT_MAX_POS - COEFFICIENT_3 * (RIGHT_MAX_POS - RIGHT_MIN_POS);
  private static final double LEFT_6_POS = LEFT_MIN_POS + COEFFICIENT_6 * (LEFT_MAX_POS - LEFT_MIN_POS);
  private static final double RIGHT_6_POS = RIGHT_MAX_POS - COEFFICIENT_6 * (RIGHT_MAX_POS - RIGHT_MIN_POS);
  private static final double CLAW_TUNE_DELTA = 0.02;
  private static final double CLAW_TUNE_DELAY = 100; // ms
  private double right_claw_pos;
  private double left_claw_pos;
  private ElapsedTime claw_timer;
  private static final double[] left_positions = {LEFT_OPEN_POS, LEFT_3_POS, LEFT_6_POS, LEFT_CLOSED_POS};
  private static final double[] right_positions = {RIGHT_OPEN_POS, RIGHT_3_POS, RIGHT_6_POS, RIGHT_CLOSED_POS};

  enum ClawSide {
    LEFT,
    RIGHT
  };

  enum ClawPosition {
    OPEN(0),
    SEP_3(1),
    SEP_6(2),
    CLOSE(3);

    private final int ind;

    ClawPosition(int ind) {
        this.ind = ind;
    }

    public int getIndex() {
        return ind;
    }

    public double getPos(ClawSide side) {
      return (side == ClawSide.LEFT) ? left_positions[this.ind]:right_positions[this.ind];
    }
  }

  private void setClawPos(ClawPosition pos) {
    left_claw_pos = pos.getPos(ClawSide.LEFT);
    right_claw_pos = pos.getPos(ClawSide.RIGHT);
  }

  private boolean clawChangeWithinBounds(double left_delta) {
    double new_left_pos = left_claw_pos + left_delta;
    double new_right_pos = right_claw_pos - left_delta;
    return new_left_pos >= LEFT_MIN_POS && new_left_pos <= LEFT_MAX_POS && new_right_pos >= RIGHT_MIN_POS && new_right_pos <= RIGHT_MAX_POS;
  }
  
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
    leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
    rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");

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
    claw_timer = new ElapsedTime();
    arm_timer = new ElapsedTime();
    setArmPos();
    setClawPos(ClawPosition.CLOSED);
    right_claw_pos = RIGHT_MAX_POS;
    left_claw_pos = LEFT_MIN_POS;
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Input Updating
        dpad_up = gamepad1.dpad_up;
        dpad_down = gamepad1.dpad_down;

        // Driving
        drive_power = gamepad1.right_stick_y;
        drive_turn = gamepad1.right_stick_x * TURN_COEFFICIENT;
        if (gamepad1.right_bumper) {
          drive_spin = drive_turn / TURN_COEFFICIENT * SPIN_COEFFICIENT;
          driveMotors(-drive_spin, drive_spin);
        } else {
          driveMotors(drive_power - drive_turn, drive_power + drive_turn);
        }

        // Arm Positioning
        if (dpad_up && !last_dpad_up && target_in_bounds(arm_target_pos - ARM_TICK_INTERVAL)) {
          arm_target_pos -= ARM_TICK_INTERVAL;
          setArmPos();
        } else if (dpad_down && ! last_dpad_down && target_in_bounds(arm_target_pos + ARM_TICK_INTERVAL)) {
          arm_target_pos += ARM_TICK_INTERVAL;
          setArmPos();
        }

        // Resetting Arm Position
        if (gamepad1.y) {
          arm_target_pos = 0;
          setArmPos();
        } else if (gamepad1.a) {
          arm_target_pos = ARM_MAX_POS;
            setArmPos();
        }

        // Arm Tuning
        if (arm_timer.milliseconds() >= ARM_TUNE_DELAY) {
          if (gamepad1.left_bumper && Math.abs(gamepad1.left_stick_y) > ARM_TUNE_THRESHOLD) {
            int direction = gamepad1.left_stick_y > 0 ? 1 : -1;
            if (target_in_bounds(arm_target_pos + (direction * ARM_TUNE_DELTA))) {
              arm_target_pos += direction * ARM_TUNE_DELTA;
            }

            setArmPos();
            arm_timer.reset();
          }
        }
        
        // Claw Autopositioning

        // Claw Tuning
        if (claw_timer.milliseconds() > CLAW_TUNE_DELAY) {
          if (gamepad1.left_trigger > 0.5 && clawChangeWithinBounds(-CLAW_TUNE_DELTA)) {
            left_claw_pos -= CLAW_TUNE_DELTA;
            right_claw_pos += CLAW_TUNE_DELTA;
            claw_timer.reset();
          } else if (gamepad1.right_trigger > 0.5 && clawChangeWithinBounds(CLAW_TUNE_DELTA)) {
            left_claw_pos += CLAW_TUNE_DELTA;
            right_claw_pos -= CLAW_TUNE_DELTA;
            claw_timer.reset();
          }
        }

        // Setting Claw Position
        leftClawServo.setPosition(left_claw_pos);
        rightClawServo.setPosition(right_claw_pos);

        // Telemetry
        telemetry.addData("Testing Button", gamepad1.x);
        telemetry.addData("Arm Height", getArmHeight());
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        telemetry.addData("Arm Angle", getArmAngle());
        telemetry.addData("Arm Target Position", arm_target_pos);
        telemetry.addData("Arm Tick Interval", ARM_TICK_INTERVAL);
        telemetry.addData("Arm Tune Delta (ticks)", ARM_TUNE_DELTA);
        telemetry.addData("Arm At Target", !armMotor.isBusy());
        telemetry.addData("Arm Tuning Clock", arm_timer.seconds());
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.update();

        last_dpad_up = dpad_up;
        last_dpad_down = dpad_down;
      }
    }
  }
}
