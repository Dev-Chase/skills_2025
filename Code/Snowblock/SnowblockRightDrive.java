package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Snowblock_right_drive (Java)")
public class SnowblockRightDrive extends LinearOpMode {
  // DriveTrain
  private DcMotor leftMotor;
  private DcMotor rightMotor;
  private static final double LEFT_SPEED = 0.7;
  private static final double RIGHT_SPEED = 0.7;
  private static final double TURN_COEFFICIENT = 0.7;
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
  private static final double START_ANGLE = 62.5;
  private static final double ARM_LENGTH = 205; // mm
  private static final double ARM_SPEED = 0.7;

  // Claw
  private Servo leftClawServo;
  private Servo rightClawServo;
  // TODO: determine positions
  private static final double LEFT_MAX_POS = 0.87;
  private static final double LEFT_MIN_POS = 0.65;
  private static final double RIGHT_MAX_POS = 0.57;
  private static final double RIGHT_MIN_POS = 0.35;
  private static final double CLAW_TUNE_DELTA = 0.02;
  private static final double CLAW_TUNE_DELAY = 100; // ms
  private double right_claw_pos;
  private double left_claw_pos;
  private ElapsedTime claw_timer;

  private boolean clawChangeWithinBounds(double left_delta) {
    double new_left_pos = left_claw_pos + left_delta;
    double new_right_pos = right_claw_pos - left_delta;
    return new_left_pos >= LEFT_MIN_POS && new_left_pos <= LEFT_MAX_POS && new_right_pos >= RIGHT_MIN_POS && new_right_pos <= RIGHT_MAX_POS;
  }

  @Override
  public void runOpMode() {
    // Init
    leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
    rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
    armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
    leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
    rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");

    waitForStart();
    claw_timer = new ElapsedTime();
    right_claw_pos = RIGHT_MAX_POS;
    left_claw_pos = LEFT_MIN_POS;
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Driving
        drive_power = gamepad1.right_stick_y;
        drive_turn = gamepad1.right_stick_x * TURN_COEFFICIENT;
        if (gamepad1.left_bumper) {
          drive_spin = drive_turn / TURN_COEFFICIENT * SPIN_COEFFICIENT;
          driveMotors(-drive_spin, drive_spin);
        } else {
          driveMotors(drive_power - drive_turn, drive_power + drive_turn);
        }

        // Claw
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
        leftClawServo.setPosition(left_claw_pos);
        rightClawServo.setPosition(right_claw_pos);

        armMotor.setPower(-gamepad1.left_stick_y * ARM_SPEED);

        // Telemetry
        telemetry.addData("Left Claw Position", leftClawServo.getPosition());
        telemetry.addData("Right Claw Position", rightClawServo.getPosition());
        telemetry.addData("Left Claw Target Position", left_claw_pos);
        telemetry.addData("Right Claw Target Position", right_claw_pos);
        telemetry.addData("Arm Speed", armMotor.getPower());
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        
        telemetry.update();
      }
    }
  }
}
