package org.firstinspires.ftc.teamcode.PostSeason;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Hannah Rajarao, Team 6037 WAGS
 */

@TeleOp(name = "May 19", group = " ")
public class Teleop0518_1 extends OpMode {
    Hardware0518 robot = new Hardware0518();
    boolean up = true;

    public void init() {
        robot.init(hardwareMap);
    }

    public void loop() {
        robot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,
                gamepad1.right_stick_y, gamepad1.right_trigger, this);

        //left and right triggers control intake wheels
        //right bumper flipper back and forth
        //left bumper holder

        robot.intakeLeft.setPower(gamepad2.left_trigger);
        robot.intakeRight.setPower(gamepad2.right_trigger);

        if(gamepad1.a || gamepad2.a) {
            robot.intakeLeft.setPower(1);
            robot.intakeRight.setPower(1);
        }

        if(gamepad1.x || gamepad2.x) {
            robot.intakeLeft.setPower(-1);
            robot.intakeRight.setPower(-1);
        }
        if(gamepad1.b || gamepad2.b) {
            robot.intakeLeft.setPower(0);
            robot.intakeRight.setPower(0);
        }

        if(gamepad1.left_bumper)
            robot.holder.setPosition(0);

        if(gamepad1.right_bumper)
            robot.holder.setPosition(1);

        double flipperPower = -gamepad2.left_stick_y * 0.75;
        robot.flipper.setPower(flipperPower);
        telemetry.addData("flipper", flipperPower);


        if(gamepad1.y)
            robot.flipper.setPower(-gamepad1.left_trigger);
        else
            robot.flipper.setPower(gamepad1.left_trigger);

    }
}
