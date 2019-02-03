package org.firstinspires.ftc.teamcode.FrozenFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Hannah Rajarao, Team 6037 WAGS, on 1/18/2018.
 */

@TeleOp(name = "Teleop1", group = "Frenzy")
//@Disabled
public class TeleopFrenzy1 extends OpMode {
    HardwareFrenzy robot = new HardwareFrenzy();
    @Override
    public void init() {
        robot.init(hardwareMap, false);
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger>0) {
            robot.pulley.setPower(gamepad1.right_stick_y);
            robot.driveTrain.setPower(gamepad1.left_stick_y);
        } else
            robot.driveTrain.drive(gamepad1, this);

        if(gamepad1.a)
            robot.clampIn();
        if(gamepad1.b)
            robot.clampOut();

        if(gamepad1.right_bumper)
            robot.jewelUp();

        if(robot.jewel.getPosition() != robot.JEWEL_UP)
            robot.jewelUp();
    }
}
