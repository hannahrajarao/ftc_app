package org.firstinspires.ftc.teamcode.FrozenFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Hannah Rajarao, Team 6037 WAGS, on 1/16/2018.
 */

@TeleOp(name = "Teleop2", group = "Frenzy")
//@Disabled
public class TeleopFrenzy extends OpMode{
    HardwareFrenzy robot = new HardwareFrenzy();
    @Override
    public void init() {
        robot.init(hardwareMap, false);
    }

    @Override
    public void loop() {
        //drive
        robot.driveTrain.drive(gamepad1, this);

        //control clamps
        if(gamepad2.a)
            robot.clampIn();
        if(gamepad2.b)
            robot.clampOut();

        //lift glyphs
        robot.pulley.setPower(gamepad2.left_stick_y);

        //relic controls
        robot.relicArm.setPower(gamepad2.right_stick_y); //pulley arm
        if(gamepad2.x)
            robot.rotateRelic(); //rotation of claw
        if(gamepad2.y)
            robot.relicClaw(); //open and close claw
    }
}
