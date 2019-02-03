package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FrozenFrenzy.HardwareFrenzy;

/**
 * Created by Hannah Rajarao, Team 6037 WAGS on 1/5/2018.
 */

@TeleOp(name = "Vision Test", group = "Test")
public class VisionTest extends OpMode {
    HardwareFrenzy robot = new HardwareFrenzy();

    public void init() {
        robot.init(hardwareMap, false);
    }

    public void loop() {
        telemetry.addData("VuMark", robot.vision.getVuMark());
        telemetry.update();
    }
}
