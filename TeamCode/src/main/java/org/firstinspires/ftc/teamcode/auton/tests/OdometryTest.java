package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.PixelManipulation;

@Autonomous(name = "OdomTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    PixelManipulation slides = new PixelManipulation(this);

    @Override
    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap);
        methods.calibrateEncoders();
        methods.resetIMU(hardwareMap);

        waitForStart();
        if (opModeIsActive()){
            methods.driveY(50);
            methods.rotateToHeading(90);
            slides.SanderArm(700);
            methods.rotateToHeading(0);
            methods.driveY(50);
            telemetry.update();
            sleep(30000);
        }
    }
}