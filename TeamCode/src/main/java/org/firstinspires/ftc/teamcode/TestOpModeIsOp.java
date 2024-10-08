package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TestOpModeIsOp")
public class TestOpModeIsOp extends LinearOpMode {

    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor BackRight;
    private Servo Grip;
    private DcMotor Intake;
    private Servo Wrist;
    private DcMotor Hanger;
    private DcMotor Slide;

    // Limelight!!
    private Limelight3A limelight;

    // Control hub gyroscope
    private IMU imu;
    private double robotOrientation;

    public double convertAngle(double angle) {
        if (angle < 0) {
            angle += 360;
        }
        return angle % 360;
    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double Power;
        int Alt;
        int numOfPresses;
        int numOfPresses2;
        int tankDrive = 0;
        double runningError = 0;

        FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        BackLeft = hardwareMap.get(DcMotor.class, "Back Left");
        FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
        BackRight = hardwareMap.get(DcMotor.class, "Back Right");
        /* Grip = hardwareMap.get(Servo.class, "GripAsServo");
        Intake = hardwareMap.get(DcMotor.class, "IntakeAsDcMotor");
        Wrist = hardwareMap.get(Servo.class, "WristAsServo");
        Hanger = hardwareMap.get(DcMotor.class, "HangerAsDcMotor");
        Slide = hardwareMap.get(DcMotor.class, "SlideAsDcMotor"); */

        // Limelight boilerplate
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.update();

        // Control hub setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize( // See https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html#orthogonal-mounting
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
            )
        );

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            Alt = 1;
            numOfPresses = 0;
            numOfPresses2 = 0;
            // Put run blocks here.
            while (opModeIsActive()) {
                // Robot angle (yaw) between -180 and 180
                robotOrientation = convertAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Robot angle (yaw): ", robotOrientation);



                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

                // Reset IMU
                if (gamepad1.b) {
                    imu.resetYaw();
                }

                // Limelight
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    Pose3D fieldPos = result.getBotpose_MT2();
                    Pose3D relativePose = result.getBotpose();

                    telemetry.addData("MT2 Botpose: ", fieldPos);
                    telemetry.addData("simple pose: ", relativePose);
                    runningError = convertAngle(convertAngle(relativePose.getOrientation().getYaw()) - robotOrientation);
                }


                // Speed Variables
                if (gamepad1.left_bumper) {
                    Power = .35;
                } else if (gamepad1.right_bumper) {
                    Power = 1;
                } else {
                    Power = 0.75;
                }
                // Drive Mode Select
                // Tank Drive selected by Default
                if (gamepad1.a || tankDrive == 1) {
                    tankDrive = 1;
                    Alt = 0;
                }
                if (gamepad1.y || Alt == 1) {
                    Alt = 1;
                    tankDrive = 0;
                }
                if (tankDrive == 1) {
                    // Tank Drive Code
                    if (gamepad1.dpad_right) {
                        // Crab Right
                        FrontLeft.setPower(-Power);
                        BackLeft.setPower(Power);
                        FrontRight.setPower(-Power);
                        BackRight.setPower(Power);
                    } else if (gamepad1.dpad_left) {
                        // Crab Left
                        FrontLeft.setPower(Power);
                        BackLeft.setPower(-Power);
                        FrontRight.setPower(Power);
                        BackRight.setPower(-Power);
                    } else {
                        // Drive
                        FrontLeft.setPower(Power * gamepad1.left_stick_y);
                        BackLeft.setPower(Power * gamepad1.left_stick_y);
                        FrontRight.setPower(-(Power * gamepad1.right_stick_y));
                        BackRight.setPower(-(Power * gamepad1.right_stick_y));
                    }
                    if (gamepad1.dpad_down) {
                        FrontLeft.setPower(Power * 1);
                        BackLeft.setPower(Power * 1);
                        FrontRight.setPower(-(Power * 1));
                        BackRight.setPower(-(Power * 1));
                    }
                    if (gamepad1.dpad_up) {
                        FrontLeft.setPower(Power * -1);
                        BackLeft.setPower(Power * -1);
                        FrontRight.setPower(-(Power * -1));
                        BackRight.setPower(-(Power * -1));
                    }
                }
                if (Alt == 1) {
                    // Alternative Drive Code

                    // Control hub gyroscope adjustment
                    double x = gamepad1.left_stick_x;
                    double y = gamepad1.left_stick_y;

                    double r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

                    double theta = Math.toDegrees(Math.atan2(y, x));

                    // Note: Add constant to roboOrient. to dictate which way is forward
                    theta += robotOrientation + runningError;

                    if (theta < 0) { theta += 360; }

                    x = r * Math.cos(Math.toRadians(theta));
                    y = r * Math.sin(Math.toRadians(theta));

                    FrontLeft.setPower(Power * (-gamepad1.right_stick_x + (y - x)));
                    BackLeft.setPower(Power * (-gamepad1.right_stick_x + y + x));
                    FrontRight.setPower(-(Power * (gamepad1.right_stick_x + y + x)));
                    BackRight.setPower(-(Power * (gamepad1.right_stick_x + (y - x))));
                    if (gamepad1.dpad_down) {
                        FrontLeft.setPower(Power * 1);
                        BackLeft.setPower(Power * 1);
                        FrontRight.setPower(-(Power * 1));
                        BackRight.setPower(-(Power * 1));
                    }
                    if (gamepad1.dpad_up) {
                        FrontLeft.setPower(Power * -1);
                        BackLeft.setPower(Power * -1);
                        FrontRight.setPower(-(Power * -1));
                        BackRight.setPower(-(Power * -1));
                    }
                    if (gamepad1.dpad_left) {
                        FrontLeft.setPower(Power * 1);
                        BackLeft.setPower(-(Power * 1));
                        FrontRight.setPower(Power * 1);
                        BackRight.setPower(-(Power * 1));
                    }
                    if (gamepad1.dpad_right) {
                        FrontLeft.setPower(-(Power * 1));
                        BackLeft.setPower(Power * 1);
                        FrontRight.setPower(-(Power * 1));
                        BackRight.setPower(Power * 1);
                    }
                }
                telemetry.update();
                // Put loop blocks here.
            }
        }
    }
}