/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name = "Test: Gyro", group = "Test")
//@Disabled

public class Gyro extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    // Declare DcMotor members
    private DcMotor Motor_Left = null;      // DcMotors for left wheels
    private DcMotor Motor_Right = null;     // DcMotors for right wheels
    private DcMotor Motor_Convey = null;    // DcMotors to pick up boxes
    private DcMotor Motor_Elevator = null;  // DcMotor to elevate boxes

    // Declare Servo members
    private Servo Servo_Color1 = null;      // Servo to control the sensor arm (up & down)
    private Servo Servo_Color2 = null;      // Servo to control the sensor arm (forward & backward)
    private Servo Servo_Hand = null;       // Servo to control the craw hand (forward & backward)
    private Servo Servo_Craw = null;        // Servo to control the craw (open & close)
    private Servo Servo_Switch = null;      // Servo to control the switch (on & off)

    // Declare variables
    double LeftPower;                       // Power for left motors
    double RightPower;                      // Power for right motors
    double ConveyPower;                     // Power for convey motors
    double Convey_Offset = 0.3;     // Offset for initial speed of convey motors
    double ElevatorPower;                   // Power for an elevator motor
    double CrawPower;                       // Power for craw control
    double Speed_Offset = 1.00;             // Set the maximum power range of wheel motors
    double Switch_offPosition = 0.5;        // Set the servo position during stacking
    double Switch_onPosition = 0;           // Set the servo position during relic recovery
    double Servo_Color1_Position;
    double Servo_Color2_Position;
    double Servo_HandPosition = 0.5;
    double Servo_CrawPosition = 1.0;
    boolean Relic_Mode = false;             // Initial status for OpMode

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Call hardware
        Motor_Left = hardwareMap.get(DcMotor.class, "Motor_L");
        Motor_Right = hardwareMap.get(DcMotor.class, "Motor_R");
        Motor_Convey = hardwareMap.get(DcMotor.class, "Motor_C");
        Motor_Elevator = hardwareMap.get(DcMotor.class, "Motor_E");

        Servo_Color1 = hardwareMap.get(Servo.class, "Servo_C1");
        Servo_Color2 = hardwareMap.get(Servo.class, "Servo_C2");
        Servo_Hand = hardwareMap.get(Servo.class, "Servo_H");
        Servo_Craw = hardwareMap.get(Servo.class, "Servo_C");
        Servo_Switch = hardwareMap.get(Servo.class, "Servo_S");

        // Set directions of DcMotors
        Motor_Left.setDirection(DcMotor.Direction.FORWARD);
        Motor_Right.setDirection(DcMotor.Direction.REVERSE);
        Motor_Convey.setDirection(DcMotor.Direction.FORWARD);
        Motor_Elevator.setDirection(DcMotor.Direction.REVERSE);

        // Set positions of Servos
        Servo_Color1_Position = Servo_Color1.getPosition();
        Servo_Color2_Position = Servo_Color2.getPosition();
        Servo_Color1.setPosition(Servo_Color1_Position);
        Servo_Switch.setPosition(Switch_offPosition);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Select the roles of the controllers
            double drive = gamepad1.left_stick_y;   // Set left Y stick of Gamepad1 to go forward & backward
            double turn = -gamepad1.right_stick_x; // Set right X stick of Gamepad1 to turn
            double convey = gamepad2.left_stick_y;   // Set left Y stick of Gamepad2 to pick the box up
            double elevator = gamepad2.right_stick_y;  // Set right Y stick of Gamepad2 to move the box up & down
            double craw = gamepad2.left_trigger;   // Set left trigger of Gamepad2 to open & close the craw

            // Select the maximum power of DcMotor
            if (gamepad1.a) {
                Speed_Offset = 1;       // Set the speed when hit 'a' button of Gamepad1
            } else if (gamepad1.x) {
                Speed_Offset = 0.60;    // Set the speed when hit 'x' button of Gamepad1
            } else if (gamepad1.b) {
                Speed_Offset = 0.35;    // Set the speed when hit 'b' button of Gamepad1
            }

            // Select the maximum power of DcMotor
            if (gamepad2.a) {
                Convey_Offset = 0.28;     // Set the speed when hit 'a' button of Gamepad2
            } else if (gamepad2.b) {
                Convey_Offset = 0.1;     // Set the speed when hit 'b' button of Gamepad2
            }

            // Select between the stacking mode and the relic recovery mode
            if (gamepad2.x) {
                Relic_Mode = true;      // Start the Relic mode when hit 'x' button of Gamepad2
            } else if (gamepad2.y) {
                Relic_Mode = false;     // Stop the Relic mode when hit 'y' button of Gamepad2
            }

            // Set the range of power for each DcMotor
//            LeftPower = Range.clip((drive + turn) * Speed_Offset, -1.0, 1.0);
//            RightPower = Range.clip((drive - turn) * Speed_Offset, -1.0, 1.0);
            ConveyPower = Range.clip(convey, -1.0, 1.0);
            ElevatorPower = Range.clip(elevator, -1.0, 1.0);
            CrawPower = Range.clip(craw, -1.0, 1.0);

            // Send calculated power to DcMotors
            Motor_Left.setPower(LeftPower);
            Motor_Right.setPower(RightPower);
            Motor_Convey.setPower(ConveyPower);
            Servo_Color1.setPosition(Servo_Color1_Position);
            Servo_Hand.setPosition(Servo_HandPosition);
            Servo_Craw.setPosition(Servo_CrawPosition);

            if (Relic_Mode == false) {
                Servo_Switch.setPosition(Switch_offPosition); // Turn the switch off
                Motor_Elevator.setPower(ElevatorPower);
                Motor_Convey.setPower(ConveyPower * Convey_Offset);
            } else if (Relic_Mode == true) {
                Servo_Switch.setPosition(Switch_onPosition);  // Turn the switch on
                Servo_HandPosition = (ElevatorPower + 1) / 2;
                Servo_CrawPosition = 0.58 + CrawPower * 0.5;
                Motor_Convey.setPower(ConveyPower);
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            if (angles.firstAngle > 45) {
                LeftPower = Range.clip((drive + turn) * Speed_Offset, -1.0, 1.0);
                RightPower = Range.clip((drive - turn) * Speed_Offset, -1.0, 1.0);
            } else {
                LeftPower = 0;
                RightPower = 0;
            }


            // Show information of game time, wheel power, and so on.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", LeftPower, RightPower);
            telemetry.addData("Convey", "power (%.2f)", ConveyPower);
            telemetry.addData("Elevator", "power (%.2f)", ElevatorPower);
            telemetry.addData("Hand", "position (%.3f)", Servo_HandPosition);
            telemetry.addData("Craw", "position (%.3f)", Servo_CrawPosition);
            telemetry.addData("Craw", "power (%.2f)", CrawPower);



            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override
                        public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override
                        public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel * gravity.xAccel
                                            + gravity.yAccel * gravity.yAccel
                                            + gravity.zAccel * gravity.zAccel));
                        }
                    });




            telemetry.update();
        }
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
