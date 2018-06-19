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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Final: Auto_Red_S", group = "Final")
//@Disabled

public class Auto_Red_S extends LinearOpMode {
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

    // Declare variables for sensing color
    String Team_Color       = "Red";        // "Blue" or "Red"
    int    rpc              = 1440;         // Value for one rotation
    double Servo_C1_Initial = 0.38;         // Initial position of Servo_Color1
    double Servo_C1_Sensing = 0.95;         // Sensing position of Servo_Color1
    double Servo_C1_Final   = 0.36;         // Final position of Servo_Color1
    double Servo_C2_Initial = 0.30;         // Initial position of Servo_Color2
    double Servo_C2_Left    = 0.15;         // Left position of Servo_Color2
    double Servo_C2_Right   = 0.45;         // Right position of Servo_Color2
    double Color_Value_Red  = 0;            // Color sensor initial value for red
    double Color_Value_Blue = 0;            // Color sensor initial value for blue

    // Declare variables for pictogram
    double Picto_Power        = 0.25;       // Motor power 0.25
    double Dropbox_Power      = 1.00;       // Motor power 1.00
    double Picto_Turn_Left    = 1.520;      // Motor rotation for Left Turn 1.520
    double Picto_Turn_Right   = 1.540;      // Motor rotation for Right Turn 1.540
    double Picto_Go;                        // Initial motor rotation

    double Picto_Middle  = 2.000;           // Motor rotation for Middle 2.000
    double Picto_Left    = 1.350;           // Motor rotation for Left 1.350
    double Picto_Center  = 0.760;           // Motor rotation for Center 0.760
    double Picto_Right   = 0.180;           // Motor rotation for Right 0.180
    double Picto_Adjust  = 0.450;           // Motor rotation for Adjust
    double Picto_Push    = 0.300;           // Motor rotation for Push

    long Picto_Left_Decision   = 0;         // Picto sensor initial value for Left
    long Picto_Center_Decision = 0;         // Picto sensor initial value for Center
    long Picto_Right_Decision  = 0;         // Picto sensor initial value for Right

    double Switch_offPosition = 0.5;        // Set the servo position during stacking
    double Servo_HandPosition = 0.5;
    double Servo_CrawPosition = 1.0;

    long Stop_Time    = 500;                // Stop time (ms)
    long Dropbox_Time = 500;                // Dropbox time (ms)

    // Declare Color Sensor members
    NormalizedColorSensor colorSensor;
    View relativeLayout;

    // Declare Vuforia members
    public static final String TAG = "Vuforia VuMark Sample";
    VuforiaLocalizer vuforia;

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Call hardware
        Motor_Left = hardwareMap.get(DcMotor.class, "Motor_L");
        Motor_Right = hardwareMap.get(DcMotor.class, "Motor_R");
        Motor_Convey = hardwareMap.get(DcMotor.class, "Motor_C");
        Motor_Elevator = hardwareMap.get(DcMotor.class, "Motor_E");
        Servo_Color1 = hardwareMap.get(Servo.class, "Servo_C1");
        Servo_Color2 = hardwareMap.get(Servo.class, "Servo_C2");
        Servo_Hand      = hardwareMap.get(Servo.class, "Servo_H");
        Servo_Craw      = hardwareMap.get(Servo.class, "Servo_C");
        Servo_Switch    = hardwareMap.get(Servo.class, "Servo_S");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "Sensor_C");
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Set Motor Encoders
        Motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set directions of DcMotors
        Motor_Left.setDirection(DcMotor.Direction.REVERSE);
        Motor_Right.setDirection(DcMotor.Direction.FORWARD);
        Motor_Convey.setDirection(DcMotor.Direction.REVERSE);
        Motor_Elevator.setDirection(DcMotor.Direction.FORWARD);

        try {
            // Initialize the positions of Servos
            Servo_Color1.setPosition(Servo_C1_Initial);
            Servo_Color2.setPosition(Servo_C2_Initial);
            Servo_Hand.setPosition(Servo_HandPosition);
            Servo_Craw.setPosition(Servo_CrawPosition);
            Servo_Switch.setPosition(Switch_offPosition);

            waitForStart();   // Wait for the start button to be pressed

            runColorSensor(); // Execute the ColorSensor task
            runVuforia();     // Execute the Vuforia task
            runDropBox();     // Execute the Dropbox task
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }

    protected void runColorSensor() throws InterruptedException {
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        sleep(Stop_Time);
        Servo_Color1.setPosition(Servo_C1_Sensing);
        sleep(Stop_Time);

        for (int i = 0; i < 5; i++) {
            // Read the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);

            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));

            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            telemetry.update();

            // convert the RGB values to HSV values.
            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            Color_Value_Red = Color_Value_Red + colors.red;
            Color_Value_Blue = Color_Value_Blue + colors.blue;
            sleep(Stop_Time);
        }

        if (Color_Value_Red > Color_Value_Blue) {
            telemetry.addLine("Red")
                     .addData("Blue", "%.3f", Color_Value_Blue)
                     .addData("Red", "%.3f", Color_Value_Red);
            telemetry.update();
            if (Team_Color.equals("Blue")) {
                HitBall(Servo_C2_Right, Servo_C2_Right);
            } else if (Team_Color.equals("Red")) {
                HitBall(Servo_C2_Left, Servo_C2_Right);
            }
        } else if (Color_Value_Red < Color_Value_Blue) {
            telemetry.addLine("Blue")
                    .addData("Blue", "%.3f", Color_Value_Blue)
                    .addData("Red", "%.3f", Color_Value_Red);
            telemetry.update();
            if (Team_Color.equals("Blue")) {
                HitBall(Servo_C2_Left, Servo_C2_Right);
            } else if (Team_Color.equals("Red")) {
                HitBall(Servo_C2_Right, Servo_C2_Right);
            }
        }
    }

    protected void runVuforia() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AeVGOdj/////AAAAGYDMTqUG0U/wpR640DA/mLE8kY7NW3lKq/PI5Jes5dJ2W73rcQ3QQevrN7Y4GDzXyKI6d2m6HFSibELcc1klm/qo24ZS5tlYiMlrb5hIX02DO5hgINBcy23/gPiwbMg5FcPGamk5q9Rjn47iPQAzFE8JXLgBUJ7S21o282xY2SZ3b9xDZ8iHl8KVmv6zgD8FC9EeG3PYd+py1vgxAPrppJbxG98FIaxP0D0IUPiJMhsEIlSab8NJ8PVlht79AFIKX2B4Av9qgy6JKz1esiaPsPUF1vHzi9Z5F3Zr8tiju1EioenJdBPLXbuvLHHWEbqBa6H7uKgp/yIjQV680UIM071DCbDpojLOWGnuhXokUwUx";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();
        ElapsedTime eTime = new ElapsedTime();
        eTime.reset();

        while (eTime.time() < 2) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                Picto_Left_Decision = Picto_Left_Decision + 1;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                Picto_Center_Decision = Picto_Center_Decision + 1;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                Picto_Right_Decision = Picto_Right_Decision + 1;
            }
        }

        if (Picto_Left_Decision > Picto_Center_Decision && Picto_Left_Decision > Picto_Right_Decision) {
            Picto_Go = Picto_Left;
        } else if (Picto_Center_Decision > Picto_Left_Decision && Picto_Center_Decision > Picto_Right_Decision) {
            Picto_Go = Picto_Center;
        } else if (Picto_Right_Decision > Picto_Left_Decision && Picto_Right_Decision > Picto_Center_Decision) {
            Picto_Go = Picto_Right;
        } else {
            Picto_Go = Picto_Center;
        }
    }

    protected void runDropBox() throws InterruptedException {
        // Approach to the crypto zone
        if (Team_Color.equals("Red")) {
            Run_encoder(Picto_Middle, Picto_Middle, Picto_Power);           // Move Forward
            Run_encoder(-Picto_Turn_Left, Picto_Turn_Left, Picto_Power);    // Turn Left
            Run_encoder(Picto_Go, Picto_Go, Picto_Power);                   // Move Forward
            Run_encoder(Picto_Turn_Right, -Picto_Turn_Right, Picto_Power);  // Turn Right
        } else if (Team_Color.equals("Blue")) {
            Run_encoder(-Picto_Middle, -Picto_Middle, Picto_Power);         // Move Backward
            Run_encoder(-Picto_Turn_Left, Picto_Turn_Left, Picto_Power);    // Turn Left
            Run_encoder(Picto_Go, Picto_Go, Picto_Power);                   // Move Forward
            Run_encoder(-Picto_Turn_Left, Picto_Turn_Left, Picto_Power);    // Turn Left
        }

        Run_encoder(Picto_Adjust, Picto_Adjust, Picto_Power);   // Adjust the robot

        // Drop the box
        Motor_Elevator.setPower(-Dropbox_Power);
        sleep(Dropbox_Time/2);
        Motor_Elevator.setPower(0);
        Motor_Convey.setPower(-Dropbox_Power);
        sleep(Dropbox_Time*2);

        // Push the box
        Run_encoder(Picto_Push, Picto_Push, Picto_Power);
        Run_encoder(-Picto_Push*1.7, -Picto_Push*1.7, Picto_Power);
        Motor_Convey.setPower(0);
    }

    // Function for Hit a ball
    public void HitBall(double position, double rollback) {
        Servo_Color2.setPosition(position);
        sleep(Stop_Time);
        Servo_Color1.setPosition(Servo_C1_Final);
        sleep(Stop_Time);
        Servo_Color2.setPosition(rollback);
        sleep(Stop_Time);
    }

    // Function for Run
    public void Run_encoder(double leftspin, double rightspin, double run_power) {
        int left_position;
        int right_position;

        // Reset the encoder
        Motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target position
        left_position =  Motor_Left.getCurrentPosition() + (int) (leftspin * rpc);
        right_position = Motor_Right.getCurrentPosition() + (int) (rightspin * rpc);
        Motor_Left.setTargetPosition(left_position);
        Motor_Right.setTargetPosition(right_position);

        // Run to position
        Motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Motor_Left.setPower(run_power);
        Motor_Right.setPower(run_power);

        while(Motor_Left.isBusy() && Motor_Right.isBusy() ){
            // Wait until target position is reached
        }
        Motor_Left.setPower(0);
        Motor_Right.setPower(0);

        Motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}