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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "Final: Auto_Blue_C", group = "Final")
@Disabled

public class Auto_turn_balance extends LinearOpMode {
    // Declare DcMotor members
    private DcMotor Motor_Left = null;      // DcMotors for left wheels
    private DcMotor Motor_Right = null;     // DcMotors for right wheels
    private DcMotor Motor_Convey = null;    // DcMotors to pick up boxes
    private DcMotor Motor_Elevator = null;  // DcMotor to elevate boxes

    // Declare Servo members
    private Servo Servo_Color1 = null;      // Servo to control the sensor arm (up & down)
    private Servo Servo_Color2 = null;      // Servo to control the sensor arm (forward & backward)

    // Declare variables for sensing color
           // "Blue" or "Red"
    int    rpc              = 1440;         // Value for one rotation
    double Servo_C1_Initial = 0.38;         // Initial position of Servo_Color1
    double Servo_C1_Sensing = 0.90;         // Sensing position of Servo_Color1
    double Servo_C1_Final   = 0.36;         // Final position of Servo_Color1
    double Servo_C2_Initial = 0.30;         // Initial position of Servo_Color2
    double Servo_C2_Left    = 0.155;         // Left position of Servo_Color2
    double Servo_C2_Right   = 0.45;         // Right position of Servo_Color2
    double Color_Value_Red  = 0;            // Color sensor initial value for red
    double Color_Value_Blue = 0;            // Color sensor initial value for blue

    // Declare variables for pictogram
    double Picto_Power        = 0.25;       // Motor power
    double Dropbox_Power      = 1.00;       // Motor power
    double Picto_Turn_Left    = 1.500;      // Motor rotation for Left Turn
    double Picto_Turn_Right   = 1.600;      // Motor rotation for Right Turn
    double Picto_Go;                        // Initial motor rotation

    double Picto_Left    = 2.040;           // Motor rotation for Left
    double Picto_Center  = 2.657;           // Motor rotation for Center
    double Picto_Right   = 3.380;           // Motor rotation for Right
    double Picto_Adjust  = 0.500;           // Motor rotation for Adjust
    double Picto_Push    = 0.300;           // Motor rotation for Push

    long Picto_Left_Decision   = 0;         // Picto sensor initial value for Left
    long Picto_Center_Decision = 0;         // Picto sensor initial value for Center
    long Picto_Right_Decision  = 0;         // Picto sensor initial value for Right

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

            waitForStart();   // Wait for the start button to be pressed

            runDropBox();     // Execute the Dropbox task
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }



    protected void runDropBox() throws InterruptedException {
        // Approach to the crypto zone


        Run_encoder(Picto_Adjust, Picto_Adjust, Picto_Power);   // Adjust the robot
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