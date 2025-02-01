package org.firstinspires.ftc.teamcode.autonomii;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
//@TeleOp


public class AlbastruStanga extends LinearOpMode {

    DcMotor fdr, sdr, fst, sst; // sasiu

    DcMotor gdr, gst; // glisiera

    Servo bratSt, bratDr, cleste;



    public void runOpMode() throws InterruptedException {


        fdr = hardwareMap.dcMotor.get("fdr");
        fdr.setDirection(DcMotorSimple.Direction.REVERSE);
        fdr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        sdr = hardwareMap.dcMotor.get("sdr");
        sdr.setDirection(DcMotorSimple.Direction.REVERSE);
        sdr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        fst = hardwareMap.dcMotor.get("fst");
        fst.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        sst = hardwareMap.dcMotor.get("sst");
        sst.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        gdr = hardwareMap.dcMotor.get("gdr");
        gdr.setDirection(DcMotorSimple.Direction.REVERSE);
        gdr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        gdr.setMode(RunMode.STOP_AND_RESET_ENCODER);


        gst = hardwareMap.dcMotor.get("gst");
        gst.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        gst.setMode(RunMode.STOP_AND_RESET_ENCODER);

        bratSt = hardwareMap.servo.get("bratSt");
        bratSt.setPosition(0);

        bratDr = hardwareMap.servo.get("bratDr");
        bratDr.setDirection(Servo.Direction.REVERSE);
        bratDr.setPosition(0);

        cleste = hardwareMap.servo.get("cleste");
        cleste.setPosition(1);

        waitForStart();

        sleep(1000);
        fata(0.4);
        sleep(780);
        brat_diagonal();
        sleep(1000);
    }


    /****************************************************************************************
     ______   _    _   _   _    _____   _______   _____   _____
     |  ____| | |  | | | \ | |  / ____| |__   __| |_   _| |_   _|
     | |__    | |  | | |  \| | | |         | |      | |     | |
     |  __|   | |  | | | . ` | | |         | |      | |     | |
     | |      | |__| | | |\  | | |____     | |     _| |_   _| |_
     |_|       \____/  |_| \_|  \_____|    |_|    |_____| |_____|
     ****************************************************************************************/

    public void fata(double power) {
        fdr.setPower(power);
        sdr.setPower(power);
        fst.setPower(power);
        sst.setPower(power);
    }
    public void spate(double power) {
        fdr.setPower(-power);
        sdr.setPower(-power);
        fst.setPower(-power);
        sst.setPower(-power);
    }

    public void dreapta(double power){
        fdr.setPower(-power);
        fst.setPower(power);
        sdr.setPower(power);
        sst.setPower(-power);
    }
    public void stanga(double power){
        fdr.setPower(power);
        fst.setPower(-power);
        sdr.setPower(-power);
        sst.setPower(power);
    }
    public void rotirestanga(double power){
        fdr.setPower(power);
        fst.setPower(-power);
        sdr.setPower(power);
        sst.setPower(-power);
    }
    public void rotiredreapta(double power){
        fdr.setPower(-power);
        fst.setPower(power);
        sdr.setPower(-power);
        sst.setPower(power);
    }

    public void brat_jos() {
        bratSt.setPosition(1);
        bratDr.setPosition(1);
    }

    public void brat_diagonal() {
        bratSt.setPosition(0.40);
        bratDr.setPosition(0.40);
    }
    public void nivel1() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(1230);
        gst.setTargetPosition(1230);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() || gst.isBusy()) {
            gdr.setPower(0.5);
            gst.setPower(0.5);
        }
    }

    public void coborare() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(0);
        gst.setTargetPosition(0);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() || gst.isBusy()) {
            gdr.setPower(0.4);
            gst.setPower(0.4);
        }
    }
}