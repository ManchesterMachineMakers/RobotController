//package org.firstinspires.ftc.teamcode.util;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
////import org.firstinspires.ftc.teamcode.navigation.org.firstinspires.ftc.teamcode.util.pathfinder.Destination;
////import org.firstinspires.ftc.teamcode.sensors.Camera;
//
//public class RingDetectorTimeout {
//    public static class Detection {
//        public RecognitionMatrix recognition = null;
////        public int ringCount = 0;
//        //public org.firstinspires.ftc.teamcode.util.pathfinder.Destination targetZone = null;
//    }
////    public static Detection detect(LinearOpMode self, Camera camera) {
////        long start = System.currentTimeMillis();
////        RecognitionMatrix recognition = camera.getRingStack();
////        Detection detect = new Detection();
////        while(!(recognition == null
////                && self.opModeIsActive()
////                && System.currentTimeMillis() - start < 3000
////        )) {
////            //if(System.currentTimeMillis() - start == 3000) break;
////            recognition = camera.getRingStack();
////            self.idle();
////        }
////        detect.recognition = recognition;
//////        detect.ringCount = camera.getCountOfRings(recognition);
//////        detect.targetZone = camera.getTargetZone(recognition);
////        return detect;
////    }
//}