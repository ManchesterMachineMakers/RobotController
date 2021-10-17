
package org.firstinspires.ftc.teamcode.navigation;

public enum FieldDestinations {
         Shoot ("Shoot", 100, -901),
         StarterStack ("starterStack", -571, -901),
         TargetZoneA ("TargetZoneA",307,-1501),
         TargetZoneB ("TargetZoneB",904,-904),
         TargetZoneC ("TargetZoneC",1501,-1501),
         StartLineLeftCenter ("StartLineLeftCenter",-1501,-622),
         StartLineRightCenter ("StartLineRightCenter",-1501,-1238),
         StartLineLeftTip ("StartLineLeftTip",-1212,-627),
         StartLineRightTip ("StartLineRightTip",-1212,-1238),
         TowerGoal ("TowerGoal",1790,-888),
         PowerShotLeft ("PS_LEFT",1790,-107),
         PowerShotCenter ("PS_CENTER",1790,-298),
         PowerShotRight ("PS_RIGHT",1790,-488),
         LaunchLinePark ("LaunchLinePark", 400,-901);

        public final Destination destination;

        private FieldDestinations(String name, float destX, float destY) {
                this.destination = new Destination(name, destX, destY);
        }

        public static Destination SHOOT = Shoot.destination;
        public static Destination STACK = StarterStack.destination;
        public static Destination TZA = TargetZoneA.destination;
        public static Destination TZB = TargetZoneB.destination;
        public static Destination TZC = TargetZoneC.destination;
        public static Destination SLC = StartLineLeftCenter.destination;
        public static Destination SRC = StartLineRightCenter.destination;
        public static Destination SLT = StartLineLeftTip.destination;
        public static Destination SRT = StartLineRightTip.destination;
        public static Destination TOW = TowerGoal.destination;
        public static Destination PSL = PowerShotLeft.destination;
        public static Destination PSC = PowerShotCenter.destination;
        public static Destination PSR = PowerShotRight.destination;
        public static Destination LLP = LaunchLinePark.destination;

}
