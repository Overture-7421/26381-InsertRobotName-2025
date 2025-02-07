package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    /* WRIST POSITIONS */
    public static class Wrist {
        public static final double WRIST_STOW = 0.1;
        public static final double WRIST_EXTEND_SHORT = 0.2;
        public static final double WRIST_EXTEND_MEDIUM = 0.4;
        public static final double WRIST_EXTEND_LONG = 0.8;
    }

    /* INTAKE POSITIONS */
    public static class Intake{
        public static final double INTAKE_STOW = 0.1;
        public static final double INTAKE_OPEN = 0.4;
    }

    /* ARM POSITIONS */
    public static class Arm {
        public static final double ARM_STOW = -31;


        public static final double ARM_GROUDGRAB_LONG = 5;

        public static final double ARM_CLIMB = 90;
        public static final double ARM_LOWBASKET = 50;
        public static final double ARM_AUTO_HIGHBASKET = 65;
        public static final double ARM_HIGHBASKET = 80;

        // CHAMBERS
        public static final double ARM_LOWCHAMBER = 0;
        public static final double ARM_HIGHCHAMBER = 90;
    }

    /* ELEVATOR POSITIONS in cm*/
    public static class Elevator {
        public static final double ELEVATOR_STOW = 2; // 0 in

        public static final double ELEVATOR_GROUDGRAB_LONG = 50; // 20 in


        public static final double ELEVATOR_CLIMB = 15; // 20 in

          /*
            Clarification:
             The following distances may seem to exceed the maximum length stipulated in rule R104, however,
             the distances are with the arm at a high angle thus, the total horizontal expansion limit
             is still acknowledged as per rule R104. This was calculated using trigonometry with the following formula.
                    HorizontalExtension = cosine(armAngle) * ElevatorExtension
           */

        public static final double ELEVATOR_LOWBASKET = 45; // 15 in
        public static final double ELEVATOR_HIGHBASKET = 68; //29 in

        // CHAMBERS
        public static final double ELEVATOR_LOWCHAMBER = 3; // 1.1 in
        public static final double ELEVATOR_HIGHCHAMBER = 39; // 15 in

    }
}
