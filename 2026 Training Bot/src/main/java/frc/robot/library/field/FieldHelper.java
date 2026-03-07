package frc.robot.library.field;

public class FieldHelper {

    public FieldHelper() {

    }

    

   
    public static int lookup(FieldColor color, FieldLocation location) {
        switch (color) {
            case RED:
                switch (location) {
                    case OUTPOST_LEFT:
                        return 14;
                    case OUTPOST_RIGHT:
                        return 13;
                    case TOWER_LEFT:
                        return 16;
                    case TOWER_RIGHT:
                        return 15;
                    case HUB_LEFT_FRONT:
                        return 9;
                    case HUB_LEFT_BACK:
                        return 4;
                    case HUB_RIGHT_FRONT:
                        return 10;
                    case HUB_RIGHT_BACK:
                        return 3;
                    case TRENCH_LEFT_FRONT:
                        return 7;
                    case TRENCH_LEFT_BACK:
                        return 6;
                    case TRENCH_RIGHT_FRONT:
                        return 12;
                    case TRENCH_RIGHT_BACK:
                        return 1;
                    case BUMP_LEFT_FRONT:
                        return 8;
                    case BUMP_LEFT_BACK:
                        return 5;
                    case BUMP_RIGHT_FRONT:
                        return 11;
                    case BUMP_RIGHT_BACK:
                        return 2;
                    case CORRAL:
                        return -1;
                }

            case BLUE:
                switch (location) {
                    case OUTPOST_LEFT:
                        return 30;
                    case OUTPOST_RIGHT:
                        return 29;
                    case TOWER_LEFT:
                        return 32;
                    case TOWER_RIGHT:
                        return 31;
                    case HUB_LEFT_FRONT:
                        return 25;
                    case HUB_LEFT_BACK:
                        return 20;
                    case HUB_RIGHT_FRONT:
                        return 26;
                    case HUB_RIGHT_BACK:
                        return 19;
                    case TRENCH_LEFT_FRONT:
                        return 23;
                    case TRENCH_LEFT_BACK:
                        return 22;
                    case TRENCH_RIGHT_FRONT:
                        return 28;
                    case TRENCH_RIGHT_BACK:
                        return 17;
                    case BUMP_LEFT_FRONT:
                        return 24;
                    case BUMP_LEFT_BACK:
                        return 21;
                    case BUMP_RIGHT_FRONT:
                        return 27;
                    case BUMP_RIGHT_BACK:
                        return 18;
                    case CORRAL:
                        return -1;

                }
        }

        return 30;
    }
}
