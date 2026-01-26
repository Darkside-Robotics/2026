package frc.robot.library;

// public enum FrontBack {
//     FRONT, BACK
// }

// public enum LeftRight {
//     LEFT, RIGHT
// }

// public enum FieldColor {
//     RED, BLUE
// }

// public enum FieldLocation {
//     OUTPOST, TOWER, HUB, TRENCH, BUMP, CORRAL
// }

public class FieldLocation {
    public FieldHelper.LeftRight leftRight;
    public FieldHelper.FieldColor fieldColor;
    public FieldHelper.FrontBack frontBack;
    public int fieldNumber;

    public FieldLocation(FieldHelper.LeftRight leftRight,
            FieldHelper.FieldColor fieldColor,
            FieldHelper.FrontBack frontBack,
            int fieldNumber) {
        this.leftRight = leftRight;
        this.fieldColor = fieldColor;
        this.frontBack = frontBack;
        this.fieldNumber = fieldNumber;
    }

    
}
