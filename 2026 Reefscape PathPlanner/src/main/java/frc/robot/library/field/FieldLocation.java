package frc.robot.library.field;

 



// public enum FieldLocation {
//     OUTPOST, TOWER, HUB, TRENCH, BUMP, CORRAL
// }

public class FieldLocation {
    public LeftRight leftRight;
    public FieldColor fieldColor;
    public FrontBack frontBack;
    public int fieldNumber;

    public FieldLocation(LeftRight leftRight,
            FieldColor fieldColor,
            FrontBack frontBack,
            int fieldNumber) {
        this.leftRight = leftRight;
        this.fieldColor = fieldColor;
        this.frontBack = frontBack;
        this.fieldNumber = fieldNumber;
    }

    
}
