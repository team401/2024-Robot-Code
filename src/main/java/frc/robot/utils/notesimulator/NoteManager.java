package frc.robot.utils.notesimulator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.function.Supplier;

public class NoteManager {

    private static ArrayList<Note> notesOnField = new ArrayList<Note>();
    private static Note noteInRobot;

    private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    private static int numberOfExistingNotes = 0;

    public NoteManager(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public static void addNote(Note note) {
        if (note.inRobot() && noteInRobot == null) noteInRobot = note;
        else {
            notesOnField.add(note);
        }
        numberOfExistingNotes++;
    }

    public static void intake() {
        if (noteInRobot == null) {
            for (Note note : notesOnField) {
                if (note.intakeNote()) {
                    noteInRobot = note;
                    notesOnField.remove(note);
                    SmartDashboard.putBoolean("noteInRobot", noteInRobot != null);
                    return;
                }
            }
        }
    }

    public static Command shoot(double aimRPM, double aimAngle) {
        if (noteInRobot != null) {
            Note shotNote = noteInRobot;
            noteInRobot = null;
            SmartDashboard.putBoolean("noteInRobot", noteInRobot != null);
            return shotNote.shoot(aimRPM, aimAngle);
        } else return null;
    }

    public static boolean noteInRobot() {
        return noteInRobot != null;
    }

    public static int numberOfExistingNotes() {
        return numberOfExistingNotes;
    }
}
