import java.lang.reflect.Array;

/** Minimal stand-in for PApplet: only what the original controller logic touches. */
class PImage {}

public class Stub {
  public static final float PI = (float) Math.PI;
  public static final float TWO_PI = (float) (Math.PI * 2.0);
  public static final float HALF_PI = (float) (Math.PI / 2.0);
  public static final float QUARTER_PI = (float) (Math.PI / 4.0);
  static final float DEG_TO_RAD = PI / 180.0f;
  static final float RAD_TO_DEG = 180.0f / PI;

  // controllable fake clock
  public int fake_millis = 0;
  public int millis() { return fake_millis; }

  public static float radians(float deg) { return deg * DEG_TO_RAD; }
  public static float degrees(float rad) { return rad * RAD_TO_DEG; }
  public static float sin(float a) { return (float) Math.sin(a); }
  public static float cos(float a) { return (float) Math.cos(a); }
  public static float tan(float a) { return (float) Math.tan(a); }
  public static float atan2(float y, float x) { return (float) Math.atan2(y, x); }
  public static float sqrt(float a) { return (float) Math.sqrt(a); }
  public static float acos(float a) { return (float) Math.acos(a); }
  public static float asin(float a) { return (float) Math.asin(a); }
  public static float abs(float a) { return (a < 0) ? -a : a; }
  public static int abs(int a) { return (a < 0) ? -a : a; }
  public static float pow(float a, float b) { return (float) Math.pow(a, b); }
  public static float constrain(float a, float lo, float hi) { return (a < lo) ? lo : ((a > hi) ? hi : a); }
  public static int constrain(int a, int lo, int hi) { return (a < lo) ? lo : ((a > hi) ? hi : a); }
  public static float map(float value, float start1, float stop1, float start2, float stop2) {
    float outgoing = start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
    return outgoing;
  }
  public static int round(float a) { return Math.round(a); }
  public static String str(int x) { return String.valueOf(x); }
  public static String str(float x) { return String.valueOf(x); }
  public static String str(boolean x) { return String.valueOf(x); }

  public static Object append(Object array, Object element) {
    int len = Array.getLength(array);
    Object out = Array.newInstance(array.getClass().getComponentType(), len + 1);
    System.arraycopy(array, 0, out, 0, len);
    Array.set(out, len, element);
    return out;
  }
  public static Object shorten(Object array) {
    int len = Array.getLength(array);
    Object out = Array.newInstance(array.getClass().getComponentType(), len - 1);
    System.arraycopy(array, 0, out, 0, len - 1);
    return out;
  }
  public static void arrayCopy(Object src, Object dst) {
    System.arraycopy(src, 0, dst, 0, Array.getLength(src));
  }

  public boolean quiet_println = true;
  public void println(String s) { if (!quiet_println) System.err.println(s); }
  public PImage loadImage(String name) { return null; }
}
