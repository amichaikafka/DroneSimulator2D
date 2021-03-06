import java.awt.*;
import java.text.DecimalFormat;

public class Point {
	public double x;
	public double y;




	
	public Point(double x,double y) {
		this.x = x;
		this.y = y;
	}
	
	public Point(Point p) {
		this.x = p.x;
		this.y = p.y;
	}
	
	public Point() {
		x = 0;
		y = 0;
	}

	public int getAngle(Point target) {
		return (int) Math.toDegrees(Math.atan2(target.x - x, target.y - y));
	}
	
	@Override
	public String toString() {
		DecimalFormat df = new DecimalFormat("#.###");
		
		return "(" + df.format(x) + "," + df.format(y) + ")";
	}

}
