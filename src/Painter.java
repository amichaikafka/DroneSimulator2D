import java.awt.*;

import javax.swing.JComponent;


public class Painter extends JComponent{
	OurAlgo algo;
	
	public Painter(OurAlgo algo) {
		this.algo = algo;
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		algo.paint(g);
	}
}
