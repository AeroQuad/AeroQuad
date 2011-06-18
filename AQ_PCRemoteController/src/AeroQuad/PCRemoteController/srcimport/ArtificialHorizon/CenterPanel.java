package AeroQuad.PCRemoteController.srcimport.ArtificialHorizon;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.RoundRectangle2D;


@SuppressWarnings("serial")
public class CenterPanel extends JPanel{
	
	private Dimension dimPanel;
	private Dimension rectDim;
	private int roundRect;
	private RoundRectangle2D upperRectBox;
	private RoundRectangle2D lowerRectBox;
	private Font upperWriting;
	private Font insideWriting;
	
	
	public CenterPanel(){
		setBackground(Color.black);
		
		rectDim = new Dimension(70, 40);
		this.roundRect = 5;
	}

	public Dimension getPreferredSize(){
		this.dimPanel = new Dimension(100, 500);
		return dimPanel;
	}
	
	public void paintComponent(Graphics g){
		
	    super.paintComponent(g);       
		Graphics2D g2d = (Graphics2D)g;

		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

		// Display the shaped round box
		upperRectBox = new RoundRectangle2D.Double(((dimPanel.getWidth() / 2) - rectDim.getWidth()) / 2 + 15, (dimPanel.getHeight() - rectDim.getHeight()) / 2, rectDim.getWidth(), rectDim.getHeight(), roundRect, roundRect);
		lowerRectBox = new RoundRectangle2D.Double(((dimPanel.getWidth() / 2) + rectDim.getWidth()) / 2 + 18, (dimPanel.getHeight() - rectDim.getHeight()) / 2, rectDim.getWidth(), rectDim.getHeight(), roundRect, roundRect);
		g2d.setPaint(Color.blue);
		g2d.setStroke(new BasicStroke(2));
		g2d.draw(upperRectBox);
		g2d.draw(lowerRectBox);
		
		// Write above the box
		upperWriting = new Font("Serif", Font.BOLD, 25);
		g2d.setFont(upperWriting);
		g2d.drawString("PITCH", (int)((dimPanel.getWidth() / 2) - rectDim.getWidth()) / 2 + 40, (int)((dimPanel.getHeight() - rectDim.getHeight()) / 2 - (rectDim.getHeight()) / 2));
		
		// Write the value inside the box
		insideWriting = new Font("Serif", Font.CENTER_BASELINE, 25);
		g2d.setFont(insideWriting);
		g2d.drawString("" + ArtificialHorizon.pitchValueFiltered, (int) (upperRectBox.getCenterX()) - 30, (int) (upperRectBox.getCenterY()) + 10);
		g2d.drawString("" + ArtificialHorizon.pitchValuePure, (int) (upperRectBox.getCenterX()) - 30 + 75, (int) (upperRectBox.getCenterY()) + 10);
	}
}
