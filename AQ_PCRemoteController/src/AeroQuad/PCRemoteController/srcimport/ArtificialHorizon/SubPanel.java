/************************************************************
 * Created and developed by @author wilhem (Davide Picchi)
 *
 * written in Java JVM 1.6.12 on Debian 5.0 (Lenny)
 *
 * created on: Feb 22, 2010
 *
 *********************************************/
package AeroQuad.PCRemoteController.srcimport.ArtificialHorizon;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.RoundRectangle2D;


@SuppressWarnings("serial")
public class SubPanel extends JPanel{
	
	private Dimension dimPanel;
	private RoundRectangle2D leftRollBox;
	private RoundRectangle2D rightRollBox;
	private Dimension rectDim;
	private int roundRect;
	private Font writing;
	private int distanceBox;       // Distance between the Boxes and the margin left and right
	private int distXWriting;
	private int distYWriting;
	
	public SubPanel(){
		setBackground(Color.black);
		
		rectDim = new Dimension(70, 40);
		this.roundRect = 5;
		
		this.distanceBox = 250;
		
		this.distXWriting = 160;
		this.distYWriting = 0;
	}


	public Dimension getPreferredSize(){
		dimPanel = new Dimension(1150, 60);
		return dimPanel;
	}
	
	
	
	public void paintComponent(Graphics g){

        super.paintComponent(g);       
		Graphics2D g2d = (Graphics2D)g;

		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

		// Display the shaped round box
		leftRollBox = new RoundRectangle2D.Double(distanceBox, ((dimPanel.getHeight() - rectDim.getHeight())/2), rectDim.getWidth(), rectDim.getHeight(), roundRect, roundRect);
		rightRollBox = new RoundRectangle2D.Double((dimPanel.getWidth() - distanceBox), (dimPanel.getHeight() - rectDim.getHeight()) / 2, rectDim.getWidth(), rectDim.getHeight(), roundRect, roundRect);
		
		// Draw the boxes
		g2d.setPaint(Color.green);
		g2d.setStroke(new BasicStroke(2));
		g2d.draw(leftRollBox);
		g2d.draw(rightRollBox);
		
		// Write in front of the box
		writing = new Font("Serif", Font.BOLD, 25);
		g2d.setFont(writing);
		g2d.drawString("ROLL", distXWriting, (int)(rectDim.getHeight() + distYWriting));
		g2d.drawString("ROLL", (int)(dimPanel.getWidth() - rectDim.getWidth() - distXWriting - 110), (int)(rectDim.getHeight() + distYWriting));
			
		// Write the value inside the box
		writing = new Font("Serif", Font.CENTER_BASELINE, 25);
		g2d.setFont(writing);
		g2d.drawString("" + ArtificialHorizon.rollValueFiltered, distXWriting + 100, (int)(rectDim.getHeight() + distYWriting));
		g2d.drawString("" + ArtificialHorizon.rollValuePure, (int)(dimPanel.getWidth() - rectDim.getWidth() - distXWriting - 10), (int)(rectDim.getHeight() + distYWriting));
	}
}
