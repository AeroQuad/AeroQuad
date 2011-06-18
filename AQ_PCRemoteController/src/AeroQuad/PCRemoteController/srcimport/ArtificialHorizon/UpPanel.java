/************************************************************
 * Created and developed by @author wilhem (Davide Picchi)
 *
 * written in Java JVM 1.6.12 on Debian 5.0 (Lenny)
 *
 * created on: Feb 22, 2010
 *
 *********************************************/
package AeroQuad.PCRemoteController.srcimport.ArtificialHorizon;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.RoundRectangle2D;

import javax.swing.JPanel;

/**
 * @author wilhem
 *
 */
@SuppressWarnings("serial")
public class UpPanel extends JPanel{

	private Dimension dimPanel;
	private RoundRectangle2D leftYawBox;
	private RoundRectangle2D rightYawBox;
	private Dimension rectDim;
	private int roundRect;
	private Font writing;
	private int distanceBox;       // Distance between the Boxes and the margin left and right
	private int distXWriting;
	private int distYWriting;
	
	public UpPanel(){
		
		setBackground(Color.black);
		
		rectDim = new Dimension(70, 40);
		this.roundRect = 5;
		
		this.distanceBox = 250;
		
		this.distXWriting = 160;
		this.distYWriting = 22;
	}
	
	public Dimension getPreferredSize(){
		dimPanel = new Dimension(1150, 80);
		return dimPanel;
	}
	
	public void paintComponent(Graphics g){

        super.paintComponent(g);       
		Graphics2D g2d = (Graphics2D)g;

		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

		// Display the shaped round box
		leftYawBox = new RoundRectangle2D.Double(distanceBox, (dimPanel.getHeight() - rectDim.getHeight()/2) / 2, rectDim.getWidth(), rectDim.getHeight(), roundRect, roundRect);
		rightYawBox = new RoundRectangle2D.Double((dimPanel.getWidth() - distanceBox), (dimPanel.getHeight() - rectDim.getHeight()/2) / 2, rectDim.getWidth(), rectDim.getHeight(), roundRect, roundRect);
		
		// Draw the boxes
		g2d.setPaint(Color.yellow);
		g2d.setStroke(new BasicStroke(2));
		g2d.draw(leftYawBox);
		g2d.draw(rightYawBox);
		
		// Writing in front of the boxes
		writing = new Font("Serif", Font.BOLD, 25);
		g2d.setFont(writing);
		g2d.drawString("YAW", distXWriting, (int)(rectDim.getHeight() + distYWriting));
		g2d.drawString("YAW", (int)(dimPanel.getWidth() - rectDim.getWidth() - distXWriting - 110), (int)(rectDim.getHeight() + distYWriting));
	
		// Write the value inside the boxes
		writing = new Font("Serif", Font.CENTER_BASELINE, 25);
		g2d.setFont(writing);
		g2d.drawString("" + ArtificialHorizon.yawValueFiltered, distXWriting + 100, (int)(rectDim.getHeight() + distYWriting));
		g2d.drawString("" + ArtificialHorizon.yawValuePure, (int)(dimPanel.getWidth() - rectDim.getWidth() - distXWriting - 10), (int)(rectDim.getHeight() + distYWriting));
	}
}