/************************************************************
 * Created and developed by @author wilhem (Davide Picchi)
 *
 * written in Java JVM 1.6.12 on Debian 5.0 (Lenny)
 *
 * created on: Feb 24, 2010
 *
 *********************************************/
package AeroQuad.PCRemoteController.srcimport.ArtificialHorizon;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.io.IOException;
import java.io.InputStream;

@SuppressWarnings("serial")
public class PanelGUI extends JPanel{
	
	/****************************
	 * Defining instance variables
	 ****************************/
	private String definePanel;     // Stores whether the panel for kalman is or for the sensor
	private Color blueSky;
	private Color orangeEarth;
	private GradientPaint outline;
	private Dimension dimPanel;
	private Arc2D upperArc;          // Upper part of the Horizon
	private Arc2D lowerArc;          // Bottom part of the Horizon
	private Point2D centerPoint;
	
	private Ellipse2D roundHorizon;
	private int radius;

	private Line2D markerLine;
	private GeneralPath triangle;
	private GeneralPath centerShape;
	private GeneralPath bankMarkerLong;
	private GeneralPath bankMarkerShort;

	
	private Font writing = null;
	
	private int dimMarker5Deg;
	private int dimMarker10Deg;
	
	private int rollAngle;
	private int pitchAngle;

	
	
	/************************************
	 * This constructor will create
	 * the initial panel for the Horizon
	 ************************************/
	
    public PanelGUI(String type) {    	
    	
    	this.definePanel = type;
    	
		setBackground(Color.black);
		
		// Define color used
		blueSky = new Color(10, 112, 156);
		orangeEarth = new Color(222, 132, 14);
		
		// Creates two arcs used to draw the outline
		upperArc = new Arc2D.Float();
		lowerArc = new Arc2D.Float();
		
		// Define a center point as a reference
		centerPoint = new Point2D.Float(250, 250);
		
		// Instance variables initialization
		this.radius = 200;
		
		this.dimMarker10Deg = 30;
		this.dimMarker5Deg = 10;
		
		
		/*****************************************
		 * Take resources from the folder for font
		 ****************************************/
    	InputStream is = this.getClass().getResourceAsStream("/01Digitall.ttf");
    	
		try {
			this.writing = Font.createFont(Font.TRUETYPE_FONT, is);
			
	    	this.writing = this.writing.deriveFont(12.0f);
	    	
		} catch (FontFormatException e) {
			System.out.println("Format fonts not correct!!!");
		} catch (IOException e) {
			System.out.println("Fonts not found!!!");
		}

    }

	
	public Dimension getPreferredSize(){
		dimPanel = new Dimension(500, 500);
		return dimPanel;
	}
	

	/****************************
	 * Main paintComponent method
	 ***************************/
    public void paintComponent(Graphics g) {

    	/*************************************************
    	 * According to the info stored in definePanel the 
    	 * filtered or pure values are picked up from the
    	 * static variables defined in the main class
    	 ************************************************/
    	if(definePanel.contentEquals("kalman")){
    		this.pitchAngle = ArtificialHorizon.pitchValueFiltered;
        	this.rollAngle = ArtificialHorizon.rollValueFiltered;
    	}
    	
    	if(definePanel.contentEquals("sensor")){
    		this.pitchAngle = ArtificialHorizon.pitchValuePure;
        	this.rollAngle = ArtificialHorizon.rollValuePure;
    	}
    	
    	
        super.paintComponent(g);       
		Graphics2D g2d = (Graphics2D)g;

		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    	
		drawHorizon(g2d);

		g2d.setStroke(new BasicStroke(2));
		g2d.setPaint(Color.white);
		
    	// Draw the Bank roll lines on the top
    	drawBankRollMarker(g2d);
    	
    	// Display the outline of the Horizon
    	roundHorizon = new Ellipse2D.Float(50, 50, 2 * radius, 2 * radius);
    	outline = new GradientPaint(20, 20, Color.white, 500, 500, Color.gray, true);
    	g2d.setPaint(outline);
    	g2d.setStroke(new BasicStroke(6));
    	g2d.draw(roundHorizon);
    }
    
    
    private void drawHorizon(Graphics2D g2d){
    	
    	// Start doing some math calculation for angles
    	int angStartUpper = 0;
    	int angExtUpper = 0;
    	int angStartLower = 0;
    	int angExtLower = 360;
    	
    	// First step is to determine the roll display position
    	AffineTransform at = AffineTransform.getRotateInstance(Math.toRadians(rollAngle), centerPoint.getX(), centerPoint.getY());
    	g2d.transform(at);
    	
    	if((pitchAngle < 90) && (pitchAngle > -90)){
        	angStartUpper = - pitchAngle;  // Minus because of the reverse way of working of the artificial horizon positive values let the blue arc to get bigger...
        	angExtUpper = (180 - (2 * angStartUpper));
    	}
    	
    	if((pitchAngle >= 90) && (pitchAngle < 180)){
        	at = AffineTransform.getRotateInstance(Math.toRadians(180), centerPoint.getX(), centerPoint.getY());
        	g2d.transform(at);
        		
           	angStartUpper = -(180 - pitchAngle);  // Minus because of the reverse way of working of the artificial horizon positive values let the blue arc to get bigger...
           	angExtUpper = (180 - (2 * angStartUpper));
    	}

   		if((pitchAngle <= -90) && (pitchAngle > -180)){
        	at = AffineTransform.getRotateInstance(Math.toRadians(180), centerPoint.getX(), centerPoint.getY());
        	g2d.transform(at);
        	
            angStartUpper = (180 + pitchAngle);  // Minus because of the reverse way of working of the artificial horizon positive values let the blue arc to get bigger...
            angExtUpper = (180 - (2 * angStartUpper));
   		}
		
    	// Draw the artificial horizon itself, composed by 2 half arcs
		lowerArc.setArcByCenter(centerPoint.getX(), centerPoint.getY(), radius, angStartLower, angExtLower, Arc2D.CHORD);
		g2d.setPaint(orangeEarth);
		g2d.fill(lowerArc);
		
		upperArc.setArcByCenter(centerPoint.getX(), centerPoint.getY(), radius, angStartUpper, angExtUpper, Arc2D.CHORD);
		g2d.setPaint(blueSky);
		g2d.fill(upperArc);

		// Draw the middle white line
		g2d.setStroke(new BasicStroke(1));
		g2d.setPaint(Color.white);
		g2d.draw(upperArc);
		
		drawMarkers(g2d);
		
		at = AffineTransform.getRotateInstance(Math.toRadians(- rollAngle), centerPoint.getX(), centerPoint.getY());
		
		g2d.transform(at);
    }

    
    private void drawMarkers(Graphics2D g2d){
    	
    	// Draw the lines on the Horizon
    	drawLines(g2d);
    	
    	// Draw the Bank roll display on the top
    	drawBankRollTriangle(g2d);

    }
    
    
    private void drawLines(Graphics2D g2d){
    	
    	int angle;
    	int distance;
    	int angleCorrUp;
    	int limitInf, limitMax;
    	
    	limitInf = (int)((this.pitchAngle / 10) - 5);
    	if(limitInf < - 18) limitInf = -18;
    	limitMax = limitInf + 11;
    	if(limitMax > 18) limitMax = 19;
    	
    	for(int i = limitInf; i < limitMax; i++){
    		
    		angle = i * 10;    // Display the text at the right "height"
    		angleCorrUp = angle - this.pitchAngle;
    		distance = Math.abs(i * 5);       // Put the text and the lines length at the right position
    			
        	g2d.setPaint(Color.white);
        	g2d.setStroke(new BasicStroke(2));
        	g2d.setFont(writing);
      	
        	// Longer markers
    		markerLine = new Line2D.Float((float)(centerPoint.getX() - dimMarker10Deg - distance), (float)(centerPoint.getY() - (radius * Math.sin(Math.toRadians(angleCorrUp)))), (float)(centerPoint.getX() + dimMarker10Deg + distance), (float)(centerPoint.getY() - (radius * Math.sin(Math.toRadians(angleCorrUp)))));

        	g2d.draw(markerLine);
        	
        	// Short markers
    		markerLine = new Line2D.Float((float)(centerPoint.getX() - dimMarker5Deg), (float)(centerPoint.getY() - (radius * Math.sin(Math.toRadians(angleCorrUp + 5)))), (float)(centerPoint.getX() + dimMarker5Deg), (float)(centerPoint.getY() - (radius * Math.sin(Math.toRadians(angleCorrUp + 5)))));

        	g2d.draw(markerLine);  
  
        	// Writing routine
        	g2d.drawString("" + (angle), (float)(centerPoint.getX() - dimMarker10Deg - distance - 25), (float)(centerPoint.getY() - (radius * Math.sin(Math.toRadians(angleCorrUp)) - 5)));
        	g2d.drawString("" + (angle), (float)(centerPoint.getX() + dimMarker10Deg + distance + 8), (float)(centerPoint.getY() - (radius * Math.sin(Math.toRadians(angleCorrUp)) - 5)));
        	
    	}

       	// Draw the center shape
       	centerShape = new GeneralPath(GeneralPath.WIND_EVEN_ODD);
       	centerShape.moveTo((centerPoint.getX() - radius/2.5 ), centerPoint.getY());
       	centerShape.lineTo((centerPoint.getX() - 25), centerPoint.getY());
       	centerShape.moveTo((centerPoint.getX() - 40), centerPoint.getY());
       	centerShape.lineTo((centerPoint.getX() - 20), (centerPoint.getY() + 20));
       	centerShape.lineTo(centerPoint.getX(), centerPoint.getY());
       	centerShape.lineTo((centerPoint.getX() + 20), (centerPoint.getY() + 20));
       	centerShape.lineTo((centerPoint.getX() + 40), centerPoint.getY());
       	centerShape.moveTo((centerPoint.getX() + radius/2.5), centerPoint.getY());
       	centerShape.lineTo((centerPoint.getX() + 25), centerPoint.getY());
       	
       	g2d.setPaint(Color.white);
       	g2d.setStroke(new BasicStroke(3));
       	g2d.draw(centerShape);
    }
    
    
    private void drawBankRollTriangle(Graphics2D g2d){
    	
    	// Draw the triangle on the upper position
    	triangle = new GeneralPath(GeneralPath.WIND_EVEN_ODD);
    	triangle.moveTo(centerPoint.getX(), (centerPoint.getY() - radius + 5));
    	triangle.lineTo((centerPoint.getX() - 15), (centerPoint.getY() - radius + 30));
    	triangle.lineTo((centerPoint.getX() + 15), (centerPoint.getY() - radius + 30));
    	triangle.closePath();
    	
    	g2d.fill(triangle);
    	
    	// Draw the triangle in the lower position
    	triangle.moveTo(centerPoint.getX(), (centerPoint.getY() + radius - 5));
       	triangle.lineTo((centerPoint.getX() - 10), (centerPoint.getY() + radius - 25));
    	triangle.lineTo((centerPoint.getX() + 10), (centerPoint.getY() + radius - 25));
    	triangle.closePath();
     	
    	g2d.draw(triangle);
    }
    
    
    private void drawBankRollMarker(Graphics2D g2d){
    	
    	// Draw the line markers for bank angle
    	bankMarkerLong = new GeneralPath(GeneralPath.WIND_EVEN_ODD);
    	bankMarkerLong.moveTo((centerPoint.getX() - radius), centerPoint.getY());
    	bankMarkerLong.lineTo((centerPoint.getX() - radius + 20), centerPoint.getY());
    
    	bankMarkerShort = new GeneralPath(GeneralPath.WIND_EVEN_ODD);
    	bankMarkerShort.moveTo((centerPoint.getX() - radius), centerPoint.getY());
    	bankMarkerShort.lineTo((centerPoint.getX() - radius + 10), centerPoint.getY());
    	
    	for(int i = 0; i < 5; i++){
        	AffineTransform ata = AffineTransform.getRotateInstance(Math.toRadians(30), centerPoint.getX(), centerPoint.getY());
        	g2d.transform(ata);
        	
        	g2d.draw(bankMarkerLong);
        	
    	}

    	AffineTransform ata = AffineTransform.getRotateInstance(Math.toRadians(260), centerPoint.getX(), centerPoint.getY());
    	g2d.transform(ata);
    	
    	for(int i = 0; i < 7; i++){
        	AffineTransform atb = AffineTransform.getRotateInstance(Math.toRadians(10), centerPoint.getX(), centerPoint.getY());
        	g2d.transform(atb);
        	
        	g2d.draw(bankMarkerShort);
    	}
    	

    	ata = AffineTransform.getRotateInstance(Math.toRadians(110), centerPoint.getX(), centerPoint.getY());
    	g2d.transform(ata);
    	
    	for(int i = 0; i < 7; i++){
        	AffineTransform atb = AffineTransform.getRotateInstance(Math.toRadians(10), centerPoint.getX(), centerPoint.getY());
        	g2d.transform(atb);
        	
        	g2d.draw(bankMarkerShort);
    	}
    }
}
