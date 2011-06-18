/*******************************************************************************
 * Artificial Horizon
 * 
 * "Copyright (C) 2010 Davide Picchi"  mailto: paveway@gmail.com
 * 
 * This program is distributed under the terms of the GNU General Public License.
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Artificial Horizon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 * (see COPYING file)
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * 
 * This program does use the Open Source rxtx (www.rxtx.org) for serial connection
 * and for controlling serial ports on the system, which are not provided with
 * this package but must be downloaded, installed and configured apart
 * (and make sure they are runnable) from the link above.
 * 
 * Written in Java JVM 1.6.12 (on Debian 5.0 Lenny)
 * 
 * @author wilhem (Davide Picchi)
 * 
 * *****************************
 * starting date: 15 Januar 2010
 * 
 * last update: Feb, 5 2010 
 **********************************************/
package AeroQuad.PCRemoteController.srcimport.ArtificialHorizon;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;


public class ArtificialHorizon{
	
	/***************************************************************
	 * Those instances variables store the value collected via USART
	 **************************************************************/
	static int pitchValuePure = 0;
	static int rollValuePure = 0;
		
	public static int pitchValueFiltered = 0;
	public static int rollValueFiltered = 0;
	
	static int yawValueFiltered = 0;
	static int yawValuePure = 0;

	/************************************************************
	 * Variables to be passed as argument to the serialPort class
	 ***********************************************************/
	static int baudRateValue = 0;
	static String portChoosen = null;

	
	public static void main(String[] args) {

		Runnable runner = new Runnable(){

			@Override
			public void run() {
				
				final float VERSION = 2.5f;
				Dimension dimScreenSize = Toolkit.getDefaultToolkit().getScreenSize();
				int xLocation;
				int yLocation;
				String[] baudRate = {"600", "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200", "576000"};
				
				
				/**************************
				 * Definition of the frame
				 * for the main application
				 *************************/	
				// Create the side panels
				// As a default the left is or kalman and the right side is from the sensors
				PanelGUI leftPanel = new PanelGUI("kalman");
				PanelGUI rightPanel = new PanelGUI("sensor");
				
				// Create the sub panel
				SubPanel subPanel = new SubPanel();
				
				// Create the center panel
				CenterPanel centerPanel = new CenterPanel();
				
				// Create the upper panel
				UpPanel upPanel = new UpPanel();
				
				// Create the SerialPort object at first
				final SerialCommunication serial = new SerialCommunication(leftPanel, rightPanel, subPanel, centerPanel, upPanel);
		
				
				/*************************
				 * Create here the menuBar
				 ************************/
				JMenuBar menuBar = new JMenuBar();

				
				/*************************
				 * Create the JComboBox for
				 * the BaudRate values
				 *************************/
				JLabel labelBaudRate = new JLabel("  Baudrate:  ");
				menuBar.add(labelBaudRate);
				
				JComboBox comboBoxBaudRate = new JComboBox(baudRate);
				menuBar.add(comboBoxBaudRate);
				
				ItemListener itemListener = new ItemListener(){
					@Override
					public void itemStateChanged(ItemEvent itemEvent) {
						if(itemEvent.getStateChange() == ItemEvent.SELECTED){
							baudRateValue = Integer.parseInt((String) itemEvent.getItem());
						}
					}
					
				};
				
				comboBoxBaudRate.addItemListener(itemListener);
				
				
				/****************************
				 * Create the JLabel for the 
				 * port found on the system
				 ****************************/
				JLabel labelPort = new JLabel("  Serial Port found:  ");
				menuBar.add(labelPort);
				
				
				/******************************
				 * Create the JComboBox for the
				 * ports found on the system
				 *****************************/
				DefaultComboBoxModel model = new DefaultComboBoxModel();
				ArrayList<String> portList = new ArrayList<String>();
				portList = serial.checkPorts();
				
				switch(serial.checkPorts().size()){
				case 0:
					model.addElement("NO PORT FOUND!!!");
					break;
					
				case 1:
					portChoosen = portList.get(0);
					model.addElement(portList.get(0));
					break;
					
				default:
					for (int i = 0; i < portList.size(); i++){
						model.addElement(portList.get(i));
					}
					break;
				}
				
				JComboBox boxPorts = new JComboBox(model);
				menuBar.add(boxPorts);
				
				ItemListener itemList = new ItemListener(){
					@Override
					public void itemStateChanged(ItemEvent itemEvent) {
						if(itemEvent.getStateChange() == ItemEvent.SELECTED){
							portChoosen = (String) (itemEvent.getItem());
						}
					}
				};

				boxPorts.addItemListener(itemList);
				
				
				/*******************************
				 * Create a JButton for starting
				 * the serial connection 
				 ******************************/
				JButton openSerial = new JButton("Open Serial");
				menuBar.add(openSerial);
				
				ActionListener openComm = new ActionListener(){
					@Override
					public void actionPerformed(ActionEvent e) {
						serial.openSerialComm(baudRateValue, portChoosen);
					}
				};
				openSerial.addActionListener(openComm);
				
				
				/******************************
				 * Create a JButton for closing
				 * the serial Connection
				 ******************************/
				JButton closeSerial = new JButton("Close Serial");
				menuBar.add(closeSerial);

				ActionListener closeComm = new ActionListener(){
					@Override
					public void actionPerformed(ActionEvent e) {
						serial.closeSerialComm();
					}
				};
				closeSerial.addActionListener(closeComm);
				

				
				/*************************************
				 * Create a Box for spacing JButton
				 ************************************/				
				menuBar.add(Box.createHorizontalStrut(300));
				
				
				/**********************
				 * Create a JButton for 
				 * closing the program
				 *********************/
				JButton closeApp = new JButton("Exit");
				menuBar.add(closeApp);
				
				ActionListener closeProg = new ActionListener(){
					@Override
					public void actionPerformed(ActionEvent evtProg) {
						serial.closeSerialComm();
						System.exit(0);
					}					
				};
				closeApp.addActionListener(closeProg);
				
				
				/*******************************
				 * Create a JButton for 
				 * information about the license
				 ******************************/
				JButton about = new JButton("About");
				menuBar.add(about);
				
				ActionListener aboutLic = new ActionListener(){
					@Override
					public void actionPerformed(ActionEvent arg0) {
						JOptionPane info = new JOptionPane("Artificial Horizon AH-3+  ver: " + VERSION + "\n\nCopyright (C) 2010 Davide Picchi" + "\n<paveway@gmail.com>" + "\n" + "\nLicensed under GNU GPL v3", JOptionPane.INFORMATION_MESSAGE);
						JDialog dialog = info.createDialog("License");
						dialog.setVisible(true);
					}
					
				};
				about.addActionListener(aboutLic);
				
			
				/*********************
				 * Create the main GUI
				 *********************/
				JFrame mainFrame = new JFrame("Artificial Horizon  Ver: " + VERSION);
				mainFrame.setUndecorated(true);
				mainFrame.getRootPane().setWindowDecorationStyle(JRootPane.FRAME);
				mainFrame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
				mainFrame.setResizable(true);
				mainFrame.setLayout(new BorderLayout());

				ImageIcon img = new ImageIcon(getClass().getResource("/Artificial_Horizon.gif"));
				Image image = img.getImage();
				mainFrame.setIconImage(image);
		
				mainFrame.getContentPane().add(BorderLayout.WEST, leftPanel);
				mainFrame.getContentPane().add(BorderLayout.EAST, rightPanel);
				mainFrame.getContentPane().add(BorderLayout.SOUTH, subPanel);
				mainFrame.getContentPane().add(BorderLayout.CENTER, centerPanel);
				mainFrame.getContentPane().add(BorderLayout.NORTH, upPanel);
				mainFrame.pack();		

				xLocation = (dimScreenSize.width - mainFrame.getWidth()) / 2;
				yLocation = (dimScreenSize.height - mainFrame.getHeight()) / 2;
				mainFrame.setLocation(xLocation, yLocation);
			
				
				mainFrame.addWindowListener(new WindowAdapter(){
					
						public void windowClosing(WindowEvent winEvt){
							int answer;
							answer = JOptionPane.showConfirmDialog(winEvt.getWindow(), "Exit the application?", "Please select:", JOptionPane.YES_NO_OPTION);
							if(answer == JOptionPane.YES_OPTION){
								serial.closeSerialComm();
								winEvt.getWindow().dispose();
								System.exit(0);
							}
						}
				});
				
				mainFrame.setJMenuBar(menuBar);
				
				mainFrame.setVisible(true);
			}
			
			
		};
		
		EventQueue.invokeLater(runner);
	}
}