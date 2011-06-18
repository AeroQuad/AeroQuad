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
 * (and make sure they are running) from the link above.
 * 
 * Written in Java JVM 1.6.12 (on Debian 5.0 Lenny)
 * 
 * @author wilhem (Davide Picchi)
 * 
 * 
 * *****************************
 * starting date: 15 Januar 2010
 * 
 * last update: 01 Februar 2010
 **********************************************/

package AeroQuad.PCRemoteController.srcimport.ArtificialHorizon;

import gnu.io.*;

import javax.swing.*;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.TooManyListenersException;


public class SerialCommunication implements SerialPortEventListener,Runnable{

	@SuppressWarnings("unchecked")
	private Enumeration portList = null;
	private CommPortIdentifier portId = null;
	private String defaultPort;
	private int baudRate;
	private SerialPort serialPort = null;
	private boolean portFound = false;
	private InputStream is = null;
	private boolean readThreadAlive = false;
	private BufferedReader inStream = null;
	private ArrayList<String> listPorts;
	
	private JPanel lPanel;
	private JPanel rPanel;
	private JPanel subPan;
	private JPanel centerPanel;
	private JPanel upPan;
	
	Thread readThread;
	
	
	public SerialCommunication(JPanel a, JPanel b, JPanel c, JPanel d, JPanel e){
		this.readThread = new Thread(this);
		this.lPanel = a;
		this.rPanel = b;
		this.subPan = c;
		this.centerPanel = d;
		this.upPan = e;
	}
	
	
	/************************************
	 * Gives a list of the present ports
	 * on the system
	 ***********************************/
	public ArrayList<String> checkPorts(){
		
		this.listPorts = new ArrayList<String>();
		
		portList = CommPortIdentifier.getPortIdentifiers();
		
		while(portList.hasMoreElements()){
			portId = (CommPortIdentifier)portList.nextElement();
			listPorts.add(portId.getName().toString());
			}
		
		return listPorts;      // Return the ports found on the system
	}
	
	
	/**********************************************
	 * Opens the serial port and sets:
	 * @param bdR: baudRate choosen in the first frame
	 * @param defPort: port choosen in the frame
	 **********************************************/
	public void openSerialComm(int bdR, String defPort){

		/**********************
		 * Open the serial port
		 *********************/
		
		this.baudRate = bdR;
		this.defaultPort = defPort;
		
		if(portId != null){               // If portId contains at least one element!!!
			if(portId.getName().equals(defaultPort)){
				portFound = true;
				System.out.println("Port found on: " + defaultPort);
			}else{
				portFound = false;  // Don't execute the cycle below
			}
		}

		
		if(portFound){
			
				try{
					serialPort = (SerialPort)portId.open("Artificial Horizont", 2000);
				} catch (PortInUseException ex){
					System.err.println("Port already in use!");
				}
				
				// Get input stream
				try{
					is = serialPort.getInputStream();
				} catch (IOException e){
					System.err.println("Cannot open Input Stream " + e);
					is = null;
				}
				
				
				try{
					inStream = new BufferedReader(new InputStreamReader(is), 5);
				} catch (IllegalArgumentException eft){
					System.err.println("Increase the buffer!!!");
				}
				
				
				try{
					serialPort.setSerialPortParams(this.baudRate,
							                       SerialPort.DATABITS_8,
							                       SerialPort.STOPBITS_1,
		                                           SerialPort.PARITY_NONE);
				} catch (UnsupportedCommOperationException ex){
					System.err.println("Wrong settings for the serial port: " + ex.getMessage());
				}
				
				
				try{
					serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
				} catch (UnsupportedCommOperationException ex){
					System.err.println("Check the flow control setting: " + ex.getMessage());
				}
				
				// Add an event Listener
				try{
					serialPort.addEventListener(this);
				} catch (TooManyListenersException ev){
					System.err.println("Too many Listeners! " + ev);
				}
				
				// Advise if data available to be read on the port
				serialPort.notifyOnDataAvailable(true);

				if(readThreadAlive){
					// Do nothing
				}else{
					try{
						readThread.start();
						readThreadAlive = true;
					} catch(IllegalThreadStateException evtTh){
						System.out.println("Problems with Threads");
					}
				}
				
				System.out.println("Port: " + defaultPort + " opened");
	      }
	}

	
	
	public void closeSerialComm(){
		
		if(readThreadAlive){
			
			if(is != null){
				try {
					is.close();
				} catch (IOException e) {
					System.err.println("Cannot close the InputStream!!!");
				}
			}
	
			if(inStream != null){
     			try {
					inStream.close();
				} catch (IOException e) {
					System.err.println("Cannot close the BufferedReader InputStream!!!");
				}
		}

			serialPort.close();
			
			System.out.println("Serial Port closed!!");
		}
	}
	
			
	
	@Override
	public void run() {
		
		while(readThreadAlive){
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
		 }
	}

	
	@Override
	public void serialEvent(SerialPortEvent event) {
	
		switch(event.getEventType()){
		case SerialPortEvent.BI:
		case SerialPortEvent.CD:
		case SerialPortEvent.CTS:
		case SerialPortEvent.DSR:
		case SerialPortEvent.FE:
		case SerialPortEvent.OE:
		case SerialPortEvent.PE:
		case SerialPortEvent.RI:
		case SerialPortEvent.OUTPUT_BUFFER_EMPTY:
			break;
			
		case SerialPortEvent.DATA_AVAILABLE:
			analizeIncomingData();
			break;
			
		default:
			break;
		}
	}
	
	
	private void analizeIncomingData(){
		
		String rawInput = null;
		int len;
		String num;
		String field;
		
		try {
			while((rawInput = inStream.readLine()) != null){
				
				len = rawInput.length();
				
				// Check the end of the incoming string
				try{
					field = rawInput.substring(len - 2, len);
					
					num = rawInput.substring(0, len - 2);
					
					if(field.contentEquals("kp")){
						ArtificialHorizon.pitchValueFiltered = Integer.parseInt(num);					
					}

					if(field.contentEquals("kr")){
						ArtificialHorizon.rollValueFiltered = Integer.parseInt(num);
					}
					
					if(field.contentEquals("ky")){
						ArtificialHorizon.yawValueFiltered = Integer.parseInt(num);
					}
					
					if(field.contentEquals("pp")){
						ArtificialHorizon.pitchValuePure = Integer.parseInt(num);
					}
					
					if(field.contentEquals("rr")){
						ArtificialHorizon.rollValuePure = Integer.parseInt(num);
					}
					
					if(field.contentEquals("yy")){
						ArtificialHorizon.yawValuePure = Integer.parseInt(num);
					}
					
				} catch (IndexOutOfBoundsException eft){
					System.out.println("Problem with string");
				} catch (NumberFormatException numEvt){
					System.out.println("Not a number, but a string");
				}

					lPanel.repaint();
					rPanel.repaint();
					subPan.repaint();
					centerPanel.repaint();
					upPan.repaint();
				}
			
		} catch (IOException e) {
		}	
    }

}
