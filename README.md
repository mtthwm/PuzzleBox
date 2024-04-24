# About

## Description
This is a project built for our ECE 5780 (Embedded Systems) final project. We built a puzzle box that requires the user to interact with it in novel ways to unlock and reveal the prize inside. 

### Puzzle 1: Knock and Response 

The buzzer plays a common knock rhythm, and the user must respond to finish the rhythm. 

### Puzzle 2:  

Led on sides of the box lights up, rotate the whole box so the LED points down. Repeat 3 times with a random side each time. 

### Puzzle 3: 

Using multiple photoresistors, a sequence of holes needs to either be lit or covered.  

### Puzzle Completion: 

Using a servo, unlock the box for opening. Puzzle restarts on close. 

## Collaborators
- Matthew Morales
- Daniel Reyna
- Casey Wolfe
- Griffin Rodgers

# Project Setup Instructions

## Pins
- I2C:  
  - SCL: B10 
  - SDA: B11 

- UART (debug):  
  - TX: PC10 
  - RX: PC11 

- ADC: TBD
  - Photoresistors
  - Piezo Sensor

- Buzzer: PB5 

- Box-Side LEDs 
	- PA8 -- Top LED 
	- PA9 – Base LED 
	- PA10 – Front LED 
	- PA11 – Back LED 
  - PA12 – Left Side LED 
  - PA13 – Right Side LED	 

- Servo: PB3 
