/*
 * shaft_controller.h
 *
 *  Created on: Oct 26, 2023
 *      Author: ADMIN
 */

#ifndef INC_SHAFT_CONTROLLER_H_
#define INC_SHAFT_CONTROLLER_H_

void setup();
void loop();
void loadlidarvalue();

void indicate_health();
void test_ll();
void test_motor();
void test_callBookingClear();
void test_door_solenoid();
void testSiren();
void checkReg3();
void switchMessages(int state);
void servoTestCode();
void zeroCrossTestCode();
void testCodeHere();

#endif /* INC_SHAFT_CONTROLLER_H_ */
