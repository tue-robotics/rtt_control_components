/***************************************************************************
 tag: Sava Marinkov, Ruud van den Bogaert,  Fri Mar 23 12:44:00 CET 2011  soem_armEthercat.cpp

 soem_armEthercat.cpp -  dedicated ethercat module TU/e
 -------------------
 begin                : Fri November 23 2012
 copyright            : (C) 2012 Sava Marinkov & Ruud van den Bogaert & Max Baeten
 email                : s.marinkov@student.tue.nl , r.v.d.bogaert@tue.nl

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include "soem_armEthercat.hpp"

using namespace soem_beckhoff_drivers;

SoemARMETHERCAT::SoemARMETHERCAT(ec_slavet* mem_loc) :
	soem_master::SoemDriver(mem_loc), 
			port_out_encoderAngle1("encoderAngle1"),
			port_out_encoderAngle2("encoderAngle2"),
			port_out_encoderAngle3("encoderAngle3"),
			port_out_positionSensors("positionSensors"),
  			port_out_forceSensors("forceSensors"),
            port_in_pwmDutyMotors("pwmDutyMotorsIn"),
            port_in_enable("enablePort"){

		m_service->doc(
			std::string("Services for custom EtherCat ") + std::string(
					m_datap->name) + std::string(" module"));

	m_service->addOperation("write_pwm", &SoemARMETHERCAT::write_pwm, this, RTT::OwnThread).doc("Write pwm duty values");
	m_service->addOperation("read_encoders", &SoemARMETHERCAT::read_encoders, this, RTT::OwnThread).doc("Read encoder values");
 	m_service->addOperation("read_forces", &SoemARMETHERCAT::read_forces, this, RTT::OwnThread).doc("Read force sensor values");
 	m_service->addOperation("read_positions", &SoemARMETHERCAT::read_positions, this, RTT::OwnThread).doc("Read position sensor values");

	m_service->addPort(port_out_encoderAngle1).doc("");
	m_service->addPort(port_out_encoderAngle2).doc("");
	m_service->addPort(port_out_encoderAngle3).doc("");
	m_service->addPort(port_out_positionSensors).doc("");
 	m_service->addPort(port_out_forceSensors).doc("");
	m_service->addPort(port_in_pwmDutyMotors).doc("");
    m_service->addPort(port_in_enable).doc("");

 	positionSensors_msg.values.resize(3);
 	forceSensors_msg.values.resize(3);
	pwmDutyMotors_msg.values.resize(3);
}

bool SoemARMETHERCAT::configure() {
	
    positionSensors.resize(3);
  	forceSensors.resize(3);
	pwmDutyMotors.resize(3);
	
	setOutputToZero = false;
	enablestatus = true;
	
    cntr = 0;	
	heartbeat = 0;
    printEnabled = 0;
    printDisabled = 1;
    j=0;
    statusregister_prev = 0;
    
    controlregister = 0x00;		// heart beat on 		+ emergency button detection on  
    //controlregister = 0x01;	// heart beat off 		+ emergency button detection on
	//controlregister = 0x02;   // heart beat on 		+ emergency button detection off
	//controlregister = 0x03;	// heart beat off 		+ emergency button detection off  
   
    // This is to avoid random outputs at startup
    if (!setOutputToZero) {
        log(Info) << "Doing extra output update to start with zero outputs" << endlog();
        m_out_armEthercat = ((out_armEthercatMemoryt*) (m_datap->outputs));
        log(Info) << "PWM Values before initialization are " << m_out_armEthercat->pwm_duty_motor_1 << "\t" << m_out_armEthercat->pwm_duty_motor_2 << "\t" << m_out_armEthercat->pwm_duty_motor_3 << endlog();
        write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
        setOutputToZero = true;
    }
        
	return true;
}

void SoemARMETHERCAT::update() {
	
    if (port_in_enable.connected()) {
        port_in_enable.read(enable);
        if (enablestatus == true)
			log(Warning)<<"Soem_armEthercat: Port_in_enable is connected"<<endlog();
        enablestatus = false;
		}
		
    else if (!port_in_enable.connected()) {
		if (enablestatus == false)	{ 
			log(Warning)<<"Soem_armEthercat: Waiting for Enable Signal, port_in_enable is not connected"<<endlog();
			enablestatus = true;
		}
    }
    
    // heartbeat
    heartbeat++;
    if (heartbeat > 250)
	{
		heartbeat = 0;  
	}	
    
    m_out_armEthercat->heart_beat = heartbeat;
	m_out_armEthercat->control_register = controlregister;														

	m_in_armEthercat = ((in_armEthercatMemoryt*) (m_datap->inputs));
	m_out_armEthercat = ((out_armEthercatMemoryt*) (m_datap->outputs));
	
	read_positions();
 	read_forces();
	read_encoders();
		
	if (port_in_pwmDutyMotors.connected()) {
		if (port_in_pwmDutyMotors.read(pwmDutyMotors_msg) == NewData) {
			write_pwm((pwmDutyMotors_msg.values[0]),(pwmDutyMotors_msg.values[1]),(pwmDutyMotors_msg.values[2]));
		}
    }

    // If enable is true no error occured in the system
    if(enable){
        if(printEnabled==0){
            printEnabled++;
            printDisabled=0;
            if(cntr != 0) {
				log(Warning)<<"Soem_armEthercat: enable = true -> PWM output enabled" <<endlog();
			cntr=0;
			}
        }
    }
    else if(!enable){
        if(printDisabled==0){
            log(Warning)<<"Soem_armEthercat: enable = false -> PWM output set to zero"<<endlog();
            printDisabled++;
            printEnabled=0;
            cntr++;
        }
        write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
    }
    
    // status_register warnings
    statusregister = m_in_armEthercat->status_register;
    
    //j++;	
    if ((statusregister & 0x03) != (statusregister_prev & 0x03)) {		
		if ((statusregister & 0x03) == 0x00 ) { log(Info)<< "Soem_armEthercat Status:  E_OK" << endlog(); }
		if ((statusregister & 0x03) == 0x01 ) {
			if ((statusregister ^ 0X7C) & 0x04 ) { 	log(Info)<< "Soem_armEthercat Status:  E_POWER_DOWN: 5V" << endlog(); }
			if ((statusregister ^ 0X7C) & 0x08 ) { 	log(Info)<< "Soem_armEthercat Status:  E_POWER_DOWN: 12V" << endlog(); }
			if ((statusregister ^ 0X7C) & 0x10 ) { 	log(Info)<< "Soem_armEthercat Status:  E_POWER_DOWN: 24V" << endlog(); }
			if ((statusregister ^ 0X7C) & 0x20 ) { 	log(Info)<< "Soem_armEthercat Status:  E_POWER_DOWN: 1.2V" << endlog(); }
			if ((statusregister ^ 0X7C) & 0x40 ) { 	log(Info)<< "Soem_armEthercat Status:  E_POWER_DOWN: 1.65V" << endlog(); }
			}			
		if ((statusregister & 0x03) == 0x02 ) {	log(Info)<< "Soem_armEthercat Status:  E_COMM_DOWN" << endlog(); }	
		if ((statusregister & 0x03) == 0x03 ) { log(Error)<< "Soem_armEthercat Status:  E_NO_OP_STATE" << endlog(); }
	
		statusregister_prev = statusregister;
	}      
}

void SoemARMETHERCAT::read_forces(){
		
	float force1 = (float) m_in_armEthercat->force_1;
	float force2 = (float) m_in_armEthercat->force_2;
	float force3 = (float) m_in_armEthercat->force_3;
	
	 forceSensors_msg.values[0] = (force1/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V
	 forceSensors_msg.values[1] = (force2/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V
	 forceSensors_msg.values[2] = (force3/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V

	 port_out_forceSensors.write(forceSensors_msg);

	//log(Warning) << "Forces: ["<< force1 << ", "<< force2 << ", " << force3 << "]" << endlog();
}

void SoemARMETHERCAT::read_encoders(){

    enc1 = m_in_armEthercat->encoder_angle_1;
    enc2 = m_in_armEthercat->encoder_angle_2;
    enc3 = m_in_armEthercat->encoder_angle_3;

    encoderAngle1_msg.value = (enc1);
    encoderAngle2_msg.value = (enc2);
    encoderAngle3_msg.value = (enc3);   

	port_out_encoderAngle3.write(encoderAngle3_msg); 
	port_out_encoderAngle2.write(encoderAngle2_msg);
	port_out_encoderAngle1.write(encoderAngle1_msg);

	//log(Info) << "Angles: ["<< enc1 << ", "<< enc2 << ", " << enc3 << "]" << endlog();
}

void SoemARMETHERCAT::read_positions(){
	float position1 = (float) m_in_armEthercat->position_1;
	float position2 = (float) m_in_armEthercat->position_2;
	float position3 = (float) m_in_armEthercat->position_3;

	positionSensors_msg.values[0] = position1;
	positionSensors_msg.values[1] = position2;
	positionSensors_msg.values[2] = position3;
	
	port_out_positionSensors.write(positionSensors_msg);

	// log(Info) << "Positions: ["<< position1 << ", "<< position2 << ", " << position3 << "]" << endlog();
}

void SoemARMETHERCAT::write_pwm(float val1, float val2, float val3) {
    int16 tmp1 = (int16) val1;
	int16 tmp2 = (int16) val2;
	int16 tmp3 = (int16) val3;

	// limit the duty cycles
	if (tmp1 > 100)
		tmp1 = 100;
	if (tmp1 < -100)
		tmp1 = -100;

	if (tmp2 > 100)
		tmp2 = 100;
	if (tmp2 < -100)
		tmp2 = -100;

	if (tmp3 > 100)
		tmp3 = 100;
	if (tmp3 < -100)
		tmp3 = -100;
		
	m_out_armEthercat->pwm_duty_motor_1 = tmp1;
	m_out_armEthercat->pwm_duty_motor_2 = tmp2;
	m_out_armEthercat->pwm_duty_motor_3 = tmp3;

	//log(Warning) << "PWM are set to:[ " << tmp1 << "," <<  tmp2 << "," << tmp3 << "]" << endlog();
    //log(Info) << "PWM duty are set to [0x" << hex << tmp1 << ", " << "0x" << hex << tmp2 << ", " << "0x" << hex << tmp3 << "]" << endlog();
}

namespace {
soem_master::SoemDriver* createSoemARMETHERCAT(ec_slavet* mem_loc) {
	return new SoemARMETHERCAT(mem_loc);
}

const bool registered0 =
		soem_master::SoemDriverFactory::Instance().registerDriver(
				"EPC ESRA FGPA Ethercat Slave", createSoemARMETHERCAT);
}
