/***************************************************************************
 tag: Sava Marinkov, Ruud van den Bogaert,  Fri Mar 23 12:44:00 CET 2011  soem_armEthercat.cpp

 soem_armEthercat.cpp -  dedicated ethercat module TU/e
 -------------------
 begin                : Fri November 23 2012
 copyright            : (C) 2012 Sava Marinkov & Ruud van den Bogaert
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
			port_out_encoderAngle0("encoderAngle0"),
			port_out_encoderAngle1("encoderAngle1"),
			port_out_encoderAngle2("encoderAngle2"),
			port_out_forceSensors("forceSensors"),
			port_out_positionSensors("positionSensors"),
			port_out_spareAnalogIns("spareAnalogIns"),
			port_out_motorCurrents("motorCurrents"),
			port_out_supplyVoltages("supplyVoltages"),
			
			port_out_spareDigitalIns("spareDigitalIns"),
			port_out_pwmDutyMotors("pwmDutyMotorsOut"),
			port_in_pwmDutyMotors("pwmDutyMotorsIn"),
			port_in_spareDigitalOuts("spareDigitalOutsIn"),
			port_out_spareDigitalOuts("spareDigitalOutsOut") {

	m_service->doc(
			std::string("Services for custom EtherCat ") + std::string(
					m_datap->name) + std::string(" module"));

	m_service->addOperation("write_pwm", &SoemARMETHERCAT::write_pwm, this, RTT::OwnThread).doc("Write pwm duty values");
	m_service->addOperation("read_encoders", &SoemARMETHERCAT::read_encoders, this, RTT::OwnThread).doc("Read encoder values");
	m_service->addOperation("read_supply", &SoemARMETHERCAT::read_supply, this, RTT::OwnThread).doc("Read power supply voltages");
	m_service->addOperation("read_forces", &SoemARMETHERCAT::read_forces, this, RTT::OwnThread).doc("Read force sensor values");
	m_service->addOperation("read_positions", &SoemARMETHERCAT::read_positions, this, RTT::OwnThread).doc("Read position sensor values");
	m_service->addOperation("read_spareanalog", &SoemARMETHERCAT::read_spareanalog, this, RTT::OwnThread).doc("read_spareanalog");
	m_service->addOperation("read_currents", &SoemARMETHERCAT::read_currents, this, RTT::OwnThread).doc("read_currents");

	//	m_service->addProperty(m_params[i].name, m_params[i].param).doc(m_params[i].description);

	m_service->addPort(port_out_encoderAngle0).doc("");
	m_service->addPort(port_out_encoderAngle1).doc("");
	m_service->addPort(port_out_encoderAngle2).doc("");
	m_service->addPort(port_out_forceSensors).doc("");
	m_service->addPort(port_out_positionSensors).doc("");
	m_service->addPort(port_out_spareAnalogIns).doc("");
	m_service->addPort(port_out_motorCurrents).doc("");
	m_service->addPort(port_out_supplyVoltages).doc("");

	m_service->addPort(port_out_spareDigitalIns).doc("");
	m_service->addPort(port_out_pwmDutyMotors).doc("");
	m_service->addPort(port_in_pwmDutyMotors).doc("");
	m_service->addPort(port_in_spareDigitalOuts).doc("");
	m_service->addPort(port_out_spareDigitalOuts).doc("");

	positionSensors_msg.values.resize(3);
	forceSensors_msg.values.resize(3);
	spareAnalogIns_msg.values.resize(2);
	supplyVoltages_msg.values.resize(5);
	motorCurrents_msg.values.resize(3);
	pwmDutyMotors_msg.values.resize(3);

}

bool SoemARMETHERCAT::configure() {
	
	positionSensors.resize(3);
	forceSensors.resize(3);
	spareAnalogIns.resize(2);
	supplyVoltages.resize(5);
	motorCurrents.resize(3);
	pwmDutyMotors.resize(3);
	setOutputToZero = false;
	
	return true;
}

void SoemARMETHERCAT::update() {
	
	// This is to avoid random outputs at startup
	// Might work but is kind off hacky
	// One (short) output sample might be a random value
	
	if (!setOutputToZero) {
		log(Info) << "Doing extra output update to start with zero outputs" << endlog();
		m_out_armEthercat = ((out_armEthercatMemoryt*) (m_datap->outputs));
		log(Debug) << "PWM Values before initialization are " << m_out_armEthercat->pwm_duty_motor_1 << "\t" << m_out_armEthercat->pwm_duty_motor_2 << "\t" << m_out_armEthercat->pwm_duty_motor_3 << endlog();
		write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
		log(Debug) << "PWM Values after initialization are  " << m_out_armEthercat->pwm_duty_motor_1 << "\t" << m_out_armEthercat->pwm_duty_motor_2 << "\t" << m_out_armEthercat->pwm_duty_motor_3 << endlog();
		setOutputToZero = true;
	}
	
	m_in_armEthercat = ((in_armEthercatMemoryt*) (m_datap->inputs));
	m_out_armEthercat = ((out_armEthercatMemoryt*) (m_datap->outputs));
	
	read_positions();
	read_forces();
	read_supply();
	read_spareanalog();
	read_encoders();
	read_currents();
		
	if (port_in_pwmDutyMotors.connected()) {
		if (port_in_pwmDutyMotors.read(pwmDutyMotors_msg) == NewData) {
			write_pwm((pwmDutyMotors_msg.values[0]),(pwmDutyMotors_msg.values[1]),(pwmDutyMotors_msg.values[2]));
		}
	}

}

void SoemARMETHERCAT::read_supply(){
	float supply_int_1V2 = (m_in_armEthercat->supply_int_1V2); 						// conversion to Volts into the FPGA
	float supply_int_1V65 = (m_in_armEthercat->supply_int_1V65);					// conversion to Volts into the FPGA
	float supply_int_5V = (m_in_armEthercat->supply_int_5V);						// conversion to Volts into the FPGA
	float supply_int_12V = (m_in_armEthercat->supply_int_12V);					 	// conversion to Volts into the FPGA
	float supply_ext_24V = (m_in_armEthercat->supply_ext_24V);					 	// conversion to Volts into the FPGA

	supplyVoltages_msg.values[0] = supply_int_1V2;
	supplyVoltages_msg.values[1] = supply_int_1V65;
	supplyVoltages_msg.values[2] = supply_int_5V;
	supplyVoltages_msg.values[3] = supply_int_12V;
	supplyVoltages_msg.values[4] = supply_ext_24V;

	port_out_supplyVoltages.write(supplyVoltages_msg);

	// log(Info) << "Supply: ["<< supply_int_1V2 << ", "<< supply_int_1V65 << ", " << supply_int_5V <<  ", " << supply_int_12V << ", " << supply_ext_24V << "]" << endlog();
}

void SoemARMETHERCAT::read_forces(){
	float force1 = (m_in_armEthercat->force_1)/2047*3.3;  	 //11 bits over 3,3V or 12 bits over 6,6V
	float force2 = (m_in_armEthercat->force_2)/2047*3.3;	 //11 bits over 3,3V or 12 bits over 6,6V
	float force3 = (m_in_armEthercat->force_3)/2047*3.3;	 //11 bits over 3,3V or 12 bits over 6,6V


	 forceSensors_msg.values[0] = force1;
	 forceSensors_msg.values[1] = force2;
	 forceSensors_msg.values[2] = force3;

	 port_out_forceSensors.write(forceSensors_msg);

	//log(Info) << "Forces: ["<< force1 << ", "<< force2 << ", " << force3 << "]" << endlog();
}

void SoemARMETHERCAT::read_encoders(){
	uint16  enc1 = m_in_armEthercat->encoder_angle_1;
	uint16  enc2 = m_in_armEthercat->encoder_angle_2;
	uint16  enc3 = m_in_armEthercat->encoder_angle_3;

	encoderAngle0_msg.value = m_in_armEthercat->encoder_angle_1;
	encoderAngle1_msg.value = m_in_armEthercat->encoder_angle_2;
	encoderAngle2_msg.value = m_in_armEthercat->encoder_angle_3;

	port_out_encoderAngle0.write(encoderAngle0_msg);
	port_out_encoderAngle1.write(encoderAngle1_msg);
	port_out_encoderAngle2.write(encoderAngle2_msg);


	//log(Info) << "Angles: ["<< enc1 << ", "<< enc2 << ", " << enc3 << "]" << endlog();

}

void SoemARMETHERCAT::read_currents(){
	float current1 = (m_in_armEthercat->current_motor_1);
	float current2 = (m_in_armEthercat->current_motor_2);
	float current3 = (m_in_armEthercat->current_motor_3);

		// motorCurrents[0] = current1;
		// motorCurrents[1] = current2;
		// motorCurrents[2] = current3;

	    motorCurrents_msg.values[0] = current1;
		motorCurrents_msg.values[1] = current2;
		motorCurrents_msg.values[2] = current3;

		port_out_motorCurrents.write(motorCurrents_msg);

		// log(Info) << "Currents: ["<< current1 << ", "<< current2 << ", " << current3 << "]" << endlog();
}

void SoemARMETHERCAT::read_positions(){
	float position1 = (m_in_armEthercat->position_1)/2047*3.3;
	float position2 = (m_in_armEthercat->position_2)/2047*3.3;
	float position3 = (m_in_armEthercat->position_3)/2047*3.3;

	positionSensors_msg.values[0] = position1;
	positionSensors_msg.values[1] = position2;
	positionSensors_msg.values[2] = position3;
	
	port_out_positionSensors.write(positionSensors_msg);

	// log(Info) << "Positions: ["<< position1 << ", "<< position2 << ", " << position3 << "]" << endlog();
}

void SoemARMETHERCAT::read_spareanalog(){
	float spareanalogin1 = (m_in_armEthercat->spare_analog_in_1)/2047*3.3;
	float spareanalogin2 = (m_in_armEthercat->spare_analog_in_1)/2047*3.3;

	spareAnalogIns_msg.values[0] = spareanalogin1;
	spareAnalogIns_msg.values[1] = spareanalogin2;

	port_out_spareAnalogIns.write(spareAnalogIns_msg);

}

void SoemARMETHERCAT::write_pwm(float val1, float val2, float val3) {
	m_out_armEthercat->ramp_rate = 100;
	// convert to signed int value
	int16 tmp1 = (int16)val1;
	int16 tmp2 = (int16)val2;
	int16 tmp3 = (int16)val3;

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
		
	// send the command to the motors
	m_out_armEthercat->pwm_duty_motor_1 = tmp1;
	m_out_armEthercat->pwm_duty_motor_2 = tmp2;
	m_out_armEthercat->pwm_duty_motor_3 = tmp3;

	//  port_in_pwmDutyMotors.write(pwmDutyMotors_msg);  (hier ben ik gebleven pop 23-11-12)

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
