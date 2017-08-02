#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "terminal.h"
#include "commands.h"
#include "timeout.h"
#include "utils.h"

#include <math.h>

/* 
 * This is to control the pedelec engine of the carla cargo clones.
 * Its controlled by an analog hall generator that gives an estimate
 * how deep the towing bar is extracted by the pulling bike.
 * This is transmitted as an analog value via connector 1; 
 * Voltage reduces on aproaching magnet. Range usually 1.6 - 0.6V
 * http://cdn-reichelt.de/documents/datenblatt/A200/A1324-26-ALL.pdf
 * The Allegro 1324 is only specified for 5V, but works with 3.3 also.
 *
 * A set of reed switches is going to deliver kill switch and end of 
 * pull mechanic. 
 *
 * - -+--XXXXX--+--XXXX--+--XXXX--- + 3.3V
 *    |         |        |
 *    +----1----+---2----+
 * 
 *   1,2: Reed switch opening on approach
 * XXXXX: 10kOhm resistor
 *
 * The cruise control will provide a maximum speed where acceleration
 * stops.
 *
 * Optionally a bluetooth sensor will later on be added to transmit the 
 * crank rotations from the pulling bike.
 */



// s -> Analog in
// m -> +3V
// - -> GND

// Settings
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8

extern /*static volatile */ adc_config config;
static volatile float ms_without_power = 0;


int killswitch_deployed;
int rangeswitch_deployed;
float adc2_value;
float adc1_value;
float adc1_value1;
float adc1_value2;
float current_out;

/* TODO where to get these? */
float rpm_lim_start = 0.0;
float rpm_lim_end = 1.1;

int maxspeed = 25;
int umfang_mm = 1860;

// M_PI;

// Example thread
static THD_FUNCTION(mag_thread, arg);
static THD_WORKING_AREA(mag_thread_wa, 2048); // 2kb stack for this thread

const char *magCommands[] = {
	"killswitch",
	"get the state of the killswitch",
	""
};
int testint = 1337;
int breakactive = 0;
void killswitch_state_command(int argc, const char ** argv) {
	commands_printf("Current killswitch status: %d - %d - %g %g %g %g %g | %d", killswitch_deployed, rangeswitch_deployed, (double)adc2_value,  (double)adc1_value, (double)adc1_value1, (double)adc1_value2, (double)current_out, breakactive); // (double)config.voltage_start, (double)config.voltage_end);
}
// Current killswitch status: 0 - 0 -                                                                                      1.112088     1.000000    0.935604     inf          0.000000
//void broadcastState(unsigned char *data, unsigned int len) {}
void app_mag_init(void) {
	// Set the UART TX pin as an input with pulldown
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	terminal_register_command_callback(magCommands[0],
					   magCommands[1],
					   magCommands[2],					   
					   killswitch_state_command);
	// commands_set_app_data_handler(broadcastState);
	// Start the example thread
	chThdCreateStatic(mag_thread_wa, sizeof(mag_thread_wa),
		NORMALPRIO, mag_thread, NULL);
}

static THD_FUNCTION(mag_thread, arg) {
	(void)arg;
 
	chRegSetThreadName("APP_MAG");
	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		// ============================================
		// Read & map the kill switch
		
		float pot = (float)ADC_Value[ADC_IND_EXT2];
		pot /= 4095.0;
		pot *= V_REG;
		adc2_value = pot;
		killswitch_deployed = (pot < 1);
		rangeswitch_deployed = (pot > 1.4);

		// Current killswitch status: 1 - 0 - 0.000000
		// Current killswitch status: 0 - 0 - 1.112088
		// Current killswitch status: 0 - 1 - 1.635092

		// ============================================
		// Read & map the input from the hall generator
		// 
		// Read the external ADC pin and convert the value to a voltage.
		float pwr = (float)ADC_Value[ADC_IND_EXT];
		pwr /= 4095;
		pwr *= V_REG;

		// don't have that :( read_voltage = pwr;

		// Optionally apply a mean value filter
		if (config.use_filter) {
			static float filter_buffer[FILTER_SAMPLES];
			static int filter_ptr = 0;

			filter_buffer[filter_ptr++] = pwr;
			if (filter_ptr >= FILTER_SAMPLES) {
				filter_ptr = 0;
			}

			pwr = 0.0;
			for (int i = 0;i < FILTER_SAMPLES;i++) {
				pwr += filter_buffer[i];
			}
			pwr /= FILTER_SAMPLES;
		}
		// Map and truncate the read voltage
		adc1_value1 = pwr;
		// TODO: ranges not saved from the UI??? pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
		pwr = utils_map(pwr, 0.5, 1.73, 0.0, 1.0);
		adc1_value2 = pwr;
		utils_truncate_number(&pwr, 0.0, 1.0);
		adc1_value = pwr;

		if (killswitch_deployed) {
			current_out = 0.0;
			mc_interface_set_brake_current(timeout_get_brake_current());
			continue;
		}

		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
			// Scale the voltage and set 0 at the center
			pwr *= 2.0;
			pwr -= 1.0;
			break;

		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
			break;

		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
			// Invert the voltage if the button is pressed
			// if (rev_button) {
			//	pwr = -pwr;
			//}
			break;

		case ADC_CTRL_TYPE_CURRENT:

			break;
		default:
			break;
		}


                // Apply deadband
		utils_deadband(&pwr, config.hyst, 1.0);
		
		float rpm_local = mc_interface_get_rpm();
		float rpm_lowest = rpm_local;

		float current = 0.0;
		bool current_mode_brake = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	
	       
		// If safe start is enabled and the output has not been zero for long enough
		/*
		if (fabsf(pwr) < 0.001) {
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}

		if (ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());

			continue;
		}		

		*/
		
		// Reset timeout
		timeout_reset();


		
		// Filter RPM to avoid glitches
		static float filter_buffer[RPM_FILTER_SAMPLES];
		static int filter_ptr = 0;
		filter_buffer[filter_ptr++] = mc_interface_get_rpm();
		if (filter_ptr >= RPM_FILTER_SAMPLES) {
			filter_ptr = 0;
		}

		float rpm_filtered = 0.0;
		for (int i = 0;i < RPM_FILTER_SAMPLES;i++) {
			rpm_filtered += filter_buffer[i];
		}
		rpm_filtered /= RPM_FILTER_SAMPLES;



/*
		if (current_mode && cc_button && fabsf(pwr) < 0.001) {
			static float pid_rpm = 0.0;

			if (!was_pid) {
				was_pid = true;
				pid_rpm = rpm_filtered;
			}

			mc_interface_set_pid_speed(pid_rpm);
			continue;
		}
*/
		if (current_mode_brake) {
			breakactive = 1;
			mc_interface_set_brake_current(current);
		} else {
			breakactive = 0;
			// Apply soft RPM limit
			if (rpm_lowest > /*config.*/rpm_lim_end && current > 0.0) {
				current = mcconf->cc_min_current;
			} else if (rpm_lowest > /*config.*/rpm_lim_start && current > 0.0) {
				current = utils_map(rpm_lowest, /*config.*/rpm_lim_start, /*config.*/rpm_lim_end, current, mcconf->cc_min_current);
			} else if (rpm_lowest < - /*config.*/rpm_lim_end && current < 0.0) {
				current = mcconf->cc_min_current;
			} else if (rpm_lowest < - /*config.*/rpm_lim_start && current < 0.0) {
				rpm_lowest = -rpm_lowest;
				current = -current;
				current = utils_map(rpm_lowest, /*config.*/rpm_lim_start, /*config.*/rpm_lim_end, current, mcconf->cc_min_current);
				current = -current;
				rpm_lowest = -rpm_lowest;
			}
			else {
				if (pwr >= 0.0) {
					current = pwr * mcconf->l_current_max;
				} else {
					current = pwr * fabsf(mcconf->l_current_min);
				}
			}

			float current_out = current;
			bool is_reverse = false;
			if (current_out < 0.0) {
				is_reverse = true;
				current_out = -current_out;
				current = -current;
				rpm_local = -rpm_local;
				rpm_lowest = -rpm_lowest;
			}


			if (is_reverse) {
				mc_interface_set_current(-current_out);
			} else {
				mc_interface_set_current(current_out);
			}
		}
	}
}
