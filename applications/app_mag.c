#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "terminal.h"
#include "commands.h"
#include "timeout.h"
#include "utils.h"
#include "app.h"

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
 *              +- ADC2 connector
 * 
 *   1,2: Reed switch opening on approach
 *   Reed 1: Kill Switch (Brown/Red)
 *   Reed 2: Range Switch / measure RPM (Red/orange)
 * XXXXX: 10kOhm resistor
 *
 * The cruise control will provide a maximum speed where acceleration
 * stops.
 * 
 * The Allegro values are mapped from 0.0 to 1.0.
 * 0.5 is the magic balance spot which we try to keep. 
 * our primary scale is RPM. If above 0.5 we increase the RPM to reach.
 * once the RPMs we try to reach, and that we set reach a delta of 10, 
 * we put more bang to it by increasing the voltage. 
 * we reduce the voltage once the 0.5 are undergone, der RPMs once its below
 * 0.3 
 * 
 * When the 'Invert reverse button' is checked, we will output measured
 * physical rpms via the commandline. This evaluates to 256 Rotations
 * for the trio bafang.
 *
 * TODO: howto calculate the power we give? we have to limit at 250W
 * via the current.
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
float want_rpm;
float rpm_filtered;
float measure_rpm;

/* TODO where to get these? */
float rpm_lim_start = 0.0;
float rpm_lim_end = 1.1;

int maxspeed = 25;
int umfang_mm = 1860;
systime_t deltaT;
systime_t now;
int rev_counter = 0;
int tacho_count = 0;
// M_PI;
static int switch_deployed = 0;
static int pass_counter = 0;
static systime_t last_time = 1;

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
  (void) argc;
  (void) argv;
	commands_printf("Current killswitch status: %d - %d - %d - %g %g %g frpm: %g wrpm: %g hrpm: %g | %d | %d",
			killswitch_deployed, rangeswitch_deployed, deltaT,
			(double)adc2_value,  (double)adc1_value, (double)adc1_value1,
			(double)rpm_filtered, (double)want_rpm, (double)measure_rpm,
			tacho_count, breakactive); // (double)config.voltage_start, (double)config.voltage_end);
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
	want_rpm = /*config.*/rpm_lim_start;
	rpm_filtered = 0.0;
}

static THD_FUNCTION(mag_thread, arg) {
	(void)arg;
 
	chRegSetThreadName("APP_MAG");
	// Sleep for a time according to the specified rate
	systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

	// At least one tick should be slept to not block the other threads
	if (sleep_time == 0) {
		sleep_time = 1;
	}
	
	for(;;) {
		chThdSleep(sleep_time);

		// ============================================
		// Read & map the kill switch
		
		float pot = app_adc_get_voltage2(); // (float)ADC_Value[ADC_IND_EXT2];
		pot /= 4095.0;
		pot *= V_REG;
		adc2_value = pot;
		killswitch_deployed = (pot < 1);
		rangeswitch_deployed = (pot > 1.4);

		// Current killswitch status: 1 - 0 - 0.000000
		// Current killswitch status: 0 - 0 - 1.112088
		// Current killswitch status: 0 - 1 - 1.635092

		// measure physical RPMs by reed switch.
		if (config.rev_button_inverted){
			if ((switch_deployed == 0) && (rangeswitch_deployed == 1)) {
				rev_counter ++;
				tacho_count = mc_interface_get_tachometer_value(true);
				if (rev_counter == 100) {
					now = chVTGetSystemTime();
					deltaT = ST2S(now - last_time);
					last_time = now;
					rev_counter = 0;
				}
				pass_counter = 0;
				switch_deployed = 1;
			}
			else {
				pass_counter ++;
				switch_deployed = rangeswitch_deployed;
			}
		}
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
		adc1_value2 = utils_map(adc1_value1, config.voltage_start, config.voltage_end, 0.0, 1.0);
		utils_truncate_number(&adc1_value2, 0.0, 1.0);
		if (config.voltage_inverted) {
			adc1_value2 = 1.0 - adc1_value2;
		}
		adc1_value = pwr = adc1_value2;

		if (killswitch_deployed) {
			rpm_filtered = 0.0;
			current_out = 0.0;
			want_rpm = /*config.*/rpm_lim_start;
			mc_interface_set_brake_current(timeout_get_brake_current());
			continue;
		}

                // Apply deadband
		utils_deadband(&pwr, config.hyst, 1.0);


		// Apply throttle curve
		pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// Apply ramping
		static systime_t this_last_time = 0;
		static float pwr_ramp = 0.0;
		const float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(this_last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&pwr_ramp, pwr, ramp_step);
			this_last_time = chVTGetSystemTimeX();
			pwr = pwr_ramp;
		}



		
		float rpm_local = mc_interface_get_rpm();
		// float rpm_lowest = rpm_local;

		static float current = 0.0;
		// bool current_mode_brake = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		
		// Reset timeout
		timeout_reset();
		
		// Filter RPM to avoid glitches
		static float filter_buffer[RPM_FILTER_SAMPLES];
		static int filter_ptr = 0;
		filter_buffer[filter_ptr++] = mc_interface_get_rpm();
		if (filter_ptr >= RPM_FILTER_SAMPLES) {
			filter_ptr = 0;
		}

		for (int i = 0;i < RPM_FILTER_SAMPLES;i++) {
			rpm_filtered += filter_buffer[i];
		}
		rpm_filtered /= RPM_FILTER_SAMPLES;

		if (pwr > 0.55) {
			if (want_rpm < /*config.*/rpm_lim_end) {
				want_rpm += 1.0;
			}
			if (want_rpm > /*config.*/rpm_lim_end) {
				want_rpm = /*config.*/rpm_lim_end;
			}
			// if either want rpm vs. have outgrows a limit,
			// or the throttle demand is bigger, we add more power:
			if ((pwr > 0.75) || (want_rpm - rpm_filtered > 10.0)) {
				if (current < mcconf->l_current_max) {
					current += 1.0;
				}
			}
		}
		else if (pwr > 0.45) {
			// Sweet spot! stay here!
			continue;
		}
		else {
			if (want_rpm > /*config.*/rpm_lim_start) {
				want_rpm -= 1.0;
			}
			if ((pwr < 0.2) || (rpm_filtered - want_rpm > 10.0)) {
				if (current > mcconf->l_current_min) {
					current -= 1.0;
				}
				if (current < 0.0) {
					current = 0.0;
				}
			}
		}

		mc_interface_set_pid_speed(want_rpm);
		mc_interface_set_current(current);

	}
}
