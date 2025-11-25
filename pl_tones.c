/*
 * PL tone detector card
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/bootrom.h"
#include "hardware/dma.h"

// Compilation options
// #define		SINGLE_SHOT
// #define		PRINT_RANGE
// #define		PRINT_DCS_DECODE

// Don't change SAMPLE_SIZE without looking carefully at wave_out
// the values of 465, 467 and 470 are in the code

#define	SAMPLE_SIZE		4000

#define	SW_VERSION		0x16

#define	MAX_CLK_GPIO		16		// GPIOs used by MAX7219
#define	MAX_LD_GPIO		17
#define	MAX_DIN_GPIO		18

#define	PI_PRESENT_GPIO		19
#define	SQUELCH_GPIO		22		// Digital signal from Radio
// PICO_DEFAULT_LED_PIN		25
#define	AUDIO_GPIO		26		// Audio input to ADC

#include "hardware/pio.h"
#include "out7seg.pio.h"

// Array values for Sin and Cos are 12 bit ... so adjust by shift for divide

int	Sin[] = {
#include	"sincos.c"
		};

int	Cos[] = {
#include	"cos.c"
		};

typedef	struct	{
	char	*t_name;
	int	t_freq;
	int	t_display;
	} TONE;

TONE	Tones[] = {
#include	"tones.c"
		};

#include	"dcs.c"		// FIR filter & DCS codes

#define	NUMBER_OF_TONES		(sizeof (Tones) / sizeof (TONE))
#define	TAP_SIZE		(sizeof (taps) / sizeof (int))

int	WaveSample0[SAMPLE_SIZE], WaveSample1[SAMPLE_SIZE];
int	wave_out[470];			// actually 467 - based on SAMPLE_SIZE of 4000

int			TableFlag = 0;
uint			pio_sm = -1;
int			DisplayTimeout = 5;
int			invert_bits = 0;
int			LosAltos_mode = 1;
uint			DMA_Channel;
dma_channel_config	DMA_Config;

void
start_dma (int *wr_ptr)
	{
	dma_channel_configure (DMA_Channel, &DMA_Config,
			wr_ptr,
			&adc_hw->fifo,
			SAMPLE_SIZE,
			true);

	adc_run (true);
	}

void
wait_for_dma (void)
	{
	dma_channel_wait_for_finish_blocking (DMA_Channel);

	adc_run (false);
	adc_fifo_drain ();	// Likely NEVER needed
	}

void
prep_samples (int *ptr)
	{
	int	i;

	// Doing 4 per loop makes the code twice as fast

	for (i = 0; i < SAMPLE_SIZE / 4; i++, ptr += 4) {

		// ADC is 12-bit, 0 to 4095. Convert to 10-bit reading
		// centered around 0, -512 to +511

		ptr[0] = (ptr[0] >> 2) - 512;
		ptr[1] = (ptr[1] >> 2) - 512;
		ptr[2] = (ptr[2] >> 2) - 512;
		ptr[3] = (ptr[3] >> 2) - 512;
		}
	}

uint
setup_pio (void)
	{
	PIO	pio;
	uint	offset, sm;

	pio = pio0;		// Pick one
	offset = pio_add_program (pio, &out7seg_program);
	sm = pio_claim_unused_sm (pio, true);
	out7seg_program_init (pio, sm, offset);
	return (sm);
	}

int
look_for_tone (int *ws_ptr, int freq)
	{
	int	total_r, total_i, j, angle;

	// As 2pi = 1024 for this number system, freq is 'f * 1024'

	total_r = 0;
	total_i = 0;

	for (j = 0; j < SAMPLE_SIZE; j++) {

		// Angle = 2pi.f.n/N .. but 2pi is 1024, n is 0 to (SAMPLE - 1) in
		// steps of 1/8601.6, N = 8601.6
		// f = frequency in Hz (67 .. 254). Shift << 10 to keep decimals
		// Angle = 1024.f.n/N,

		// Angle = 1024.f<<10.n/8601.6 = 128.1024.f.n/1075.2
		// Add accuracy of further 10 bits
		// angle = 1024.128.1024.f.n/(1024 x 1075.2)
		// Pull out constant f/537.6 ... 1024.128.f/537.6 - call this k
		// angle = 1024.n.k/2.1024

		angle = (j * freq + 1024) >> 11;
		angle &= 1023;

		total_i += (Sin[angle] * ws_ptr[j]) >> 6;		// Don't round, numbers can be -ve
		total_r += (Cos[angle] * ws_ptr[j]) >> 6;
		}

	return ((total_i >> 16) * (total_i >> 16) + (total_r >> 16) * (total_r >> 16));
	}

void
check_stdin (void)
	{
	int	ch;

	ch = getchar_timeout_us (0);
	if (ch != -1) {

		if (ch == 't')
			TableFlag = 1;
		else if (ch == 'u')
			TableFlag = 0;
		else if (ch == 'i') {
			if (invert_bits == 0)
				invert_bits = 0x7FFFFF;
			    else
				invert_bits = 0;
			printf ("DCS: Invert\n");
			}
		else if (ch == 'L') {
			printf ("DCS: Los Altos mode\n");
			LosAltos_mode = 1;
			}
		else if (ch == 'D') {
			printf ("DCS: Full Decode mode\n");
			LosAltos_mode = 0;
			}
		else if (ch == 'B') {
			printf ("ReBooting Now ...\n");
			reset_usb_boot (0, 0);
			}
		else if (ch == 'v')
			printf ("PL_tones [K6BAW] .. Version %d.%d\n", SW_VERSION >> 4, SW_VERSION & 15);
		}

#ifdef	SINGLE_SHOT

	for (;;) {
		ch = getchar_timeout_us (0);
		if (ch != -1) {
			if (ch == 's')
				return;
			}
		}
#endif
	}

/*
 * Send 2 12-bit words to the 7-segment display. The words are in value and are sent
 * bits 31..20 and 19..8. Bits 7..0 are not sent to the display, so are generally 0's
 *
 * Send two at once to be more efficient. When just one is sent, the other is all
 * zeros, which is a NOP to the 7-seg display controller
 */

static inline void
out_7seg (int value)
	{
	pio_sm_put_blocking (pio0, pio_sm, value);
	}

void
display_frequency (int freq)
	{
	// Lots of magic numbers here ... See the MAX7219 datasheet for description

			// Send digit 0 and digit 1
	out_7seg (0x10020000 | (((freq >> 12) & 0xF) << 20) | (((freq >> 8) & 0xF) << 8));

			// Send digit 2 and digit 3
	out_7seg (0x38040000 | (((freq >> 4) & 0xF) << 20) | ((freq & 0xF) << 8));

			// Intensity (brightness) to F, Display into "normal mode"
	out_7seg (0xA0FC0100);
	DisplayTimeout = 20;
	}

void
display_dcs_code (int code)
	{
	out_7seg (0x10020000 | (((code >> 6) & 0x7) << 20) | (((code >> 3) & 0x7) << 8));
	out_7seg (0x30040F00 | ((code & 0x7) << 20));
	out_7seg (0xA0FC0100);
	DisplayTimeout = 20;
	}

void
init_7seg (void)
	{
	// Lots of magic numbers here ... See the MAX7219 datasheet for description

			// Set Decode Mode for digits 3 & 4
			// Set Intensity (brightness) to F (could go to F)
	out_7seg (0x90CA0F00);

			// Just use digits 0-3 (scan out 4 digits)
			// Set digit 0 to 'H'
	out_7seg (0xB0313700);

			// Set digit 1 to 'i'
			// Set digit 2 to SW_VERSION major
	out_7seg (0x20638000 | (SW_VERSION << 4));

			// Set digit 3 to SW_VERSION minor
			// Put display into "normal mode"
	out_7seg (0x400C0100 | ((SW_VERSION & 15) << 20));

	sleep_ms (3000);

			// Put display into "standby mode"
			// Set Decode Mode to BCD
	out_7seg (0xC0090F00);

			// Set digit 0 to blank
			// Set digit 1 to '-'
	out_7seg (0x10F20A00);

			// Set digit 2 to '-' and turn on decimal point
			// Set digit 3 to '-'
	out_7seg (0x38A40A00);

			// Put display into "normal mode", then NOP
	out_7seg (0xC0100000);
	}

void
setup_gpio_and_stdio (void)
	{
	gpio_init (PI_PRESENT_GPIO);
	gpio_set_dir (PI_PRESENT_GPIO, GPIO_IN);
	gpio_pull_down (PI_PRESENT_GPIO);

	gpio_init (PICO_DEFAULT_LED_PIN);		// GPIO 25
	gpio_set_dir (PICO_DEFAULT_LED_PIN, GPIO_OUT);

	gpio_init (SQUELCH_GPIO);
	gpio_set_dir (SQUELCH_GPIO, GPIO_IN);
	gpio_disable_pulls (SQUELCH_GPIO);		// Hardware has pulldown
	gpio_set_input_hysteresis_enabled (SQUELCH_GPIO, true);

	if (gpio_get (PI_PRESENT_GPIO) == 0)
		stdio_usb_init ();
	    else {
		stdio_uart_init ();
		gpio_pull_up (PI_PRESENT_GPIO);
		}
	}

void
init_dma (void)
	{
	DMA_Channel = dma_claim_unused_channel (true);
	DMA_Config = dma_channel_get_default_config (DMA_Channel);

	channel_config_set_transfer_data_size (&DMA_Config, DMA_SIZE_32);
	channel_config_set_read_increment (&DMA_Config, false);
	channel_config_set_write_increment (&DMA_Config, true);
	channel_config_set_dreq (&DMA_Config, DREQ_ADC);
	}

void
setup_adc (void)
	{
	float	freq;

	adc_gpio_init (AUDIO_GPIO);	// Make sure GPIO is high-impedance, no pullups etc
	gpio_disable_pulls (AUDIO_GPIO);
	adc_select_input (0);		// Select ADC input 0 (GPIO26)
	adc_init ();

	/*
	 * arguments to adc_fifo_setup ...
	 *
	 *	en		Enables write each conversion result to the FIFO
	 *	dreq_en		Enable DMA requests when FIFO contains data
	 *	dreq_thresh	Threshold for DMA requests/FIFO IRQ if enabled.
	 *	err_in_fifo	If enabled, bit 15 of the FIFO contains error flag for each sample
	 *	byte_shift	Shift FIFO contents to be one byte in size (for byte DMA)
	 *			 - enables DMA to byte buffers.
	 */

	adc_fifo_setup (true, true, 1, false, false);

	/*
	 * Fundamental rate is 134.4Hz for DCS. 48MHz is ADC clock rate
	 *
	 * 134.4 * 8 * 8 = 8601.6, used as DFT sample rate
	 * 48MHz / 8601.6 = 5580.357142857143
	 * 5580 + 91/256  = 5580.35546875
	 *
	 * 48MHz / 5580.35546875 = 8601.60258... close as I can get
	 */

	freq = 91.0;
	freq = freq / 256.0;
	freq += 5580.0;                 // This gets to 134.4Hz
	freq -= 1.0;                    // adc_set_clkdiv needs "freq - 1"

	adc_set_clkdiv (freq);          // Read at 8601.6Hz
	}

int display_sweep = 1;

void
sweep_display (void)
	{
	switch (display_sweep++ % 6) {

		case 0:
			out_7seg (0x10F20F00);		// "   ."
			out_7seg (0x30F48F00);
			break;

		case 1:
		case 5:
			out_7seg (0x10F20F00);		// "  . "
			out_7seg (0x38F40F00);
			break;

		case 2:
		case 4:
			out_7seg (0x10F28F00);		// " .  "
			out_7seg (0x30F40F00);
			break;

		case 3:
			out_7seg (0x18F20F00);		// ".   "
			out_7seg (0x30F40F00);
			break;
		}
	}

// return value is INDEX of Tones or -1 if none

int
look_for_pl (int *ws_ptr)
	{
	int     i, total, average, index_of_max, max_score, score;

	// Get the "scores" for each of the 50 PL frequencies

	for (total = max_score = i = 0; i < NUMBER_OF_TONES; i++) {

		score = look_for_tone (ws_ptr, Tones[i].t_freq);
		total += score;

		if (TableFlag == 1)
			printf ("Tab %s  %5d\n", Tones[i].t_name, score);

		// Take out the single highest value ... This
		// currently looks for "at most" just one tone

		if (score > max_score) {
			max_score = score;
			index_of_max = i;
			}
		}

	average = (total - max_score) / NUMBER_OF_TONES;
	if (average == 0) average = 1;

	if (TableFlag == 1)
		printf ("Average %d\n", average);

	// The 0th index is for 134.4, which is being filtered out
	// because it's the DCS end of transmission frequency

	if (max_score > (average * 50) && index_of_max != 0) {

			printf ("PL %s\n", Tones[index_of_max].t_name);
			return (index_of_max);
		}

	printf ("PL NONE\n");
	return (-1);
	}

int	sweep_count = 0;

void
display_results (int pl_index, int code)
	{
	if (code != 0) {

		display_dcs_code (code);
		return;
		}

	if (pl_index != -1) {
		
		display_frequency (Tones[pl_index].t_display);
		return;
		}

	if (DisplayTimeout > 0) {
		out_7seg (0xA0200000);		// Set Intensity (brightness) to 2
		DisplayTimeout--;
		display_sweep = 1;
		}
	    else
		if ((sweep_count++ % 3) == 0)
			sweep_display ();
	}

// Make the CRC for the 23-bit DCS golay code
// 12-bits of data with 11-bits of CRC

int
encode (int code)
	{
	int	p;

	p = 0;
	if ((code & 0x001) != 0) p ^= 0x475;
	if ((code & 0x002) != 0) p ^= 0x49f;
	if ((code & 0x004) != 0) p ^= 0x54b;
	if ((code & 0x008) != 0) p ^= 0x6e3;
	if ((code & 0x010) != 0) p ^= 0x1b3;
	if ((code & 0x020) != 0) p ^= 0x366;
	if ((code & 0x040) != 0) p ^= 0x6CC;
	if ((code & 0x080) != 0) p ^= 0x1ED;
	if ((code & 0x100) != 0) p ^= 0x3da;
	if ((code & 0x200) != 0) p ^= 0x7b4;
	if ((code & 0x400) != 0) p ^= 0x31d;
	if ((code & 0x800) != 0) p ^= 0x63a;

	return ((p << 12) | (code & 0xFFF));
	}

// Return value: Code or 0

int
code_lookup (int dcs_word)
	{
	int	i, code;

	if (dcs_word != 0) {

		for (i = 0; i < 23; i++) {		// DCS code is 23 bits long

			// Look for DCS marker bits

			if ((dcs_word & 0xE00) == 0x800) {

				// Alias resolution

				if (LosAltos_mode == 1)
					code = los_altos_map[dcs_word & 0x1FF];
				    else
					code = code_map[dcs_word & 0x1FF];
				printf ("DCS %3o\n", code);
				return (code);
				}

			// Rotate DCS word into next position

			dcs_word = (dcs_word >> 1) | ((dcs_word & 1) << 22);
			}
		}

	printf ("DCS NONE\n");
	return (0);
	}

// return a DCS word (good CRC) or zero

int
look_for_dcs (int *ws_ptr)
	{
	int	i, j, *ptr, *ptr2;
	int	x;				// long long ??
	int	highest, lowest;
	int	word, last, bits, count, pwo;

	// This is the low-pass FIR filter, only process every 8th sample

	highest = lowest = 0;
	for (i = 0; i < SAMPLE_SIZE - 2 * TAP_SIZE; i += 8) {

		x = 0;

		ptr = &ws_ptr[i];				// Index i + 0
		ptr2 = &ws_ptr[i + 2 * (TAP_SIZE - 1)];		// Index i + 264

		for (j = 0; j < TAP_SIZE - 1; ) {
			x += (*ptr++ + *ptr2--) * taps[j++];
			x += (*ptr++ + *ptr2--) * taps[j++];
			x += (*ptr++ + *ptr2--) * taps[j++];
			x += (*ptr++ + *ptr2--) * taps[j++];
			}
		x += *ptr * taps[j];

		// Output of FIR filter, 8 values per bit-time

		wave_out[i >> 3] = x;

		if (x > highest)	highest = x;
		if (x < lowest)		lowest = x;
		}

	// Use 5/16 of the peak

	lowest = (lowest * 5) >> 4;
	highest = (highest * 5) >> 4;

#ifdef	PRINT_RANGE
	printf ("Range %d to %d\n", lowest, highest);
#endif

	count = -1;
	pwo = word = bits = 0;

	// 465 comes from (SAMPLE_SIZE - 2 * TAP_SIZE) / 8
	// Index i must allow for wo[i-1] and wo[i+1] below

	for (i = 1; i < 465; i++) {

		int	diff;

#ifdef	PRINT_DCS_DECODE
		printf ("%3d: %8d  (diff %8d)   ", (count == -1) ? 0 : count,
				wave_out[i], wave_out[i] - pwo);
#endif

		if ((count & 6) == 6 || (count & 7) == 0) {

			// Startup, early or on schedule

			diff = wave_out[i] - pwo;
			if (wave_out[i] > highest && diff > highest &&
						wave_out[i-1] <= wave_out[i] &&
						wave_out[i] > wave_out[i+1]) {

#ifdef	PRINT_DCS_DECODE
				printf (" ++ ");
#endif
				last = 1;
				count = 0;
				goto speed_up;
				}
			if (wave_out[i] < lowest && diff < lowest &&
						wave_out[i-1] >= wave_out[i] &&
						wave_out[i] < wave_out[i+1]) {

#ifdef	PRINT_DCS_DECODE
				printf (" -- ");
#endif
				last = 0;
				count = 0;
				goto speed_up;
				}
			}

		if ((count & 7) == 0) {

			// If the next is a peak .. wait a cycle

			diff = wave_out[i+1] - pwo;
			if (wave_out[i+1] > highest && diff > highest &&
						wave_out[i] <= wave_out[i+1] ) {
				i++;
#ifdef	PRINT_DCS_DECODE
				printf ("\n%3d: %8d  (diff %8d)    +  ", (count == -1) ? 0 : count,
						wave_out[i], wave_out[i] - pwo);
#endif
				last = 1;
				count = 0;
				goto speed_up;
				}
			if (wave_out[i+1] < lowest && diff < lowest &&
						wave_out[i] >= wave_out[i+1] ) {
				i++;
#ifdef	PRINT_DCS_DECODE
				printf ("\n%3d: %8d  (diff %8d)    -  ", (count == -1) ? 0 : count,
						wave_out[i], wave_out[i] - pwo);
#endif
				last = 0;
				count = 0;
				goto speed_up;
				}

			// Get here every 8th sample time

		    speed_up:
			word = (last << 30) | ((word >> 1));
#ifdef	PRINT_DCS_DECODE
			printf ("BIT %d ... 0x%x  ", last, word);
#endif

			pwo = wave_out[i];

			if (++bits >= 31) {

				// Check Bits ... first 8 bits and last 8 bits (of 31)

				if (word != 0 && word != 0x7FFFFFFF && (word & 0xFF) == (word >> 23))

					// Check the 11 parity bits are correct

					if ((word & 0x7FFFFF) == encode (word))

						return ((word & 0x7FFFFF) ^ invert_bits);
				}
			}

#ifdef	PRINT_DCS_DECODE
		printf ("\n");
#endif

		if (count != -1)
			count++;
		}
	return (0);
	}

int
main () {
	int	code, pl_index;

	setup_gpio_and_stdio ();
	pio_sm = setup_pio ();
	setup_adc ();
	init_dma ();
	init_7seg ();

	for (;;) {

#ifdef	SINGLE_SHOT

		check_stdin ();

		start_dma (&WaveSample0[0]);
		wait_for_dma ();		// WS0

		prep_samples (&WaveSample0[0]);
		pl_index = look_for_pl (&WaveSample0[0]);
		code = code_lookup (look_for_dcs (&WaveSample0[0]));
		display_results (pl_index, code);

#else	// SINGLE_SHOT
		start_dma (&WaveSample0[0]);

		prep_samples (&WaveSample1[0]);
		pl_index = look_for_pl (&WaveSample1[0]);
		code = code_lookup (look_for_dcs (&WaveSample1[0]));
		display_results (pl_index, code);

		check_stdin ();
		wait_for_dma ();		// WS0

		start_dma (&WaveSample1[0]);

		prep_samples (&WaveSample0[0]);
		pl_index = look_for_pl (&WaveSample0[0]);
		code = code_lookup (look_for_dcs (&WaveSample0[0]));
		display_results (pl_index, code);

		check_stdin ();
		wait_for_dma ();		// WS1
#endif	// SINGLE_SHOT
		}
	}

