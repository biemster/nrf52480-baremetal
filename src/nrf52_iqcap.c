#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>

#include "nrf.h"
#include "rfx.h"
#include "tusb.h"

#define NRF_CMD_USBTEST          0xa1
#define NRF_CMD_REBOOT           0xa2
#define NRF_CMD_IQCAPTURE_STREAM 0xca
#define NRF_CMD_IQCAPTURE_BURST  0xcb

#define LED     (1 << 15) // the red led on the nice!nano
#define MAXSAMP (62*1024)
__attribute__((aligned(4))) static uint32_t iq_buf[MAXSAMP];

static volatile int gs_usb_cmd;
static volatile int gs_capture_freq;
static volatile int gs_streaming;

/* Dummy implementations to satisfy the linker and silence warnings */
int _write(int handle, char *buffer, int size) { return size; }
int _close(int file) { return -1; }
int _fstat(int file, struct stat *st) { return -1; }
int _isatty(int file) { return 1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _read(int file, char *ptr, int len) { return 0; }
int _kill(int pid, int sig) { return -1; }
int _getpid(void) { return 1; }

extern void tusb_hal_nrf_power_event(uint32_t event);

// Value is chosen to be as same as NRFX_POWER_USB_EVT_* in nrfx_power.h
enum {
	USB_EVT_DETECTED = 0,
	USB_EVT_REMOVED = 1,
	USB_EVT_READY = 2
};

void HardFault_Handler(void) {
	NRF_P0->OUTSET = LED; // Force LED on to show we crashed
	while(1) { __NOP(); }
}

void USBD_IRQHandler(void) {
	tud_int_handler(0);
}

void POWER_CLOCK_IRQHandler(void) {
	uint32_t inten = NRF_POWER->INTENSET;

	// Cable plugged in
	if ((inten & POWER_INTENSET_USBDETECTED_Msk) && NRF_POWER->EVENTS_USBDETECTED) {
		NRF_POWER->EVENTS_USBDETECTED = 0;
		tusb_hal_nrf_power_event(USB_EVT_DETECTED);
	}

	// Cable unplugged
	if ((inten & POWER_INTENSET_USBREMOVED_Msk) && NRF_POWER->EVENTS_USBREMOVED) {
		NRF_POWER->EVENTS_USBREMOVED = 0;
		tusb_hal_nrf_power_event(USB_EVT_REMOVED);
	}

	// Power ready to use
	if ((inten & POWER_INTENSET_USBPWRRDY_Msk) && NRF_POWER->EVENTS_USBPWRRDY) {
		NRF_POWER->EVENTS_USBPWRRDY = 0;
		tusb_hal_nrf_power_event(USB_EVT_READY);
	}
}

void init_usb_power_irq(void) {
	// Enable events for USB insertion, removal, and power ready
	NRF_POWER->INTENSET = (POWER_INTENSET_USBDETECTED_Msk |
						   POWER_INTENSET_USBREMOVED_Msk  |
						   POWER_INTENSET_USBPWRRDY_Msk);

	// Enable the POWER_CLOCK IRQ in the NVIC
	NVIC_EnableIRQ(POWER_CLOCK_IRQn);

	// USB power may already be ready at this time -> no event generated
	// We need to invoke the handler based on the status initially
	uint32_t usb_reg = NRF_POWER->USBREGSTATUS;
	if ( usb_reg & POWER_USBREGSTATUS_VBUSDETECT_Msk ) {
		tusb_hal_nrf_power_event(USB_EVT_DETECTED);
	}
	if ( usb_reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk  ) {
		tusb_hal_nrf_power_event(USB_EVT_READY);
	}
}

void clock_init() {
	// Start LFCLK (Low Frequency Clock)
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {
		// Wait for LFCLK to start
	}

	// Start HFCLK (High Frequency 32MHz Crystal)
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
		// Wait for HFCLK to stabilize
	}
}

void delay_init(void) {
	// Enable the Trace and Debug block
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0; // Clear the cycle counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable the cycle counter
}

void delay_us(uint32_t us) {
	uint32_t start = DWT->CYCCNT;

	// nRF52840 runs at 64 MHz. Therefore, 64 cycles = 1 microsecond.
	uint32_t delay_ticks = us * 64;
	while ((DWT->CYCCNT - start) < delay_ticks) {
		__NOP();
	}
}
void delay_ms(uint32_t ms) { delay_us(ms *1000); }

void jump_bootloader() {
	// go back to uf2 bootloader
	NRF_POWER->GPREGRET = 0x57; // 0x57 tells the Adafruit UF2 bootloader to stay in bootloader mode
	NVIC_SystemReset();         // Reboot the chip
}

void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		NRF_P0->OUTSET = LED;
		delay_ms(33);
		NRF_P0->OUTCLR = LED;
		if(i) delay_ms(33);
	}
}

void tud_mount_cb(void) {
	tud_vendor_write_clear();

	uint8_t dump_buf[64];
	while (tud_vendor_available()) {
		tud_vendor_read(dump_buf, sizeof(dump_buf));
	}

	gs_usb_cmd = 0;
}

void tud_vendor_rx_cb(uint8_t intf, const uint8_t *buffer, uint32_t bufsize) {
	// - CFG_TUD_VENDOR_TXRX_BUFFERED = 1: buffer and bufsize must not be used (both NULL,0) since data is in RX FIFO
	uint8_t buf[64]; // Buffer to hold the incoming command

	while (tud_vendor_available() > 0) {
		uint32_t bytes_read = tud_vendor_read(buf, sizeof(buf));
		if( (bytes_read == 4) && !gs_usb_cmd) {
			gs_usb_cmd = buf[0];
			switch(gs_usb_cmd) {
			case NRF_CMD_REBOOT:
			case NRF_CMD_USBTEST:
				gs_streaming = 0;
				break;
			case NRF_CMD_IQCAPTURE_STREAM:
				gs_capture_freq = 2400 +buf[1];
				gs_streaming = buf[2];
				break;
			case NRF_CMD_IQCAPTURE_BURST:
				gs_capture_freq = 2400 +buf[1];
				break;
			default:
				break;
			}
		}
	}
}

void iqcapture_burst(int freq) {
	init_radio(/*access address*/0, freq);
	nrf_radio_event_clear(NRF_RADIO, RFX_RADIO_EVENT_IQCAPSTART);
	nrf_radio_event_clear(NRF_RADIO, RFX_RADIO_EVENT_IQCAPEND);

	radio_start_rx();
	while (nrf_radio_state_get(NRF_RADIO) != NRF_RADIO_STATE_RX) { delay_us(1); } // settle PLL

	radio_set_iq_capture(iq_buf, MAXSAMP);
	radio_trigger_iq_capture();
	// wait for capture to start
	while (!nrf_radio_event_check(NRF_RADIO, RFX_RADIO_EVENT_IQCAPSTART)) { delay_us(1); }
	delay_us(10);
	while (!nrf_radio_event_check(NRF_RADIO, RFX_RADIO_EVENT_IQCAPEND)) { delay_us(1); }
}

void bulk_send_burst() {
	uint32_t total_bytes = MAXSAMP * sizeof(iq_buf[0]);
	uint32_t bytes_sent = 0;
	uint8_t* ptr = (uint8_t*)iq_buf;

	// Convert in place to sign extend 12-bit samples
	for(int i = 0; i < MAXSAMP; i++) {
		uint32_t val = iq_buf[i];

		int16_t i_sample = (int16_t)( ((int32_t)(val << 20)) >> 20 );
		int16_t q_sample = (int16_t)( ((int32_t)(val << 8))  >> 20 );

		iq_buf[i] = ((uint8_t *)&i_sample)[0] << 24 |
					((uint8_t *)&i_sample)[1] << 16 |
					((uint8_t *)&q_sample)[0] << 8  |
					((uint8_t *)&q_sample)[1];

		// Yield to TinyUSB every 512 iterations so the USB connection doesn't drop
		if ((i % 512) == 0) {
			tud_task();
		}
	}

	while ((bytes_sent < total_bytes) && tud_vendor_mounted()) {
		// Calculate remaining bytes, but cap the request to 1024 bytes
		uint32_t chunk = total_bytes - bytes_sent;
		if (chunk > 1024) {
			chunk = 1024;
		}

		uint32_t pushed = tud_vendor_write(ptr, chunk);
		if (pushed > 0) {
			ptr += pushed;
			bytes_sent += pushed;
			tud_vendor_write_flush(); // Tell TinyUSB the data is ready to go
		}

		// Keep the USB state machine moving
		tud_task();
	}
}

static inline uint32_t decimate_1bit_avg4(const uint32_t* buf, int base, int k) {
	// 1. Load the 4 samples from RAM
	uint32_t w0 = buf[base + k*8 + 0];
	uint32_t w1 = buf[base + k*8 + 2];
	uint32_t w2 = buf[base + k*8 + 4];
	uint32_t w3 = buf[base + k*8 + 6];

	// 2. Extract and sum I (bits 0-11)
	int32_t sum_i = ((int32_t)(w0 << 20) >> 20) +
					((int32_t)(w1 << 20) >> 20) +
					((int32_t)(w2 << 20) >> 20) +
					((int32_t)(w3 << 20) >> 20);

	// 3. Extract and sum Q (bits 12-23)
	int32_t sum_q = ((int32_t)(w0 << 8) >> 20) +
					((int32_t)(w1 << 8) >> 20) +
					((int32_t)(w2 << 8) >> 20) +
					((int32_t)(w3 << 8) >> 20);

	// 4. Extract the sign bits and shift to the correct position
	uint32_t bit_i = ((uint32_t)sum_i >> 31) << (31 - 2*k);
	uint32_t bit_q = ((uint32_t)sum_q >> 31) << (30 - 2*k);

	return bit_i | bit_q;
}

void iqcapture_stream(int freq) {
	init_radio(/*access address*/0, freq);
	radio_start_rx();
	while (nrf_radio_state_get(NRF_RADIO) != NRF_RADIO_STATE_RX) { delay_us(1); } // settle PLL

	nrf_radio_event_clear(NRF_RADIO, RFX_RADIO_EVENT_IQCAPSTART);
	nrf_radio_event_clear(NRF_RADIO, RFX_RADIO_EVENT_IQCAPEND);

	uint32_t usbbuf[CFG_TUD_VENDOR_EPSIZE /4] = {0}; // initialize as uint32_t for easier decimation of the capture buffer
	int decimation = 8*16; // 16Msps -> 2Msps, 16bit I/Q -> 1bit I/Q,
	int streambuf_size = (CFG_TUD_VENDOR_EPSIZE /4) * decimation;
	streambuf_size = (streambuf_size > MAXSAMP) ? MAXSAMP : streambuf_size;

	uint32_t *iq_buf_2ndhalf = &iq_buf[MAXSAMP /2]; // move the capture buffer further in RAM, it does not like USB in parallel
	uint32_t *stream_buf = iq_buf_2ndhalf;
	while(gs_streaming) {
		radio_set_iq_capture(stream_buf, streambuf_size);
		radio_trigger_iq_capture();

		// now we have time to send the other capture buffer over USB
		stream_buf = (stream_buf == iq_buf_2ndhalf) ? iq_buf_2ndhalf +streambuf_size : iq_buf_2ndhalf;
		int usbbuf_idx = 0;
		for(int i = 0; i < streambuf_size; i+=decimation) {
			usbbuf[usbbuf_idx++] =
					decimate_1bit_avg4(stream_buf, i, 0)  |
					decimate_1bit_avg4(stream_buf, i, 1)  |
					decimate_1bit_avg4(stream_buf, i, 2)  |
					decimate_1bit_avg4(stream_buf, i, 3)  |
					decimate_1bit_avg4(stream_buf, i, 4)  |
					decimate_1bit_avg4(stream_buf, i, 5)  |
					decimate_1bit_avg4(stream_buf, i, 6)  |
					decimate_1bit_avg4(stream_buf, i, 7)  |
					decimate_1bit_avg4(stream_buf, i, 8)  |
					decimate_1bit_avg4(stream_buf, i, 9)  |
					decimate_1bit_avg4(stream_buf, i, 10) |
					decimate_1bit_avg4(stream_buf, i, 11) |
					decimate_1bit_avg4(stream_buf, i, 12) |
					decimate_1bit_avg4(stream_buf, i, 13) |
					decimate_1bit_avg4(stream_buf, i, 14) |
					decimate_1bit_avg4(stream_buf, i, 15);
		}

		if(tud_vendor_mounted()) {
			uint32_t written = 0;
			// Loop guarantees we don't drop data if the FIFO is momentarily full
			while (written < CFG_TUD_VENDOR_EPSIZE && tud_vendor_mounted() && gs_streaming) {
				uint32_t pushed = tud_vendor_write(((uint8_t*)usbbuf) + written, CFG_TUD_VENDOR_EPSIZE - written);
				written += pushed;
				tud_vendor_write_flush();
				tud_task();
			}
		}

		// Keep the USB state machine moving while waiting for the capture to end
		while (!nrf_radio_event_check(NRF_RADIO, RFX_RADIO_EVENT_IQCAPEND)) {
			tud_task();
		}

		nrf_radio_event_clear(NRF_RADIO, RFX_RADIO_EVENT_IQCAPSTART);
		nrf_radio_event_clear(NRF_RADIO, RFX_RADIO_EVENT_IQCAPEND);
	}
}

void usb_cmd_handler() {
	if(gs_usb_cmd) {
		switch(gs_usb_cmd) {
		case NRF_CMD_REBOOT:
			blink(3);
			jump_bootloader();
			break;
		case NRF_CMD_USBTEST:
			blink(2);
			break;
		case NRF_CMD_IQCAPTURE_BURST:
			iqcapture_burst(gs_capture_freq);
			bulk_send_burst();
			blink(2);
			break;
		case NRF_CMD_IQCAPTURE_STREAM:
			gs_usb_cmd = 0; // iqcapture_stream() is blocking
			iqcapture_stream(gs_capture_freq);
			break;
		}
		gs_usb_cmd = 0;
	}
}

// entry point
void main(void) {
	// Relocate the interrupt vector table to the app start address for UF2
	SCB->VTOR = 0x26000;

	clock_init();
	delay_init();

	// LED
	NRF_P0->DIRSET = LED;
	NRF_P0->OUTCLR = LED;

	// Init USB
	init_usb_power_irq();
	tusb_init();
	NVIC_EnableIRQ(USBD_IRQn);

	blink(3);
	while(1) {
		tud_task();
		usb_cmd_handler();
	}
}
