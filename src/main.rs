#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::{
	delay_ms,
	hal::port,
	pac::{tc1::tccr1b::CS1_A, TC1},
	port::{mode::Output, Pin},
	prelude::*,
};
use atmega_hal::{port::PinOps, usart::BaudrateExt};
use core::{
	cmp, mem,
	sync::atomic::{AtomicBool, Ordering},
};
use ebyte_e32::{
	mode::{Mode, Normal, Program},
	parameters::{AirBaudRate, Persistence},
	Ebyte,
};
use embedded_hal::digital::v2::InputPin;
use nb::block;
use panic_halt as _;

const MESSAGE_SEPARATOR: u8 = b';';
const SECONDS_IN_DAY: u32 = 24 * 60 * 60;

const OPEN_WINDOW_REQUEST: &[u8; 1] = b"o";
const CLOSE_WINDOW_REQUEST: &[u8; 1] = b"c";
const GET_STATE_REQUEST: &[u8; 1] = b"s";
const GET_GLOBAL_TIME_REQUEST: &[u8; 1] = b"t";
const UPDATE_GLOBAL_TIME_REQUEST: &[u8; 1] = b"u";
const SET_TIME_MODE_REQUEST: &[u8; 1] = b"r";
const ENABLE_SCHEDULE_REQUEST: &[u8; 15] = b"enable_schedule";
const DISABLE_SCHEDULE_REQUEST: &[u8; 16] = b"disable_schedule";
const DISABLE_TIME_MODE_REQUEST: &[u8; 1] = b"d";
const ENABLE_TIME_MODE_REQUEST: &[u8; 1] = b"e";
const GET_TIME_MODE_RANGE_REQUEST: &[u8; 1] = b"a";
const SET_SCHEDULE_REQUEST: &[u8; 1] = b"h";
const GET_SCHEDULE_TIME: &[u8; 1] = b"b";

const WINDOW_OPENED_RESPONSE: u8 = b'o';
const WINDOW_CLOSED_RESPONSE: u8 = b'c';
const SET_TIME_OK_RESPONSE: &[u8; 11] = b"set_time_ok";
const SET_TIME_ERR_RESPONSE: &[u8; 12] = b"set_time_err";
const SCHEDULE_ENABLED_RESPONSE: &[u8; 16] = b"schedule_enabled";
const SHCEDULE_ERR_RESPONSE: &[u8; 12] = b"schedule_err";
const SHCEDULE_DISABLED_RESPONSE: &[u8; 17] = b"schedule_disabled";
const TIME_MODE_ENABLED_OK_RESPONSE: &[u8; 9] = b"enable_ok";
const TIME_MODE_ENABLED_ERR_RESPONSE: &[u8; 10] = b"enable_err";
const TIME_MODE_DISABLED_RESPONSE: &[u8; 10] = b"disable_ok";

enum TimeModeActionState {
	ShouldBeOpened,
	ShouldBeClosed,
}

struct TimeMode {
	active_time_in_sec: u32,
	delay_time_in_sec: u32,
	active_time_offset: u32,
	delay_time_offset: u32,
	enabled: bool,
	action_state: TimeModeActionState,
}

struct Schedule {
	open_time: u32,
	close_time: u32,
	enabled: bool,
	action_state: TimeModeActionState,
}

static mut WINDOW_IS_OPENED: AtomicBool = AtomicBool::new(false);
static mut WINDOW_IS_CLOSED: AtomicBool = AtomicBool::new(true);
static mut WINDOW_IS_OPENING: AtomicBool = AtomicBool::new(false);
static mut WINDOW_IS_CLOSING: AtomicBool = AtomicBool::new(false);
static mut GLOBAL_TIME_IN_SEC: u32 = 0;
static mut TIME_MODE: TimeMode = TimeMode {
	active_time_in_sec: 0,
	delay_time_in_sec: 0,
	active_time_offset: 0,
	delay_time_offset: 0,
	enabled: false,
	action_state: TimeModeActionState::ShouldBeOpened,
};
static mut SCHEDULE: Schedule = Schedule {
	open_time: 0,
	close_time: 0,
	enabled: false,
	action_state: TimeModeActionState::ShouldBeOpened,
};

#[arduino_hal::entry]
fn main() -> ! {
	let mut receiving_message: [u8; 50] = [0; 50];
	let mut receiving_message_position = 0;
	let mut message_received = false;

	let dp = atmega_hal::Peripherals::take().unwrap();

	let pins = atmega_hal::pins!(dp);

	let mut serial1 = arduino_hal::Usart::new(
		dp.USART0,
		pins.pd0,
		pins.pd1.into_output(),
		BaudrateExt::into_baudrate(9600),
	);

	let timer1 = dp.TC1;

	// engine rotation direction
	// low -> close; high -> open
	let mut engine_direction_pin = pins.pd2.into_output();
	engine_direction_pin.set_high();

	let mut engine_inverse_pin = pins.pd3.into_output();
	engine_inverse_pin.set_high();

	// open/close sensors
	let close_sensor = pins.pd5.into_floating_input();
	let open_sensor = pins.pd6.into_floating_input();

	reg_timer(&timer1);

	// add uart tx interrupt
	// serial1.listen(atmega_hal::usart::Event::TxComplete);
	unsafe {
		avr_device::interrupt::enable();
	}

	loop {
		if let Ok(byte) = serial1.read() {
			// serial1.write_byte(byte);
			if !message_received {
				receiving_message[receiving_message_position] = byte;
				match receiving_message_position < receiving_message.len() {
					true => receiving_message_position += 1,
					false => {
						receiving_message.fill(0);
						receiving_message_position = 0;
						message_received = false;
					}
				}
				if byte == MESSAGE_SEPARATOR {
					message_received = true;
				}
			}
		}

		if message_received {
			if let Some(length) = receiving_message
				.iter()
				.position(|c| c.eq(&MESSAGE_SEPARATOR))
			{
				let message = &receiving_message[0..length];
				unsafe {
					if message.starts_with(OPEN_WINDOW_REQUEST) {
						// open window
						try_open(&mut engine_inverse_pin, &mut engine_direction_pin);
					} else if message.starts_with(CLOSE_WINDOW_REQUEST) {
						// close window
						try_close(&mut engine_inverse_pin, &mut engine_direction_pin);
					} else if message.starts_with(GET_STATE_REQUEST) {
						// get state
						if WINDOW_IS_OPENED.load(Ordering::SeqCst) {
							serial1.write_byte(WINDOW_OPENED_RESPONSE);
						}
						if WINDOW_IS_CLOSED.load(Ordering::SeqCst) {
							serial1.write_byte(WINDOW_CLOSED_RESPONSE);
						}
					} else if message.starts_with(GET_GLOBAL_TIME_REQUEST) {
						// get time
						serial1.write_byte(b't');
						for byte in GLOBAL_TIME_IN_SEC.to_be_bytes() {
							serial1.write_byte(byte);
						}
					} else if message.starts_with(UPDATE_GLOBAL_TIME_REQUEST) {
						// update global time
						let updated_time =
							u32::from_be_bytes([message[1], message[2], message[3], message[4]]);
						match updated_time.cmp(&SECONDS_IN_DAY) {
							cmp::Ordering::Less => {
								GLOBAL_TIME_IN_SEC = updated_time;
								for &byte in SET_TIME_OK_RESPONSE {
									serial1.write_byte(byte);
								}
							}
							_ => {
								for &byte in SET_TIME_ERR_RESPONSE {
									serial1.write_byte(byte);
								}
							}
						}
					} else if message.starts_with(SET_TIME_MODE_REQUEST) {
						// set time mode
						TIME_MODE.active_time_in_sec =
							u32::from_be_bytes([message[1], message[2], message[3], message[4]]);
						TIME_MODE.delay_time_in_sec =
							u32::from_be_bytes([message[5], message[6], message[7], message[8]]);
						TIME_MODE.enabled = true;
						SCHEDULE.enabled = false;
						for &byte in TIME_MODE_ENABLED_OK_RESPONSE {
							serial1.write_byte(byte);
						}
						delay_ms(10);
						for &byte in SHCEDULE_DISABLED_RESPONSE {
							serial1.write_byte(byte);
						}
					} else if message.starts_with(DISABLE_SCHEDULE_REQUEST) {
						SCHEDULE.enabled = false;
						for &byte in SHCEDULE_DISABLED_RESPONSE {
							serial1.write_byte(byte);
						}
					} else if message.starts_with(ENABLE_SCHEDULE_REQUEST) {
						match SCHEDULE.open_time.eq(&SCHEDULE.close_time) {
							true => {
								for &byte in SHCEDULE_ERR_RESPONSE {
									serial1.write_byte(byte);
								}
							}
							false => {
								SCHEDULE.enabled = true;
								TIME_MODE.enabled = false;
								for &byte in SCHEDULE_ENABLED_RESPONSE {
									serial1.write_byte(byte);
								}
								delay_ms(10);
								for &byte in TIME_MODE_DISABLED_RESPONSE {
									serial1.write_byte(byte);
								}
							}
						}
					} else if message.starts_with(DISABLE_TIME_MODE_REQUEST) {
						// disable time mode
						TIME_MODE.enabled = false;
						for &byte in TIME_MODE_DISABLED_RESPONSE {
							serial1.write_byte(byte);
						}
					} else if message.starts_with(ENABLE_TIME_MODE_REQUEST) {
						// enable time mode
						match TIME_MODE.active_time_in_sec > 0 && TIME_MODE.active_time_in_sec > 0 {
							true => {
								TIME_MODE.enabled = true;
								SCHEDULE.enabled = false;
								for &byte in TIME_MODE_ENABLED_OK_RESPONSE {
									serial1.write_byte(byte);
								}
								delay_ms(10);
								for &byte in SHCEDULE_DISABLED_RESPONSE {
									serial1.write_byte(byte);
								}
							}
							false => {
								for &byte in TIME_MODE_ENABLED_ERR_RESPONSE {
									serial1.write_byte(byte);
								}
							}
						}
					} else if message.starts_with(GET_TIME_MODE_RANGE_REQUEST) {
						// get time range
						for byte in TIME_MODE.active_time_in_sec.to_be_bytes() {
							serial1.write_byte(byte);
						}
						for byte in TIME_MODE.delay_time_in_sec.to_be_bytes() {
							serial1.write_byte(byte);
						}
					} else if message.starts_with(SET_SCHEDULE_REQUEST) {
						// set schedule
						let open_time =
							u32::from_be_bytes([message[1], message[2], message[3], message[4]]);
						let close_time =
							u32::from_be_bytes([message[5], message[6], message[7], message[8]]);
						match open_time < SECONDS_IN_DAY - 1
							&& close_time < SECONDS_IN_DAY - 1
							&& !open_time.eq(&close_time)
						{
							true => {
								SCHEDULE.open_time = open_time;
								SCHEDULE.close_time = close_time;
								SCHEDULE.enabled = true;
								TIME_MODE.enabled = false;
								for &byte in SCHEDULE_ENABLED_RESPONSE {
									serial1.write_byte(byte);
								}
								delay_ms(10);
								for &byte in TIME_MODE_DISABLED_RESPONSE {
									serial1.write_byte(byte);
								}
							}
							false => {
								for &byte in SHCEDULE_ERR_RESPONSE {
									serial1.write_byte(byte);
								}
							}
						}
					} else if message.starts_with(GET_SCHEDULE_TIME) {
						// get schedule state
						for byte in SCHEDULE.open_time.to_be_bytes() {
							serial1.write_byte(byte);
						}
						for byte in SCHEDULE.close_time.to_be_bytes() {
							serial1.write_byte(byte);
						}
					}
				}
			}
			receiving_message.fill(0);
			receiving_message_position = 0;
			message_received = false;
		}

		unsafe {
			if TIME_MODE.enabled {
				match TIME_MODE.action_state {
					TimeModeActionState::ShouldBeOpened => {
						try_open(&mut engine_inverse_pin, &mut engine_direction_pin);
					}
					TimeModeActionState::ShouldBeClosed => {
						try_close(&mut engine_inverse_pin, &mut engine_direction_pin);
					}
				}
			} else if SCHEDULE.enabled {
				match SCHEDULE.action_state {
					TimeModeActionState::ShouldBeOpened => {
						try_open(&mut engine_inverse_pin, &mut engine_direction_pin);
					}
					TimeModeActionState::ShouldBeClosed => {
						try_close(&mut engine_inverse_pin, &mut engine_direction_pin);
					}
				}
			}

			if WINDOW_IS_OPENING.load(Ordering::SeqCst) && open_sensor.is_high() {
				delay_ms(10);
				if open_sensor.is_high() {
					WINDOW_IS_CLOSED.store(false, Ordering::SeqCst);
					WINDOW_IS_OPENED.store(true, Ordering::SeqCst);
					WINDOW_IS_OPENING.store(false, Ordering::SeqCst);
					engine_inverse_pin.set_high();
					engine_direction_pin.set_high();
				}
			}

			if WINDOW_IS_CLOSING.load(Ordering::SeqCst) && close_sensor.is_high() {
				delay_ms(10);
				if close_sensor.is_high() {
					WINDOW_IS_OPENED.store(false, Ordering::SeqCst);
					WINDOW_IS_CLOSED.store(true, Ordering::SeqCst);
					WINDOW_IS_CLOSING.store(false, Ordering::SeqCst);
					engine_inverse_pin.set_high();
					engine_direction_pin.set_high();
				}
			}
		}
	}
}

#[inline]
pub fn try_open<P1, P2>(
	engine_inverse_pin: &mut Pin<Output, P1>,
	engine_direction_pin: &mut Pin<Output, P2>,
) where
	P1: PinOps,
	P2: PinOps,
{
	unsafe {
		if WINDOW_IS_CLOSED.load(Ordering::SeqCst) {
			engine_direction_pin.set_high();
			engine_inverse_pin.set_low();
			WINDOW_IS_OPENING.store(true, Ordering::SeqCst);
		}
	}
}

#[inline]
pub fn try_close<P1, P2>(
	engine_inverse_pin: &mut Pin<Output, P1>,
	engine_direction_pin: &mut Pin<Output, P2>,
) where
	P1: PinOps,
	P2: PinOps,
{
	unsafe {
		if WINDOW_IS_OPENED.load(Ordering::SeqCst) {
			engine_direction_pin.set_low();
			engine_inverse_pin.set_low();
			WINDOW_IS_CLOSING.store(true, Ordering::SeqCst);
		}
	}
}

pub const fn calc_overflow(clock_hz: u32, target_hz: u32, prescale: u32) -> u32 {
	/*
	https://github.com/Rahix/avr-hal/issues/75
	reversing the formula F = 16 MHz / (256 * (1 + 15624)) = 4 Hz
	 */
	clock_hz / target_hz / prescale - 1
}

pub fn reg_timer(timer1: &TC1) {
	use arduino_hal::clock::Clock;

	const ATMEGA_CLOCK_FREQUENCY: u32 = arduino_hal::DefaultClock::FREQ;
	const CLOCK_SOURCE: CS1_A = CS1_A::PRESCALE_1024;
	let clock_divisor: u32 = match CLOCK_SOURCE {
		CS1_A::DIRECT => 1,
		CS1_A::PRESCALE_8 => 8,
		CS1_A::PRESCALE_64 => 64,
		CS1_A::PRESCALE_256 => 256,
		CS1_A::PRESCALE_1024 => 1024,
		_ => 1024, // CS1_A::NO_CLOCK | CS1_A::EXT_FALLING | CS1_A::EXT_RISING => {
		           // 		uwriteln!(serial, "uhoh, code tried to set the clock source to something other than a static prescaler {}", CLOCK_SOURCE as usize)
		           // 				.void_unwrap();
		           // 		1
		           // }
	};

	let ticks = calc_overflow(ATMEGA_CLOCK_FREQUENCY, 1, clock_divisor) as u16;

	timer1.tccr1a.write(|w| w.wgm1().bits(0b00));
	timer1.tccr1b.write(|w| {
		w.cs1()
			//.prescale_256()
			.variant(CLOCK_SOURCE)
			.wgm1()
			.bits(0b01)
	});
	timer1.ocr1a.write(|w| w.bits(ticks));
	timer1.timsk1.write(|w| w.ocie1a().set_bit()); //enable this specific interrupt
}

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
	unsafe {
		GLOBAL_TIME_IN_SEC = match GLOBAL_TIME_IN_SEC.cmp(&(SECONDS_IN_DAY - 1)) {
			cmp::Ordering::Less => GLOBAL_TIME_IN_SEC + 1,
			_ => 0,
		};

		if TIME_MODE.enabled {
			match TIME_MODE.action_state {
				TimeModeActionState::ShouldBeOpened => {
					TIME_MODE.active_time_offset = match TIME_MODE
						.active_time_offset
						.cmp(&TIME_MODE.active_time_in_sec)
					{
						cmp::Ordering::Less => TIME_MODE.active_time_offset + 1,
						_ => {
							TIME_MODE.action_state = TimeModeActionState::ShouldBeClosed;
							0
						}
					}
				}
				TimeModeActionState::ShouldBeClosed => {
					TIME_MODE.delay_time_offset = match TIME_MODE
						.delay_time_offset
						.cmp(&TIME_MODE.delay_time_in_sec)
					{
						cmp::Ordering::Less => TIME_MODE.delay_time_offset + 1,
						_ => {
							TIME_MODE.action_state = TimeModeActionState::ShouldBeOpened;
							0
						}
					}
				}
			}
		}

		if SCHEDULE.enabled {
			match SCHEDULE.open_time.cmp(&SCHEDULE.close_time) {
				cmp::Ordering::Less => {
					SCHEDULE.action_state = match GLOBAL_TIME_IN_SEC >= SCHEDULE.open_time
						&& GLOBAL_TIME_IN_SEC <= SCHEDULE.close_time
					{
						true => TimeModeActionState::ShouldBeOpened,
						false => TimeModeActionState::ShouldBeClosed,
					}
				}
				_ => {
					SCHEDULE.action_state = match GLOBAL_TIME_IN_SEC >= SCHEDULE.open_time
						|| GLOBAL_TIME_IN_SEC <= SCHEDULE.close_time
					{
						true => TimeModeActionState::ShouldBeOpened,
						false => TimeModeActionState::ShouldBeClosed,
					}
				}
			}
		}

		(*atmega_hal::pac::TC1::PTR).tcnt1.write(|w| w.bits(0))
	}
}

// #[no_mangle]
// pub unsafe extern "avr-interrupt" fn __vector_17() {
// 	// port::B5::toggle();
// 	// port::E0::toggle();
// 	// (*arduino_hal::pac::USART0::PTR).udr0.as_ptr()
// }

// #[avr_device::interrupt(atmega128rfa1)]
// fn USART1_TX() {
// 	let led = unsafe { &mut *LED.as_mut_ptr() };

// 	led.toggle();
// }
