# Avr window opener project

this crate will work only with atmega328p based boards

### Compile the crate to an ELF executable.
cargo build -Z build-std=core --target avr-atmega128rfa1.json --release

### SIGNALS

each signal must end with ';' char

| Signal type | Request | Response |
|:-:|:-:|:-:|
| open window | 'o' | null |
| close window | 'c' | null |
| get window state | 's' | 'o' or 'c' |
| get time | 't' | 't' + 4 bytes |
| update time | 'u' + 4 bytes | 'ok' |
| set range | 'r' + 8 bytes | 'enable_ok' |
| disable time mode | 'd' | 'disable_ok' |
| enable time mode | 'e' | 'enable_ok' or 'enable_err' |