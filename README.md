# AVR Embedded Systems Labs

Selected ATmega128A course exercises by Jaejun Lee. These examples complement the larger AVR Care Companion team project, which is maintained separately.

| Lab | Exercise | Main topics |
| --- | --- | --- |
| [09](09-light-controlled-curtain) | Light-controlled curtain simulation using a DC motor | ADC, timer-driven FSM, elapsed-time travel estimate |
| [10](10-uart-number-display) | UART-controlled four-digit display | UART RX interrupt, ring buffer, input parser FSM, multiplexed display |
| [11](11-adc-lcd-light-sensor) | Light sensor ADC readout on character LCD | ADC interrupt, atomic snapshot, 4-bit LCD interface |
| [12](12-ultrasonic-height-meter) | Ultrasonic height meter | Timer1, external edge interrupts, button-driven FSM, character LCD |
| [13](13-i2c-temperature-monitor) | LM75A temperature monitor | TWI/I2C register reads, seven-segment display, hold mode |

## Scope

These are historical coursework snapshots, not production drivers. Lab 09 estimates curtain travel from motor running time; it does not measure position with an encoder. Lab 11 displays ADC readings, not calibrated lux. Lab 13 follows the course example structure, with application behavior added around it.

Sources were recovered from submitted report attachments or separately supplied C files. Text encoding and line endings were normalized to UTF-8/LF. Code has not been rebuilt or retested for this archive; exact build settings and wiring are not yet packaged. Historical reports and videos are omitted from this initial release.

The course reports contain observations and design rationales, but these have not been independently verified here. In particular, this repository makes no guarantee of buffer-overflow immunity, timing accuracy, or measured power savings. Course-provided/example code and student changes are not yet separated line by line.

Lab 12 was added from a separately supplied source snapshot. The author reports historical hardware operation; its immediate completion-flag check remains an unresolved source-review observation, documented in the lab README.
