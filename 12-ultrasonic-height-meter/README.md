# Lab 12: Ultrasonic Height Meter

ATmega128A course exercise combining Timer1, INT7 rising/falling-edge interrupts, a button-driven FSM, and a character LCD. Height is computed from the difference between a floor-to-ceiling measurement and a head-to-ceiling measurement.

The author reports successful hardware operation during the course. This supplied source snapshot has not been rebuilt or hardware-tested during portfolio preparation.

## Source review note

In this snapshot, `ulsonic_measure_ticks()` checks the completion flag immediately after triggering and returns zero if the flag is not set. There is no elapsed-time timeout or explicit wait for the new echo measurement to complete. The relationship between this snapshot and the historically demonstrated binary remains unverified. The source is preserved unchanged rather than described as independently validated.
