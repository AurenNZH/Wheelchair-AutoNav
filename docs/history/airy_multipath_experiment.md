# AIRY Multipath Filter Experiment

The shadow-map filter was evaluated in July 2026 after scattered cells appeared
near reflective obstacles and wheelchair hardware.

It retained multi-cell evidence immediately and required isolated cells to
persist across two of three frames. The filtered and rejected maps were never
connected to shared control.

Physical inspection later identified contamination on the AIRY dome as the
dominant source. Cleaning with isopropyl-alcohol wipes reduced ghost cells to
almost zero, including within 0.5 m of obstacles. The software filter had
removed few cells before cleaning and essentially none afterward.

The live filter and its diagnostic topics were therefore retired in favour of:

- a clean-dome inspection before testing;
- a non-reflective hood preventing the AIRY from viewing wheelchair steel;
- the measured chassis/mount self-filter;
- raw sensor evidence for all safety decisions.

The implementation remains recoverable from commits `7569695` and `418702d`.
Any future reflection filter must again run as a shadow output and demonstrate
that it never suppresses a real narrow obstacle before downstream adoption.

