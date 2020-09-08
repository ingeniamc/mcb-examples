# MCB master CoC example

## Description

This example shows a fully working MCBus application, with the following steps:

1. Set up the MCB environment and send several config messages.
2. Set the cyclic mappings and change the MCB state to the cyclic mode. Cyclic mode is executed through a timer each 2 ms.
3. While being in cyclic state, the Config Over Cyclic (CoC) mechanism is used to send acyclic messages while cyclic communication is working.
4. After a certain number of cyclic transactions, cyclic mode is deactivated and MCB returns to config mode. Finally application ends.

## Files

Code is commented to help understanding.
Following files are the most important to understand the example:

* **main.c**. It contains the main program, the needed initializations and is where the application functions are called.
* **mcb_al.c**. This is the adaptation layer for the MCB library. It contains all the needed functions to be adapted depending on each platform.
* **mcb-master-coc.c**. It is the example application.

## Who do I talk to?

This repository is maintained by Ingenia FW team.
