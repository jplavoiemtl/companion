# OwnedPubSubClient

Project-local renamed PubSubClient 2.8 (Nick O'Leary), from the installed Arduino
PubSubClient_2.8_48867b22d3bf7501 package. Original license retained. No global library
edits. Local changes: operation cancellation/deadline guard, vTaskDelay(1) in network
polling/packet read loops, bounded packet/remaining-length validation, and read errors.
Finite local buffer-copy/header-encoding loops are not network polling loops.
Only the MQTT worker may use this client. Its Client facade rejects hidden reconnects
and enforces the operation guard on every transport boundary.
