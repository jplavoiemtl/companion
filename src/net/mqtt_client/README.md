# OwnedPubSubClient

Project-local renamed PubSubClient 2.8 (Nick O'Leary), from the installed Arduino
PubSubClient_2.8_48867b22d3bf7501 package. Original license retained. No global library
edits. Local changes: operation cancellation/deadline guard, vTaskDelay(1) in network
empty-input polling and chunked writes, bounded packet/remaining-length validation,
and read errors. Buffered packet bodies yield once per 64 bytes, not per byte; the
four-byte remaining-length parser delegates empty waits to readByte. Packets exceeding
the 512-byte wire buffer are counted and discarded under the unchanged absolute 5 s
operation deadline, preserving the session on a complete discard. Malformed lengths
and remaining lengths above 16 KiB close the connection, as do deadline/cancellation
failures. Oversized messages are not streamed or delivered to the application.
Finite local buffer-copy/header-encoding loops are not network polling loops.
Only the MQTT worker may use this client. Its Client facade rejects hidden reconnects
and enforces the operation guard on every transport boundary.
