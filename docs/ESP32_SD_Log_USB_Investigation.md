# ESP32-S3 SD Card Log Retrieval over USB — Investigation Task

## Objective

I use a **Waveshare ESP32-S3-Touch-AMOLED-1.8-3D** board and develop/flash it from **VS Code** over its USB-C connection.

My firmware already uses the USB serial connection extensively for diagnostic output. In VS Code's Serial Monitor I can see information such as:

- Wi-Fi connection/status
- MQTT connection and messages
- Images being received/displayed
- General firmware status and debugging information

The board also has a **microSD card slot**. I now want the firmware to write persistent diagnostic/log files to the SD card.

My goal is to retrieve those SD-card log files from my Windows PC **through the existing USB-C cable, without physically removing the SD card**.

Please investigate practical solutions and help me prototype the simplest reliable approach.

### USB-only constraint

The companion board normally connects to a **Wi-Fi hotspot provided by my iPhone**. That Wi-Fi connection is part of the board's normal operation and must not be repurposed or disturbed for diagnostic-file retrieval.

Therefore, **all solutions in this investigation must retrieve the SD-card files through the physical USB-C cable only**. Do not propose an HTTP server, Wi-Fi file transfer, access point, network file server, or any other network-based retrieval mechanism. Wi-Fi and MQTT should simply continue their normal operation independently while USB is used for diagnostics.

---

## Existing Project and Hardware

Work from the **current Companion software project**, not from a new standalone project. The board and its hardware configuration are already well defined in the project documentation and existing source code, so use those as the authoritative reference rather than re-identifying or re-documenting the board.

Create a **new Git branch dedicated to this feature/investigation** and make the minimal test changes there.

The objective is to add the smallest amount of code necessary to the existing Companion software to test SD-card logging and USB retrieval while preserving the application's normal behavior.

Do not redesign unrelated parts of the Companion software.

## Desired User Experience

I would like something simple.

### Option A — Temporarily use the existing serial port

Normal operation:

```text
ESP32-S3 -- USB-C --> COM port --> VS Code Serial Monitor
```

When I want diagnostic files:

1. Stop/close the VS Code Serial Monitor so it releases the COM port.
2. Open a browser-based diagnostic/file utility in Chrome or Edge.
3. Click Connect.
4. Select the ESP32 COM port.
5. Browse/list files on the ESP32 microSD card.
6. Download one or more log files to the PC.
7. Disconnect the browser from the serial port.
8. Reopen the VS Code Serial Monitor.

The ESP32 application should preferably continue running during this operation. Wi-Fi, MQTT, display processing, sensors, etc. should not need to stop.

This currently seems like the **preferred first solution** because it should require minimal changes and minimal USB complexity.

A simple command protocol could be implemented over the existing serial connection, for example:

```text
DIAG_INFO
DIAG_SDINFO
DIAG_LIST /logs
DIAG_GET /logs/system.log
DIAG_DELETE /logs/old.log
```

The exact protocol is open for discussion.

During a file transfer, normal `Serial.print()` diagnostic messages must not corrupt the transferred file. Please propose a simple robust mechanism for handling this.

---

## Browser / Web Serial Idea

Investigate using the browser **Web Serial API** in Chrome/Edge.

A small HTML/JavaScript application could connect directly to the ESP32 COM port and provide something like:

```text
ESP32 Diagnostic Manager

Device: ESP32-S3
SD Card: 32 GB
Free: 27 GB

/logs

system.log       78 KB     [Download]
mqtt.log         12 KB     [Download]
errors.log        4 KB     [Download]

[Refresh]
[Download All]
[Disconnect]

Serial Monitor:
--------------------------------
WiFi connected
MQTT connected
Image received
...
```

While the browser owns the COM port, it would be useful if the web application could also display the ordinary serial/debug messages so that I do not completely lose my Serial Monitor.

The web page could eventually be hosted at a normal HTTPS URL, but for initial testing a simple local HTML application/server is perfectly acceptable.

---

## Option B — Composite USB / Two Interfaces

Because the ESP32-S3 has native USB, investigate whether we can reliably expose two interfaces over the same physical USB-C cable.

For example:

```text
                     ESP32-S3
                        |
                     USB-C
                        |
              +---------+---------+
              |                   |
          USB CDC #1          USB CDC #2
              |                   |
            COM7                COM8
              |                   |
          VS Code              Browser/
       Serial Monitor        Diagnostic Tool
```

This would potentially allow VS Code to keep the normal Serial Monitor open while a second interface handles SD-card file access.

Please determine:

- Whether this is practical with the current Arduino-ESP32 / TinyUSB stack.
- Whether the exact Waveshare board supports it cleanly.
- How much firmware complexity it adds.
- Whether Windows enumerates both interfaces reliably.
- Whether Web Serial can select/use the second interface.
- Whether this is worth doing compared with Option A.

Do **not** start by implementing a complicated composite USB solution if the simple shared-COM-port approach works well.

---

## Option C — USB Mass Storage

Also investigate, but treat as secondary, exposing the SD card through **USB Mass Storage (MSC)** so Windows sees it as a drive.

Conceptually:

```text
ESP32-S3
   |
   +-- USB CDC --> VS Code Serial Monitor
   |
   +-- USB MSC --> Windows drive containing logs
```

I am concerned about filesystem corruption if both the ESP32 firmware and Windows access the FAT filesystem simultaneously.

If MSC is investigated, explicitly consider ownership:

```text
Normal:
ESP32 owns SD and writes logs

PC access requested:
ESP32 flushes/closes log files
ESP32 relinquishes filesystem
Windows/USB MSC owns SD

Finished:
Windows releases/ejects drive
ESP32 remounts SD
Logging resumes
```

Determine whether this is sufficiently reliable and simple to justify using it. I suspect it may be unnecessarily complicated for this project.

---

## What I Want You to Do

Please approach this experimentally rather than designing a large framework immediately.

1. Inspect the current Companion project, its documentation, and existing source code to understand how USB Serial and the SD card are configured.
2. Create a **new Git branch** for this SD-log/USB-retrieval feature and perform the investigation there.
3. Treat the board definition already present in the project documentation as authoritative. Only consult external hardware documentation when a specific USB capability needs verification.
4. Compare the USB approaches above and identify the **simplest robust solution**.
5. Prefer **Option A (existing COM port + simple file-transfer protocol + Web Serial)** initially unless testing reveals a significant problem.
6. Add a **minimal proof-of-concept directly to the existing Companion software**. Do not create a separate firmware project from scratch.
7. Keep the changes small and localized so the Companion application continues its normal operation.
8. Have this test code:
   - initialize the SD card;
   - create a small diagnostic file;
   - append a few test log entries;
   - list files/directories;
   - respond to a command requesting a file;
   - transfer that file over USB Serial.
9. Create the smallest practical PC/browser test client to retrieve the file.
10. Verify the downloaded file byte-for-byte if possible.
11. Test disconnecting the browser and reconnecting VS Code's Serial Monitor.
12. Observe whether opening/closing the USB connection resets the ESP32 or otherwise affects normal operation.
13. Document any issues with DTR/RTS, USB CDC, baud rate, browser permissions, Windows COM-port ownership, or ESP32-S3 USB configuration.
14. Only after the minimal test works should we decide whether to integrate it into the full companion software.

---

## Important Design Priorities

In order of importance:

1. **Simple**
2. **Reliable**
3. **Minimal changes to my existing firmware**
4. **No removal of the SD card**
5. **Uses only the existing USB-C cable**
6. **Does not interfere with normal Wi-Fi/MQTT/display operation or the iPhone Wi-Fi hotspot connection**
7. **Easy to reuse in my other ESP32 projects**
8. Nice browser interface later

Please avoid overengineering the first prototype.

I want a small proof of concept that answers:

> Can I reliably stop the VS Code Serial Monitor, connect to the same ESP32 USB serial port from Chrome/Edge, list and download files from the SD card, disconnect, and then return to the VS Code Serial Monitor?

If the answer is yes, that is likely sufficient for Version 1.

---

## Expected Output From This Investigation

Please report back with:

- What you found about the USB implementation already used by the Companion project, plus any specific hardware/USB details that needed external verification.
- Which approach you recommend and why.
- Any problems discovered during minimal testing.
- The minimal changes made to the existing Companion software for the test.
- The minimal browser/PC-side test code.
- Exact steps for testing it.
- Results of the SD-card file transfer test.
- Whether the file downloaded correctly.
- Whether VS Code Serial Monitor works normally again afterward.
- What changes would eventually be required in my existing companion firmware.

Please **do not make major or unrelated changes to the existing Companion project without discussing the findings with me first**. Work in a new branch and implement the proof of concept as a small, localized addition to the current software rather than as a separate project.
