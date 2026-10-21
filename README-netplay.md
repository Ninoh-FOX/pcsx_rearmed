# Netplay on the Miyoo Mini (lockstep)

The Miyoo Mini build (`make -f Makefile.libretro platform=miyoomini`, gives
`pcsx_rearmed_miyoo_libretro.so`) plays netplay in **lockstep** through the
libretro netpacket interface: RetroArch only carries the packets, there are
no savestates every frame and no rollback (a PSX state is 4 MB, the Miyoo
Mini has 128 MB and can't re-run frames anyway).

- The host is **player 1**, the client **player 2** (2 players).
- When they connect, **both consoles reset** (a power-on reset: nothing of
  what each one was playing stays) and from then on compute the very same
  frames. Only the pads cross the network.
- Both play with the **host's memory cards** (128 KB each, sent once when
  linking). Saves made while linked go to the host's card. The client's own
  card isn't touched (also not by RetroArch's autosave) and is back when the
  session ends.
- The input delay comes from the measured round trip (2 frames or more).
- Both screens (and sounds) are kept within a frame of each other: two
  consoles never run at exactly the same speed, so the one that gets a
  frame ahead shows a frame twice (about once every 10 s).
- If the WiFi has a bad moment and a console has to wait for the other's
  pads, that one sends its pads a frame earlier from then on (up to 4
  more), one less after a calm minute.
- Every 2 seconds both consoles compare their PSX RAM; if they ever differ
  the link is cut with a message ("went out of sync") and each one goes on
  alone.

## What must match

The same core build, game (disc and disc number), BIOS (or HLE on both)
and the core options that change the machine: CPU clock, dynarec and its
hacks, region, dithering, multitap, vibration, threaded SPU, CD and XA/CDDA
options... If anything differs both consoles say "games, BIOS, core or
settings differ, not linked" and each one plays alone.

## While linked

- Savestates can't be loaded, cheats are off, discs can't be changed and
  fast forward is disabled (they would make the consoles differ).
- Reset (RetroArch's) resets both consoles on the same frame.
- The DualShock ANALOG button combo goes through the link.
- Pads only (standard, analog, DualShock): a gun or mouse is used as a pad.
- Frameskip is off; the dynarec compile thread is off (when a block gets
  compiled would depend on the host's thread timing).

## Testing without consoles

`tools/lockstep_test/run.sh` runs two consoles (two processes joined by a
socket) under `qemu-arm` on the real arm32 build:

    tools/lockstep_test/run.sh /path/game.chd /path/bios_dir 600 2400 20
    TEST_RESET=1 TEST_PACE=1 TEST_JITTER=30 tools/lockstep_test/run.sh ...
