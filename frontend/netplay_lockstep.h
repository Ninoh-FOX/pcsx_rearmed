/*
 * Lockstep netplay over the libretro netpacket interface.
 *
 * Both consoles run the very same PSX: the host is pad 1, the client pad 2.
 * When they connect both do a power-on reset (the host's memory cards are
 * used by both), then only the pads travel over the network, a few frames
 * ahead of when they're used. No savestates, no rollback.
 */
#ifndef NETPLAY_LOCKSTEP_H
#define NETPLAY_LOCKSTEP_H

#include <stdbool.h>
#include <stdint.h>
#include "libretro.h"

/* What a console sends for each frame: its own pad. */
struct lsk_pad {
   uint32_t keys;      /* PSX buttons (in_keystate bits) + LSK_KEY_* */
   int16_t axes[4];    /* left x/y, right x/y */
};

#define LSK_KEY_ANALOG_TOGGLE 0x40000000u /* DualShock ANALOG button combo */
#define LSK_KEY_RESET         0x80000000u /* everybody resets this frame */

enum { LSK_SKIP = 0, LSK_RUN };

void lsk_init(retro_environment_t environ_cb);
void lsk_deinit(void);

/* Session running (pads synced): the core must not load states, apply
 * cheats, swap discs... and must take the pads from lsk_frame_begin(). */
bool lsk_running(void);
/* Anything going on with a peer (also while it's being set up). */
bool lsk_busy(void);

/* Called every retro_run, after the local pad was read into *local.
 * LSK_RUN: the PSX pads are set, run the frame.
 * LSK_SKIP: don't run the frame (waiting for the other console). */
int lsk_frame_begin(const struct lsk_pad *local);
void lsk_frame_end(void);

void lsk_request_reset(void);
void lsk_request_analog_toggle(void);
/* Our pad (0 or 1) while running, -1 otherwise. */
int lsk_local_port(void);
/* Settings that change the machine were changed in the menu. */
void lsk_check_config(void);

/* Provided by libretro.c */
void core_power_on_reset(void);
void lsk_core_reset(void);
void lsk_core_message(const char *msg, unsigned ms);
uint32_t lsk_core_cfg_hash(void);
uint32_t lsk_core_game_hash(void);
uint32_t lsk_core_bios_hash(void);
const char *lsk_core_build_id(void);
int lsk_core_local_pad_type(void);
void lsk_core_session_begin(int pad1_type, int pad2_type);
void lsk_core_session_end(void);
void lsk_core_analog_toggle(int port);

#endif
