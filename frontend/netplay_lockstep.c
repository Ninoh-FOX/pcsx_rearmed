/*
 * Lockstep netplay over the libretro netpacket interface. See the header.
 *
 *   host (pad 1)                          client (pad 2)
 *                         <-------------  HELLO (build, game, BIOS, settings)
 *   GIVEUP (something differs) -------->  (each one plays alone)
 *   PING  ----------------------------->  PONG  (round trip -> input delay)
 *   (stops its game)
 *   MCD (its memory cards) ------------>
 *   START (delay, pad types) ---------->  power-on reset
 *                         <-------------  READY
 *   power-on reset
 *   INPUT (pad for frame N+delay) <---->  INPUT, every frame
 *   HASH (RAM, sometimes) <------------>  HASH: must agree
 *
 * The pads for frame N are sent while running frame N - delay, so they're
 * normally there in time. If they aren't, the frame isn't run (the last
 * picture stays) and it's tried again on the next retro_run.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "netplay_lockstep.h"
#include "plugin_lib.h"
#include "../libpcsxcore/psxcommon.h"
#include "../libpcsxcore/r3000a.h"
#include "../libpcsxcore/sio.h"

#define LSK_MAGIC         0x4c585350u   /* "PSXL" */
#define LSK_VERSION       1
#define LSK_PROTOCOL      "pcsx_rearmed lockstep 1"

#define PK_HELLO          1
#define PK_GIVEUP         2
#define PK_PING           3
#define PK_PONG           4
#define PK_MCD            5
#define PK_START          6
#define PK_READY          7
#define PK_INPUT          8
#define PK_HASH           9
#define PK_BYE            10

#define RING              128           /* frames of pads kept */
#define HASH_EVERY        120           /* frames between RAM checks */
#define HASH_SLOTS        8
#define PINGS             8
#define PING_US           1500000       /* max time measuring */
#define PONG_US           250000        /* no answer: ping again */
#define STALL_US          12000         /* longest wait for a pad in a frame */
#define PEER_US           15000000      /* peer silent this long: gone */
#define SETUP_US          30000000      /* whole setup */
#define HELLO_EVERY_US    250000        /* HELLO again until the host answers */
#define FRAME_US          16683
#define DELAY_MIN         2
#define DELAY_MAX         10
#define AHEAD_MAX         4             /* frames our pads may go earlier */
#define AHEAD_GAP_US      1000000       /* between two of those */
#define AHEAD_CALM_US     60000000      /* peer happy this long: one less */
#define MCD_CHUNK         (16 * 1024)
#define MCD_BLOB          (2 * MCD_SIZE)

enum {
   ST_OFF = 0,
   ST_FAILED,        /* session on, but each plays alone */
   ST_HOST_WAIT,     /* hosting, nobody yet */
   ST_HOST_PING,
   ST_HOST_SEND,     /* next frame: send the cards and START */
   ST_HOST_READY,    /* waiting for READY (game held) */
   ST_HOST_GO,       /* next frame: reset and run */
   ST_CLIENT_HELLO,  /* waiting for the host */
   ST_CLIENT_GO,     /* next frame: install the cards, reset and run */
   ST_RUN,
};

enum { GIVEUP_DIFFER = 1, GIVEUP_VERSION, GIVEUP_FULL };

static retro_environment_t env_cb;
static retro_netpacket_send_t send_fn;
static retro_netpacket_poll_receive_t poll_fn;

static struct {
   int state;
   int is_host;
   uint16_t my_id;          /* frontend client id: 0 host */
   int peer_id;             /* host: the client's id, -1 none */
   uint64_t t_state;        /* when the state was entered */
   uint64_t t_heard;        /* last packet from the peer */

   /* setup */
   int host_answered;       /* client: the host got our HELLO */
   uint64_t hello_sent;
   uint32_t ping_seq, pongs;
   uint64_t ping_sent, rtt_sum;
   int peer_pad_type, pad_type[2];
   uint32_t delay;

   /* memory cards: host's copy on the client */
   uint8_t *mcd_in;         /* received */
   uint8_t mcd_disable[2];
   uint8_t mcd_disable_backup[2];
   char mcd_path_backup[2][MAXPATHLEN];
   int mcd_installed;

   /* running */
   uint32_t frame;          /* next frame to run */
   uint32_t sent;           /* our pad sent up to (excluding) this frame */
   uint32_t tag[RING][2];   /* frame + 1 each pad is for, 0: none */
   struct lsk_pad pad[RING][2];
   uint32_t pending_flags;  /* reset/analog toggle for our next pad */
   uint32_t hash_frame[HASH_SLOTS][2], hash_val[HASH_SLOTS][2];
   int hash_have[HASH_SLOTS][2];
   uint32_t stalls;         /* times we waited for the peer */
   uint32_t peer_stalls;    /* same, the peer's (comes with its pads) */
   uint32_t ahead;          /* our pads go this many frames earlier */
   uint64_t t_ahead;
} s;

static uint64_t now_us(void)
{
   struct timespec ts;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   return (uint64_t)ts.tv_sec * 1000000u + ts.tv_nsec / 1000;
}

static void set_state(int st)
{
   s.state = st;
   s.t_state = now_us();
}

static uint32_t crc32_buf(uint32_t crc, const void *data, size_t len)
{
   const uint8_t *p = data;
   crc = ~crc;
   while (len--) {
      int k;
      crc ^= *p++;
      for (k = 0; k < 8; k++)
         crc = (crc >> 1) ^ (0xedb88320u & -(crc & 1));
   }
   return ~crc;
}

/* ---- packets ---- */

static void put32(uint8_t *p, uint32_t v)
{
   p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static uint32_t get32(const uint8_t *p)
{
   return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void send_pk(uint32_t type, const uint32_t *w, unsigned nw,
      const void *data, unsigned len)
{
   uint8_t buf[16 + 16 * 4 + MCD_CHUNK];
   unsigned i, n = 0;
   if (!send_fn || s.peer_id < 0 || nw > 16 || len > MCD_CHUNK)
      return;
   put32(buf + n, LSK_MAGIC); n += 4;
   put32(buf + n, type | (nw << 8)); n += 4;
   for (i = 0; i < nw; i++, n += 4)
      put32(buf + n, w[i]);
   if (len)
      memcpy(buf + n, data, len);
   n += len;
   send_fn(RETRO_NETPACKET_RELIABLE | RETRO_NETPACKET_FLUSH_HINT, buf, n,
         s.is_host ? (uint16_t)s.peer_id : 0);
}

static void msg(const char *text)
{
   SysPrintf("netplay: %s\n", text);
   lsk_core_message(text, 4000);
}

/* ---- session ---- */

/* The client plays with the host's cards: the emulated machine is pointed
 * to them, its own cards (what the frontend saves, also every autosave
 * while linked) aren't touched. */
static void restore_memcards(void)
{
   if (!s.mcd_installed)
      return;
   Mcd1Data = Mcd1Store;
   Mcd2Data = Mcd2Store;
   McdDisable[0] = s.mcd_disable_backup[0];
   McdDisable[1] = s.mcd_disable_backup[1];
   memcpy(Config.Mcd1, s.mcd_path_backup[0], sizeof(Config.Mcd1));
   memcpy(Config.Mcd2, s.mcd_path_backup[1], sizeof(Config.Mcd2));
   // let the game know the card changed
   McdFlag[0] |= 8;
   McdFlag[1] |= 8;
   s.mcd_installed = 0;
   free(s.mcd_in);
   s.mcd_in = NULL;
}

static void install_memcards(void)
{
   s.mcd_disable_backup[0] = McdDisable[0];
   s.mcd_disable_backup[1] = McdDisable[1];
   memcpy(s.mcd_path_backup[0], Config.Mcd1, sizeof(Config.Mcd1));
   memcpy(s.mcd_path_backup[1], Config.Mcd2, sizeof(Config.Mcd2));

   Mcd1Data = (char *)s.mcd_in;
   Mcd2Data = (char *)s.mcd_in + MCD_SIZE;
   McdDisable[0] = s.mcd_disable[0];
   McdDisable[1] = s.mcd_disable[1];
   // the host's cards are only borrowed: never write them to our files
   Config.Mcd1[0] = 0;
   Config.Mcd2[0] = 0;
   s.mcd_installed = 1;
}

static void ff_inhibit(bool on)
{
   struct retro_fastforwarding_override ff;
   memset(&ff, 0, sizeof(ff));
   ff.inhibit_toggle = on;
   if (env_cb)
      env_cb(RETRO_ENVIRONMENT_SET_FASTFORWARDING_OVERRIDE, &ff);
}

/* Back to each one playing alone. failed: the peer is still there but
 * there's no link with it (the host waits for it to leave). */
static void session_end(const char *why, int failed)
{
   if (s.state == ST_OFF)
      return;
   if (s.state == ST_RUN) {
      restore_memcards();
      lsk_core_session_end();
      ff_inhibit(false);
   }
   if (!s.mcd_installed) {
      free(s.mcd_in);
      s.mcd_in = NULL;
   }
   if (why)
      msg(why);
   if (!send_fn)
      set_state(ST_OFF);
   else if (s.is_host && !failed)
      set_state(ST_HOST_WAIT);
   else
      set_state(ST_FAILED);
}

/* Pads and checks start from scratch (set when the GO is known: the other
 * console's first pads can come in the same batch of packets). */
static void run_prepare(void)
{
   memset(s.tag, 0, sizeof(s.tag));
   memset(s.hash_have, 0, sizeof(s.hash_have));
   s.frame = 0;
   s.sent = s.delay;        /* frames below the delay: no buttons */
   s.pending_flags = 0;
   s.stalls = 0;
   s.peer_stalls = 0;
   s.ahead = 0;
   s.t_ahead = now_us();
}

static uint32_t session_cfg;

static void run_begin(void)
{
   ff_inhibit(true);
   lsk_core_session_begin(s.pad_type[0], s.pad_type[1]);
   core_power_on_reset();
   session_cfg = lsk_core_cfg_hash();
   set_state(ST_RUN);
   s.t_heard = now_us();
   {
      char text[96];
      snprintf(text, sizeof(text), "Netplay: linked, you are player %d (%u frames of delay)",
            s.is_host ? 1 : 2, s.delay);
      msg(text);
   }
}

static void host_send_cards_and_start(void)
{
   uint8_t *blob = malloc(MCD_BLOB);
   uint32_t w[8], off;
   if (!blob) {
      session_end("Netplay: out of memory", 1);
      return;
   }
   memcpy(blob, Mcd1Data, MCD_SIZE);
   memcpy(blob + MCD_SIZE, Mcd2Data, MCD_SIZE);
   for (off = 0; off < MCD_BLOB; off += MCD_CHUNK) {
      w[0] = off;
      send_pk(PK_MCD, w, 1, blob + off, MCD_CHUNK);
   }
   w[0] = s.delay;
   w[1] = s.pad_type[0];
   w[2] = s.pad_type[1];
   w[3] = crc32_buf(0, blob, MCD_BLOB);
   w[4] = McdDisable[0] | (McdDisable[1] << 8);
   send_pk(PK_START, w, 5, NULL, 0);
   free(blob);
   set_state(ST_HOST_READY);
}

static uint32_t compute_delay(void)
{
   uint64_t rtt = s.pongs ? s.rtt_sum / s.pongs : 2 * FRAME_US;
   /* one way + 1.6 frames, rounded up */
   uint32_t d = (uint32_t)((rtt / 2 + FRAME_US * 16 / 10 + FRAME_US - 1) / FRAME_US);
   if (d < DELAY_MIN) d = DELAY_MIN;
   if (d > DELAY_MAX) d = DELAY_MAX;
   SysPrintf("netplay: rtt %u us over %u pings: delay %u\n",
         (unsigned)rtt, s.pongs, d);
   return d;
}

static void send_ping(void)
{
   uint32_t w[1] = { ++s.ping_seq };
   s.ping_sent = now_us();
   send_pk(PK_PING, w, 1, NULL, 0);
}

/* ---- receiving ---- */

static void hash_check(int slot)
{
   char text[96];
   if (!s.hash_have[slot][0] || !s.hash_have[slot][1])
      return;
   if (s.hash_frame[slot][0] != s.hash_frame[slot][1])
      return;
   if (s.hash_val[slot][0] == s.hash_val[slot][1]) {
      SysPrintf("netplay: frame %u in sync (%08x), %u waits\n",
            s.hash_frame[slot][0], s.hash_val[slot][0], s.stalls);
      s.hash_have[slot][0] = s.hash_have[slot][1] = 0;
      return;
   }
   snprintf(text, sizeof(text), "Netplay: the consoles went out of sync (frame %u), unlinked",
         s.hash_frame[slot][0]);
   send_pk(PK_BYE, NULL, 0, NULL, 0);
   session_end(text, 1);
}

static void hash_store(int who, uint32_t frame, uint32_t val)
{
   int slot = (frame / HASH_EVERY) % HASH_SLOTS;
   s.hash_frame[slot][who] = frame;
   s.hash_val[slot][who] = val;
   s.hash_have[slot][who] = 1;
   hash_check(slot);
}

static void on_hello(const uint32_t *w, unsigned nw)
{
   uint32_t mine[5];
   int differ;
   if (!s.is_host || s.state != ST_HOST_WAIT || nw < 6)
      return;
   if (w[0] != LSK_VERSION) {
      uint32_t r = GIVEUP_VERSION;
      send_pk(PK_GIVEUP, &r, 1, NULL, 0);
      msg("Netplay: the other console has another version of the core");
      set_state(ST_FAILED);
      return;
   }
   mine[0] = crc32_buf(0, lsk_core_build_id(), strlen(lsk_core_build_id()));
   mine[1] = lsk_core_game_hash();
   mine[2] = lsk_core_bios_hash();
   mine[3] = lsk_core_cfg_hash();
   differ = memcmp(mine, w + 1, 4 * sizeof(mine[0])) != 0;
   if (differ) {
      uint32_t r = GIVEUP_DIFFER;
      SysPrintf("netplay: build %08x/%08x game %08x/%08x bios %08x/%08x cfg %08x/%08x\n",
            mine[0], w[1], mine[1], w[2], mine[2], w[3], mine[3], w[4]);
      send_pk(PK_GIVEUP, &r, 1, NULL, 0);
      msg("Netplay: games, BIOS, core or settings differ, not linked");
      set_state(ST_FAILED);
      return;
   }
   s.peer_pad_type = w[5];
   s.pad_type[0] = lsk_core_local_pad_type();
   s.pad_type[1] = s.peer_pad_type;
   s.pongs = 0;
   s.rtt_sum = 0;
   set_state(ST_HOST_PING);
   msg("Netplay: player 2 connected, linking...");
   send_ping();
}

static void on_input(const uint32_t *w, unsigned nw)
{
   uint32_t f, i;
   int who = s.is_host ? 1 : 0;
   if ((s.state != ST_RUN && s.state != ST_HOST_GO && s.state != ST_CLIENT_GO)
         || nw < 5)
      return;
   /* the peer had to wait for our pads: send them a frame earlier */
   if (w[4] > s.peer_stalls) {
      uint64_t t = now_us();
      s.peer_stalls = w[4];
      if (s.ahead < AHEAD_MAX && t - s.t_ahead > AHEAD_GAP_US) {
         s.ahead++;
         s.t_ahead = t;
         SysPrintf("netplay: the other console waits, our pads %u frames earlier\n",
               s.delay + s.ahead);
      }
   }
   f = w[0];
   if (f < s.frame || f >= s.frame + RING)
      return;
   i = f % RING;
   s.pad[i][who].keys = w[1];
   s.pad[i][who].axes[0] = (int16_t)(w[2] & 0xffff);
   s.pad[i][who].axes[1] = (int16_t)(w[2] >> 16);
   s.pad[i][who].axes[2] = (int16_t)(w[3] & 0xffff);
   s.pad[i][who].axes[3] = (int16_t)(w[3] >> 16);
   s.tag[i][who] = f + 1;
}

static void RETRO_CALLCONV np_receive(const void *buf, size_t len, uint16_t client_id)
{
   const uint8_t *p = buf;
   uint32_t w[16], type;
   unsigned nw, i;

   if (len < 8 || get32(p) != LSK_MAGIC)
      return;
   if (s.is_host && (int)client_id != s.peer_id)
      return;
   type = get32(p + 4);
   nw = (type >> 8) & 0xff;
   type &= 0xff;
   if (nw > 16 || len < 8 + nw * 4)
      return;
   for (i = 0; i < nw; i++)
      w[i] = get32(p + 8 + i * 4);
   p += 8 + nw * 4;
   len -= 8 + nw * 4;
   s.t_heard = now_us();
   if (!s.is_host && (type == PK_PING || type == PK_GIVEUP || type == PK_MCD
         || type == PK_START))
      s.host_answered = 1;

   switch (type) {
   case PK_HELLO:
      on_hello(w, nw);
      break;
   case PK_GIVEUP:
      if (!s.is_host) {
         session_end(w[0] == GIVEUP_VERSION
               ? "Netplay: the other console has another version of the core"
               : w[0] == GIVEUP_FULL ? "Netplay: the game is full (2 players)"
               : "Netplay: games, BIOS, core or settings differ, not linked", 1);
      }
      break;
   case PK_PING:
      if (!s.is_host)
         send_pk(PK_PONG, w, nw, NULL, 0);
      break;
   case PK_PONG:
      if (s.is_host && s.state == ST_HOST_PING && nw >= 1 && w[0] == s.ping_seq) {
         s.rtt_sum += now_us() - s.ping_sent;
         s.pongs++;
         if (s.pongs >= PINGS || now_us() - s.t_state > PING_US) {
            s.delay = compute_delay();
            set_state(ST_HOST_SEND);
         }
         else
            send_ping();
      }
      break;
   case PK_MCD:
      if (!s.is_host && s.state == ST_CLIENT_HELLO && nw >= 1
            && w[0] + len <= MCD_BLOB) {
         if (!s.mcd_in)
            s.mcd_in = calloc(1, MCD_BLOB);
         if (s.mcd_in)
            memcpy(s.mcd_in + w[0], p, len);
      }
      break;
   case PK_START:
      if (!s.is_host && s.state == ST_CLIENT_HELLO && nw >= 5) {
         if (!s.mcd_in || crc32_buf(0, s.mcd_in, MCD_BLOB) != w[3]) {
            send_pk(PK_BYE, NULL, 0, NULL, 0);
            session_end("Netplay: memory cards didn't arrive, not linked", 1);
            break;
         }
         s.delay = w[0];
         s.pad_type[0] = w[1];
         s.pad_type[1] = w[2];
         s.mcd_disable[0] = w[4] & 0xff;
         s.mcd_disable[1] = (w[4] >> 8) & 0xff;
         run_prepare();
         set_state(ST_CLIENT_GO);
      }
      break;
   case PK_READY:
      if (s.is_host && s.state == ST_HOST_READY) {
         run_prepare();
         set_state(ST_HOST_GO);
      }
      break;
   case PK_INPUT:
      on_input(w, nw);
      break;
   case PK_HASH:
      if (s.state == ST_RUN && nw >= 2)
         hash_store(1, w[0], w[1]);
      break;
   case PK_BYE:
      session_end("Netplay: the other console left, playing alone", 1);
      break;
   }
}

/* ---- frontend callbacks ---- */

/* The frontend starts the client's session before it asks the host to
 * play: until then the host doesn't know it (and drops what it sends), so
 * HELLO goes again every little while until the host answers. */
static void send_hello(void)
{
   uint32_t w[6];
   const char *b = lsk_core_build_id();
   w[0] = LSK_VERSION;
   w[1] = crc32_buf(0, b, strlen(b));
   w[2] = lsk_core_game_hash();
   w[3] = lsk_core_bios_hash();
   w[4] = lsk_core_cfg_hash();
   w[5] = lsk_core_local_pad_type();
   s.hello_sent = now_us();
   send_pk(PK_HELLO, w, 6, NULL, 0);
}

static void RETRO_CALLCONV np_start(uint16_t client_id,
      retro_netpacket_send_t send, retro_netpacket_poll_receive_t poll)
{
   /* a session that was never stopped: undo it first */
   if (s.state != ST_OFF)
      session_end(NULL, 0);
   free(s.mcd_in);
   memset(&s, 0, sizeof(s));
   send_fn = send;
   poll_fn = poll;
   s.my_id = client_id;
   s.is_host = client_id == 0;
   s.peer_id = s.is_host ? -1 : 0;
   s.t_heard = now_us();
   if (s.is_host)
      set_state(ST_HOST_WAIT);
   else {
      set_state(ST_CLIENT_HELLO);
      send_hello();
   }
}

static void RETRO_CALLCONV np_stop(void)
{
   session_end(s.state == ST_RUN ? "Netplay: session ended, playing alone" : NULL, 0);
   free(s.mcd_in);
   memset(&s, 0, sizeof(s));
   send_fn = NULL;
   poll_fn = NULL;
   set_state(ST_OFF);
}

static bool RETRO_CALLCONV np_connected(uint16_t client_id)
{
   if (!s.is_host)
      return false;
   if (s.peer_id >= 0 && s.state != ST_HOST_WAIT && s.state != ST_FAILED)
      return false; /* 2 players only */
   s.peer_id = client_id;
   s.t_heard = now_us();
   set_state(ST_HOST_WAIT);
   return true;
}

static void RETRO_CALLCONV np_disconnected(uint16_t client_id)
{
   if (!s.is_host || (int)client_id != s.peer_id)
      return;
   session_end(s.state == ST_RUN ? "Netplay: player 2 left, playing alone" : NULL, 0);
   s.peer_id = -1;
   set_state(ST_HOST_WAIT);
}

static const struct retro_netpacket_callback np_iface = {
   np_start,
   np_receive,
   np_stop,
   NULL,
   np_connected,
   np_disconnected,
   LSK_PROTOCOL,
};

void lsk_init(retro_environment_t environ_cb)
{
   env_cb = environ_cb;
   memset(&s, 0, sizeof(s));
   s.peer_id = -1;
   if (env_cb)
      env_cb(RETRO_ENVIRONMENT_SET_NETPACKET_INTERFACE, (void *)&np_iface);
}

void lsk_deinit(void)
{
   if (s.state != ST_OFF)
      session_end(NULL, 0);
   free(s.mcd_in);
   memset(&s, 0, sizeof(s));
}

/* ---- per frame ---- */

bool lsk_running(void)
{
   return s.state == ST_RUN;
}

bool lsk_busy(void)
{
   return s.state == ST_RUN || s.state == ST_HOST_SEND || s.state == ST_HOST_READY
      || s.state == ST_HOST_GO || s.state == ST_CLIENT_GO;
}

int lsk_local_port(void)
{
   return s.state == ST_RUN ? (s.is_host ? 0 : 1) : -1;
}

void lsk_request_reset(void)
{
   s.pending_flags |= LSK_KEY_RESET;
}

void lsk_request_analog_toggle(void)
{
   s.pending_flags |= LSK_KEY_ANALOG_TOGGLE;
}

void lsk_check_config(void)
{
   if (s.state != ST_RUN || lsk_core_cfg_hash() == session_cfg)
      return;
   /* the other console's machine wouldn't follow */
   send_pk(PK_BYE, NULL, 0, NULL, 0);
   session_end("Netplay: settings changed while linked, unlinked", 1);
}

static void setup_tick(void)
{
   uint64_t t = now_us();
   switch (s.state) {
   case ST_HOST_PING:
      if (t - s.ping_sent > PONG_US)
         send_ping();
      if (t - s.t_state > PING_US + PONG_US) {
         s.delay = compute_delay();
         set_state(ST_HOST_SEND);
      }
      break;
   case ST_CLIENT_HELLO:
      if (!s.host_answered && t - s.hello_sent > HELLO_EVERY_US)
         send_hello();
      /* fallthrough */
   case ST_HOST_READY:
      if (t - s.t_state > SETUP_US) {
         send_pk(PK_BYE, NULL, 0, NULL, 0);
         session_end("Netplay: the other console didn't answer, not linked", 1);
      }
      break;
   default:
      break;
   }
}

static void apply_pads(uint32_t i)
{
   int port;
   for (port = 0; port < 2; port++) {
      const struct lsk_pad *pd = &s.pad[i][port];
      in_keystate[port] = pd->keys & 0xffff;
      in_analog_left[port][0]  = pd->axes[0];
      in_analog_left[port][1]  = pd->axes[1];
      in_analog_right[port][0] = pd->axes[2];
      in_analog_right[port][1] = pd->axes[3];
   }
   for (port = 2; port < 8; port++)
      in_keystate[port] = 0;
}

int lsk_frame_begin(const struct lsk_pad *local)
{
   int me, other;
   uint32_t i, f, flags;
   uint64_t t0;

   if (s.state == ST_OFF || s.state == ST_FAILED || s.state == ST_HOST_WAIT)
      return LSK_RUN;

   setup_tick();
   switch (s.state) {
   case ST_HOST_SEND:
      host_send_cards_and_start();
      return LSK_SKIP;
   case ST_HOST_READY:
      return LSK_SKIP;      /* hold the game: the cards were sent */
   case ST_HOST_GO:
      run_begin();
      break;
   case ST_CLIENT_GO:
      install_memcards();
      send_pk(PK_READY, NULL, 0, NULL, 0);
      run_begin();
      break;
   case ST_RUN:
      break;
   default:
      return LSK_RUN;       /* setting up: our game goes on meanwhile */
   }
   if (s.state != ST_RUN)
      return LSK_RUN;

   me = s.is_host ? 0 : 1;
   other = me ^ 1;
   f = s.frame;

   /* a calm while: our pads can go a frame later again */
   if (s.ahead && now_us() - s.t_ahead > AHEAD_CALM_US) {
      s.ahead--;
      s.t_ahead = now_us();
   }

   /* our pad, for frame f + delay (+ more if the other console had to
    * wait for it: then it goes for a few frames at once) */
   while (s.sent <= f + s.delay + s.ahead) {
      uint32_t w[5], sf = s.sent;
      i = sf % RING;
      s.pad[i][me] = *local;
      s.pad[i][me].keys = (local->keys & 0xffff) | s.pending_flags;
      s.pending_flags = 0;
      s.tag[i][me] = sf + 1;
      w[0] = sf;
      w[1] = s.pad[i][me].keys;
      w[2] = (uint16_t)s.pad[i][me].axes[0] | ((uint32_t)(uint16_t)s.pad[i][me].axes[1] << 16);
      w[3] = (uint16_t)s.pad[i][me].axes[2] | ((uint32_t)(uint16_t)s.pad[i][me].axes[3] << 16);
      w[4] = s.stalls;
      send_pk(PK_INPUT, w, 5, NULL, 0);
      s.sent = sf + 1;
   }

   i = f % RING;
   if (f < s.delay) {
      /* before anybody could press anything: sticks centered */
      int k, port;
      memset(&s.pad[i], 0, sizeof(s.pad[i]));
      for (port = 0; port < 2; port++)
         for (k = 0; k < 4; k++)
            s.pad[i][port].axes[k] = 127;
   }
   else if (s.tag[i][other] != f + 1) {
      /* wait a little for the other console's pad */
      t0 = now_us();
      while (s.state == ST_RUN && s.tag[i][other] != f + 1) {
         if (poll_fn)
            poll_fn();
         if (s.state != ST_RUN)
            return LSK_RUN;    /* the session ended meanwhile */
         if (s.tag[i][other] == f + 1)
            break;
         if (now_us() - t0 > STALL_US) {
            if (now_us() - s.t_heard > PEER_US) {
               send_pk(PK_BYE, NULL, 0, NULL, 0);
               session_end("Netplay: the other console stopped answering, playing alone", 1);
               return LSK_RUN;
            }
            s.stalls++;
            return LSK_SKIP;
         }
         usleep(500);
      }
   }

   apply_pads(i);
   flags = s.pad[i][0].keys | s.pad[i][1].keys;
   if (flags & LSK_KEY_RESET)
      lsk_core_reset();
   if (s.pad[i][0].keys & LSK_KEY_ANALOG_TOGGLE)
      lsk_core_analog_toggle(0);
   if (s.pad[i][1].keys & LSK_KEY_ANALOG_TOGGLE)
      lsk_core_analog_toggle(1);
   return LSK_RUN;
}

static uint32_t ram_hash(void)
{
   const uint32_t *p = (const uint32_t *)psxRegs.ptrs.psxM;
   uint32_t h = 0x811c9dc5u, k;
   for (k = 0; k < 0x200000 / 4; k += 4) {
      h = (h ^ p[k]) * 16777619u;
      h = (h ^ p[k + 1]) * 16777619u;
      h = (h ^ p[k + 2]) * 16777619u;
      h = (h ^ p[k + 3]) * 16777619u;
   }
   h = (h ^ psxRegs.cycle) * 16777619u;
   h = (h ^ psxRegs.pc) * 16777619u;
   return h;
}

void lsk_frame_end(void)
{
   if (s.state != ST_RUN)
      return;
   s.frame++;
   if (s.frame % HASH_EVERY == 0) {
      uint32_t w[2];
      w[0] = s.frame;
      w[1] = ram_hash();
      send_pk(PK_HASH, w, 2, NULL, 0);
      hash_store(0, w[0], w[1]);
   }
}
