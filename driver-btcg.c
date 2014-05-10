/*
 * cgminer driver for BitCoin Garden.
 *
 * Copyright 2014 Yichao Ma, Hongzhi Song
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "btcg-config.h"
#include "btcg-hw-ctrl.h"
#include "btcg-vector.h"

///////////////////////////////////////////////////////////////////////////


/********** work queue */
struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};

static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);

	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL){
		applog(LOG_ERR, "queue nont init");
		return NULL;
	}
	if (wq->num_elems == 0){
		applog(LOG_ERR, "queue length is 0");
		return NULL;
	}
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}


/********** chip and chain context structures */
struct BTCG_chip {
    unsigned int id;
    bool enabled;

    /*******/
    /* FSM */
    /*******/
#define CHIP_STATE_RUN                  0
#define CHIP_STATE_GOING_TO_HIBERNATE   1
#define CHIP_STATE_HIBERNATE            2
    int state;
    /* After this time, the chip goes back to RUN state */
    struct timeval hibernate_deadline;

    
    /********************************/
    /* data relates to current work */
    /********************************/
	struct work *work;
    /* After this time, if no nonce calculated by the chip,
     * the work is considered time out
     */
    struct timeval this_work_deadline;
    /* Actual nonces that have been submitted by this work */
    struct BTCG_vec *this_work_submitted_actual_nonces;
    /* A mark that show w_allow has been pulled to low for this chip */
    bool this_work_w_allow_has_been_low;

    /*********************/
	/* global statistics */
    /*********************/
    unsigned int total_works;
    unsigned int have_nonce_works;
    unsigned int no_nonce_works;
	unsigned int total_nonces;
    unsigned int local_rejected_nonces;

    /* consecutive errors */
    unsigned int consec_errs;
    unsigned int max_consec_errs;

	uint32_t hw_errs;
    float ave_hw_errs;

    unsigned long long total_hibernate_ms;
};

/**********************************************/
/* MACROS that operate on BTCG_chip structure */
/**********************************************/
#define __CHIP_INC_AVE(val)     do {    \
    val = 0.5 + (val) / 2.0;            \
} while(0)

#define __CHIP_DEC_AVE(val)     do {    \
    val = 0.0 + (val) / 2.0;            \
} while(0)

/* Operations on err */
static void CHIP_WORK_DONE_WITH_ERR( struct BTCG_chip *chip) {
    assert( chip->work != NULL);
    if ( vec_size(chip->this_work_submitted_actual_nonces) > 0) {
        chip->have_nonce_works += 1;
    }
    else {
        chip->no_nonce_works += 1;
    }

    chip->consec_errs += 1;
    if (chip->max_consec_errs < chip->consec_errs) {
        chip->max_consec_errs = chip->consec_errs;
    }
    chip->hw_errs += 1;
    __CHIP_INC_AVE(chip->ave_hw_errs);
}

static void CHIP_WORK_DONE_WITHOUT_ERR( struct BTCG_chip *chip) {
    assert( chip->work != NULL);
    if ( vec_size(chip->this_work_submitted_actual_nonces) > 0) {
        chip->have_nonce_works += 1;
    }
    else {
        chip->no_nonce_works += 1;
    }
    chip->consec_errs = 0;
    __CHIP_DEC_AVE(chip->ave_hw_errs);
}

#define CHIP_WORK_DONE_WITH_NONCE(chip) do {    \
    assert( chip->work != NULL);                \
    chip->have_nonce_works += 1;                \
    chip->consec_errs = 0;                      \
    __CHIP_DEC_AVE(chip->ave_hw_errs);          \
} while(0)

#define CHIP_WORK_DONE_WITHOUT_NONCE(chip) do { \
    assert( chip->work != NULL);                \
    chip->no_nonce_works += 1;                  \
    chip->consec_errs = 0;                      \
    __CHIP_DEC_AVE(chip->ave_hw_errs);          \
} while(0)

/* Operations on work */
/* Get a future time which is 'ms' milliseconds after current time.
 * The future time is stored to tv.
 */
static void __future_time(unsigned ms, struct timeval *tv) {
    struct timeval incremental;
    incremental.tv_sec = ms / 1000;
    incremental.tv_usec = (ms % 1000) * 1000;

    struct timeval curtime;
    cgtime( &curtime);

    timeradd( &curtime, &incremental, tv);
}

static inline void CHIP_SET_HIBERNATE_DEADLINE(struct BTCG_chip *chip) {
    chip->consec_errs = 0;
    __future_time( btcg_config()->hibernate_time_ms, &chip->hibernate_deadline);
    chip->total_hibernate_ms += btcg_config()->hibernate_time_ms;
}

static bool CHIP_IS_TIME_TO_WAKE_UP(struct BTCG_chip *chip) {
    struct timeval curtime;
    cgtime( &curtime);
    return timercmp( &chip->hibernate_deadline, &curtime, <);
}

/* Check if a nonce has been submitted.
 */
static bool CHIP_IS_DUPLICATED_NONCE( struct BTCG_chip *chip, uint32_t nonce) {
    assert( chip->this_work_submitted_actual_nonces != NULL);
    return vec_find_fst( chip->this_work_submitted_actual_nonces, &nonce) != VEC_NPOS;
}

static void CHIP_APPEND_SUBMITTED_NONCE( struct BTCG_chip *chip, uint32_t nonce) {
    assert( chip->this_work_submitted_actual_nonces != NULL);
    vec_push_back( chip->this_work_submitted_actual_nonces, &nonce);
    chip->total_nonces += 1;
}

static void CHIP_NEW_WORK(struct cgpu_info *cgpu, struct BTCG_chip *chip, struct work *newwork) {
    if (chip->work) {
        work_completed( cgpu, chip->work);
    }
    chip->work = newwork;
    if (newwork) {
        __future_time( btcg_config()->work_timeout_ms, &chip->this_work_deadline);
        chip->total_works += 1;
    }
    else {
        timerclear( &chip->this_work_deadline);
    }
    assert( chip->this_work_submitted_actual_nonces != NULL);
    vec_clear( chip->this_work_submitted_actual_nonces);
    chip->this_work_w_allow_has_been_low = false;
}

static inline bool CHIP_IS_WORK_TIMEOUT( const struct BTCG_chip *chip) {
    assert( chip->work != NULL);
    struct timeval curtime;
    cgtime( &curtime);
    return timercmp( &chip->this_work_deadline, &curtime, <);
}

/* Show various info of a chip to LOG_ERR */
static void CHIP_SHOW( const struct BTCG_chip *chip, bool show_work_info) {
    // Show nothing for disabled chips
    if ( !chip->enabled) {
        return;
    }
    applog(LOG_WARNING, " ");
    applog(LOG_WARNING, "********** chip %u **********", chip->id);
    if (show_work_info) {
        applog(LOG_WARNING, "work: %p", chip->work);
        applog(LOG_WARNING, "this work nonces: %zu", vec_size( chip->this_work_submitted_actual_nonces));
        applog(LOG_WARNING, "this work w_allow has been low: %u", chip->this_work_w_allow_has_been_low);
    }
    applog(LOG_WARNING, "total works: %u", chip->total_works);
    applog(LOG_WARNING, "have nonce works: %u (%f%%)", chip->have_nonce_works, (float)(chip->have_nonce_works * 100.0 / chip->total_works));
    applog(LOG_WARNING, "no noce works: %u (%f%%)", chip->no_nonce_works, (float)(chip->no_nonce_works * 100.0 / chip->total_works));
    applog(LOG_WARNING, "total nonces: %u", chip->total_nonces);
    applog(LOG_WARNING, "local rejected nonces: %u", chip->local_rejected_nonces);
    applog(LOG_WARNING, "consecutive errors: %u", chip->consec_errs);
    applog(LOG_WARNING, "max consecutive errors: %u", chip->max_consec_errs);
    applog(LOG_WARNING, "hardware errors: %u", chip->hw_errs);
    applog(LOG_WARNING, "average hardware errors: %f", chip->ave_hw_errs);
    applog(LOG_WARNING, "total hibernate time: %llus", chip->total_hibernate_ms / 1000);
}


/*
 * id: chip id
 */
static bool init_a_chip( struct BTCG_chip *chip, struct spi_ctx *ctx, unsigned int id) {
    if ( !chip_reset( ctx, btcg_config()->core_clk_mhz)) {
        applog(LOG_ERR, "Chip %u: failed to reset", id);
        return false;
    }
    memset(chip, 0, sizeof(*chip));
    chip->id = id;
    chip->enabled = btcg_config()->enabled_chips[id];
    chip->state = CHIP_STATE_RUN;
    chip->this_work_submitted_actual_nonces = vec_open( sizeof(uint32_t));
    return true;
}

static void release_a_chip( struct BTCG_chip *chip) {
    vec_close( chip->this_work_submitted_actual_nonces);
    chip->this_work_submitted_actual_nonces = NULL;
}


struct BTCG_board {
	struct cgpu_info *cgpu;
	int num_chips;
	struct spi_ctx *spi_ctx;
	struct BTCG_chip *chips;
    unsigned int chip_to_be_scanned;
	pthread_mutex_t lock;

	struct work_queue active_wq;
};

/* Select a chip to scan */
static inline unsigned int next_chip_id( struct BTCG_board *bd) {
    assert( bd->num_chips > 0);
    assert( bd->chip_to_be_scanned <= bd->num_chips);
    if ( bd->chip_to_be_scanned == bd->num_chips) {
        bd->chip_to_be_scanned = 0;
    }
    return bd->chip_to_be_scanned++;
}

/* Operations on queue */
static inline int __board_queue_max_buf_size( const struct BTCG_board *bd) {
    assert( bd->num_chips > 0);
    if ( bd->num_chips == 1) {
        return 3;
    }
    else {
        return bd->num_chips * 2;
    }
}

static inline bool board_queue_full(const struct BTCG_board *bd) {
    return bd->active_wq.num_elems >= __board_queue_max_buf_size(bd);
}

static inline bool board_queue_need_more_work(const struct BTCG_board *bd) {
    assert( bd->num_chips > 0);
    return bd->active_wq.num_elems <= 3 * __board_queue_max_buf_size(bd) / 5;
}


/********** temporary helper for hexdumping SPI traffic */
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(LOG_DEBUG, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(LOG_DEBUG, "%s", line);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}


/********** disable / re-enable related section (temporary for testing) */
static int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
}

/********** job creation and result evaluation */
uint32_t get_diff(double diff)
{
	uint32_t n_bits;
	int shift = 29;
	double f = (double) 0x0000ffff / diff;
	while (f < (double) 0x00008000) {
		shift--;
		f *= 256.0;
	}
	while (f >= (double) 0x00800000) {
		shift++;
		f /= 256.0;
	}
	n_bits = (int) f + (shift << 24);
	return n_bits;
}


/********** driver interface */

static struct BTCG_board *init_BTCG_board( struct cgpu_info *cgpu, struct spi_ctx *ctx)
{
	struct BTCG_board *bd = malloc(sizeof(*bd));
	assert(bd != NULL);

	applog(LOG_DEBUG, "BTCG init board");
	memset(bd, 0, sizeof(*bd));
    bd->cgpu = cgpu;
	bd->num_chips = btcg_config()->num_chips;
	if (bd->num_chips == 0)
		goto failure;
	bd->spi_ctx = ctx;

	applog(LOG_WARNING, "spidev%d.%d: Found %d BTCG chips",
	       bd->spi_ctx->config.bus, bd->spi_ctx->config.cs_line,
	       bd->num_chips);

	bd->chips = calloc(bd->num_chips, sizeof(struct BTCG_chip));
	assert (bd->chips != NULL);

    bd->chip_to_be_scanned = 0;

	applog(LOG_WARNING, "found %d chips", bd->num_chips);

	mutex_init(&bd->lock);
	INIT_LIST_HEAD(&bd->active_wq.head);

    size_t i;
    for ( i = 0; i < bd->num_chips; ++i) {
        if (!init_a_chip( bd->chips + i, ctx, i)) {
            goto failure;
        }
    }

    // Show enabled/disabled chips
    {
        char buf[1024];
        size_t off = 0;
        off += snprintf( buf, sizeof( buf), "Enabled chips: ");
        bool id_printed = false;
        for ( i = 0; i < bd->num_chips; ++i) {
            assert( off < sizeof(buf));
            if ( bd->chips[ i].enabled) {
                if ( id_printed) {
                    assert( off < sizeof(buf));
                    off += snprintf( buf + off, sizeof( buf) - off, ", ");
                }
                assert( off < sizeof(buf));
                off += snprintf( buf + off, sizeof( buf) - off, "%zu", i);
                id_printed = true;
            }
        }
        applog(LOG_WARNING, "%s", buf);
    }

	return bd;

failure:
    if (bd) {
        if (bd->chips) {
            free(bd->chips);
        }
        free(bd);
    }
    return NULL;
}

static bool submit_a_nonce(struct thr_info *thr, struct BTCG_chip *chip,
        const uint32_t nonce, uint32_t *actual_nonce) {
    assert( actual_nonce);
    const uint32_t nonce_candies[] = {
       nonce + 1, nonce + 2, nonce + 3, nonce + 4,
       nonce - 4, nonce - 3, nonce - 2, nonce - 1, nonce};

    struct work *work = chip->work;

    bool has_nonce = false;
    bool has_dup_nonce = false;
    size_t i;
    for ( i = 0; i < sizeof( nonce_candies) / sizeof( nonce_candies[0]); ++i) {
        const uint32_t a_nonce = nonce_candies[i];
        if (!test_nonce( work, a_nonce)) {
            continue;
        }
        if (CHIP_IS_DUPLICATED_NONCE( chip, a_nonce)) {
            applog( LOG_ERR, "Chip %u: duplicated nonce %u, actual duplicated nonce %u", chip->id, nonce, a_nonce);
            has_dup_nonce = true;
            continue;
        }
        if (submit_nonce( thr, work, a_nonce)) {
            *actual_nonce = a_nonce;
            applog(LOG_DEBUG, "Chip %u: submit nonce ok, nonce %u, actual nonce %u",
                    chip->id, nonce, a_nonce);
            CHIP_APPEND_SUBMITTED_NONCE( chip, a_nonce);
            has_nonce = true;
            break;
        }
    }

    /* has_nonce has_dup_nonce return
     *     Y          N         true 
     *     Y          Y         false
     *     N          x         false
     */
    if ( has_nonce && !has_dup_nonce) {
        return true;
    }
    if ( !has_nonce) {
        chip->local_rejected_nonces += 1;
    }
    inc_hw_errors( thr);
    return false;
}


/*
 * Read valid nonces from a chip.
 */
static bool submit_ready_nonces( struct thr_info *thr, struct BTCG_chip *chip, const uint8_t status) {
#if 0
    usleep(10*1000);
#endif
    assert( STATUS_R_READY( status));
    unsigned int grps[4];
    size_t num_grps = 0;

    if ( STATUS_NONCE_GRP0_RDY( status)) {
        grps[ num_grps++] = 0;
    }
    if ( STATUS_NONCE_GRP1_RDY( status)) {
        grps[ num_grps++] = 1;
    }
    if ( STATUS_NONCE_GRP2_RDY( status)) {
        grps[ num_grps++] = 2;
    }
    if ( STATUS_NONCE_GRP3_RDY( status)) {
        grps[ num_grps++] = 3;
    }

    if ( num_grps == 0) {
        applog(LOG_ERR, "Chip %u: R_READY is high, but no group ready", chip->id);
        return false;
    }

    // Get ready groups
    struct BTCG_board *bd = thr->cgpu->device_data;
    struct spi_ctx *ctx = bd->spi_ctx;

    bool all_submit_succ = true;
    size_t i;
    for ( i = 0; i < num_grps; ++i) {
        uint32_t nonce;
        if (!chip_read_nonce( ctx, grps[i], &nonce)) {
            applog(LOG_ERR, "Chip %u: failed to get nonce", chip->id);
            return false;
        }

        // submit nonce
        uint32_t actual_nonce;
        if (!submit_a_nonce( thr, chip, nonce, &actual_nonce)) {
            applog(LOG_ERR, "Chip %u: error encountered while submitting nonce %u from nonce group %d",
                     chip->id, nonce, grps[i]);
            all_submit_succ = false;
        }
    }
    return all_submit_succ;
}

static void may_submit_may_get_work(struct thr_info *thr, unsigned int id) {
    struct cgpu_info *cgpu = thr->cgpu;
    struct BTCG_board *bd = cgpu->device_data;

    assert( id < bd->num_chips);

    struct spi_ctx *ctx = bd->spi_ctx;
    struct BTCG_chip *chip = bd->chips + id;
    assert( chip);
    // Do nothing for disabled chips
    if ( !chip->enabled) {
        assert( chip->work == NULL && chip->total_works == 0);
        return;
    }

    if ( !chip_select(id)) {
        applog(LOG_ERR, "Chip %u: failed to select chip", id);
        return;
    }


    switch( chip->state) {
        case CHIP_STATE_RUN:

#define __RESET_AND_GIVE_BACK_WORK()  do {          \
    (void)chip_reset( ctx, btcg_config()->core_clk_mhz);  \
    applog(LOG_DEBUG, "Chip %u: reset", chip->id);  \
    CHIP_NEW_WORK( cgpu, chip, NULL);               \
} while(0)

#define FIX_CHIP_ERR_MAY_GOING_HIBERNATE_AND_RETURN do {            \
    CHIP_WORK_DONE_WITH_ERR(chip);                                  \
    if (chip->consec_errs > btcg_config()->consecutive_err_threshold) {   \
        chip->state = CHIP_STATE_GOING_TO_HIBERNATE;                \
        applog(LOG_ERR,                                             \
                "Chip %u: %u consecutive errors, "                  \
                "which is larger than threshold %u. "               \
                "Going to hibernate.",                              \
                chip->id,                                           \
                chip->consec_errs,                                  \
                btcg_config()->consecutive_err_threshold);                \
    }                                                               \
    __RESET_AND_GIVE_BACK_WORK();                                   \
    return;                                                         \
} while(0)

            if ( chip->work) {
                uint8_t status;
                if ( !chip_status( ctx, &status)) {
                    applog(LOG_ERR, "Chip %u: failed to get status", id);
                    FIX_CHIP_ERR_MAY_GOING_HIBERNATE_AND_RETURN;
                }

                /* The chip status check order is important.
                 * Do NOT change the order without strong reason.
                 */
                if (STATUS_R_READY( status)) {
                    // READ nonces
                    const bool submit_succ =  submit_ready_nonces( thr, chip, status);

                    /* DO always clean chip status */
                    if ( !chip_clean( ctx)) {
                        applog(LOG_ERR, "Chip %u: failed to clean status", id);
                        FIX_CHIP_ERR_MAY_GOING_HIBERNATE_AND_RETURN;
                    }
                    if ( !submit_succ) {
                        applog(LOG_ERR, "Chip %u: failed to submit nonce", id);
                        FIX_CHIP_ERR_MAY_GOING_HIBERNATE_AND_RETURN;
                    }
                }

                if (STATUS_W_ALLOW(status)) {
                    assert( chip->work);
                    if (vec_size( chip->this_work_submitted_actual_nonces) == 0) {
                        // Without nonce
                        if ( chip->this_work_w_allow_has_been_low) {
                            CHIP_WORK_DONE_WITHOUT_ERR( chip);
                            CHIP_NEW_WORK( cgpu, chip, NULL);
                        }
                        else {
                            applog(LOG_ERR, "Chip %u: No w_allow low level detected, and no nonce calculated for work %p",
                                    id, chip->work);
                            FIX_CHIP_ERR_MAY_GOING_HIBERNATE_AND_RETURN;
                        }
                    }
                    else {
                        // With nonce
                        CHIP_WORK_DONE_WITHOUT_ERR( chip);
                        CHIP_NEW_WORK( cgpu, chip, NULL);
                    }
                }
                else {
                    chip->this_work_w_allow_has_been_low = true;

                    if ( CHIP_IS_WORK_TIMEOUT( chip)) {
                        // check w_allow timeout
                        applog(LOG_ERR, "Chip %u: work time out", id);
                        FIX_CHIP_ERR_MAY_GOING_HIBERNATE_AND_RETURN;
                    }
                    else {
                        assert( STATUS_R_READY( status) || STATUS_BUSY( status));
                        return;
                    }
                }
            }

            if ( chip->work == NULL) {
                struct work *new_work = wq_dequeue(&bd->active_wq);
                if (new_work == NULL) {
                    applog(LOG_ERR, "queue under flow");
                    return;
                }
                CHIP_NEW_WORK( cgpu, chip, new_work);
                applog(LOG_DEBUG, "Chip %u: new work %p", id, chip->work);
                if (!chip_write_job( ctx, chip->work->midstate, chip->work->data + 64)) {
                    // give back job
                    applog( LOG_ERR, "Chip %u: failed to write job", id);
                    FIX_CHIP_ERR_MAY_GOING_HIBERNATE_AND_RETURN;
                }
            }
            break;

        case CHIP_STATE_GOING_TO_HIBERNATE:
            assert( chip->work == NULL);
            CHIP_SET_HIBERNATE_DEADLINE( chip);
            chip->state = CHIP_STATE_HIBERNATE;
            break;

        default:
            assert( chip->state == CHIP_STATE_HIBERNATE);
            assert( chip->work == NULL);
            if ( CHIP_IS_TIME_TO_WAKE_UP( chip)) {
                applog(LOG_WARNING, "Chip %u: wake up", chip->id);
                chip->state = CHIP_STATE_RUN;
            }
            break;
    }   // End of switch( chip->state)

    return;
}


/* Probe SPI channel and register chip board */
void BTCG_detect(bool hotplug)
{
	/* no hotplug support for now */
	if (hotplug)
		return;

    if (!btcg_parse_opt()) {
        applog(LOG_ERR, "Failed to parse BitCoin Garden options");
        return;
    }
    applog(LOG_WARNING, "Core clock: %uMHz", btcg_config()->core_clk_mhz);
    applog(LOG_WARNING, "SPI clock: %uKHz", btcg_config()->spi_clk_khz);
 
    if (!chip_selector_init()) {
        applog(LOG_ERR, "Failed to initialize chip selector");
        return;
    }
	
    /* SPI configuration */
    struct spi_config cfg = default_spi_config;
    cfg.mode = SPI_MODE_0;
    cfg.speed = btcg_config()->spi_clk_khz * 1000;
    cfg.delay = 30;         // TODO: may use default value

    struct spi_ctx *ctx = spi_init(&cfg);
    if (ctx == NULL) {
        applog(LOG_ERR, "Failed to initialize SPI");
        return;
    }
	
    struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
    assert(cgpu != NULL);

    struct BTCG_board *bd = init_BTCG_board(cgpu, ctx);
    if (bd == NULL)
        return;

    memset(cgpu, 0, sizeof(*cgpu));
    cgpu->drv = &bitmineA1_drv;
    cgpu->name = "BitmineA1";
    cgpu->threads = 1;
    cgpu->device_data = bd;

    // Finally, add the cgpu
    add_cgpu(cgpu);
}




static int64_t BTCG_scanwork(struct thr_info *thr)
{
    struct cgpu_info *cgpu = thr->cgpu;
    struct BTCG_board *bd = cgpu->device_data;

    applog(LOG_DEBUG, "BTCG running scanwork");
    mutex_lock(&bd->lock);

    struct work *work;
    do {
        if (cgpu->shutdown) {
            break;
        }
        const unsigned int id = next_chip_id(bd);
        may_submit_may_get_work(thr, id);
    } while(!board_queue_need_more_work(bd));
    
    mutex_unlock(&bd->lock);
    
    // TODO: SHOULD RETURN (int64_t)(number of hashes done)
    return 0;
}


/* queue two work items per chip in board */
static bool BTCG_queue_full(struct cgpu_info *cgpu)
{
	struct BTCG_board *bd = cgpu->device_data;
	int queue_full = false;
	struct work *work;

	mutex_lock(&bd->lock);
	applog(LOG_DEBUG, "BTCG running queue_full: %d/%d",
            bd->active_wq.num_elems, bd->num_chips);

	if (board_queue_full(bd))
		queue_full = true;
	else{
		//push queue
		applog(LOG_DEBUG," queue elem add and num is %d",bd->active_wq.num_elems);
		wq_enqueue(&bd->active_wq, get_queued(cgpu));
	}
	mutex_unlock(&bd->lock);

	return queue_full;
}

static void BTCG_flush_work(struct cgpu_info *cgpu)
{
	struct BTCG_board *bd = cgpu->device_data;

	applog(LOG_DEBUG, "BTCG running flushwork");

	int i;

	mutex_lock(&bd->lock);
	/* Reset all chips first */
    for( i = 0; i < bd->num_chips; ++i) {
        if ( !chip_select( i) || !chip_reset( bd->spi_ctx, btcg_config()->core_clk_mhz)) {
            applog(LOG_ERR, "Chip %d: failed to abort work", i);
        }
    }
	/* flush the work chips were currently hashing */
	for (i = 0; i < bd->num_chips; i++) {
		struct BTCG_chip *chip = &bd->chips[i];
        applog(LOG_DEBUG, "flushing chip %d, work: 0x%p", i, chip->work);
        CHIP_NEW_WORK( cgpu, chip, NULL);
    }
	/* flush queued work */
	applog(LOG_DEBUG, "flushing queued work...");
	while (bd->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&bd->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	mutex_unlock(&bd->lock);
}

static void BTCG_get_statline_before(char *buf, size_t len, struct cgpu_info *cgpu)
{
	struct BTCG_board *bd = cgpu->device_data;
	tailsprintf(buf, len, "%2d ", bd->num_chips);
}

static void BTCG_thread_shutdown(struct thr_info *thr) {
    struct BTCG_board *bd = thr->cgpu->device_data;
    unsigned i;
	mutex_lock(&bd->lock);
    for ( i = 0; i < bd->num_chips; ++i) {
        CHIP_SHOW( bd->chips + i, false);
    }

    /* release all chips */
    for ( i = 0; i < bd->num_chips; ++i) {
        struct BTCG_chip *chip = bd->chips + i;
        release_a_chip( chip);
    }
	mutex_unlock(&bd->lock);
}

struct device_drv bitmineA1_drv = {
	.drv_id = DRIVER_btcg,
	.dname = "BitCoinGarden",
	.name = "BTCG",
	.drv_detect = BTCG_detect,

	.hash_work = hash_queued_work,
	.scanwork = BTCG_scanwork,
	.queue_full = BTCG_queue_full,
	.flush_work = BTCG_flush_work,
	.get_statline_before = BTCG_get_statline_before,

    .thread_shutdown = BTCG_thread_shutdown,
};
