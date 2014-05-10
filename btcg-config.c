#include <stdio.h>
#include <assert.h>

#include "miner.h"
#include "logging.h"

#include "btcg-config.h"
#include "btcg-vector.h"


char *opt_btcg_clks = NULL;
char *opt_btcg_only_enable_chips = NULL;

static struct BTCG_config g_config = {
    .num_chips = 32,

    .spi_clk_khz = 200,

    .core_clk_mhz = 200,

    // The min freq of the chip is 200MHz.
    // With 32 cores each chip, the min hash rate is 6.4G/s.
    // The full search space is 4G, so the max time is about
    // 4G/(6.4G/s) = (4/6.4)s, which is less than 1s.
    // Now, set the time out to 2s, the safe margin is large
    // enough, and no too much failure messages.
    .work_timeout_ms = 2 * 1000,

    .consecutive_err_threshold = 15,
    .hibernate_time_ms = 30 * 1000,

    /* This field initialized in btcg_parse_opt() */
    .enabled_chips = NULL,
};

static struct BTCG_config *p_config = NULL;

static bool parse_clks_opt() {
    if ( opt_btcg_clks == NULL) {
        return true;
    }

    int core_clk;
    int spi_clk;

    if( sscanf(opt_btcg_clks, "%d:%d", &core_clk, &spi_clk) < 2) {
        applog( LOG_ERR, "Failed to parse clock option");
        return false;
    }
    if ( core_clk <= 0 || spi_clk <= 0) {
        applog( LOG_ERR, "A clock must be a positive integer");
        return false;
    }
    g_config.core_clk_mhz = core_clk;
    g_config.spi_clk_khz = spi_clk;

    return true;
}

/*
 * Parse a list of numbers.
 * The list should be in the form:
 * num0,num1,num2,...,numN
 * Return NULL on failure.
 * Return a pointer to a long array, and store number of elements in to n.
 */
static const long* parse_num_list(const char *opt, size_t *n) {
    static struct BTCG_vec *nums = NULL;
    if ( nums == NULL) {
        nums = vec_open( sizeof( long));
    }

    const char *nptr = opt;
    char *endptr = (char*)nptr;

    size_t i;
    for (i = 0; *endptr != '\0'; ++i) {
        /* To distinguish failure after call */
        errno = 0;
        long ret = strtol( nptr, &endptr, 10);

        if (errno != 0) {
            if ( errno == ERANGE) {
                if ( ret == LONG_MAX) {
                    applog(LOG_ERR, "Number over flow");
                }
                else {
                    assert (ret == LONG_MIN);
                    applog(LOG_ERR, "Number under flow");
                }
            }
            else {
                applog( LOG_ERR, "Error occurred while parsing a number");
            }
            return NULL;
        }
        else if (ret == 0 && nptr == endptr) {
            applog( LOG_ERR, "Non-digit character encountered");
            return NULL;
        }
        vec_push_back( nums, &ret);
        nptr = endptr + 1;
    }
    *n = vec_size( nums);
    return vec_size( nums) > 0 ? ((long*)vec_at(nums, 0)) : NULL;
}

static bool parse_only_enable_chips_opt() {
    assert( g_config.enabled_chips);

    if (opt_btcg_only_enable_chips == NULL) {
        // Enable all chips
        size_t i;
        for ( i = 0; i < g_config.num_chips; ++i) {
            g_config.enabled_chips[i] = true;
        }
        return true;
    }

    size_t n;
    const long *ids = parse_num_list( opt_btcg_only_enable_chips, &n);
    if ( ids == 0) {
        applog(LOG_ERR, "Failed to parse enable chips list");
        return false;
    }

    size_t i;
    /* Disable all chips first */
    for ( i = 0; i < g_config.num_chips; ++i) {
        g_config.enabled_chips[i] = false;
    }
    /* Then enable selected chips */
    for ( i = 0; i < n; ++i) {
        const long id = ids[i];
        if ( id < 0 || id >= g_config.num_chips) {
            applog( LOG_ERR, "Invalid chip id: %ld", id);
            return false;
        }
        if ( g_config.enabled_chips[ id]) {
            applog( LOG_WARNING, "Chip id %ld has been enabled", id);
        }
        g_config.enabled_chips[ id] = true;
    }
    return true;
}

bool btcg_parse_opt() {
    bool succ = true;

    // Initialize g_config first.
    const size_t enabled_chips_size = sizeof( bool) * g_config.num_chips;
    assert( g_config.enabled_chips == NULL);
    g_config.enabled_chips = malloc( enabled_chips_size);
    assert( g_config.enabled_chips);
    memset( g_config.enabled_chips, 0, enabled_chips_size);

    succ &= parse_clks_opt();
    succ &= parse_only_enable_chips_opt();
    if ( succ) {
        p_config = &g_config;
    }
    else {
        free( g_config.enabled_chips);
        g_config.enabled_chips = NULL;
    }
    return succ;
}

const struct BTCG_config *btcg_config() {
    return p_config;
}
