#include <stdio.h>

#include "miner.h"
#include "logging.h"

#include "btcg-config.h"


char *opt_btcg_clks = NULL;
char *opt_btcg_only_allow_chips = NULL;

static struct BTCG_config g_config = {
    .num_chips = 32,

    .spi_clk_khz = 200,

    .core_clk_mhz = 200,

    // The min freq of the chip is 200MHz.
    // With 32 cores each chip, the min hash rate is 6.4G/s.
    // The full search space is 4G, so the max time is about
    // 4G/(6.4G/s) = (4/6.4)s, which is less than 1s.
    // Now, set the time out to 10s, the safe margin is large
    // enough, and no too much failure messages.
    .work_timeout_ms = 10 * 1000,

    .consecutive_err_threshold = 15,
    .hibernate_time_ms = 30 * 1000,
};

static struct BTCG_config *p_config = NULL;

bool btcg_parse_opt() {
    int core_clk;
    int spi_clk;

    if (opt_btcg_clks) {
        
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
    }
    if (opt_btcg_only_allow_chips) {
        // TODO: parse only allow chips
        // ...
        // ...
    }
    p_config = &g_config;
    return true;
}

const struct BTCG_config *btcg_config() {
    return p_config;
}
