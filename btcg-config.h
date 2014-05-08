#ifndef BTCG_CONFIG_H
#define BTCG_CONFIG_H

/********** global driver configuration */
struct BTCG_config {
    unsigned num_chips;

    unsigned int spi_clk_khz;

    unsigned int core_clk_mhz;

    unsigned int work_timeout_ms;

    /* When number of consecutive errors
     * is larger than this number, the chip
     * should goes to hibernate state.
     */
    unsigned int consecutive_err_threshold;
    /* How many milliseconds should the chip hibernates,
     * when the chip enters hibernate state.
     */
    unsigned int hibernate_time_ms;
};

/* These two are used for command line options */
extern char *opt_btcg_clks;
extern char *opt_btcg_only_allow_chips;

/* Parse command options and set global config.
 * This is intended to be called at driver initialization time.
 */
bool btcg_parse_opt();

/* Call this function to get global config.
 * This function must be called after btcg_parse_opt().
 * When btcg_parse_opt() is not called, this function returns NULL.
 */
const struct BTCG_config *btcg_config();
#endif  /* BTCG_CONFIG_H */
