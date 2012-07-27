/*
 * Defines for encore boards
 */
#include <video/omapdss.h>

extern void __init encore_peripherals_init(void);
extern void __init encore_display_init(void);

#define BOARD_ENCORE_REV_EVT1A      0x1
#define BOARD_ENCORE_REV_EVT1B      0x2
#define BOARD_ENCORE_REV_EVT2       0x3

#define EVT0  0
#define EVT1A 1
#define EVT1B 2
#define EVT2  3
#define DVT   4
#define PVT   5

extern int has_3G_support(void);
extern int has_1GHz_support(void);

static inline int encore_board_type(void)
{
	return system_rev;  // This is set by U-Boot
}

static inline int is_encore_board_evt2(void)
{
    return (system_rev >= BOARD_ENCORE_REV_EVT2);
}

static inline int is_encore_board_evt1b(void)
{
    return (system_rev == BOARD_ENCORE_REV_EVT1B);
}

