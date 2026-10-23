/** @file s1_bridge.c
 *  @brief Weak defaults for the MQTT bridge (see s1_bridge.h). The strong
 *         versions in mqtt_utk.c replace these at link time when the bridge
 *         change is applied and the image is built with NET=lwip MQTT=1.
 *         With these defaults the bridge is simply "never connected".
 */

#include "s1_bridge.h"

__attribute__((weak)) void
mqtt_utk_bridge_attach (int32_t out_mbf, int32_t in_mbf,
                        char const * p_cmd_topic)
{
    (void) out_mbf;
    (void) in_mbf;
    (void) p_cmd_topic;
}

__attribute__((weak)) uint32_t
mqtt_utk_bridge_connected (void)
{
    return 0U;
}

__attribute__((weak)) uint32_t
mqtt_utk_bridge_backoff_s (void)
{
    return 0U;
}

__attribute__((weak)) uint32_t
mqtt_utk_bridge_pub_fail (void)
{
    return 0U;
}

__attribute__((weak)) uint32_t
mqtt_utk_bridge_stalls (void)
{
    return 0U;
}
