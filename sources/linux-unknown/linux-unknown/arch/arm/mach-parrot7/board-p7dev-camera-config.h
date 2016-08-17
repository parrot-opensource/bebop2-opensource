/*
 * Configuration of CAMERA board
 */

/* configuration test synchronization:
 * - 3 sensors MT9F002 (14 Mpixels)
 */
/*#define CAMERA_CONFIG_3XMT9F002*/

/* configuration ADAS use case:
 * - 4 sensors MT9V117 (VGA)
 */
/*#define CAMERA_CONFIG_4XMT9V117*/

/* configuration Analog:
 * - 4 sensors TVP5151 (SD SOC camera)
 */
/*#define CAMERA_CONFIG_4XTVP5150*/

/* configuration Analog:
 * - 4 sensors TVP5151 (SD SOC platform)
 */
/*#define CAMERA_CONFIG_4XTVP5151*/

/* configuration test P7R2:
 * - 1 sensor MT9M114 (HD) slot #2
 */
/*#define CAMERA_CONFIG_1XMT9M114*/

/* configuration test P7R2:
 * - 1 sensor MT9F002 (14 Mpixels) slot #2
 */
/*#define CAMERA_CONFIG_1XMT9F002*/

/* configuration test validation AVI51 & AVI52:
 * - 4 sensors MT9F002 (14 Mpixels)
 * - 1 sensor MT9M114 (HD)
 * - 1 sensor AS0260 (Full HD)
 */
/*#define CAMERA_CONFIG_TEST_AVI51*/

/* configuration default:
 * - 3 sensors MT9F002 (14 Mpixels)
 * - 1 sensor MT9M114 (HD)
 * - 2 sensors AS0260 (Full HD)
 */
/*#define CAMERA_CONFIG_DEFAULT*/

#if defined(CAMERA_CONFIG_3XMT9F002)
#include "board-p7dev-camera-config-3xmt9f002.h"
#elif defined(CAMERA_CONFIG_4XMT9V117)
#include "board-p7dev-camera-config-4xmt9v117.h"
#elif defined(CAMERA_CONFIG_4XTVP5150)
#include "board-p7dev-camera-config-4xtvp5150.h"
#elif defined(CAMERA_CONFIG_4XTVP5151)
#include "board-p7dev-camera-config-4xtvp5151.h"
#elif defined(CAMERA_CONFIG_1XMT9M114)
#include "board-p7dev-camera-config-1xmt9m114.h"
#elif defined(CAMERA_CONFIG_1XMT9F002)
#include "board-p7dev-camera-config-1xmt9f002.h"
#elif defined(CAMERA_CONFIG_TEST_AVI51)
#include "board-p7dev-camera-config-test_AVI51.h"
#else
#include "board-p7dev-camera-config-default.h"
#warning "Use default configuration for P7dev camera board (cam6)"
#endif


