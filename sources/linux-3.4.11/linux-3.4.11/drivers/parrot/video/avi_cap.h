#ifndef _AVI_CAP_H_
#define _AVI_CAP_H_

/*
 * Capabilities descriptions. Each segment has a set of capabilities.
 *
 * Warning: if you add/delete/modify these entries be sure to update
 *          avi_caps_to_string
 */
#define AVI_CAP_DMA_IN          (1UL <<  0)
#define AVI_CAP_DMA_OUT         (1UL <<  1)
#define AVI_CAP_BUFFER          (1UL <<  2) /* FIFO Buffer after a scaler */
#define AVI_CAP_CONV            (1UL <<  3)
#define AVI_CAP_GAM             (1UL <<  4)
#define AVI_CAP_SCAL            (1UL <<  5)
#define AVI_CAP_ROT             (1UL <<  6)
#define AVI_CAP_PLANAR          (1UL <<  7)
/* Do not use these capabilities directly, use AVI_CAPS_ISP instead, as it does
 * not make sense to take only a part of the ISP */
#define AVI_CAP_ISP_CHAIN_BAYER (1UL <<  8)
#define AVI_CAP_STATS_YUV       (1UL <<  9)
#define AVI_CAP_ISP_CHAIN_YUV   (1UL << 10)

#define AVI_CAP_STATS_BAYER_0   (1UL << 11)
#define AVI_CAP_STATS_BAYER_1   (1UL << 12)

// XXX add the remaining capabilities
/* We need one capability for each camera and LCD to be able to pick one
 * explicitely (it doesn't make sense to dynamically allocate them) */
#define AVI_CAP_CAM_0            (1UL << 24)
#define AVI_CAP_CAM_1            (1UL << 25)
#define AVI_CAP_CAM_2            (1UL << 26)
#define AVI_CAP_CAM_3            (1UL << 27)
#define AVI_CAP_CAM_4            (1UL << 28)
#define AVI_CAP_CAM_5            (1UL << 29)

#define AVI_CAP_LCD_0            (1UL << 30)
#define AVI_CAP_LCD_1            (1UL << 31)

/* It does not make sense to take only a part of the ISP-chain */
#define AVI_CAPS_ISP     (AVI_CAP_ISP_CHAIN_BAYER | \
                          AVI_CAP_GAM             | \
                          AVI_CAP_CONV            | \
                          AVI_CAP_STATS_YUV       | \
                          AVI_CAP_ISP_CHAIN_YUV   )

#define AVI_CAPS_CAM_ALL (AVI_CAP_CAM_0 |        \
                          AVI_CAP_CAM_1 |        \
                          AVI_CAP_CAM_2 |        \
                          AVI_CAP_CAM_3 |        \
                          AVI_CAP_CAM_4 |        \
                          AVI_CAP_CAM_5 )

#define AVI_CAPS_LCD_ALL (AVI_CAP_LCD_0 | AVI_CAP_LCD_1)

/* Check that caps does not contain more than one capability set in exclusive.
 *
 * Returns 1 if there's a conflict, 0 otherwise.
 */
extern int avi_cap_check(unsigned long caps);

/* Get a non-busy node with given capability
 *
 * returns NULL if there no node is found or if more than one cap is requested
 */
extern struct avi_node *avi_cap_get_node(unsigned long cap);

/* Get a non-busy blender node.
 *
 * There is a separate function for blenders and capabilities as no capability
 * should be associated with a blender.
 *
 * returns NULL if there no node is found or if more than one cap is requested
 */
extern struct avi_node *avi_cap_get_blender(void);

#endif /* _AVI_CAP_H_ */
