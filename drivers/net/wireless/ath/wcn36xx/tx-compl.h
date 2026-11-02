/* SPDX-License-Identifier: ISC */
#ifndef WCN36XX_TX_COMPL_H
#define WCN36XX_TX_COMPL_H

/* Caller provides u8/u32, size_t, bool and get_unaligned_le16/32.
 * Legacy HAL: 8-byte header + status. Prima appends a dialog token.
 * Legacy completion consumes only status (vendor WDA gates token use on
 * ENHANCED_TXBD_COMPLETION). This driver does not advertise enhanced completion
 * and serializes TX-status requests, so the trailing word is opaque here.
 * Do not update *status on malformed input or consume a pending TX skb.
 */
static inline bool wcn36xx_parse_tx_compl(const void *buf, size_t len,
					u32 *status)
{
	const u8 *p = buf;

	if (len != 12 && len != 16)
		return false;
	if (get_unaligned_le16(p + 2) != 0 ||
	    get_unaligned_le32(p + 4) != len)
		return false;
	*status = get_unaligned_le32(p + 8);
	return true;
}

#endif
