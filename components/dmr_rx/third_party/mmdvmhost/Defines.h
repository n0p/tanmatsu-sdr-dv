/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2023,2025,2026 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

// Modified for dmr_rx (tanmatsu-sdr), see NOTICE.md: trimmed to the parts
// needed by the vendored DMR FEC / PDU classes. Only USE_DMR is enabled and
// the class names are prefixed so that they cannot collide with other
// components that vendor MMDVMHost code.

#if !defined(Defines_H)
#define	Defines_H

#define	USE_DMR

#define CBPTC19696       DmrRx_CBPTC19696
#define CCRC             DmrRx_CCRC
#define CHamming         DmrRx_CHamming
#define CGolay2087       DmrRx_CGolay2087
#define CQR1676          DmrRx_CQR1676
#define CRS129           DmrRx_CRS129
#define CDMRSlotType     DmrRx_CDMRSlotType
#define CDMREMB          DmrRx_CDMREMB
#define CDMRLC           DmrRx_CDMRLC
#define CDMRFullLC       DmrRx_CDMRFullLC
#define CDMREmbeddedData DmrRx_CDMREmbeddedData
#define CDMRShortLC      DmrRx_CDMRShortLC
#define CUtils           DmrRx_CUtils

const unsigned char TAG_HEADER = 0x00U;
const unsigned char TAG_DATA   = 0x01U;
const unsigned char TAG_LOST   = 0x02U;
const unsigned char TAG_EOT    = 0x03U;
const unsigned char TAG_RSSI   = 0x04U;

#endif
