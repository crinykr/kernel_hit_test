/*
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __NEXELL_PCM_H
#define __NEXELL_PCM_H

#define	NX_SND_PCM_DAI_NAME	"nx-pcm"

/* platform data */
extern struct snd_soc_platform nx_snd_pcm_platform;

#if defined(CFG_DMA_AUDIO_PLAY)
#define PCM_DMA_PLAYBACK
#define	PCM_DMA_PLAYBACK_CH		CFG_DMA_AUDIO_PLAY
#endif

#if defined(CFG_DMA_AUDIO_REC)
#define PCM_DMA_CAPTURE
#define	PCM_DMA_CAPTURE_CH		CFG_DMA_AUDIO_REC
#endif

/*------------------------------------------------------------------------------
 * 	local data for DMA trans
 */
struct dma_buffer {
	unsigned int buf_base;
	unsigned int dma_phys;
	unsigned int buf_size;
};


struct nx_runtime_data {
	spinlock_t 			lock;
	struct dma_trans  * dma_tr;
	int					irq;
	bool 				dma_run;
	int					dma_width;
	/* DMA buffering */
	u_int 				buf_point;
	u_int 				buf_base;
	struct dma_buffer 	buf_mute;
};



#endif	/* __NEXELL_PCM_H */
