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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/mach/map.h>

/* nexell soc headers */
#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/vmem.h>

#if (1)
#define DBGOUT(msg...)	{ printk(KERN_INFO "vmem: " msg); }
#define	KNL_BREAK		do{} while(1)
#define KNL_ASSERT(expr) {								\
		if (!(expr)) {                                  \
			printk(KERN_ERR "%s(%d) : %s (%s)\n",		\
				__FILE__, __LINE__, "_ASSERT", #expr);	\
			KNL_BREAK;                          		\
		}                                               \
	}
#else
#define DBGOUT(msg...)		do {} while (0)
#define KNL_ASSERT(expr)
#define KNL_BREAK
#endif

#define ERROUT(msg...)		{ 	\
		printk(KERN_ERR "ERROR: %s, %s line %d: \n",	\
			__FILE__, __FUNCTION__, __LINE__),			\
		printk(KERN_ERR msg); }

#define	COLLECT_STATUS	(1)

/*-------------------------------------------------------------------------*/
#define	VMEM_BLOCK_ZONE		0x20000000
#define	SEG_MIN_BYTE		(1024 * 1024)		/*  1M (0x00100000) */
#define	SEG_MAX_BYTE		(16 * 1024 * 1024)	/* 16M (0x01000000) */
#define	SEG_STRIDE_BYTE		(4096)				/* 0x1000 */

/*--------------------------------------------------------------------------
 *	Set block memory map table
 --------------------------------------------------------------------------*/
/* reserved memory struct for 2D/3D */
typedef struct {
	unsigned int phybase;
	unsigned int virbase;
	unsigned int length;
} RESERVEMEMBASE;

static RESERVEMEMBASE g_n2d_part[] = {
	/* partition 1 for video */
#if defined(CFG_MEM_PHY_BLOCK_BASE)
	{
		.phybase = CFG_MEM_PHY_BLOCK_BASE,
		.virbase = CFG_MEM_VIR_BLOCK_BASE,
		.length  = CFG_MEM_PHY_BLOCK_SIZE,
	},
#endif
};


#define NUM_OF_PARTS	(sizeof(g_n2d_part)/sizeof(g_n2d_part[0]))	/* total partition number */
#define NUM_OF_NODES	NUM_OF_PARTS								/* total root node number */
struct list_head 		g_n2d_root[NUM_OF_NODES];					/* root node */

/* lock */
static DEFINE_MUTEX(mem_lock);
#define	VM_LOCK()		{ mutex_lock(&mem_lock); }
#define	VM_UNLOCK()		{ if (mutex_is_locked(&mem_lock)) mutex_unlock(&mem_lock); }

/* vmem device */
struct device *	vm_device  = NULL;
static u64  	vm_dmamask = DMA_BIT_MASK(32);

//--------------------------------------------------------------------------
//
//	2D memory manager - begin
//
//	exports
//		- KNLMCB2D
//		- __KnlInitializeMCB2D
//		- __KnlFinalizeMCB2D
//		- __KnlMalloc
//		- __KnlFree
//		- __KnlAvailSpace
//		- __KnlShowNodes
//
//------------------------------------------------------------------------------
/* 2D/3D struct */
typedef struct stKNLMCB2D {
	unsigned int 		uiX;
	unsigned int		uiY;
	unsigned int		uiWidth;
	unsigned int		uiHeight;

	unsigned int		bSolid;
	struct stKNLMCB2D * pParent;
	struct stKNLMCB2D * pChild1;
	struct stKNLMCB2D * pChild2;

} KNLMCB2D;

/* node struct */
typedef struct __NODE2D {
	unsigned int 		segbase;
	unsigned int 		width;
	unsigned int 		height;
	unsigned int 		reserve_h;
	KNLMCB2D			node2d;
	struct list_head 	list;

} NODE2D, *PNODE2D;

static void		    __KnlInitializeMCB2D( KNLMCB2D * pMCB2D, unsigned int width, unsigned int  height,
									      unsigned int x, unsigned int y,	KNLMCB2D * pParent );
static void		    __KnlFinalizeMCB2D  (KNLMCB2D * pMCB2D);
static KNLMCB2D *   __KnlMalloc         ( KNLMCB2D * pMCB2D, unsigned int width, unsigned int height,
								          unsigned int align_x, unsigned int align_y );
static void		    __KnlFree           (KNLMCB2D * pMCB2D);
static void         __KnlShowNodes      (KNLMCB2D * pMCB2D, int (*pLogFunc)(const char *, ...), int Depth );

/*
static unsigned int	__KnlAvailSpace     (KNLMCB2D * pMCB2D);
*/
//------------------------------------------------------------------------------
// implementation
//		- __KnlIsPowerOf2
//		- __KnlSplitHorizontal
//		- __KnlSplitVertical
//		- __KnlSplit
//		- __KnlSplitPowerOf2Horizontal
//		- __KnlSplitPowerOf2Vertical
//		- __KnlSplitToPowerOf2
//		- __KnlIsFreeNode
//------------------------------------------------------------------------------
#define KNL_MALLOC(size)	kmalloc(size, GFP_KERNEL)
#define KNL_FREE(ptr)		kfree(ptr)
#define KNL_NULL			0
#define KNL_TRUE			1
#define KNL_FALSE			0

#ifdef __cplusplus
#define KNL_ASSERT_VALIDRECT(rect)									\
			KNL_ASSERT( IsPowerOf2((rect)->uiWidth) );				\
			KNL_ASSERT( IsPowerOf2((rect)->uiHeight) );            \
			KNL_ASSERT( 0 == ((rect)->m_nX%(rect)->uiWidth ) );    \
			KNL_ASSERT( 0 == ((rect)->m_nY%(rect)->uiHeight) );
#else
#define KNL_ASSERT_VALIDRECT(rect)
#endif

static void			__KnlSplitToPowerOf2( KNLMCB2D * );
static KNLMCB2D *	__KnlSplit( KNLMCB2D *, unsigned int, unsigned int );

/*
static int __KnlIsPowerOf2( unsigned int value )
{
	return ( (value & (value-1)) == 0 );
}
*/

static void		__KnlInitializeMCB2D( KNLMCB2D * pMCB2D, unsigned int width, unsigned int  height,
							unsigned int x, unsigned int y,	KNLMCB2D * pParent )
{
	KNL_ASSERT( 0 < width  );
	KNL_ASSERT( 0 < height );

	if ( ! pMCB2D ){ return; }

	//	Creating initial Node2d
	pMCB2D->bSolid = KNL_FALSE;

	pMCB2D->uiX = x;
	pMCB2D->uiY = y;
	pMCB2D->uiWidth  = width;
	pMCB2D->uiHeight = height;

	pMCB2D->pParent = pParent;
	pMCB2D->pChild1 = (KNLMCB2D *)KNL_NULL;
	pMCB2D->pChild2 = (KNLMCB2D *)KNL_NULL;
	if ( ! pParent ){ __KnlSplitToPowerOf2( pMCB2D ); }
}

static void __KnlFinalizeMCB2D  (KNLMCB2D * pMCB2D)
{
	if ( ! pMCB2D ){ return; }

	if ( pMCB2D->pChild1 )
	{
		__KnlFinalizeMCB2D(pMCB2D->pChild1);
		KNL_FREE(pMCB2D->pChild1);
	}

	if ( pMCB2D->pChild2 )
	{
		__KnlFinalizeMCB2D(pMCB2D->pChild2);
		KNL_FREE(pMCB2D->pChild2);
	}

	pMCB2D->bSolid  = KNL_FALSE;
	pMCB2D->pChild1 = (KNLMCB2D *)KNL_NULL;
	pMCB2D->pChild2 = (KNLMCB2D *)KNL_NULL;

	if ( ! pMCB2D->pParent ){ __KnlSplitToPowerOf2( pMCB2D ); }
}

static void		__KnlGetClosest( KNLMCB2D *pMCB2D,
			unsigned int width, unsigned int height, unsigned int align_x, unsigned int align_y,
			unsigned int *pLeastWaste, KNLMCB2D **ppBestFit )
{
	if ( pMCB2D->bSolid )
		return; // this node is in use
	if ( pMCB2D->uiWidth < width || pMCB2D->uiHeight < height )
		return; // this one (and any children) are too small
	if ( pMCB2D->pChild2 ) // This is a parent node, just check child nodes for better fit
	{
//		KNL_ASSERT( CNULL != pMCB2D->pChild1 );
		__KnlGetClosest( pMCB2D->pChild1, width, height, align_x, align_y, pLeastWaste, ppBestFit );

		if ( 0 == pLeastWaste ){ return; }
		__KnlGetClosest( pMCB2D->pChild2, width, height, align_x, align_y, pLeastWaste, ppBestFit );
		//if ( 0 == pLeastWaste ){ return; }
	}
	else
	{
		KNL_ASSERT_VALIDRECT( this );
		if ( 0 == (pMCB2D->uiX % align_x) && 0 == (pMCB2D->uiY % align_y) )
		{
			// This node is free & big enough - check for fit
			unsigned int waste = pMCB2D->uiWidth * pMCB2D->uiHeight - width * height;
			if ( *ppBestFit == (KNLMCB2D *)KNL_NULL || waste < *pLeastWaste )
			{
				*pLeastWaste = waste;
				*ppBestFit   = pMCB2D;
			}
		}
	}
}

static KNLMCB2D * __KnlSplitHorizontal( KNLMCB2D * pMCB2D, unsigned int width, unsigned int height )
{
	// 좌측상단의 width*height를 제외한 나머지 영역에서
	// 가장큰 2의 승수꼴 사각형을 잘라낸다.
	unsigned int newsize;

	for( newsize=1; width+newsize <= pMCB2D->uiWidth; newsize<<=1 ){ /* do nothing */ }

	newsize>>=1;
//	KNL_ASSERT( 0 < newsize );
//	KNL_ASSERT( 0 == ( (m_nX+(uiWidth-newsize)) % newsize ) );

	// Alloc the child1
	pMCB2D->pChild1 = (KNLMCB2D *)KNL_MALLOC(sizeof(KNLMCB2D));
	__KnlInitializeMCB2D( pMCB2D->pChild1, (pMCB2D->uiWidth - newsize), pMCB2D->uiHeight,
					pMCB2D->uiX, pMCB2D->uiY, pMCB2D );

	// Alloc the child2
	pMCB2D->pChild2 = (KNLMCB2D *)KNL_MALLOC(sizeof(KNLMCB2D));
	__KnlInitializeMCB2D( pMCB2D->pChild2, newsize, pMCB2D->uiHeight,
					pMCB2D->uiX + (pMCB2D->uiWidth - newsize), pMCB2D->uiY, pMCB2D );

	__KnlSplitToPowerOf2(pMCB2D->pChild2);
	return __KnlSplit( pMCB2D->pChild1, width, height );
}

static KNLMCB2D * __KnlSplitVertical ( KNLMCB2D * pMCB2D, unsigned int width, unsigned int height )
{
	// 좌측상단의 width*height를 제외한 나머지 영역에서
	// 가장큰 2의 승수꼴 사각형을 잘라낸다.
	unsigned int newsize;
	for( newsize=1; height+newsize <= pMCB2D->uiHeight; newsize<<=1 ){ /* do nothing */ }

	newsize>>=1;
//	KNL_ASSERT( 0 < newsize );
//	KNL_ASSERT( 0 == ( (m_nY+(uiHeight-newsize)) % newsize ) );

	// Alloc the child1
	pMCB2D->pChild1 = (KNLMCB2D *)KNL_MALLOC(sizeof(KNLMCB2D));
	__KnlInitializeMCB2D( pMCB2D->pChild1, pMCB2D->uiWidth, (pMCB2D->uiHeight - newsize),
					pMCB2D->uiX, pMCB2D->uiY, pMCB2D );

	// Alloc the child2
	pMCB2D->pChild2 = (KNLMCB2D *)KNL_MALLOC(sizeof(KNLMCB2D));
	__KnlInitializeMCB2D( pMCB2D->pChild2, pMCB2D->uiWidth, newsize,
					pMCB2D->uiX, pMCB2D->uiY + (pMCB2D->uiHeight - newsize), pMCB2D );

	__KnlSplitToPowerOf2(pMCB2D->pChild2);
	return __KnlSplit( pMCB2D->pChild1, width, height );
}

static KNLMCB2D * __KnlSplit( KNLMCB2D * pMCB2D, unsigned int width, unsigned int height )
{
	unsigned int dw = pMCB2D->uiWidth  - width ;
	unsigned int dh = pMCB2D->uiHeight - height;

	// Splitting Node2d (m_nWidthxm_nHeight) to give (widthxheight) space
//	KNL_ASSERT( ! pMCB2D->m_Solid );
//	KNL_ASSERT( width <= pMCB2D->uiWidth );
//	KNL_ASSERT( height <= pMCB2D->uiHeight );

	if ( dh > 0 || dw > 0 )
	{
		int bDivide = (dw > dh) ? KNL_TRUE : KNL_FALSE;

		if ( bDivide )	{ return __KnlSplitHorizontal( pMCB2D, width, height ); }
		else 			{ return __KnlSplitVertical( pMCB2D, width, height ); }
	}
	return pMCB2D;
}

static int	__KnlSplitPowerOf2Horizontal ( KNLMCB2D * pMCB2D )
{
	unsigned int newsize;

	// 사각형의 위치값(m_nY)을 나머지 없이 나눌수 있는 최대 2의 승수를 구함.
    for( newsize=1; (0 == (pMCB2D->uiX & newsize)) && (newsize<pMCB2D->uiWidth); newsize<<=1 ){ /* do nothing */ }

    // 구해진 값이 현재 크기(uiHeight)보다 작다면 쪼개야 한다.
    if ( newsize < pMCB2D->uiWidth )
	{
		// Alloc the child1
		pMCB2D->pChild1 = (KNLMCB2D *)KNL_MALLOC(sizeof(KNLMCB2D));
		__KnlInitializeMCB2D( pMCB2D->pChild1, newsize, pMCB2D->uiHeight,
						pMCB2D->uiX, pMCB2D->uiY, pMCB2D );

		// Alloc the child2
		pMCB2D->pChild2 = (KNLMCB2D *)KNL_MALLOC(sizeof(KNLMCB2D));
		__KnlInitializeMCB2D( pMCB2D->pChild2, pMCB2D->uiWidth - newsize, pMCB2D->uiHeight,
						pMCB2D->uiX + newsize, pMCB2D->uiY, pMCB2D );

		__KnlSplitToPowerOf2(pMCB2D->pChild1);
		__KnlSplitToPowerOf2(pMCB2D->pChild2);
		return KNL_TRUE;
	}
	return KNL_FALSE;
}

static int	__KnlSplitPowerOf2Vertical ( KNLMCB2D * pMCB2D )
{
	unsigned int newsize;

    // 사각형의 위치값(m_nY)을 나머지 없이 나눌수 있는 최대 2의 승수를 구함.
    for( newsize=1;(0 == (pMCB2D->uiY & newsize)) && (newsize<pMCB2D->uiHeight); newsize<<=1 ){ /* do nothing */ }

    // 구해진 값이 현재 크기(uiHeight)보다 작다면 쪼개야 한다.
    if ( newsize < pMCB2D->uiHeight )
	{
		pMCB2D->pChild1 = (struct stKNLMCB2D *)KNL_MALLOC(sizeof(KNLMCB2D));
		__KnlInitializeMCB2D( pMCB2D->pChild1, pMCB2D->uiWidth, newsize,
						pMCB2D->uiX, pMCB2D->uiY, pMCB2D );

		pMCB2D->pChild2 = (struct stKNLMCB2D *)KNL_MALLOC(sizeof(KNLMCB2D));
		__KnlInitializeMCB2D( pMCB2D->pChild2, pMCB2D->uiWidth, pMCB2D->uiHeight - newsize,
						pMCB2D->uiX, pMCB2D->uiY+newsize, pMCB2D );

		__KnlSplitToPowerOf2(pMCB2D->pChild1);
		__KnlSplitToPowerOf2(pMCB2D->pChild2);
		return KNL_TRUE;
	}
	return KNL_FALSE;
}

static void		__KnlSplitToPowerOf2 ( KNLMCB2D * pMCB2D )
{
	int	bDivide;

	KNL_ASSERT( 0 < pMCB2D->uiWidth );
	KNL_ASSERT( 0 < pMCB2D->uiHeight );

	bDivide = ( pMCB2D->uiWidth >= pMCB2D->uiHeight ) ? KNL_TRUE : KNL_FALSE; // 큰변을 쪼갠다
	if ( bDivide )
	{
		if ( __KnlSplitPowerOf2Horizontal(pMCB2D) ){ return; }
		if ( __KnlSplitPowerOf2Vertical(pMCB2D) ){ return; }
	}
	else
	{
		if ( __KnlSplitPowerOf2Vertical(pMCB2D) ){ return; }
		if ( __KnlSplitPowerOf2Horizontal(pMCB2D) ){ return; }
	}
}

static int	__KnlIsFreeNode( KNLMCB2D * pMCB2D )
{
	if ( pMCB2D->bSolid )
		return KNL_FALSE;

	if ( pMCB2D->pChild1 )
	{
		if ( !__KnlIsFreeNode(pMCB2D->pChild1) )
			return KNL_FALSE;
	}

	if ( pMCB2D->pChild2 )
	{
		if ( !__KnlIsFreeNode(pMCB2D->pChild2) )
			return KNL_FALSE;
	}
	return KNL_TRUE;
}

static KNLMCB2D * __KnlMalloc( KNLMCB2D * pMCB2D, unsigned int width, unsigned int height, unsigned int align_x, unsigned int align_y )
{
	unsigned int waste;
	KNLMCB2D * pNode = (KNLMCB2D *)KNL_NULL;

	if (0 == align_x) align_x = 1;
	if (0 == align_y) align_y = 1;

	//width = ( width + pMCB2D->uiAlign - 1 ) & ( - (int)pMCB2D->uiAlign );	// Round up to alignment

	if ( ! pMCB2D ){ return (KNLMCB2D *)KNL_NULL; }

	// Find the best node to contain the new area
	// Attempting to allocate (widthxheight) space
	__KnlGetClosest( pMCB2D, width, height, align_x, align_y, &waste, &pNode );
	if ( pNode )
	{
		// Split the node into the required space & remainder piece(s)
		pNode = __KnlSplit( pNode, width, height );
		// Succesfully allocated pNode (m_nX,m_nY,uiWidth,uiHeight)
		KNL_ASSERT( CNULL != pNode );
		// node가 사용중임을 표시해둔다.
		pNode->bSolid = KNL_TRUE;
	//	KNL_ASSERT( CNULL == pNode->pChild1 );
	//	KNL_ASSERT( CNULL == pNode->pChild2 );
	}
	else
	{
		// Failed to allocate Node2d !!
	}
	return pNode;
}

static void		__KnlFree(KNLMCB2D * pMCB2D)
{
	if ( ! pMCB2D ){ return; }

	if ( pMCB2D->bSolid )
	{
	//	KNL_ASSERT( pMCB2D->pChild1 == KNL_NULL );
	//	KNL_ASSERT( pMCB2D->pChild2 == KNL_NULL );
		pMCB2D->bSolid = KNL_FALSE;
	}
	else
	{
	//	KNL_ASSERT( pMCB2D->pChild1 );
	//	KNL_ASSERT( pMCB2D->pChild2 );
		// We are being told to delete both child nodes as they are now free
		if ( __KnlIsFreeNode(pMCB2D->pChild1) && __KnlIsFreeNode(pMCB2D->pChild2) )
		{
			KNL_FREE(pMCB2D->pChild1);
			KNL_FREE(pMCB2D->pChild2);
			pMCB2D->pChild1 = pMCB2D->pChild2 = (KNLMCB2D *)KNL_NULL;
		}
	}
	if ( pMCB2D->pParent )
	{
		if ( __KnlIsFreeNode(pMCB2D->pParent) )
		{
			// Now this & sibling are free so we can erase both & coalesce
			__KnlFree(pMCB2D->pParent);
		}
		else
		{
			__KnlSplitToPowerOf2(pMCB2D);
		}
	}
}

/*
static unsigned int	__KnlAvailSpace(KNLMCB2D * pMCB2D)
{
	if ( ! pMCB2D ){ return 0; }
	if ( pMCB2D->pChild2 )
		return __KnlAvailSpace(pMCB2D->pChild1) + __KnlAvailSpace(pMCB2D->pChild2);
	else if ( pMCB2D->pChild1 != (KNLMCB2D *)KNL_NULL )
		return 0;
	else
		return pMCB2D->uiWidth * pMCB2D->uiHeight;
}
*/

static void __KnlShowNodes (KNLMCB2D * pMCB2D, int (*pLogFunc)(const char *, ...), int Depth )
{
	int  i;

	if ( ! pMCB2D )
		return;

	for(i = 0; i < Depth; i++)
		pLogFunc("|");

	pLogFunc("+%s(0x%08x, y:%d~%d, x:%d~%d)\n",
	 	pMCB2D->bSolid ? "*" : "-",
	 	(int)(pMCB2D->uiX+pMCB2D->uiY*4096),
	 	(int)pMCB2D->uiY, (int)pMCB2D->uiY + (int)pMCB2D->uiHeight,
	 	(int)pMCB2D->uiX, (int)pMCB2D->uiX + (int)pMCB2D->uiWidth);

	if ( pMCB2D->pChild1 ) __KnlShowNodes (pMCB2D->pChild1, pLogFunc, Depth+1 );
	if ( pMCB2D->pChild2 ) __KnlShowNodes (pMCB2D->pChild2, pLogFunc, Depth+1 );
}

/*--------------------------------------------------------------------------------
 * 2D memory manager
 ---------------------------------------------------------------------------------*/
struct vm_node {
	pid_t			  pid;
	char			* comm;
	void			* handle;
	unsigned int	  length;
	unsigned int	  address;
	unsigned int 	  flags;
	struct list_head  list;
};
static LIST_HEAD(vm_node_list);

#if (COLLECT_STATUS)
#define K(x) ((x) << (PAGE_SHIFT-10))

static void show_vmem_dmazone(void)
{
	struct zone *zone;
	for_each_populated_zone(zone) {
 		unsigned long nr[MAX_ORDER], flags, order, total = 0;
		printk("%s: ", zone->name);
		spin_lock_irqsave(&zone->lock, flags);
		for (order = 0; order < MAX_ORDER; order++) {
			nr[order] = zone->free_area[order].nr_free;
			total += nr[order] << order;
		}
		spin_unlock_irqrestore(&zone->lock, flags);
		for (order = 0; order < MAX_ORDER; order++)
			printk("%lu*%lukB ", nr[order], K(1UL) << order);
		printk("= %lukB\n", K(total));
	}
}

static void show_vmem_status(void)
{
	struct list_head *tls = NULL;
	struct vm_node	 *vmn = NULL;
	uint total = 0;

	VM_LOCK();
	list_for_each(tls, &vm_node_list) {
		vmn = list_entry(tls, struct vm_node, list);
		total += PAGE_ALIGN(vmn->length);
		printk(KERN_INFO "[%4d] curr a:0x%08x, p:0x%08x, s:%8d, [%s], %s\n",
			vmn->pid, (u_int)vmn->handle, vmn->address, vmn->length,
			vmn->flags & VMEM_BLOCK_BUFFER ? "2D":"1D", vmn->comm);
	}
	show_vmem_dmazone();
	printk(KERN_INFO "total : %8d\n", total);
	VM_UNLOCK();
}
#endif

static int collect_vmem_garbage(int minor)
{
	struct task_struct *p   = NULL;	/* task */
	struct task_struct *t   = NULL;	/* thread */
	struct list_head   *tls = NULL;
	struct vm_node	   *vmn = NULL;

	int  die = 1;
	int  ret = ERR_VMEM_ALLOC;
	uint total = 0;

	DBGOUT("%s(minor:%d, pid:%d)\n", __func__, minor, current->pid);

	if (NUM_OF_PARTS && minor >= NUM_OF_PARTS)
		return ret;

	/* lock */
	VM_LOCK();

	list_for_each(tls, &vm_node_list) {
		vmn = list_entry(tls, struct vm_node, list);
		die = 1;
		if (vmn->pid) {
			/* find dead process */
			for_each_process(p) {
				t = next_thread(p);
				if (p == t) {	/* empty thread */
					if (vmn->pid == task_pid_nr(p)) {	/* alive */
						die = 0;
						continue;
					}
				} else {		/* check thread */
					for (; p != t; t = next_thread(t)) {
						if (vmn->pid == task_pid_nr(t)) {	/* alive */
							die = 0;
							break;
						}
					}
				}
			}

			/* free node */
			if (die) {
				struct list_head *dls = tls;
				pid_t  pid = vmn->pid;
				int	i;
				tls = tls->prev;

				for (i = 0; dls != &vm_node_list; dls = dls->next, i++) {
					vmn = list_entry(dls, struct vm_node, list);
					if (pid != vmn->pid)
						continue;

					dls = dls->prev;

				#if (COLLECT_STATUS)
					total += PAGE_ALIGN(vmn->length);
					printk(KERN_INFO "[%4d] free a:0x%08x, p:0x%08x, s:%8d, [%s]\n",
						vmn->pid, (u_int)vmn->handle, vmn->address, vmn->length,
						vmn->flags & VMEM_BLOCK_BUFFER ? "2D":"1D");
				#endif
					if (vmn->flags & VMEM_BLOCK_BUFFER)	{
						__KnlFree((KNLMCB2D*)(vmn->handle));
					} else {
						struct device *dev = vm_device;
						unsigned long size = PAGE_ALIGN(vmn->length);
						dma_free_coherent(dev, size, vmn->handle, vmn->address);
					}
					list_del(&vmn->list);
					kfree(vmn);
				}
				printk(KERN_INFO "total : %8d\n", total);
				ret = ERR_VMEM_NO;
			}
		}
	}

	/* unlock */
	VM_UNLOCK();

#if (COLLECT_STATUS)
	show_vmem_status();
#endif
	return ret;
}

/*---------------------------------------------------------------------------------*/
void 	vmem_show(int minor);
void 	vmem_reset(unsigned int type, int minor);
int 	vmem_alloc(VM_IMEMORY *imem, int minor);
int 	vmem_free(VM_IMEMORY *imem);

void vmem_show(int minor)
{
	struct list_head *ln2dh = NULL;
	struct list_head *ldata = NULL;
	NODE2D * n2d = NULL;

	DBGOUT("%s(minor:%d)\n", __func__, minor);
	if (minor >= NUM_OF_PARTS)
		return;

	VM_LOCK();

	if (1 == NUM_OF_NODES && 1 == minor)
		minor = 0;

	printk("\nPARTS [%d]\n", minor);

	// show all segment
	ln2dh = &g_n2d_root[minor];
	list_for_each_prev(ldata, ln2dh) {
		n2d = list_entry(ldata, NODE2D, list);
		if (n2d) {
			printk("\nSEG [0x%08x]\n", n2d->segbase);
			__KnlShowNodes(&n2d->node2d, printk, 0);
		}
	}
	VM_UNLOCK();
}

void vmem_reset(unsigned int type, int minor)
{
	struct list_head *tls   = NULL;
	struct vm_node	 *vmn   = NULL;
	int	del = 0;

	DBGOUT("%s(minor:%d)\n", __func__, minor);

	if (NUM_OF_PARTS && minor >= NUM_OF_PARTS)
		return;

	VM_LOCK();

	// free all node and memory
	list_for_each(tls, &vm_node_list) {

		vmn = list_entry(tls, struct vm_node, list);
		DBGOUT("[%4d] free a:0x%08x, p:0x%08x, s:%8d, [%s]\n",
			vmn->pid, (u_int)vmn->handle, vmn->address, vmn->length,
			vmn->flags & VMEM_BLOCK_BUFFER ? "2D":"1D");

		if (VMEM_BLOCK_BUFFER == vmn->flags) {
			__KnlFree((KNLMCB2D*)(vmn->handle));
			del = 1;
		}

		if (VMEM_LINEAR_BUFFER == vmn->flags) {
			struct device *dev = vm_device;
			unsigned long size = PAGE_ALIGN(vmn->length);
			dma_free_coherent(dev, size, vmn->handle, vmn->address);
			del = 1;
		}

		if (del) {
			tls = tls->prev;
			list_del(&vmn->list);
			kfree(vmn);
		}
		del = 0;
	}

	VM_UNLOCK();
	DBGOUT("done:%s(minor:%d)\n", __func__, minor);
}

int vmem_alloc(VM_IMEMORY *imem, int minor)
{
	struct vm_node * vmn = NULL;
	int ret = ERR_VMEM_NO;
	int cnt = 0;

	DBGOUT("%s (minor:%d)\n", __func__, minor);
	KNL_ASSERT(imem);

	if (NUM_OF_PARTS && minor >= NUM_OF_PARTS) {
		ret = ERR_VMEM_ALLOC;
		imem->MemPoint = NULL;
		return ret;
	}

	if (1 == NUM_OF_NODES && 1 == minor)
		minor = 0;

	/* when linear cb/cr */
	if (! imem->MemWidth || ! imem->MemHeight) {
		imem->MemPoint = NULL;
		return ret;
	}

	vmn = kmalloc(sizeof(struct vm_node), GFP_KERNEL);
	if (! vmn) {
		ret = ERR_VMEM_ALLOC;
		ERROUT("Fail, kmalloc (%d) !!!\n", sizeof(struct vm_node));
		return ret;
	}

	/* lock */
	VM_LOCK();

retry:
	imem->MemPoint = NULL;
	ret = ERR_VMEM_NO;

	if (imem->Flags & VMEM_BLOCK_BUFFER)	{
		struct list_head *ln2dh = &g_n2d_root[minor];
		struct list_head *ldata = NULL;
		NODE2D   * n2d   = NULL;
		KNLMCB2D * mcb2d = NULL;

		ln2dh = &g_n2d_root[minor];

		list_for_each_prev(ldata, ln2dh) {
			n2d = list_entry(ldata, NODE2D, list);
			if (n2d) {
				DBGOUT("[minor:%d] seg base:0x%08x, w:%8d, h:%8d, r:%4d \n",
					minor, n2d->segbase, n2d->width, n2d->height, n2d->reserve_h);
				mcb2d = __KnlMalloc(&n2d->node2d, imem->MemWidth, imem->MemHeight,
									imem->HorAlign, imem->VerAlign );
				if (mcb2d)
					break;
			}
		}
		if (0 == mcb2d) {
			ret = ERR_VMEM_ALLOC;
			goto __err;
		}
		imem->MemPoint = mcb2d;
		imem->Address  = (VMEM_BLOCK_ZONE | n2d->segbase) + (mcb2d->uiY<<12) + mcb2d->uiX;

		DBGOUT("->  0x%08x = w:%8d, h:%8d - x:%4d ~ %4d, y:%4d ~ %4d\n",
			imem->Address, mcb2d->uiWidth, mcb2d->uiHeight,
			mcb2d->uiX, (mcb2d->uiX + mcb2d->uiWidth),
			mcb2d->uiY, (mcb2d->uiY + mcb2d->uiHeight));
	} else {
		struct device *dev = vm_device;
		unsigned long size = PAGE_ALIGN(imem->MemWidth * imem->MemHeight);
		gfp_t gfp_mask = (GFP_KERNEL | GFP_DMA);

		#if (0)
		show_vmem_dmazone();
		#endif

		imem->MemPoint = dma_alloc_coherent(dev, size, &imem->Address, gfp_mask);
		if (! imem->MemPoint) {
			gfp_mask &= ~GFP_DMA;
			imem->MemPoint = dma_alloc_coherent(dev, size, &imem->Address, gfp_mask);
			if (! imem->MemPoint) {
				ret = ERR_VMEM_ALLOC;
				goto __err;
			}
		}
		DBGOUT("Linear +v:0x%08x, p:0x%08x, w:%8d, h:%8d, s:%d\n",
			(u_int)imem->MemPoint, imem->Address, imem->MemWidth, imem->MemHeight, (int)size);
	}

	/*
	 * add node to list manager
	 */
	vmn->pid     = task_pid_nr(get_current());
	vmn->comm    = get_current()->comm;
	vmn->handle  = imem->MemPoint;
	vmn->address = imem->Address;
	vmn->length  = imem->MemWidth * imem->MemHeight;
	vmn->flags   = imem->Flags;

	list_add(&vmn->list, &vm_node_list);

	DBGOUT("[%4d] +a:0x%08x, p:0x%08x, s:%8d, %s\n",
		vmn->pid, (u_int)vmn->handle, vmn->address, vmn->length, vmn->comm);

	/* unlock */
	VM_UNLOCK();
	DBGOUT("done:%s(minor:%d)\n", __func__, minor);
	return ret;

__err:
	/* unlock */
	VM_UNLOCK();

	if (! cnt++) {
		if (ERR_VMEM_NO == collect_vmem_garbage(minor)) {
			printk(KERN_INFO "\nWARN, try %s meory collection (%s)...\n",
				imem->Flags & VMEM_BLOCK_BUFFER ? "block": "linear", __func__);
			goto retry;
		}
	}
	ERROUT("Fail, allocate %s memory (%d by %d) !!!\n",
		imem->Flags & VMEM_BLOCK_BUFFER ? "block": "linear",
		(int)imem->MemWidth, (int)imem->MemHeight);
	show_vmem_dmazone();

	if (vmn)
		kfree(vmn);

	DBGOUT("err :%s(minor:%d)\n", __func__, minor);
	return ret;
}

int vmem_free(VM_IMEMORY *imem)
{
	struct list_head *tls = NULL;
	struct vm_node	 *vmn = NULL;

	DBGOUT("%s\n", __func__);
	KNL_ASSERT(imem)

	if (! imem->MemPoint)
		return ERR_VMEM_NO;

	if (! imem->MemWidth || ! imem->MemHeight) {
		printk(KERN_ERR "%s: Warn, invalid free size (0x%x, %d*%d)\n",
			__func__, (u_int)imem->MemPoint, imem->MemWidth, imem->MemHeight);
		return ERR_VMEM_NO;
	}

	/* lock */
	VM_LOCK();

	if (imem->Flags & VMEM_BLOCK_BUFFER)	{
		__KnlFree((KNLMCB2D*)(imem->MemPoint));
	} else {
		struct device *dev = vm_device;
		unsigned long size = PAGE_ALIGN(imem->MemWidth * imem->MemHeight);
		dma_free_coherent(dev, size, imem->MemPoint, imem->Address);
		DBGOUT("Linear -v:0x%08x, p:0x%08x, s:%d \n",
			(unsigned int)imem->MemPoint, imem->Address, (unsigned int)size);
	}

	/*
	 * delete node from list manager
	 */
	list_for_each(tls, &vm_node_list) {
		vmn = list_entry(tls, struct vm_node, list);
		if (vmn && imem->MemPoint == vmn->handle) {
			list_del(&vmn->list);
			DBGOUT("[%4d] -a:0x%08x, p:0x%08x, s:%8d, %s\n",
				vmn->pid, (u_int)vmn->handle, vmn->address, vmn->length, vmn->comm);
			kfree(vmn);
			break;
		}
	}

	/* clear */
	imem->Address  = 0;
	imem->MemPoint = NULL;

	/* unlock */
	VM_UNLOCK();

	DBGOUT("%s, done\n", __func__);
	return ERR_VMEM_NO;
}

unsigned int vmem_get_blkvirbase(int minor)
{
	DBGOUT("%s(minor:%d)\n", __func__, minor);

	if (NUM_OF_PARTS && NUM_OF_PARTS > minor)
		return g_n2d_part[minor].virbase;
	else
		ERROUT("Fail, not exist memory table[%d] ...\n", minor);
	return 0;
}

unsigned int vmem_get_blkphybase(int minor)
{
	DBGOUT("%s(minor:%d)\n", __func__, minor);

	if (NUM_OF_PARTS && NUM_OF_PARTS > minor)
		return g_n2d_part[minor].phybase;
	else
		ERROUT("Fail, not exist memory table[%d] ...\n", minor);
	return 0;
}

unsigned int vmem_get_blklength(int minor)
{
	DBGOUT("%s(minor:%d)\n", __func__, minor);

	if (NUM_OF_PARTS && NUM_OF_PARTS > minor)
		return g_n2d_part[minor].length;
	else
		ERROUT("Fail, not exist memory table[%d] ...\n", minor);
	return 0;
}

EXPORT_SYMBOL_GPL(vmem_show);
EXPORT_SYMBOL_GPL(vmem_reset);
EXPORT_SYMBOL_GPL(vmem_alloc);
EXPORT_SYMBOL_GPL(vmem_free);
EXPORT_SYMBOL_GPL(vmem_get_blkvirbase);
EXPORT_SYMBOL_GPL(vmem_get_blkphybase);
EXPORT_SYMBOL_GPL(vmem_get_blklength);

/*--------------------------------------------------------------------------------
 * file_operations
 ---------------------------------------------------------------------------------*/
static int nx_vmem_open(struct inode *inode, struct file *flip)
{
	int minor = MINOR(inode->i_rdev);
	DBGOUT("%s (minor:%d, table:%d)\n", __func__, minor, NUM_OF_PARTS);

	if (NUM_OF_PARTS && minor >= NUM_OF_PARTS) {
		ERROUT("Fail, not exist memory table[%d] ...\n", minor);
		return -1;
	}
	return 0;
}

static int nx_vmem_release(struct inode *inode, struct file *flip)
{
	int minor = MINOR(inode->i_rdev);
	DBGOUT("%s (minor:%d, table:%d)\n", __func__, minor, NUM_OF_PARTS);

	if (NUM_OF_PARTS && minor >= NUM_OF_PARTS) {
		ERROUT("Fail, not exist memory table[%d] ...\n", minor);
		return -1;
	}
	return 0;
}

static long nx_vmem_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
	int ret   = ERR_VMEM_INVALID;
	int minor = (int)MINOR(file->f_path.dentry->d_inode->i_rdev);

	DBGOUT("%s (minor:%d, table:%d, cmd:0x%x, nr:%d)\n",
		__func__, minor, NUM_OF_PARTS, cmd, _IOC_NR(cmd));

	if (NUM_OF_PARTS && minor >= NUM_OF_PARTS) {
		ERROUT("Fail, not exist memory table[%d] ...\n", minor);
		return ret;
	}

	switch(cmd)	{
		case IOCTL_VMEM_STATUS:
			{
				vmem_show(minor);
				ret = ERR_VMEM_NO;
			}
			break;

		case IOCTL_VMEM_RESET:
			{
				unsigned int type;
				if (copy_from_user((void*)&type, (const void*)arg, sizeof(unsigned int)))
					break;

				vmem_reset(type, minor);
				ret = ERR_VMEM_NO;
			}
			break;

		case IOCTL_VMEM_INFO:
			{
				VM_MEMBASE base;
				if (copy_from_user((void*)&base, (const void*)arg, sizeof(VM_MEMBASE)))
					break;

				base.LinPhyBase = 0;
				base.LinPhySize = 0;
				base.LinVirBase = 0;
				base.LinVirSize = 0;

				base.BlkPhyBase = 0;
				base.BlkPhySize = 0;
				base.BlkVirBase = 0;
				base.BlkVirSize = 0;

				if (NUM_OF_PARTS > minor) {
					base.BlkPhyBase = g_n2d_part[minor].phybase;
					base.BlkPhySize = g_n2d_part[minor].length;
					base.BlkVirBase = g_n2d_part[minor].virbase;
					base.BlkVirSize = g_n2d_part[minor].length;
				}

				if (copy_to_user((void*)arg, (const void*)&base, sizeof(VM_MEMBASE)))
					break;

				ret = ERR_VMEM_NO;
			}
			break;

		case IOCTL_VMEM_ALLOC:
			{
				VM_IMEMORY imem;
				if (copy_from_user((void*)&imem, (const void*)arg, sizeof(VM_IMEMORY)))
					break;

				ret = vmem_alloc(&imem, minor);

				if (ERR_VMEM_NO == ret) {
					if (copy_to_user((void*)arg, (const void*)&imem, sizeof(VM_IMEMORY))) {
						vmem_free(&imem);
						return ERR_VMEM_INVALID;
					}
				}
			}
			break;

		case IOCTL_VMEM_FREE:
			{
				VM_IMEMORY imem;
				if (copy_from_user((void*)&imem, (const void*)arg, sizeof(VM_IMEMORY)))
					break;

				vmem_free((VM_IMEMORY*)&imem);
				ret = ERR_VMEM_NO;
			}
			break;

		case IOCTL_VMEM_VALLOC:
			{
				VM_VMEMORY vmem;
				if (copy_from_user((void*)&vmem, (const void*)arg, sizeof(VM_VMEMORY)))
 					break;

				/* allocate Lu */
				ret = vmem_alloc(&vmem.LuMemory, minor);
				if (ERR_VMEM_NO != ret)
 					return ret;

				/* allocate Cb */
				ret  = vmem_alloc(&vmem.CbMemory, minor);
				if (ERR_VMEM_NO != ret) {
 					vmem_free(&vmem.LuMemory);
 					return ret;
 				}

 				/* allocate Cr */
				ret  = vmem_alloc(&vmem.CrMemory, minor);
				if (ERR_VMEM_NO != ret) {
 					vmem_free(&vmem.LuMemory);
 					vmem_free(&vmem.CbMemory);
 					return ret;
 				}

				if (copy_to_user((void*)arg, (const void*)&vmem, sizeof(VM_VMEMORY))) {
					vmem_free(&vmem.LuMemory);
 					vmem_free(&vmem.CbMemory);
 					vmem_free(&vmem.CrMemory);
 					ret = ERR_VMEM_INVALID;
					break;
				}
			}
			break;

		case IOCTL_VMEM_VFREE:
			{
				VM_VMEMORY  vmem;
				if (copy_from_user((void*)&vmem, (const void*)arg, sizeof(VM_VMEMORY)))
					break;

				/* free Lu, Cb, Cr*/
				vmem_free(&vmem.LuMemory);
				vmem_free(&vmem.CbMemory);
				vmem_free(&vmem.CrMemory);
				ret = ERR_VMEM_NO;
			}
			break;

		default:
			ERROUT("Fail, unknown ioctl ...\n");
			return -1;
	}
	DBGOUT("IoCtl (cmd:0x%x, nr:%d, ret:%d) \n\n", cmd, _IOC_NR(cmd), ret);

	return ret;
}

struct file_operations nx_vmem_fops = {
	.owner 			= THIS_MODULE,
	.open 			= nx_vmem_open,
	.release		= nx_vmem_release,
	.unlocked_ioctl	= nx_vmem_ioctl,
};

/*--------------------------------------------------------------------------------
 * Nexell SoC Memory allocator module functions.
 ---------------------------------------------------------------------------------*/
static int init_vmem_node(void)
{
	RESERVEMEMBASE   *n2d_parts = NULL;
	struct list_head *n2d_lhead = NULL;
	struct map_desc   md[NUM_OF_PARTS];
	int i = 0;
	DBGOUT("%s (%d)\n", __func__, NUM_OF_NODES);

 	/* set root node */
	for(i = 0; NUM_OF_NODES > i; i++)	{
		uint phy_base = 0, seg_size = 0, ext_size = 0;

		n2d_parts = &g_n2d_part[i];
		n2d_lhead = &g_n2d_root[i];

		INIT_LIST_HEAD(n2d_lhead);

		KNL_ASSERT(!(n2d_parts->phybase & (SEG_MIN_BYTE - 1)));
		KNL_ASSERT(n2d_parts->length >= 4*1024*1024);

		phy_base = n2d_parts->phybase;
		ext_size = (phy_base + n2d_parts->length - (phy_base & ~(SEG_MAX_BYTE - 1)));

		DBGOUT("[%d] base:0x%08x, size:0x%08x, ext:0x%08x \n",
			i, phy_base, n2d_parts->length, ext_size);

		/* only 1 segment */
		if (phy_base == (phy_base + (n2d_parts->length & ~(SEG_MAX_BYTE - 1))) )
			seg_size = (phy_base + n2d_parts->length - (phy_base & ~(SEG_MAX_BYTE - 1)));
		else
			seg_size = SEG_MAX_BYTE;

		/* create root node each segment */
		while(1) {
			NODE2D * n2d = kmalloc(sizeof(NODE2D), GFP_KERNEL);
			if (! n2d) {
				ERROUT("error, kmalloc for root node[%d], base=0x%x, height=0x%x\n",
					i, phy_base, seg_size/SEG_STRIDE_BYTE);
				return -1;
			}

			n2d->segbase   = (phy_base & ~(SEG_MAX_BYTE - 1));	// align 16Mbyte
			n2d->width     = SEG_STRIDE_BYTE;
			n2d->height    = seg_size/SEG_STRIDE_BYTE;
			n2d->reserve_h = (phy_base - (phy_base & ~(SEG_MAX_BYTE - 1)))/SEG_STRIDE_BYTE;

			DBGOUT("seg base:0x%08x, size:0x%08x, w:%8d, h:%8d, r:%4d \n",
				n2d->segbase, seg_size, n2d->width, n2d->height, n2d->reserve_h);

			/* initialize 2d node */
			__KnlInitializeMCB2D(&n2d->node2d, n2d->width, n2d->height, 0, 0, NULL);

			/* pre-allocate reserved area. */
			if (n2d->reserve_h)
				__KnlMalloc(&n2d->node2d, SEG_STRIDE_BYTE, n2d->reserve_h, 0, 0);

			/* add to root node list */
			list_add(&n2d->list, n2d_lhead);

			/* check next segment */
			ext_size -= seg_size;
			if (0 == ext_size)
				break;

			/* next segment */
			seg_size = (ext_size & ~(SEG_MAX_BYTE - 1)) ? SEG_MAX_BYTE : ext_size;
			phy_base = ((phy_base + SEG_MAX_BYTE) & ~(SEG_MAX_BYTE - 1));
		}
	}

	/* Virtual mapping */
	for(i = 0; NUM_OF_PARTS > i; i++) {
		md[i].virtual = g_n2d_part[i].virbase;
		md[i].pfn	  = __phys_to_pfn(g_n2d_part[i].phybase);
		md[i].length  = g_n2d_part[i].length;
		md[i].type    = MT_MEMORY_NONCACHED;	/* MT_DEVICE, MT_MEMORY_NONCACHED */

		iotable_init(&md[i], i+1);

		printk(KERN_INFO "%s: map[%d]: p 0x%08x -> v 0x%08x len=%dM\n",
			VMEM_DEV_NAME, i, (u_int)(md[i].pfn<<12), (u_int)(md[i].virtual),
			(u_int)(md[i].length/SZ_1M));
	}
	return 0;
}

static int exit_vmem_node(void)
{
	struct list_head *ln2dh = NULL;
	struct list_head *ldata = NULL;
	NODE2D *n2d = NULL;
	int i;
	DBGOUT("%s\n", __func__);

	for( i = 0; NUM_OF_NODES > i; i++ )	{
		DBGOUT("free node 2d\n");
		ln2dh = &g_n2d_root[i];
		list_for_each_prev(ldata, ln2dh) {
			n2d = list_entry(ldata, NODE2D, list);
			if (n2d) {
				DBGOUT("[%d] seg base:0x%08x, w:%8d, h:%8d, r:%4d \n",
					i, n2d->segbase, n2d->width, n2d->height, n2d->reserve_h);
				__KnlFinalizeMCB2D(&n2d->node2d);
				list_del(&n2d->list);
				kfree((void*)n2d);
			}
		}
	}
	return 0;
}

/*
 * cat /sys/devices/platform/vmem.0/vmem
 */
static ssize_t show_vmem_attr(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	DBGOUT("%s:\n", __func__);
	show_vmem_status();
	return 1;
}

static DEVICE_ATTR(vmem, 0666, show_vmem_attr, NULL);

static struct attribute *vmem_attrs[] = {
	&dev_attr_vmem.attr,
	NULL
};

static struct attribute_group vmem_attr_group = {
	.attrs = vmem_attrs,
};

struct class *vm_class;

static int nx_vmem_probe(struct platform_device *pdev)
{
	int ret = -1;
	DBGOUT("%s\n", __func__);

	/* 2d block memory initialze */
	ret = init_vmem_node();
	if (0 > ret)	{
		ERROUT("Error, memory node init ...\n");
		return ret;
	}

	/* register character driver */
	ret = register_chrdev(VMEM_DEV_MAJOR, "Video Memory Allocator", &nx_vmem_fops);
	if (0 > ret)	{
		ERROUT("Fail, register device (%s, major:%d)\n",
			VMEM_DEV_NAME, VMEM_DEV_MAJOR);
		return ret;
	}

	/* register attribute function */
	ret = sysfs_create_group(&pdev->dev.kobj, &vmem_attr_group);
	if (ret) {
		printk(KERN_ERR "Fail, %s create sysfs group ...\n", pdev->name);
		return ret;
	}

	/* for hotplug */
	vm_class = class_create(THIS_MODULE, "vmem");
	device_create(vm_class, NULL, MKDEV(VMEM_DEV_MAJOR, 0), NULL, "vmem");

	/* set device info */
	vm_device = &pdev->dev;
	vm_device->dma_mask = &vm_dmamask;
	vm_device->coherent_dma_mask = 0xffffffff;

	DBGOUT("DONE : %s\n", __func__);
	return ret; /* success */
}

static int nx_vmem_remove(struct platform_device *pdev)
{
	DBGOUT("%s\n", __func__);

	exit_vmem_node();

	sysfs_remove_group(&pdev->dev.kobj, &vmem_attr_group);
	device_destroy(vm_class, MKDEV(VMEM_DEV_MAJOR, 0));
	class_destroy (vm_class);

	unregister_chrdev(VMEM_DEV_MAJOR, "Video Memory Allocator");
	return 0;
}

static struct platform_driver vmem_plat_driver = {
	.probe		= nx_vmem_probe,
	.remove		= nx_vmem_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= VMEM_DEV_NAME,
	},
};

static struct platform_device vmem_plat_device = {
    .name   = VMEM_DEV_NAME,
    .id     = 0,
};

static int __init nx_vmem_init(void)
{
	DBGOUT("%s\n", __func__);
	platform_device_register(&vmem_plat_device);
	return platform_driver_register(&vmem_plat_driver);
}

static void __exit nx_vmem_exit(void)
{
	DBGOUT("%s\n", __func__);
	platform_driver_unregister(&vmem_plat_driver);
}

module_init(nx_vmem_init);
module_exit(nx_vmem_exit);

MODULE_AUTHOR("jhkim <jhkin@nexell.co.kr>");
MODULE_DESCRIPTION("Video Memory Allocator driver for the Nexell");
MODULE_LICENSE("GPL");
