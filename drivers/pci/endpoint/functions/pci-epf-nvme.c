// SPDX-License-Identifier: GPL-2.0
/*
 * NVMe function driver for PCI Endpoint Framework
 *
 * Copyright (C) 2019 SiFive
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/nvme.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <net/sock.h>
#include <generated/utsrelease.h>

#include "../../../nvme/host/nvme.h"
#include "../../../nvme/host/fabrics.h"
#include "../../../nvme/target/nvmet.h"

/*
 * To test the no wait version for submitting commands
 */
#define PCI_EPF_NVME_TEST_CMD_NOWAIT	0

#define CMB_PAGE_SHIFT			12
#define CMB_PAGE_SIZE			(1 << CMB_PAGE_SHIFT)
#define CMB_BIR_MASK			0x7
#define CMB_BIR				2
#define CMBMSC_CRE_BIT			(1 << 0)
#define CMBMSC_CMSE_BIT			(1 << 1)
#define CMBMSC_CBA_MASK			GENMASK(63, 12)

/* Network relay related below */
#define TSP_RELAY_BUFFER_SIZE (SZ_4K - sizeof(u32))
#define TSP_RELAY_MAX_RELAYS 4

struct Buffer4k {
	//struct list_head list;
	struct __attribute__((packed)) pack {
		u32	size;
		char	data[TSP_RELAY_BUFFER_SIZE];
	} pack;
};

struct tsp_relay {
	struct socket		*socket;
	struct task_struct	*thread;
	struct Buffer4k		buffer;
	bool			buffer_full;
	bool			shutdown;
	struct wait_queue_head	wq;
};

struct pci_epf_nvme;

struct tsp_relay_thread_args {
	struct pci_epf_nvme	*nvme;
	int			relay_desc;
};

struct __attribute__((packed)) addr_info {
	char	node[256];
	char	service[256];
	//s32	flags;
	//s32	family;
	//s32	socktype;
	//s32	protocol
};

/* Local Memory Management */
typedef struct LinkedList_t {
    size_t size;
    void *ptr;
    struct LinkedList_t *prev;
    struct LinkedList_t *next;
    int used;
} LinkedList_t;

typedef struct Context_t {
    void *base_address;
    size_t size;
    size_t used;
    LinkedList_t *head;
} Context_t;

#define CONTEXT(ctx) ((Context_t*)ctx)

int tsp_mm_init_context(void *memory_region, size_t memory_region_size, void **opaque_p) {
	Context_t *ctx;

	if (!opaque_p) {
		return -EINVAL;
	}

	*opaque_p = kmalloc(sizeof(Context_t), GFP_USER);
	if (!*opaque_p) {
		return -ENOMEM;
	}

	ctx = CONTEXT(*opaque_p);
	ctx->base_address = memory_region;
	ctx->size = memory_region_size;
	ctx->used = 0;

	ctx->head = kmalloc(sizeof(LinkedList_t), GFP_USER);
	if (!ctx->head) {
		kfree(*opaque_p);
		*opaque_p = NULL;
		return -ENOMEM;
	}

	ctx->head->size = memory_region_size;
	ctx->head->ptr = memory_region;
	ctx->head->next = NULL;
	ctx->head->prev = NULL;
	ctx->head->used = 0;

	return 0;
}

static void *_reserve(size_t size, LinkedList_t* ll, Context_t* ctx) {
	LinkedList_t *next, *new;
	size_t total_size;

	total_size = ll->size;
	next = ll->next;

	// Update linked list
	ll->size = size;

	// Split linked list if not exact allocation
	if (size != total_size) {
		new = kmalloc(sizeof(LinkedList_t), GFP_USER);
		if (!new)
			return NULL;

		ll->next = new;

		new->prev = ll;
		new->size = total_size - size;
		new->ptr = (void *)((char *)ll->ptr + size);
		new->next = next;
		if (next) {
			next->prev = new;
		}
		new->used = 0;
	}

	// Block is now used (reserved)
	ll->used = 1;
	ctx->used += size;
	return ll->ptr;
}

void *tsp_mm_alloc(size_t size, void *opaque) {
	Context_t *ctx;
	LinkedList_t *ll;

	if (!opaque)
		return NULL;
	if (!size)
		return NULL;
	ctx = CONTEXT(opaque);

	ll = ctx->head;
	while(ll) {
		if (!ll->used && (ll->size >= size)) {
			return _reserve(size, ll, ctx);
		} else {
			ll = ll->next;
		}
	}

	return NULL; // Cannot fit
}

static void _try_merge_next(LinkedList_t *ll) {
	LinkedList_t *next;
	next = ll->next;

	if (next && !next->used) {
		// Merge next into current
		ll->size += next->size;
		ll->next = next->next;
		if (next->next) {
			next->next->prev = ll;
		}
		kfree(next); // Block disapears
	}
}

static void _unreserve(LinkedList_t *ll, Context_t *ctx) {
	ll->used = 0;
	ctx->used -= ll->size;

	_try_merge_next(ll);

	// If previous block is unused merge it into current block
	if (ll->prev && !ll->prev->used) {
		_try_merge_next(ll->prev);
		// ll is not necessarily a valid ptr anymore here
	}
}

static void tsp_mm_free(void *ptr, void *opaque) {
	Context_t *ctx;
	LinkedList_t *ll;

	if (!opaque)
		return;

	ctx = CONTEXT(opaque);

	ll = ctx->head;

	// Search entry
	while(ll && ll->ptr != ptr) {
		ll = ll->next;
	}

	if (ll && ll->used) {
		_unreserve(ll, ctx);
		// ll is not necessarily a valid ptr anymore here
	} else {
		// Should not happen
		pr_err("TSP Memory Management Pointer %#llx was not allocated... cannot free\n", (phys_addr_t)ptr);
	}
}

static void mm_quit_context(void *opaque) {
	Context_t *ctx;
	LinkedList_t *ll;

	if (!opaque)
		return;

	ctx = CONTEXT(opaque);

	ll = ctx->head;
	while(ll) {
		LinkedList_t *to_be_freed = ll;
		ll = ll->next;
		kfree(to_be_freed);
	}

	kfree(ctx);
}

#ifndef __TSP_CS_H__
#define __TSP_CS_H__

typedef struct __attribute__((packed)) CSEProperties {
	u16 HwVersion;
	u16 SwVersion;
	char UniqueName[32];     // an identifiable string for this CSE
	u16 NumBuiltinFunctions; // number of available preloaded functions
	u32 MaxRequestsPerBatch; // maximum number of requests supported per batch request
	u32 MaxFunctionParametersAllows;    // maximum number of parameters supported
	u32 MaxConcurrentFunctionInstances; // maximum number of function instances supported
} CSEProperties;

typedef struct __attribute__((packed)) CSxProperties {
	u16 HwVersion;         // specifies the hardware version of this CSx
	u16 SwVersion;         // specifies the software version that runs on this CSx
	u16 VendorId;          // specifies the vendor id of this CSx
	u16 DeviceId;          // specifies the device id of this CSx
	char FriendlyName[32];      // an identifiable string for this CSx
	u32 CFMinMB;           // Amount of CFM in megabytes installed in device
	u32 FDMinMB;           // amount of FDM in megabytes installed in device
	struct __attribute__((packed)) {
		u64 FDMIsDeviceManaged : 1;     // FDM allocations managed by device
		u64 FDMIsHostVisible : 1;       // FDM may be mapped to host address space
		u64 BatchRequestsSupported : 1; // CSx supports batch requests in hardware
		u64 StreamsSupported : 1;       // CSx supports streams in hardware
		u64 Reserved : 60;
	} Flags;
	u16 NumCSEs;
	CSEProperties CSE[1];  // see 6.3.4.1.14
} CSxProperties;

typedef struct __attribute__((packed)) CsCapabilities {
	// specifies the fixed functionality device capability
	struct __attribute__((packed)) {
		u64 Compression : 1;
		u64 Decompression : 1;
		u64 Encryption : 1;
		u64 Decryption : 1;
		u64 RAID : 1;
		u64 EC : 1;
		u64 Dedup : 1;
		u64 Hash : 1;
		u64 Checksum : 1;
		u64 RegEx : 1;
		u64 DbFilter : 1;
		u64 ImageEncode : 1;
		u64 VideoEncode : 1;
		u64 CustomType : 48;
	} Functions;
} CsCapabilities;

typedef CsCapabilities CsFunctionBitSelect;

typedef enum {
	CS_AFDM_TYPE = 1,
	CS_32BIT_VALUE_TYPE = 2,
	CS_64BIT_VALUE_TYPE = 3,
	// https://stackoverflow.com/questions/35380279/avoid-name-collisions-with-enum-in-c-c99
	// Added _ to avoid collision with struct CS_STREAM_TYPE below
	CS_STREAM_TYPE_ = 4,
	CS_DESCRIPTOR_TYPE = 5
} CS_COMPUTE_ARG_TYPE;

typedef struct {
	void *MemHandle;  // an opaque memory handle for AFDM
	unsigned long ByteOffset; // denotes the offset with AFDM
} __attribute__((packed)) CsDevAFDM;

typedef void* CS_STREAM_HANDLE;
typedef s32 CS_CSE_HANDLE;
typedef u32 CS_FUNCTION_ID;

typedef struct {
	CS_COMPUTE_ARG_TYPE Type;
	union {
		CsDevAFDM DevMem;  // see 6.3.4.2.1
		u64 Value64;
		u32 Value32;
		CS_STREAM_HANDLE StreamHandle;
	} u;
} __attribute__((packed)) CsComputeArg;

typedef struct {
	CS_CSE_HANDLE CSEHandle;
	CS_FUNCTION_ID FunctionId;
	int NumArgs;               // set to total arguments to CSF
	CsComputeArg Args[1];      // see 6.3.4.2.6
			       // allocate enough space past this for multiple
			       // arguments
} __attribute__((packed)) CsComputeRequest;

typedef enum {
	TSP_CS_IDENTIFY = 0,
	TSP_CS_GET = 8,
	TSP_CS_ALLOCATE = 16,
	TSP_CS_DEALLOCATE = 17,
	TSP_CS_COMPUTE = 32,
} TSP_CDW10;

typedef enum {
	TSP_CS_CSX = 0,
	TSP_CS_PROPS = 8,
	TSP_CS_CAPS = 16,
	TSP_CS_FUN = 32,
	TSP_CS_MEM = 64,
	TSP_CS_COMM = 64,
	TSP_CS_OPEN_RELAY = 128,
	TSP_CS_CLOSE_RELAY = 129,
} TSP_CDW11;

enum {
	TSP_CHECKSUM_FUNCTION_ID = 42,
	TSP_SLEEP_FUNCTION_ID = 100,
};

typedef struct AsyncComputeRequest {
	struct pci_epf_nvme *nvme;
	struct pci_epf_nvme_cmd *epcmd;
	CsComputeRequest *creq;
	size_t creq_size;
} AsyncComputeRequest;

#endif /* __TSP_CS_H__ */

static CSEProperties tsp_cse_props = {
	.HwVersion = 0x1,
	.SwVersion = 0x1,
	.UniqueName = "CSE Number 1",
	.NumBuiltinFunctions = 1,
	.MaxRequestsPerBatch = 1,
	.MaxFunctionParametersAllows = 4,
	.MaxConcurrentFunctionInstances = 1,
};

static CSxProperties tsp_csx_props = {
	.HwVersion = 0x1,
	.SwVersion = 0x1,
	.VendorId = 0x42,
	.DeviceId = 0x1337,
	.FriendlyName = "Friendly CSX",
	.CFMinMB = 0,
	.FDMinMB = 0,
	.Flags.FDMIsDeviceManaged = 1,
	.Flags.FDMIsHostVisible = 0,
	.Flags.BatchRequestsSupported = 0,
	.Flags.StreamsSupported = 0,
	.Flags.Reserved = 0,
	.NumCSEs = 1,
};

static CsCapabilities tsp_cs_caps = {
	.Functions.Compression = 0,
	.Functions.Decompression = 0,
	.Functions.Encryption = 0,
	.Functions.Decryption = 0,
	.Functions.RAID = 0,
	.Functions.EC = 0,
	.Functions.Dedup = 0,
	.Functions.Hash = 0,
	.Functions.Checksum = 1,
	.Functions.RegEx = 0,
	.Functions.DbFilter = 0,
	.Functions.ImageEncode = 0,
	.Functions.VideoEncode = 0,
	.Functions.CustomType = 0,
};

static const char *TSP_CS_ID_STRING = "This device has compute";

/*
 * Maximum data transfer size: limit to 128 KB to avoid excessive local
 * memory use for buffers.
 */
#define PCI_EPF_NVME_MDTS		(128 * 1024)

/* PRP manipulation macros */
#define pci_epf_nvme_prp_addr(ctrl, prp)	((prp) & ~(ctrl)->mps_mask)
#define pci_epf_nvme_prp_ofst(ctrl, prp)	((prp) & (ctrl)->mps_mask)
#define pci_epf_nvme_prp_size(ctrl, prp)	\
	((size_t)((ctrl)->mps - pci_epf_nvme_prp_ofst(ctrl, prp)))

static struct workqueue_struct *epf_nvme_reg_wq;
static struct workqueue_struct *epf_nvme_sq_wq;
static struct kmem_cache *epf_nvme_cmd_cache;

/*
 * Host PCI memory segment for admin and IO commands.
 */
struct pci_epf_nvme_segment {
	phys_addr_t	pci_addr;
	size_t		size;
};

/*
 * Queue definition and mapping for a local PCI controller.
 */
struct pci_epf_nvme_queue {
	int			ref;

	u16			qid;
	u16			cqid;
	u16			size;
	u16			depth;
	u16			flags;
	u16			vector;
	u16			head;
	u16			tail;
	u16			phase;
	u32			db;

	size_t			qes;

	struct pci_epc_map	map;
};

struct pci_epf_nvme;

/*
 * Local PCI controller exposed with an endpoint function.
 */
struct pci_epf_nvme_ctrl {
	/* Backing fabrics host controller */
	struct nvme_ctrl		*ctrl;

	/* Registers of the local PCI controller */
	void				*reg;
	u64				cap;
	u32				vs;
	u32				cc;
	u32				csts;
	u32				aqa;
	u64				asq;
	u64				acq;
	u32				cmbloc;
	u32				cmbsz;

	size_t				adm_sqes;
	size_t				adm_cqes;
	size_t				io_sqes;
	size_t				io_cqes;

	size_t				mps_shift;
	size_t				mps;
	size_t				mps_mask;

	spinlock_t			qlock;
	unsigned int			nr_queues;
	struct pci_epf_nvme_queue	*sq;
	struct pci_epf_nvme_queue	*cq;
};

/*
 * Command flags.
 */
#define PCI_EPF_NVME_CMD_ASYNC		(1LU << 0)

/*
 * Descriptor for commands sent by the host. This is also used internally for
 * fabrics commands to control our fabrics target.
 */
struct pci_epf_nvme_cmd {
	struct pci_epf_nvme		*epf_nvme;
	unsigned long			flags;

	struct nvme_ns			*ns;

	int				sqid;
	int				cqid;
	unsigned int			status;
	struct nvme_command 		cmd;
	struct nvme_completion		cqe;
	struct completion		done;
	struct list_head		list_node;
	struct task_struct		*thread;

	/* waiting on relay */
	bool				relay;
	int				relay_desc;

	/* Internal buffer that we will transfer over PCI */
	size_t				buffer_size;
	void				*buffer;

	/*
	 * Host PCI adress segments: if nr_segs is 1, we use only "seg",
	 * otherwise, the segs array is allocated and used to store
	 * multiple segments.
	 */
	unsigned int			nr_segs;
	struct pci_epf_nvme_segment	seg;
	struct pci_epf_nvme_segment	*segs;
};

/*
 * EPF function private data representing our NVMe subsystem.
 */
struct pci_epf_nvme {
	struct pci_epf			*epf;
	const struct pci_epc_features	*epc_features;

	void				*reg[PCI_STD_NUM_BARS];
	enum pci_barno			reg_bar;
	size_t				msix_table_offset;

	unsigned int			irq_type;
	unsigned int			nr_vectors;

	__le64				*prp_list_buf;

        bool				dma_supported;
        bool				dma_private;
	struct dma_chan			*dma_chan_tx;
        struct dma_chan			*dma_chan_rx;
        struct dma_chan			*dma_chan;
	struct device			*dma_dev;
        dma_cookie_t			dma_cookie;
        enum dma_status			dma_status;
        struct completion		dma_complete;

	struct delayed_work		reg_poll;
	struct delayed_work		sq_poll;

	unsigned int			max_nr_queues;

	struct list_head		deferred_reqs;

	struct pci_epf_nvme_ctrl	ctrl;

	/* Function configfs attributes */
	struct config_group		group;
	char				*ctrl_opts_buf;
	bool				dma_enable;

	/* Towards Storage Processing (TSP) */
	void				*tsp_alloc_ctx;
	struct tsp_relay		*tsp_relays[TSP_RELAY_MAX_RELAYS];
};

/*
 * Read a 32-bits BAR register (equivalent to readl()).
 */
static inline u32 pci_epf_nvme_reg_read32(struct pci_epf_nvme_ctrl *ctrl,
					  u32 reg)
{
	volatile __le32 *ctrl_reg = ctrl->reg + reg;

	return le32_to_cpu(*ctrl_reg);
}

/*
 * Write a 32-bits BAR register (equivalent to readl()).
 */
static inline void pci_epf_nvme_reg_write32(struct pci_epf_nvme_ctrl *ctrl,
					    u32 reg, u32 val)
{
	volatile __le32 *ctrl_reg = ctrl->reg + reg;

	*ctrl_reg = cpu_to_le32(val);
}

/*
 * Read a 64-bits BAR register (equivalent to lo_hi_readq()).
 */
static inline u64 pci_epf_nvme_reg_read64(struct pci_epf_nvme_ctrl *ctrl,
					  u32 reg)
{
	return (u64)pci_epf_nvme_reg_read32(ctrl, reg) |
		((u64)pci_epf_nvme_reg_read32(ctrl, reg + 4) << 32);
}

/*
 * Write a 64-bits BAR register (equivalent to lo_hi_writeq()).
 */
static inline void pci_epf_nvme_reg_write64(struct pci_epf_nvme_ctrl *ctrl,
					    u32 reg, u64 val)
{
	pci_epf_nvme_reg_write32(ctrl, reg, val & 0xFFFFFFFF);
	pci_epf_nvme_reg_write32(ctrl, reg + 4, (val >> 32) & 0xFFFFFFFF);
}

struct pci_epf_nvme_dma_filter {
        struct device *dev;
        u32 dma_mask;
};

static bool pci_epf_nvme_dma_filter(struct dma_chan *chan, void *arg)
{
        struct pci_epf_nvme_dma_filter *filter = arg;
        struct dma_slave_caps caps;

        memset(&caps, 0, sizeof(caps));
        dma_get_slave_caps(chan, &caps);

        return chan->device->dev == filter->dev &&
               (filter->dma_mask & caps.directions);
}

static bool pci_epf_nvme_init_dma(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf *epf = epf_nvme->epf;
	struct device *dev = &epf->dev;
	struct pci_epf_nvme_dma_filter filter;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	int ret;

	epf_nvme->dma_dev = epf_nvme->epf->epc->dev.parent;
	init_completion(&epf_nvme->dma_complete);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	filter.dev = epf_nvme->dma_dev;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);
	chan = dma_request_channel(mask, pci_epf_nvme_dma_filter, &filter);
	if (!chan)
		goto generic;
	epf_nvme->dma_chan_rx = chan;

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	chan = dma_request_channel(mask, pci_epf_nvme_dma_filter, &filter);
	if (!chan)
		goto release_rx;
	epf_nvme->dma_chan_tx = chan;

	dev_info(dev, "DMA RX channel %s: maximum segment size %d B\n",
		 dma_chan_name(epf_nvme->dma_chan_rx),
		 dma_get_max_seg_size(epf_nvme->dma_chan_rx->device->dev));
	dev_info(dev, "DMA TX channel %s: maximum segment size %d B\n",
		 dma_chan_name(epf_nvme->dma_chan_tx),
		 dma_get_max_seg_size(epf_nvme->dma_chan_tx->device->dev));

	epf_nvme->dma_private = true;

	return true;

release_rx:
	dma_release_channel(epf_nvme->dma_chan_rx);
	epf_nvme->dma_chan_rx = NULL;

generic:
	/* Fallback to a generic memcpy channel if we have one */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(chan)) {
		ret = PTR_ERR(chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get generic DMA channel\n");
		return false;
	}

	dev_info(dev, "Generic DMA channel %s: maximum segment size %d B\n",
		 dma_chan_name(chan),
		 dma_get_max_seg_size(chan->device->dev));

	epf_nvme->dma_chan_tx = chan;
	epf_nvme->dma_chan_rx = chan;

	return true;
}

static void pci_epf_nvme_clean_dma(struct pci_epf_nvme *epf_nvme)
{
	if (!epf_nvme->dma_supported)
		return;

	dma_release_channel(epf_nvme->dma_chan_tx);
	if (epf_nvme->dma_chan_rx != epf_nvme->dma_chan_tx)
		dma_release_channel(epf_nvme->dma_chan_rx);

	epf_nvme->dma_chan_tx = NULL;
	epf_nvme->dma_chan_rx = NULL;
	epf_nvme->dma_chan = NULL;
	epf_nvme->dma_supported = false;
}

static void pci_epf_nvme_dma_callback(void *param)
{
        struct pci_epf_nvme *epf_nvme = param;
        struct dma_tx_state state;
	enum dma_status status;

        status = dmaengine_tx_status(epf_nvme->dma_chan,
				     epf_nvme->dma_cookie, &state);
	if (status == DMA_COMPLETE || status == DMA_ERROR) {
		epf_nvme->dma_status = status;
		complete(&epf_nvme->dma_complete);
	}
}

static int pci_epf_nvme_dma(struct pci_epf_nvme *epf_nvme,
			    dma_addr_t dma_dst, dma_addr_t dma_src,
			    size_t len, dma_addr_t dma_remote,
			    enum dma_transfer_direction dir)
{
	struct dma_async_tx_descriptor *tx;
	struct dma_slave_config sconf = {};
	unsigned long time_left;
	struct dma_chan *chan;
	dma_addr_t dma_local;
	int ret;

	if (dir == DMA_DEV_TO_MEM) {
		chan = epf_nvme->dma_chan_tx;
		dma_local = dma_dst;
	} else {
		dma_local = dma_src;
		chan = epf_nvme->dma_chan_rx;
	}
	if (IS_ERR_OR_NULL(chan)) {
		dev_err(&epf_nvme->epf->dev, "Invalid DMA channel\n");
		return -EINVAL;
	}

	if (epf_nvme->dma_private) {
		sconf.direction = dir;
		if (dir == DMA_MEM_TO_DEV)
			sconf.dst_addr = dma_remote;
		else
			sconf.src_addr = dma_remote;

		if (dmaengine_slave_config(chan, &sconf)) {
			dev_err(&epf_nvme->epf->dev,
				"DMA slave config failed\n");
			return -EIO;
		}

		tx = dmaengine_prep_slave_single(chan, dma_local, len, dir,
					DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	} else {
		tx = dmaengine_prep_dma_memcpy(chan, dma_dst, dma_src, len,
					DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	}
	if (!tx) {
		dev_err(&epf_nvme->epf->dev, "Prepare DMA memcpy failed\n");
		return -EIO;
	}

	reinit_completion(&epf_nvme->dma_complete);
	epf_nvme->dma_chan = chan;
	tx->callback = pci_epf_nvme_dma_callback;
	tx->callback_param = epf_nvme;
	epf_nvme->dma_cookie = dmaengine_submit(tx);

	ret = dma_submit_error(epf_nvme->dma_cookie);
	if (ret) {
		dev_err(&epf_nvme->epf->dev, "DMA tx_submit failed %d\n", ret);
		goto terminate;
	}

	dma_async_issue_pending(chan);

	time_left = wait_for_completion_timeout(&epf_nvme->dma_complete,
						HZ * 10);
	if (!time_left) {
		dev_err(&epf_nvme->epf->dev, "DMA transfer timeout\n");
		ret = -ETIMEDOUT;
		goto terminate;
	}

	if (epf_nvme->dma_status != DMA_COMPLETE) {
		dev_err(&epf_nvme->epf->dev, "DMA transfer failed\n");
		ret = -EIO;
	}

terminate:
	if (ret)
		dmaengine_terminate_sync(chan);

	return ret;
}

static int pci_epf_nvme_dma_transfer(struct pci_epf_nvme *epf_nvme,
				     struct pci_epc_map *map,
				     void *buf, enum dma_data_direction dir)
{
	phys_addr_t dma_phys_addr;
	int ret;

	dma_phys_addr = dma_map_single(epf_nvme->dma_dev, buf, map->size, dir);
	if (dma_mapping_error(epf_nvme->dma_dev, dma_phys_addr)) {
		dev_err(&epf_nvme->epf->dev,
			"Failed to map source buffer addr\n");
		return -ENOMEM;
	}

	switch (dir) {
	case DMA_FROM_DEVICE:
		ret = pci_epf_nvme_dma(epf_nvme, dma_phys_addr, map->phys_addr,
				       map->size, map->pci_addr,
				       DMA_DEV_TO_MEM);
		break;
	case DMA_TO_DEVICE:
		ret = pci_epf_nvme_dma(epf_nvme, map->phys_addr, dma_phys_addr,
				       map->size, map->pci_addr,
				       DMA_MEM_TO_DEV);
		break;
	default:
		ret = -EINVAL;
	}

	dma_unmap_single(epf_nvme->dma_dev, dma_phys_addr, map->size, dir);

	return ret;
}

static int pci_epf_nvme_mmio_transfer(struct pci_epf_nvme *nvme,
				      struct pci_epc_map *map,
				      void *buf, enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_FROM_DEVICE:
		memcpy_fromio(buf, map->virt_addr, map->size);
		return 0;
	case DMA_TO_DEVICE:
		memcpy_toio(map->virt_addr, buf, map->size);
		return 0;
	default:
		return -EINVAL;
	}
}

static void pci_epf_nvme_print_cmb_info(struct pci_epf_nvme *epf_nvme)
{
	u64 cmbmsc;
	phys_addr_t cba;
	const size_t CMB_SIZE = epf_nvme->epf->bar[CMB_BIR].size;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;

	dev_info(&epf_nvme->epf->dev, "CMB phys addr: %#0llx end: %#0llx CMSE %sabled CRE %sabled\n",
		cba, cba + CMB_SIZE, ((cmbmsc & CMBMSC_CMSE_BIT) ? "en" : "dis"),
		((cmbmsc & CMBMSC_CRE_BIT) ? "en" : "dis"));
}

static int pci_epf_nvme_addr_in_cmb(struct pci_epf_nvme *epf_nvme,
				    phys_addr_t addr)
{
	u64 cmbmsc;
	phys_addr_t cba;
	const size_t CMB_SIZE = epf_nvme->epf->bar[CMB_BIR].size;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;

	/* If Controller Memory Space Enabled and Capabilities Register Enabled */
	if (!!(cmbmsc & CMBMSC_CMSE_BIT) && !!(cmbmsc & CMBMSC_CRE_BIT)) {
		/* If the PCI addr is in the CMB */
		if (addr >= cba && addr < (cba + CMB_SIZE)) {
			return true;
		}
	}
	return false;
}

/* The address must be in the CMB, there is no check here */
static void *pci_epf_nvme_cmb_addr_as_local_addr(struct pci_epf_nvme *epf_nvme,
						 phys_addr_t addr)
{
	u64 cmbmsc;
	phys_addr_t cba, offset;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;
	offset = addr - cba;

	return (char*)(epf_nvme->reg[CMB_BIR]) + offset;
}

static int pci_epf_nvme_segment_in_cmb(struct pci_epf_nvme *epf_nvme,
				       struct pci_epf_nvme_segment *seg)
{
	return pci_epf_nvme_addr_in_cmb(epf_nvme, seg->pci_addr);
}

static int pci_epf_nvme_map_cmb_segment(struct pci_epf_nvme *epf_nvme,
					struct pci_epf_nvme_segment *seg,
					struct pci_epc_map *map)
{
	u64 cmbmsc;
	phys_addr_t cba, offset;
	const size_t CMB_SIZE = epf_nvme->epf->bar[CMB_BIR].size;

	if (!seg | !map | !seg->size)
		return -EINVAL;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;
	offset = seg->pci_addr - cba;

	if (seg->pci_addr < cba ||
	    seg->pci_addr > (cba + CMB_SIZE)) {
		dev_err(&epf_nvme->epf->dev, "Segment not in CMB");
		return -EINVAL;
	}

	if (seg->pci_addr + seg->size > cba + CMB_SIZE) {
		dev_err(&epf_nvme->epf->dev, "Segment overflows CMB\n");
		return -ENOMEM;
	}

	map->pci_addr = seg->pci_addr;
	map->phys_size = seg->size;
	map->phys_addr = epf_nvme->epf->bar[CMB_BIR].phys_addr + offset;
	map->virt_addr = (char*)(epf_nvme->reg[CMB_BIR]) + offset;

	return 0;
}

static int pci_epf_nvme_transfer(struct pci_epf_nvme *epf_nvme,
				 struct pci_epf_nvme_segment *seg,
				 enum dma_data_direction dir, void *buf)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf *epf = epf_nvme->epf;
	phys_addr_t addr = seg->pci_addr;
	size_t size = seg->size;
	struct pci_epc_map map;
	ssize_t map_size;
	bool in_cmb = pci_epf_nvme_segment_in_cmb(epf_nvme, seg);
	int ret;

	while (size) {

		/* If we have to read/write our own CMB we do not do it through
		 * PCI so the map is different based on CMB/PCI */
		if (in_cmb) {
			ret = pci_epf_nvme_map_cmb_segment(epf_nvme, seg, &map);
			if (ret)
				return ret;
			map_size = seg->size;
		} else {
			map_size = pci_epf_mem_map(epf, addr, size, &map);
			if (map_size < 0)
				return map_size;
		}

		/* Do not bother with DMA for small transfers */
		if (!epf_nvme->dma_enable || map.size < ctrl->mps)
			ret = pci_epf_nvme_mmio_transfer(epf_nvme, &map,
							 buf, dir);
		else
			ret = pci_epf_nvme_dma_transfer(epf_nvme, &map,
							buf, dir);

		if (!in_cmb)
			pci_epf_mem_unmap(epf, &map);

		if (ret)
			return ret;

		size -= map_size;
		addr += map_size;
		buf += map_size;
	}

	return 0;
}

static inline struct pci_epf_nvme_cmd *
pci_epf_nvme_alloc_cmd(struct pci_epf_nvme *nvme)
{
	return kmem_cache_alloc(epf_nvme_cmd_cache, GFP_KERNEL);
}

static void pci_epf_nvme_init_cmd(struct pci_epf_nvme *epf_nvme,
				  struct pci_epf_nvme_cmd *epcmd,
				  int sqid, int cqid)
{
	memset(epcmd, 0, sizeof(*epcmd));
	epcmd->epf_nvme = epf_nvme;
	epcmd->sqid = sqid;
	epcmd->cqid = cqid;
	epcmd->status = NVME_SC_SUCCESS;
	init_completion(&epcmd->done);
	/* The assignments below could be omitted because of memset 0 above */
	epcmd->thread = NULL;
	epcmd->relay = false;
	epcmd->relay_desc = 0;
}

static int pci_epf_nvme_alloc_cmd_buffer(struct pci_epf_nvme_cmd *epcmd,
					 size_t size)
{
	void *buffer;

	buffer = kvzalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	epcmd->buffer = buffer;
	epcmd->buffer_size = size;

	return 0;
}

static int pci_epf_nvme_alloc_cmd_segs(struct pci_epf_nvme_cmd *epcmd,
				       int nr_segs)
{
	struct pci_epf_nvme_segment *segs;

	/* Single map case: use the command map structure */
	if (nr_segs == 1) {
		epcmd->segs = &epcmd->seg;
		epcmd->nr_segs = 1;
		return 0;
	}

	/* More than one map needed: allocate an array */
	segs = kcalloc(nr_segs, sizeof(struct pci_epf_nvme_segment), GFP_KERNEL);
	if (!segs)
		return -ENOMEM;

	epcmd->nr_segs = nr_segs;
	epcmd->segs = segs;

	return 0;
}

static void pci_epf_nvme_free_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	if (epcmd->ns)
		nvme_put_ns(epcmd->ns);

	if (epcmd->buffer)
		kfree(epcmd->buffer);

	if (epcmd->segs && epcmd->segs != &epcmd->seg)
		kfree(epcmd->segs);

	kmem_cache_free(epf_nvme_cmd_cache, epcmd);
}

static const char *pci_epf_nvme_cmd_name(struct pci_epf_nvme_cmd *epcmd)
{
	u8 opcode = epcmd->cmd.common.opcode;

	if (epcmd->sqid)
		return nvme_get_opcode_str(opcode);
	return nvme_get_admin_opcode_str(opcode);
}

static int pci_epf_nvme_cmd_transfer(struct pci_epf_nvme *epf_nvme,
				     struct pci_epf_nvme_cmd *epcmd,
				     enum dma_data_direction dir)
{
	struct pci_epf_nvme_segment *seg;
	void *buf = epcmd->buffer;
	size_t size = 0;
	int i, ret;

	/* Do nothing for commands already marked as failed */
	if (epcmd->status != NVME_SC_SUCCESS)
		return -EIO;

	/* Go through the command segments and transfer each one */
	for (i = 0; i < epcmd->nr_segs; i++) {
		seg = &epcmd->segs[i];

		if (size >= epcmd->buffer_size) {
			dev_err(&epf_nvme->epf->dev, "Invalid transfer size\n");
			goto xfer_err;
		}

		ret = pci_epf_nvme_transfer(epf_nvme, seg, dir, buf);
		if (ret)
			goto xfer_err;

		buf += seg->size;
		size += seg->size;
	}

	return 0;

xfer_err:
	epcmd->status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
	return -EIO;
}

static void pci_epf_nvme_raise_irq(struct pci_epf_nvme *epf_nvme,
				   struct pci_epf_nvme_queue *cq)
{
	struct pci_epf *epf = epf_nvme->epf;
	int ret;

	if (!(cq->flags & NVME_CQ_IRQ_ENABLED))
		return;

	switch (epf_nvme->irq_type) {
	case PCI_IRQ_MSIX:
	case PCI_IRQ_MSI:
		ret = pci_epf_raise_irq(epf, epf_nvme->irq_type,
					cq->vector + 1);
		if (!ret)
			return;
		/*
		 * If we got an error, it is likely because the host is using
		 * legacy IRQs (e.g. BIOS, grub), so fallthrough.
		 */
		fallthrough;
	case PCI_IRQ_LEGACY:
		ret = pci_epf_raise_irq(epf, PCI_IRQ_LEGACY, 0);
		if (!ret)
			return;
		break;
	default:
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		break;
	}

	if (ret)
		dev_err(&epf->dev, "Raise IRQ failed %d\n", ret);
}

static inline bool pci_epf_nvme_ctrl_ready(struct pci_epf_nvme_ctrl *ctrl)
{
	return (ctrl->cc & NVME_CC_ENABLE) && (ctrl->csts & NVME_CSTS_RDY);
}

static void pci_epf_nvme_queue_response(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf *epf = epf_nvme->epf;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[epcmd->sqid];
	struct pci_epf_nvme_queue *cq = &ctrl->cq[epcmd->cqid];
	struct nvme_completion *cqe = &epcmd->cqe;
	unsigned long flags;

	/*
	 * Do not try to complete commands if the controller is not ready
	 * anymore, e.g. after the host cleared CC.EN.
	 */
	if (!pci_epf_nvme_ctrl_ready(ctrl))
		goto free;

	spin_lock_irqsave(&ctrl->qlock, flags);

	/* XXX Check completion queue full state XXX */
	cq->head = pci_epf_nvme_reg_read32(ctrl, cq->db);

	/* Setup the completion entry */
	cqe->sq_id = cpu_to_le16(epcmd->sqid);
	cqe->sq_head = cpu_to_le16(sq->head);
	cqe->command_id = epcmd->cmd.common.command_id;
	cqe->status = cpu_to_le16((epcmd->status << 1) | cq->phase);

	/* Post the completion entry */
	dev_dbg(&epf->dev, "cq[%d]: status 0x%x, phase %d, tail %d -> %d/%d\n",
		epcmd->cqid, epcmd->status, cq->phase, cq->tail,
		(int)cq->tail, (int)cq->depth);

	memcpy_toio(cq->map.virt_addr + cq->tail * cq->qes, cqe,
		    sizeof(struct nvme_completion));

	/* Advance cq tail */
	cq->tail++;
	if (cq->tail >= cq->depth) {
		cq->tail = 0;
		cq->phase ^= 1;
	}

	spin_unlock_irqrestore(&ctrl->qlock, flags);

	pci_epf_nvme_raise_irq(epf_nvme, cq);

free:
	pci_epf_nvme_free_cmd(epcmd);
}

static struct pci_epf_nvme_cmd *
pci_epf_nvme_fetch_cmd(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[qid];
	struct pci_epf_nvme_cmd *epcmd;
	unsigned long flags;

	if (!sq->size)
		return NULL;

	sq->tail = pci_epf_nvme_reg_read32(ctrl, sq->db);
	if (sq->tail == sq->head) {
		/* Queue empty */
		return NULL;
	}

	epcmd = pci_epf_nvme_alloc_cmd(epf_nvme);
	if (!epcmd)
		return NULL;

	/* Get the NVMe command submitted by the host */
	pci_epf_nvme_init_cmd(epf_nvme, epcmd, sq->qid, sq->cqid);
	memcpy_fromio(&epcmd->cmd, sq->map.virt_addr + sq->head * sq->qes,
		      sizeof(struct nvme_command));

	dev_dbg(&epf_nvme->epf->dev,
		"sq[%d]: head %d/%d, tail %d, command %s\n",
		qid, (int)sq->head, (int)sq->depth, (int)sq->tail,
		pci_epf_nvme_cmd_name(epcmd));

	spin_lock_irqsave(&ctrl->qlock, flags);
	sq->head++;
	if (sq->head == sq->depth)
		sq->head = 0;
	spin_unlock_irqrestore(&ctrl->qlock, flags);

	return epcmd;
}

/*
 * Transfer a prp list from the host and return the number of prps.
 */
static int pci_epf_nvme_get_prp_list(struct pci_epf_nvme *epf_nvme, u64 prp,
				     size_t xfer_len)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	size_t nr_prps = (xfer_len + ctrl->mps_mask) >> ctrl->mps_shift;
	struct pci_epf_nvme_segment seg;
	int ret;

	/*
	 * Compute the number of PRPs required for the number of bytes to
	 * transfer (xfer_len). If this number overflows the memory page size
	 * with the PRP list pointer specified, only return the space available
	 * in the memory page, the last PRP in there will be a PRP list pointer
	 * to the remaining PRPs.
	 */
	seg.pci_addr = prp;
	seg.size = min(pci_epf_nvme_prp_size(ctrl, prp), nr_prps << 3);
	ret = pci_epf_nvme_transfer(epf_nvme, &seg, DMA_FROM_DEVICE,
				    epf_nvme->prp_list_buf);
	if (ret)
		return ret;

	return seg.size >> 3;
}

static int pci_epf_nvme_cmd_parse_prp_list(struct pci_epf_nvme *epf_nvme,
					   struct pci_epf_nvme_cmd *epcmd,
					   size_t transfer_len)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	__le64 *prps = epf_nvme->prp_list_buf;
	struct pci_epf_nvme_segment *seg;
	size_t size = 0, ofst, prp_size, xfer_len;
	int nr_segs, nr_prps = 0;
	phys_addr_t pci_addr;
	int i = 0, ret;
	u64 prp;

	/*
	 * Allocate segments for the command: this considers the worst case
	 * scenario where all prps are discontiguous, so get as many segments
	 * as we can have prps. In practice, most of the time, we will have
	 * far less segments than prps.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp1);
	if (!prp)
		goto invalid_field;

	nr_segs = (transfer_len + ofst + NVME_CTRL_PAGE_SIZE - 1)
		>> NVME_CTRL_PAGE_SHIFT;

	ret = pci_epf_nvme_alloc_cmd_segs(epcmd, nr_segs);
	if (ret)
		goto internal;

	/* Set the first segment using prp1 */
	seg = &epcmd->segs[0];
	seg->pci_addr = prp;
	seg->size = pci_epf_nvme_prp_size(ctrl, prp);
	ofst = pci_epf_nvme_prp_ofst(ctrl, prp);

	size = seg->size;
	pci_addr = prp + size;
	nr_segs = 1;

	/*
	 * Now build the pci address segments using the prp lists, starting
	 * from prp2.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp2);
	if (!prp)
		goto invalid_field;

	while (size < transfer_len) {
		xfer_len = transfer_len - size;

		if (!nr_prps) {
			/* Get the prp list */
			nr_prps = pci_epf_nvme_get_prp_list(epf_nvme, prp,
							    xfer_len);
			if (nr_prps < 0)
				goto internal;

			i = 0;
			ofst = 0;
		}

		/* Current entry */
		prp = le64_to_cpu(prps[i]);
		if (!prp)
			goto invalid_field;

		/* Did we reach the last prp entry of the list ? */
		if (xfer_len > ctrl->mps && i == nr_prps - 1) {
			/* We need more PRPs: prp is a list pointer */
			nr_prps = 0;
			continue;
		}

		/* Only the first prp is allowed to have an offset */
		if (pci_epf_nvme_prp_ofst(ctrl, prp))
			goto invalid_offset;

		if (prp != pci_addr) {
			/* Discontiguous prp: new segment */
			nr_segs++;
			if (WARN_ON_ONCE(nr_segs > epcmd->nr_segs))
				goto internal;

			seg++;
			seg->pci_addr = prp;
			seg->size = 0;
			pci_addr = prp;
		}

		prp_size = min_t(size_t, ctrl->mps, xfer_len);
		seg->size += prp_size;
		pci_addr += prp_size;
		size += prp_size;

		i++;
	}

	epcmd->nr_segs = nr_segs;
	ret = 0;

	if (size != transfer_len) {
		dev_err(&epf_nvme->epf->dev,
			"PRPs transfer length mismatch %zu / %zu\n",
			size, transfer_len);
		goto internal;
	}

	return 0;

internal:
	epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	return -EINVAL;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;
}

static int pci_epf_nvme_cmd_parse_prp_simple(struct pci_epf_nvme *epf_nvme,
					     struct pci_epf_nvme_cmd *epcmd,
					     size_t transfer_len)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	int ret, nr_segs = 1;
	u64 prp1, prp2 = 0;
	size_t prp1_size;

	/* prp1 */
	prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	prp1_size = pci_epf_nvme_prp_size(ctrl, prp1);

	/* For commands crossing a page boundary, we should have a valid prp2 */
	if (transfer_len > prp1_size) {
		prp2 = le64_to_cpu(cmd->common.dptr.prp2);
		if (!prp2)
			goto invalid_field;
		if (pci_epf_nvme_prp_ofst(ctrl, prp2))
			goto invalid_offset;
		if (prp2 != prp1 + prp1_size)
			nr_segs = 2;
	}

	/* Create segments using the prps */
	ret = pci_epf_nvme_alloc_cmd_segs(epcmd, nr_segs);
	if (ret )
		goto internal;

	epcmd->segs[0].pci_addr = prp1;
	if (nr_segs == 1) {
		epcmd->segs[0].size = transfer_len;
	} else {
		epcmd->segs[0].size = prp1_size;
		epcmd->segs[1].pci_addr = prp2;
		epcmd->segs[1].size = transfer_len - prp1_size;
	}

	return 0;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;

internal:
	epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	return ret;
}

static int pci_epf_nvme_cmd_parse_dptr(struct pci_epf_nvme *epf_nvme,
				       struct pci_epf_nvme_cmd *epcmd,
				       size_t transfer_len)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	u64 prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	size_t ofst;
	int ret;

	if (transfer_len > PCI_EPF_NVME_MDTS)
		goto invalid_field;

	/* We do not support SGL for now */
	if (epcmd->cmd.common.flags & NVME_CMD_SGL_ALL)
		goto invalid_field;

	/* Get pci segments for the command using its prps */
	ofst = pci_epf_nvme_prp_ofst(ctrl, prp1);
	if (ofst & 0x3)
		goto invalid_offset;

	if (transfer_len + ofst <= NVME_CTRL_PAGE_SIZE * 2)
		ret = pci_epf_nvme_cmd_parse_prp_simple(epf_nvme, epcmd,
						        transfer_len);
	else
		ret = pci_epf_nvme_cmd_parse_prp_list(epf_nvme, epcmd,
						      transfer_len);
	if (ret)
		return ret;

	/* Get an internal buffer for the command */
	ret = pci_epf_nvme_alloc_cmd_buffer(epcmd, transfer_len);
	if (ret) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		return ret;
	}

	return 0;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;
}

static void pci_epf_nvme_unmap_cq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *cq = &epf_nvme->ctrl.cq[qid];

	if (cq->ref < 1)
		return;

	cq->ref--;
	if (cq->ref)
		return;

	pci_epf_mem_unmap(epf_nvme->epf, &cq->map);
	memset(cq, 0, sizeof(*cq));
}

static int pci_epf_nvme_map_cq(struct pci_epf_nvme *epf_nvme, int qid,
			       int flags, int size, int vector,
			       phys_addr_t pci_addr)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *cq = &ctrl->cq[qid];
	struct pci_epf *epf = epf_nvme->epf;
	size_t qsize;
	ssize_t ret;

	/*
	 * Increment the queue ref count: if the queue was already mapped,
	 * we have nothing to do.
	 */
	cq->ref++;
	if (cq->ref > 1)
		return 0;

	/* Setup and map the completion queue */
	cq->qid = qid;
	cq->size = size;
	cq->flags = flags;
	cq->depth = size + 1;
	cq->vector = vector;
	cq->phase = 1;
	cq->db = NVME_REG_DBS + (((qid * 2) + 1) * sizeof(u32));
	pci_epf_nvme_reg_write32(ctrl, cq->db, 0);

	if (!qid)
		cq->qes = ctrl->adm_cqes;
	else
		cq->qes = ctrl->io_cqes;
	qsize = cq->qes * cq->depth;

	ret = pci_epf_mem_map(epf, pci_addr, qsize, &cq->map);
	if (ret != qsize) {
		if (ret > 0) {
			dev_err(&epf->dev, "Partial CQ %d mapping\n", qid);
			ret = -ENOMEM;
		} else {
			dev_err(&epf->dev, "Map CQ %d failed\n", qid);
		}
		memset(cq, 0, sizeof(*cq));
		return ret;
	}

	dev_dbg(&epf->dev,
		"CQ %d: PCI addr 0x%llx, virt addr 0x%llx, size %zu B\n",
		qid, cq->map.pci_addr, (u64)cq->map.virt_addr, qsize);
	dev_dbg(&epf->dev,
		"CQ %d: %d entries of %zu B, vector IRQ %d\n",
		qid, cq->size, cq->qes, (int)cq->vector + 1);

	return 0;
}

static void pci_epf_nvme_unmap_sq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *sq = &epf_nvme->ctrl.sq[qid];

	if (!sq->ref)
		return;

	if (WARN_ON_ONCE(sq->ref != 1))
		return;

	WARN_ON_ONCE(epf_nvme->ctrl.cq[sq->cqid].ref < 1);
	epf_nvme->ctrl.cq[sq->cqid].ref--;

	pci_epf_mem_unmap(epf_nvme->epf, &sq->map);
	memset(sq, 0, sizeof(*sq));
}

static int pci_epf_nvme_map_sq(struct pci_epf_nvme *epf_nvme, int qid,
			       int cqid, int flags, int size,
			       phys_addr_t pci_addr)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[qid];
	struct pci_epf *epf = epf_nvme->epf;
	size_t qsize;
	ssize_t ret;

	/* Setup and map the submission queue */
	sq->ref = 1;
	sq->qid = qid;
	sq->cqid = cqid;
	sq->size = size;
	sq->flags = flags;
	sq->depth = size + 1;
	sq->db = NVME_REG_DBS + (qid * 2 * sizeof(u32));
	pci_epf_nvme_reg_write32(ctrl, sq->db, 0);

	if (!qid)
		sq->qes = ctrl->adm_sqes;
	else
		sq->qes = ctrl->io_sqes;
	qsize = sq->qes * sq->depth;

	ret = pci_epf_mem_map(epf, pci_addr, qsize, &sq->map);
	if (ret != qsize) {
		if (ret > 0) {
			dev_err(&epf->dev, "Partial SQ %d mapping\n", qid);
			ret = -ENOMEM;
		} else {
			dev_err(&epf->dev, "Map SQ %d failed\n", qid);
		}
		memset(sq, 0, sizeof(*sq));
		return ret;
	}

	/* Get a reference on the completion queue */
	epf_nvme->ctrl.cq[cqid].ref++;

	dev_dbg(&epf->dev,
		"SQ %d: PCI addr 0x%llx, virt addr 0x%llx, size %zu B\n",
		qid, sq->map.pci_addr, (u64)sq->map.virt_addr, qsize);
	dev_dbg(&epf->dev,
		"SQ %d: %d queue entries of %zu B, CQ %d\n",
		qid, size, sq->qes, cqid);

	return 0;
}

static void pci_epf_nvme_drain_sq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *sq = &epf_nvme->ctrl.sq[qid];
	struct pci_epf_nvme_cmd *epcmd;

	if (!sq->size)
		return;

	while((epcmd = pci_epf_nvme_fetch_cmd(epf_nvme, qid))) {
		epcmd->status = NVME_SC_ABORT_QUEUE | NVME_SC_DNR;
		pci_epf_nvme_queue_response(epcmd);
	}
}

static void pci_epf_nvme_init_ctrl_regs(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;

	ctrl->reg = epf_nvme->reg[epf_nvme->reg_bar];

	/* Copy the fabrics controller capabilities as a base */
	ctrl->cap = ctrl->ctrl->cap;

	/* Contiguous Queues Required (CQR) */
	ctrl->cap |= 0x1ULL << 16;

	/* Set Doorbell stride to 4B (DSTRB) */
	ctrl->cap &= ~GENMASK(35, 32);

	/* Clear NVM Subsystem Reset Supported (NSSRS) */
	ctrl->cap &= ~(0x1ULL << 36);

	/* Clear Boot Partition Support (BPS) */
	ctrl->cap &= ~(0x1ULL << 45);

	/* Memory Page Size minimum (MPSMIN) = 4K */
	ctrl->cap |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Memory Page Size maximum (MPSMAX) = 4K */
	ctrl->cap |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Clear Persistent Memory Region Supported (PMRS) */
	ctrl->cap &= ~(0x1ULL << 56);

	/* Clear Controller Memory Buffer Supported (CMBS) */
	//ctrl->cap &= ~(0x1ULL << 57);

	/* Controller Memory Buffer Supported (CMBS) */
	ctrl->cap |= (0x1ULL << 57);
	ctrl->cmbloc = CMB_BIR;
	/* Controller Memory Buffer Size (CMB page 4kB when SZU set to 0) */
	ctrl->cmbsz = (epf_nvme->epf->bar[CMB_BIR].size >> CMB_PAGE_SHIFT) << 12;
	ctrl->cmbsz |= 1 << 4; /* WDS */
	ctrl->cmbsz |= 1 << 3; /* RDS */
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CMBLOC, ctrl->cmbloc);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CMBSZ, ctrl->cmbsz);

	/* NVMe version supported */
	ctrl->vs = ctrl->ctrl->vs;

	/* Controller configuration */
	ctrl->cc = ctrl->ctrl->ctrl_config & (~NVME_CC_ENABLE);

	/* Controller Status (not ready) */
	ctrl->csts = 0;

	pci_epf_nvme_reg_write64(ctrl, NVME_REG_CAP, ctrl->cap);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_VS, ctrl->vs);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CC, ctrl->cc);

	return;
}

static void pci_epf_nvme_delete_ctrl(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;

	if (ctrl->ctrl) {
		nvme_delete_ctrl(ctrl->ctrl);
		ctrl->ctrl = NULL;
	}

	ctrl->nr_queues = 0;
	kfree(ctrl->cq);
	ctrl->cq = NULL;
	kfree(ctrl->sq);
	ctrl->sq = NULL;
}

static int pci_epf_nvme_create_ctrl(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct device *dev = &epf->dev;
	struct nvme_ctrl *nctrl;
	int ret;

	/* We must have nvme fabrics options. */
	if (!epf_nvme->ctrl_opts_buf) {
		dev_err(dev, "No nvme options specified\n");
		return ret;
	}

	memset(ctrl, 0, sizeof(*ctrl));
	spin_lock_init(&ctrl->qlock);

	nctrl = nvmf_create_ctrl(dev, epf_nvme->ctrl_opts_buf);
	if (IS_ERR(nctrl)) {
		dev_err(dev, "Create nvme controller failed\n");
		return PTR_ERR(nctrl);
	}

	/* Only support IO controllers */
	if (nctrl->cntrltype != NVME_CTRL_IO) {
		dev_err(dev, "Unsupported controller type\n");
		ret = -EINVAL;
		goto out_delete_ctrl;
	}

	dev_info(dev, "NVMe controller created, %u I/O queues\n",
		 nctrl->queue_count - 1);

	/* Allocate queues */
	ctrl->nr_queues = nctrl->queue_count;
	if (ctrl->nr_queues > epf_nvme->max_nr_queues) {
		dev_err(dev, "Too many queues (maximum allowed: %u)\n",
			epf_nvme->max_nr_queues);
		ret = -EINVAL;
		goto out_delete_ctrl;
	}

	ctrl->sq = kcalloc(ctrl->nr_queues, sizeof(struct pci_epf_nvme_queue),
			   GFP_KERNEL);
	if (!ctrl->sq) {
		ret = -ENOMEM;
		goto out_delete_ctrl;
	}

	ctrl->cq = kcalloc(ctrl->nr_queues, sizeof(struct pci_epf_nvme_queue),
			   GFP_KERNEL);
	if (!ctrl->cq) {
		ret = -ENOMEM;
		goto out_delete_ctrl;
	}

	epf_nvme->ctrl.ctrl = nctrl;

	pci_epf_nvme_init_ctrl_regs(epf_nvme);

	return 0;

out_delete_ctrl:
	pci_epf_nvme_delete_ctrl(epf);

	return ret;
}

static void pci_epf_nvme_disable_ctrl(struct pci_epf_nvme *epf_nvme,
				      bool shutdown)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf *epf = epf_nvme->epf;
	int qid;

	dev_info(&epf->dev, "%s controller\n",
		 shutdown ? "Shutting down" : "Disabling");

	/* Stop polling the submission queues */
	cancel_delayed_work_sync(&epf_nvme->sq_poll);

	for (qid = ctrl->nr_queues - 1; qid >= 0; qid--)
		pci_epf_nvme_drain_sq(epf_nvme, qid);

	/*
	 * Unmap the submission queues first to release all references
	 * on the completion queues.
	 */
	for (qid = ctrl->nr_queues - 1; qid >= 0; qid--)
		pci_epf_nvme_unmap_sq(epf_nvme, qid);

	for (qid = ctrl->nr_queues - 1; qid >= 0; qid--)
		pci_epf_nvme_unmap_cq(epf_nvme, qid);

	if (shutdown) {
		pci_epf_nvme_delete_ctrl(epf);
		ctrl->cc &= ~NVME_CC_SHN_NORMAL;
		ctrl->csts |= NVME_CSTS_SHST_CMPLT;
	}

	/* Tell the host we are done */
	ctrl->csts &= ~NVME_CSTS_RDY;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	ctrl->cc &= ~NVME_CC_ENABLE;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CC, ctrl->cc);
}

static void pci_epf_nvme_enable_ctrl(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf *epf = epf_nvme->epf;
	int ret;

	dev_info(&epf->dev, "Enabling controller\n");

	ctrl->mps_shift = ((ctrl->cc >> NVME_CC_MPS_SHIFT) & 0xf) + 12;
	ctrl->mps = 1UL << ctrl->mps_shift;
	ctrl->mps_mask = ctrl->mps - 1;

	ctrl->adm_sqes = 1UL << NVME_ADM_SQES;
	ctrl->adm_cqes = sizeof(struct nvme_completion);
	ctrl->io_sqes = 1UL << ((ctrl->cc >> NVME_CC_IOSQES_SHIFT) & 0xf);
	ctrl->io_cqes = 1UL << ((ctrl->cc >> NVME_CC_IOCQES_SHIFT) & 0xf);

	if (ctrl->io_sqes < sizeof(struct nvme_command)) {
		dev_err(&epf->dev, "Unsupported IO sqes %zu (need %zu)\n",
			ctrl->io_sqes, sizeof(struct nvme_command));
		return;
	}

	if (ctrl->io_cqes < sizeof(struct nvme_completion)) {
		dev_err(&epf->dev, "Unsupported IO cqes %zu (need %zu)\n",
			ctrl->io_sqes, sizeof(struct nvme_completion));
		return;
	}

	ctrl->aqa = pci_epf_nvme_reg_read32(ctrl, NVME_REG_AQA);
	ctrl->asq = pci_epf_nvme_reg_read64(ctrl, NVME_REG_ASQ);
	ctrl->acq = pci_epf_nvme_reg_read64(ctrl, NVME_REG_ACQ);

	/*
	 * Map the controller admin submission and completion queues. The
	 * target admin queues were already created when the target was
	 * enabled.
	 */
	ret = pci_epf_nvme_map_cq(epf_nvme, 0,
				  NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED,
				  (ctrl->aqa & 0x0fff0000) >> 16, 0,
				  ctrl->acq & GENMASK(63, 12));
	if (ret)
		return;

	ret = pci_epf_nvme_map_sq(epf_nvme, 0, 0, NVME_QUEUE_PHYS_CONTIG,
				  ctrl->aqa & 0x0fff,
				  ctrl->asq & GENMASK(63, 12));
	if (ret) {
		pci_epf_nvme_unmap_cq(epf_nvme, 0);
		return;
	}

	/* Tell the host we are now ready */
	ctrl->csts |= NVME_CSTS_RDY;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);

	/* Start polling the submission queues */
	queue_delayed_work(epf_nvme_sq_wq, &epf_nvme->sq_poll,
			   msecs_to_jiffies(1));

	return;
}

static enum rq_end_io_ret pci_epf_nvme_cmd_done(struct request *req,
						blk_status_t blk_status)
{
	struct pci_epf_nvme_cmd *epcmd = req->end_io_data;

	epcmd->status = nvme_req(req)->status;
	blk_mq_free_request(req);
	complete(&epcmd->done);
	return RQ_END_IO_NONE;
}

static int pci_epf_nvme_submit_cmd_nowait(struct pci_epf_nvme *epf_nvme,
					  struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	struct request_queue *q;
	struct request *req;
	void *buffer = epcmd->buffer;
	unsigned bufflen = epcmd->buffer_size;
	int ret;

	if (epcmd->ns)
		q = epcmd->ns->queue;
	else
		q = epf_nvme->ctrl.ctrl->admin_q;

	/**
	 * @todo timeout ?
	 * @note code inspired by __nvme_submit_sync_cmd() in drivers/nvme/host/core.c
	 * and nvmet_passthru_execute_cmd() in drivers/nvme/target/passthru.c
	 */

	req = blk_mq_alloc_request(q, nvme_req_op(cmd), 0);

	if (IS_ERR(req))
		return PTR_ERR(req);
	nvme_init_request(req, cmd);

	if (buffer && bufflen) {
		ret = blk_rq_map_kern(q, req, buffer, bufflen, GFP_KERNEL);
		if (ret)
			goto out;
	}

//	if (timeout)
//		req->timeout = timeout;

	req->end_io = pci_epf_nvme_cmd_done;
	req->end_io_data = epcmd;
	blk_execute_rq_nowait(req, false);

	return 0;

out:
	epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	blk_mq_free_request(req);
	return ret;
}

static void pci_epf_nvme_submit_sync_cmd(struct pci_epf_nvme *epf_nvme,
					 struct pci_epf_nvme_cmd *epcmd)
{
#if PCI_EPF_NVME_TEST_CMD_NOWAIT
	int ret;
	// Test the new mechanism in a sequential manner
	ret = pci_epf_nvme_submit_cmd_nowait(epf_nvme, epcmd);
	if (ret < 0)
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	else
		wait_for_completion(&epcmd->done);
#else
	struct nvme_command *cmd = &epcmd->cmd;
	struct request_queue *q;
	int ret;

	if (epcmd->ns)
		q = epcmd->ns->queue;
	else
		q = epf_nvme->ctrl.ctrl->admin_q;

	ret = nvme_submit_sync_cmd(q, cmd, epcmd->buffer, epcmd->buffer_size);
	if (ret < 0)
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	else if (ret > 0)
		epcmd->status = ret;
#endif
	if (epcmd->status)
		dev_err(&epcmd->epf_nvme->epf->dev,
			"QID %d: command %s (0x%x) failed, status 0x%0x\n",
			epcmd->cqid, pci_epf_nvme_cmd_name(epcmd),
			epcmd->cmd.common.opcode, epcmd->status);
}

static inline void pci_epf_nvme_submit_cmd(struct pci_epf_nvme *epf_nvme,
				    struct pci_epf_nvme_cmd *epcmd,
				    bool deferred)
{
	if (deferred) {
		list_add_tail(&epcmd->list_node,
			      &epcmd->epf_nvme->deferred_reqs);
		pci_epf_nvme_submit_cmd_nowait(epf_nvme, epcmd);
	}
	else
		pci_epf_nvme_submit_sync_cmd(epf_nvme, epcmd);
}

static void pci_epf_nvme_admin_create_cq(struct pci_epf_nvme *epf_nvme,
					 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	int mqes = NVME_CAP_MQES(epf_nvme->ctrl.cap);
	u16 cqid, cq_flags, qsize, vector;
	int ret;

	cqid = le16_to_cpu(cmd->create_cq.cqid);
	if (cqid >= epf_nvme->ctrl.nr_queues || epf_nvme->ctrl.cq[cqid].ref) {
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cq_flags = le16_to_cpu(cmd->create_cq.cq_flags);
	if (!(cq_flags & NVME_QUEUE_PHYS_CONTIG)) {
		epcmd->status = NVME_SC_INVALID_QUEUE | NVME_SC_DNR;
		return;
	}

	qsize = le16_to_cpu(cmd->create_cq.qsize);
	if (!qsize || qsize > NVME_CAP_MQES(epf_nvme->ctrl.cap)) {
		if (qsize > mqes)
			dev_warn(&epf_nvme->epf->dev,
				 "Create CQ %d, qsize %d > mqes %d: buggy driver?\n",
				 cqid, (int)qsize, mqes);
		epcmd->status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		return;
	}

	vector = le16_to_cpu(cmd->create_cq.irq_vector);
	if (vector >= epf_nvme->nr_vectors) {
		epcmd->status = NVME_SC_INVALID_VECTOR | NVME_SC_DNR;
		return;
	}

	ret = pci_epf_nvme_map_cq(epf_nvme, cqid, cq_flags, qsize, vector,
				  le64_to_cpu(cmd->create_cq.prp1));
	if (ret)
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
}

static void pci_epf_nvme_admin_create_sq(struct pci_epf_nvme *epf_nvme,
					 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	int mqes = NVME_CAP_MQES(epf_nvme->ctrl.cap);
	u16 sqid, cqid, sq_flags, qsize;
	int ret;

	sqid = le16_to_cpu(cmd->create_sq.sqid);
	if (sqid > epf_nvme->ctrl.nr_queues || epf_nvme->ctrl.sq[sqid].ref) {
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cqid = le16_to_cpu(cmd->create_sq.cqid);
	if (sqid && !epf_nvme->ctrl.cq[cqid].ref) {
		epcmd->status = NVME_SC_CQ_INVALID | NVME_SC_DNR;
		return;
	}

	sq_flags = le16_to_cpu(cmd->create_sq.sq_flags);
	if (sq_flags != NVME_QUEUE_PHYS_CONTIG) {
		epcmd->status = NVME_SC_INVALID_QUEUE | NVME_SC_DNR;
		return;
	}

	qsize = le16_to_cpu(cmd->create_sq.qsize);
	if (!qsize || qsize > mqes) {
		if (qsize > mqes)
			dev_warn(&epf_nvme->epf->dev,
				 "Create SQ %d, qsize %d > mqes %d: buggy driver?\n",
				 sqid, (int)qsize, mqes);
		epcmd->status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		return;
	}

	ret = pci_epf_nvme_map_sq(epf_nvme, sqid, cqid, sq_flags, qsize,
				  le64_to_cpu(cmd->create_sq.prp1));
	if (ret)
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
}

static void pci_epf_nvme_admin_identify_hook(struct pci_epf_nvme *epf_nvme,
					     struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	struct nvme_id_ctrl *id = epcmd->buffer;
	unsigned int page_shift;

	if (cmd->identify.cns != NVME_ID_CNS_CTRL)
		return;

	/* Set device vendor IDs */
	id->vid = cpu_to_le16(epf_nvme->epf->header->vendorid);
	id->ssvid = id->vid;

	/* Set Maximum Data Transfer Size (MDTS) */
	page_shift = NVME_CAP_MPSMIN(epf_nvme->ctrl.ctrl->cap) + 12;
	id->mdts = ilog2(PCI_EPF_NVME_MDTS) - page_shift;

	/* Clear Controller Multi-Path I/O and Namespace Sharing Capabilities */
	id->cmic = 0;

	/* Do not report support for Autonomous Power State Transitions */
	id->apsta = 0;

	/* Indicate no support for SGLs */
	id->sgls = 0;
}

static u16 pci_epf_nvme_h2c(struct pci_epf_nvme *nvme,
			    size_t len,
			    struct pci_epf_nvme_cmd *epcmd)
{
	int ret;

	ret = pci_epf_nvme_cmd_parse_dptr(nvme, epcmd, len);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	ret = pci_epf_nvme_cmd_transfer(nvme, epcmd, DMA_FROM_DEVICE /* h2c */);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	return NVME_SC_SUCCESS;
}

static u16 tsp_nvme_cs_identify(struct pci_epf_nvme *nvme,
				struct pci_epf_nvme_cmd *epcmd)
{
	int ret;
	struct nvme_command *cmd;
	u8 *buffer;
	static const int data_len = 4096;

	ret = pci_epf_nvme_cmd_parse_dptr(nvme, epcmd, data_len);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	buffer = epcmd->buffer;
	cmd = &epcmd->cmd;
	memset(buffer, 0, data_len);

	switch (cmd->common.cdw11) {
	case TSP_CS_CSX:
		strncpy((char*)buffer, TSP_CS_ID_STRING, data_len);
		break;

	default:
		memset(buffer, 0, data_len);
		break;
	}

	ret = pci_epf_nvme_cmd_transfer(nvme, epcmd, DMA_TO_DEVICE /* c2h */);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	return NVME_SC_SUCCESS;
}

static u16 tsp_nvme_cs_get(struct pci_epf_nvme *nvme,
			   struct pci_epf_nvme_cmd *epcmd)
{
	int ret;
	struct nvme_command *cmd;
	u64 fun;
	s32 function_id;
	u8 *buffer;
	CsFunctionBitSelect f;
	static const int data_len = 4096;

	ret = pci_epf_nvme_cmd_parse_dptr(nvme, epcmd, data_len);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	buffer = epcmd->buffer;
	cmd = &epcmd->cmd;
	memset(buffer, 0, data_len);

	switch (cmd->common.cdw11) {
	case TSP_CS_PROPS:
		//trace_tsp_nvme_get_properties();
		memcpy(&(tsp_csx_props.CSE), &tsp_cse_props, sizeof(tsp_cse_props));
		memcpy(buffer, &tsp_csx_props, sizeof(tsp_csx_props));
		break;

	case TSP_CS_CAPS:
		//trace_tsp_nvme_get_capabilities();
		memcpy(buffer, &tsp_cs_caps, sizeof(CsCapabilities));
		//printf("Caps : 0x%08x\n", tsp_cs_caps);
		break;

	case TSP_CS_FUN:
		fun = (((u64)cmd->common.cdw12) | (((u64)cmd->common.cdw13) << 32));
		memcpy(&f, &fun, sizeof(f));
		//trace_tsp_nvme_get_function(fun);
		if (f.Functions.Checksum) {
			function_id = TSP_CHECKSUM_FUNCTION_ID; // Arbitrary FunctionId ... /// @todo change this
		} else {
			// No other functions supported yet
			function_id = -1;
		}
		memcpy(buffer, &function_id, sizeof(function_id));
	break;

	default:
		// Should not happen... return 0's
		memset(buffer, 0, data_len);
		break;
	}

	ret = pci_epf_nvme_cmd_transfer(nvme, epcmd, DMA_TO_DEVICE /* c2h */);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	return NVME_SC_SUCCESS;
}

static phys_addr_t tsp_allocate_in_cmb(struct pci_epf_nvme *nvme, s32 bytes_requested)
{
	u64 cmbmsc;
	phys_addr_t cba;
	u8 *ptr, *base_ptr;

	cmbmsc = pci_epf_nvme_reg_read64(&nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;

	if (cba) {
		if (bytes_requested > 0) {
			if (bytes_requested & (4096-1)) {
				/// @todo change this ? maybe to lba size ?
				return (phys_addr_t)NULL; // Do not allow requests that are not a multiple of 4096 bytes
				// This will make sure we are 4k aligned in the CMB and that transfers don't transfer outside of buffer
			}
			// Allocate inside locally allocated cmb buffer
			ptr = tsp_mm_alloc((size_t)bytes_requested, nvme->tsp_alloc_ctx);
			if (ptr) {
				base_ptr = nvme->reg[CMB_BIR];
				// Conversion from pointer in local buffer to physical address (for host)
				dev_dbg(&nvme->epf->dev, "bytes req: %u base_ptr: %#0llx ptr: %#0llx\n", bytes_requested, (phys_addr_t)base_ptr, (phys_addr_t)ptr);
				return cba + (ptr - base_ptr);
			}
		}
	}
	// All error / failed allocation cases return "NULL"
	return (phys_addr_t)NULL;
}

static void tsp_deallocate_in_cmb(struct pci_epf_nvme *nvme, phys_addr_t addr)
{
	u64 cmbmsc;
	phys_addr_t cba;
	void *ptr;

	cmbmsc = pci_epf_nvme_reg_read64(&nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;

	if (cba) {
		if (addr > cba) {
			ptr = ((char*)(nvme->reg[CMB_BIR])) + (addr - cba);
			tsp_mm_free(ptr, nvme->tsp_alloc_ctx);
		}
	}
}

static u16 tsp_nvme_cs_mm(struct pci_epf_nvme *nvme,
			  struct pci_epf_nvme_cmd *epcmd)
{
	int ret;
	struct nvme_command *cmd;
	u8 *buffer;
	u64 cmbmsc;
	s32 bytes_requested;
	phys_addr_t phys_addr, to_be_freed;
	static const int data_len = 4096;

	ret = pci_epf_nvme_cmd_parse_dptr(nvme, epcmd, data_len);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	buffer = epcmd->buffer;
	cmd = &epcmd->cmd;
	memset(buffer, 0, data_len);
	cmbmsc = pci_epf_nvme_reg_read64(&nvme->ctrl, NVME_REG_CMBMSC);

	switch (cmd->common.cdw11) {
	case TSP_CS_MEM:
		phys_addr = (phys_addr_t)NULL;
		if (!(cmbmsc & CMBMSC_CMSE_BIT) || !(cmbmsc & CMBMSC_CRE_BIT)) {
			// No CMB enabled, cannot manage memory...
			// (return a null addr)
		} else {
			if (cmd->common.cdw10 == TSP_CS_ALLOCATE) {
				/// @note int32_t because CS API specifies "int" ...
				bytes_requested = cmd->common.cdw12;
				phys_addr = tsp_allocate_in_cmb(nvme, bytes_requested);
			} else if (cmd->common.cdw10 == TSP_CS_DEALLOCATE) {
				to_be_freed = cmd->common.cdw12;
				if (sizeof(phys_addr_t) == 8) {
					// 64 bit hwaddr
					to_be_freed |= ((phys_addr_t)cmd->common.cdw13) << 32;
				}
				tsp_deallocate_in_cmb(nvme, to_be_freed);
			}
		}
		memcpy(buffer, &phys_addr, sizeof(phys_addr));
		break;

	default:
		memset(buffer, 0, data_len);
		break;
	}

	ret = pci_epf_nvme_cmd_transfer(nvme, epcmd, DMA_TO_DEVICE /* c2h */);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	return NVME_SC_SUCCESS;
}

static u16 tsp_compute_checksum(struct pci_epf_nvme *nvme,
				phys_addr_t source, u32 len,
				phys_addr_t dest)
{
	void *cmb_source, *cmb_dest;
	u32 *source_as_u32, *dest_as_u32, checksum, i;

	pci_epf_nvme_print_cmb_info(nvme);

	if (pci_epf_nvme_addr_in_cmb(nvme, source) &&
	    pci_epf_nvme_addr_in_cmb(nvme, dest) &&
	    pci_epf_nvme_addr_in_cmb(nvme, source+len-1) &&
	    pci_epf_nvme_addr_in_cmb(nvme, dest+sizeof(checksum)-1)) {
		cmb_source = pci_epf_nvme_cmb_addr_as_local_addr(nvme, source);
		cmb_dest = pci_epf_nvme_cmb_addr_as_local_addr(nvme, dest);
		source_as_u32 = (u32*)cmb_source;
		dest_as_u32 = (u32*)cmb_dest;
		checksum = 0;
		dev_info(&nvme->epf->dev, "Computing checksum :\n");
		for (i = 0; i < len/sizeof(u32); ++i) {
			checksum += source_as_u32[i];
			//dev_info(&nvme->epf->dev, "value : %#08lx\n", source_as_u32[i]);
		}
		dev_info(&nvme->epf->dev, "Result : %#08x\n", checksum);
		*dest_as_u32 = checksum;
	} else {
		dev_err(&nvme->epf->dev, "Data is outside of CMB\n"
		"source: %#0llx, dest: %#0llx, len: %u\n", source, dest, len);
		// Data is outside of CMB ... don't even try
		return NVME_SC_DATA_XFER_ERROR;
	}
	return NVME_SC_SUCCESS;
}

static u16 tsp_do_compute_request(struct pci_epf_nvme *nvme,
				  CsComputeRequest *creq)
{
	phys_addr_t source, dest;
	u32 len;

	switch (creq->FunctionId) {
	case TSP_CHECKSUM_FUNCTION_ID:
		/// @todo check arguments
		source = (phys_addr_t)creq->Args[0].u.DevMem.MemHandle;
		dest = (phys_addr_t)creq->Args[2].u.DevMem.MemHandle;
		len = creq->Args[1].u.Value32;
		return tsp_compute_checksum(nvme, source, len, dest);
	case TSP_SLEEP_FUNCTION_ID:
		len = creq->Args[0].u.Value32;
		/* Sleeping in deferred requests (handled by a thread) makes
		 * sense, sleeping in a synchronous command will block the
		 * entire controller. This sleep is for testing purposes,
		 * to estimate the time to handle compute requests */
		//dev_info(&nvme->epf->dev, "Sleeping for %d msecs\n", len);
		msleep_interruptible(len);
		//dev_info(&nvme->epf->dev, "Done sleeping\n");
		return NVME_SC_SUCCESS;
	default:
		/// @todo change error handling ?
		return NVME_SC_INVALID_OPCODE;
	}
}

/* This is a function to be threaded */
static int tsp_do_async_compute_request_fn(void *opaque)
{
	AsyncComputeRequest *acreq = (AsyncComputeRequest*)opaque;
	struct completion *done;

	if (!acreq) {
		pr_warn("Async compute request pointer is NULL\n");
		return -EINVAL;
	}

	if (!acreq->creq) {
		pr_warn("Async compute request compute request is NULL\n");
		return -EINVAL;
	}

	done = &acreq->epcmd->done;

	/* Set the status inside the request */
	acreq->epcmd->status = tsp_do_compute_request(acreq->nvme, acreq->creq);
	/* The thread frees the memory when it is no longer needed
	 * (allocated by caller) */
	kfree(acreq->creq);
	kfree(acreq);

	/* Tell the controller that the request is completed so that it can join
	 * the thread */
	complete(done);

	return 0;
}

static u16 tsp_do_compute_request_sync(struct pci_epf_nvme *nvme,
				       struct pci_epf_nvme_cmd *epcmd)
{
	return tsp_do_compute_request(nvme, (CsComputeRequest *)epcmd->buffer);
}

static u16 tsp_launch_async_compute(struct pci_epf_nvme *nvme,
				    struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	CsComputeRequest *creq;
	AsyncComputeRequest *acreq;

	/* Allocate memory for the thread, because the buffer is local */
	creq = kzalloc(cmd->common.cdw12 /* req_size */, GFP_KERNEL);
	if (creq == NULL) {
		return NVME_SC_INTERNAL;
	}
	/** @note this copy could be omitted since the buffer lives as long as
	 *  the command (so until the completion is sent) */
	memcpy(creq, epcmd->buffer, cmd->common.cdw12 /* req_size */);
	/* Allocate memory to pass to threaded function */
	acreq = kzalloc(sizeof(AsyncComputeRequest), GFP_KERNEL);
	if (acreq == NULL) {
		kfree(creq);
		return NVME_SC_INTERNAL;
	}
	/* Request lives as long as the queue does and the thread lives as
	 * long as the queue lives, this is a shared structure between the
	 * thread and the nvme controller code, the thread only sets the status
	 * and the nvme code periodically checks (polls) the completion to see
	 * if the thread finished, then joins the thread */
	acreq->nvme = nvme; /* Very important ! */
	acreq->epcmd = epcmd;
	// WHY? req->status = NVME_SUCCESS; // Make sure it is not NVME_NO_COMPLETE
	acreq->creq = creq;
	//dev_info(&nvme->epf->dev, "Launching thread\n");
	epcmd->thread = kthread_run(tsp_do_async_compute_request_fn, (void *)acreq, "TSP Async thread");

	if (IS_ERR_OR_NULL(epcmd->thread)) {
		// The thread could not be created
		dev_err(&nvme->epf->dev, "TSP Error creating async compute thread");
		kfree(acreq->creq);
		kfree(acreq);
		return NVME_SC_INTERNAL;
	}

	/* The dynamically allocated memory above is freed by the thread (maybe
	 * not the best idea), the thread could return the acreq pointer and
	 * both creq and acreq could be freed upon join... */

	/* Dirty way to tell the caller not to enqueue the completion */
	return (u16)-1;
}

static u16 tsp_nvme_cs_compute(struct pci_epf_nvme *nvme,
			       struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	u16 ret;

	ret = pci_epf_nvme_h2c(nvme, SZ_4K, epcmd);
	if (ret != NVME_SC_SUCCESS)
		return ret;

	if (cmd->common.cdw11) {
		/* Synchronous */
		return tsp_do_compute_request_sync(nvme, epcmd);
	} else {
		/* Asynchronous - will return -1 (deferred) or error code */
		return tsp_launch_async_compute(nvme, epcmd);
	}

	return NVME_SC_SUCCESS;
}

u32 tsp_inet_addr(const char *str) {
	int ret;
	u32 addr;
	u8 *addr_arr = (u8 *)&addr;

	ret = sscanf(str, "%hhd.%hhd.%hhd.%hhd",
		     &addr_arr[0], &addr_arr[1], &addr_arr[2], &addr_arr[3]);

	if (ret < 4)
		return -1;

	return addr;
}

static int tsp_relay_thread_fn(void *opaque)
{
	struct tsp_relay_thread_args *rta =
		(struct tsp_relay_thread_args *)opaque;
	struct pci_epf_nvme *nvme = rta->nvme;
	struct tsp_relay *relay = nvme->tsp_relays[rta->relay_desc];
	struct kvec vec;
	struct msghdr msg;
	int ret = 0;

	vec.iov_len = sizeof(relay->buffer.pack.data);
	vec.iov_base = &relay->buffer.pack.data[0];

	msg.msg_name = 0;
	msg.msg_namelen = 0;
	msg.msg_control = NULL;
	msg.msg_flags = 0;

	dev_dbg(&nvme->epf->dev, "Relay %d : Thread launched\n",
		rta->relay_desc);

	while (1) {
		/* Wait if the buffer is full */
		wait_event_interruptible(relay->wq, !relay->buffer_full ||
						    relay->shutdown);
		/* For the moment a single buffer is used to simplify
		 * concurrent access */

		/* Here the buffer is not full */

		/* Stop the thread */
		if (relay->shutdown) {
			break;
		}

		/* Fill the buffer */
		dev_dbg(&nvme->epf->dev, "v Relay %d : Blocking read on socket\n",
			rta->relay_desc);
		ret = kernel_recvmsg(relay->socket, &msg, &vec,
				     TSP_RELAY_BUFFER_SIZE,
				     TSP_RELAY_BUFFER_SIZE, msg.msg_flags);
		dev_dbg(&nvme->epf->dev, "^ Relay %d : read %d bytes\n",
			rta->relay_desc, ret);

		if (ret < 0) {
			dev_warn(&nvme->epf->dev, "Relay %d failed to read\n",
				 rta->relay_desc);
			break;
		}

		relay->buffer.pack.size = ret;
		relay->buffer_full = true;
	}

	kfree(rta);
	return 0;
}

static int tsp_find_unallocated_relay(struct pci_epf_nvme *nvme)
{
	int i;
	for (i = 0; i < TSP_RELAY_MAX_RELAYS; ++i) {
		if (nvme->tsp_relays[i] == NULL) {
			return i;
		}
	}
	return -1;
}

static int tsp_new_relay(struct pci_epf_nvme *nvme,
			 struct addr_info *ai)
{
	struct socket *sock = NULL;
	struct sockaddr_in saddr = {0,};
	struct tsp_relay *relay;
	struct tsp_relay_thread_args *rta;
	u32 addr;
	u16 port;
	int ret, relay_number = tsp_find_unallocated_relay(nvme);

	if (relay_number < 0)
		return relay_number;

	dev_dbg(&nvme->epf->dev, "Relay %d : addr (node) : %s\n",
		relay_number, ai->node);
	dev_dbg(&nvme->epf->dev, "Relay %d : port (service) : %s\n",
		relay_number, ai->service);

	addr = tsp_inet_addr(ai->node);
	if (addr == (u32)-1) {
		dev_warn(&nvme->epf->dev,
			 "Relay %d : Could not parse addr (node) : %s\n",
			 relay_number, ai->node);
		return -1;
	}

	ret = kstrtou16(ai->service, 10, &port);
	if (ret < 0) {
		dev_warn(&nvme->epf->dev,
			 "Relay %d : Could not parse port (serice) : %s\n",
			 relay_number, ai->node);
		return -1;
	}

	saddr.sin_family = AF_INET;
	saddr.sin_port = htons(port);
	saddr.sin_addr.s_addr = addr;

	ret = sock_create(PF_INET, SOCK_STREAM, IPPROTO_TCP, &sock);
	if (ret < 0) {
		dev_warn(&nvme->epf->dev, "Relay %d : Socket creation failed\n",
			 relay_number);
		return -1;
	}

	ret = sock->ops->connect(sock, (struct sockaddr *)&saddr, sizeof(saddr),
				 O_RDWR);
	if (ret) {
		dev_warn(&nvme->epf->dev,
			 "Relay %d : Could not connect, ret = %d\n",
			 relay_number, ret);
		sock_release(sock);
		return -1;
	}

	relay = kzalloc(sizeof(struct tsp_relay), GFP_KERNEL);
	if (!relay) {
		dev_warn(&nvme->epf->dev, "Could not allocate relay\n");
		sock_release(sock);
		return -1;
	}

	nvme->tsp_relays[relay_number] = relay;
	dev_dbg(&nvme->epf->dev, "Relay %d : Connected !\n", relay_number);
	relay->socket = sock;
	relay->buffer_full = false;
	relay->shutdown = false;
	init_waitqueue_head(&relay->wq);

	/* These are freed by the thread itself */
	rta = kzalloc(sizeof(struct tsp_relay_thread_args), GFP_KERNEL);
	rta->nvme = nvme;
	rta->relay_desc = relay_number;

	relay->thread = kthread_run(tsp_relay_thread_fn, (void *)rta,
				    "TSP relay thread %d", relay_number);

	if (IS_ERR_OR_NULL(relay->thread)) {
		/* The thread could not be created */
		dev_err(&nvme->epf->dev, "TSP Error creating relay thread");
		kfree(rta);
		sock_release(sock);
		return -1;
	}

	return relay_number;
}

static u16 tsp_nvme_address_info_from_req(struct pci_epf_nvme *nvme,
					  struct pci_epf_nvme_cmd *epcmd,
					  struct addr_info *ai) {
	u16 ret;

	ret = pci_epf_nvme_h2c(nvme, SZ_4K, epcmd);

	if (ret != NVME_SC_SUCCESS)
		return ret;

	memcpy(ai, epcmd->buffer, sizeof(*ai));
	return NVME_SC_SUCCESS;
}

static u16 tsp_nvme_cs_open_relay(struct pci_epf_nvme *nvme,
				  struct pci_epf_nvme_cmd *epcmd)
{
	struct addr_info ai;
	int desc;
	u16 ret;

	ret = tsp_nvme_address_info_from_req(nvme, epcmd, &ai);
	if (ret != NVME_SC_SUCCESS)
		return ret;

	desc = tsp_new_relay(nvme, &ai);
	if (desc < 0)
		return NVME_SC_INTERNAL;

	/* buffer is allocated in tsp_nvme_address_info_from_req */
	*(s32*)epcmd->buffer = (s32)desc;
	ret = pci_epf_nvme_cmd_transfer(nvme, epcmd, DMA_TO_DEVICE /* c2h */);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	return NVME_SC_SUCCESS;
}

static int tsp_nvme_cs_stop_relay_thread(struct pci_epf_nvme *nvme,
					 struct tsp_relay *relay)
{
	int ret;
	relay->shutdown = true;

	/* The thread could be waiting on a buffer */
	wake_up(&relay->wq);

	/* The thread could be blocked on recv() on socket */
	kernel_sock_shutdown(relay->socket, SHUT_RDWR);
	ret = kthread_stop(relay->thread);
	sock_release(relay->socket);

	return ret;
}

static u16 tsp_nvme_cs_close_relay(struct pci_epf_nvme *nvme,
				   struct pci_epf_nvme_cmd *epcmd)
{
	int ret;
	int desc = epcmd->cmd.common.cdw13;
	struct tsp_relay *relay;

	if (desc < 0 || desc >= TSP_RELAY_MAX_RELAYS) {
		/* Out of range... */
		return NVME_SC_INVALID_FIELD;
	}

	relay = nvme->tsp_relays[desc];

	if (!relay) {
		dev_warn(&nvme->epf->dev, "Relay %d is already closed\n", desc);
	} else {
		ret = tsp_nvme_cs_stop_relay_thread(nvme, relay);

		kfree(relay);
		nvme->tsp_relays[desc] = NULL;
	}

	return NVME_SC_SUCCESS;
}

/* Do not use this function in a thread, call only if filled_buffers is not
 * empty, this is a very specific helper function, don't use */
static u16 tsp_nvme_cs_comm_to_host(struct pci_epf_nvme *nvme,
				    struct pci_epf_nvme_cmd *epcmd)
{
	/// @todo check data_len
	int ret = NVME_SC_SUCCESS;
	int desc = epcmd->cmd.common.cdw13;
	struct tsp_relay *relay = nvme->tsp_relays[desc];

	if (!relay) {
		dev_warn(&nvme->epf->dev, "Relay %d isn't allocated\n", desc);
		epcmd->status = NVME_SC_DATA_XFER_ERROR;
		return NVME_SC_DATA_XFER_ERROR;
	}

	/** @todo it could be better to use the CQE "result" field to pass the
	 * size */
	dev_dbg(&nvme->epf->dev,
		"Relay %d : Transferring 4k buffer with %d bytes to host\n",
		desc, relay->buffer.pack.size);

	ret = pci_epf_nvme_cmd_parse_dptr(nvme, epcmd, SZ_4K);
	if (ret < 0) {
		return epcmd->status;
	}
	/** @todo this copy could be avoided but requires to have the relay
	 * thread fill the data directly into the requests */
	memcpy(epcmd->buffer, &relay->buffer.pack, SZ_4K);
	ret = pci_epf_nvme_cmd_transfer(nvme, epcmd, DMA_TO_DEVICE /* c2h */);
	if (ret < 0) {
		return epcmd->status;
	}

	relay->buffer_full = false;
	wake_up(&relay->wq);

	if (relay->buffer.pack.size == 0) {
		/* Disconnection, close relay (stops thread) */
		dev_dbg(&nvme->epf->dev,
			"Relay %d : socket disconnected, closing relay\n", desc);

		return tsp_nvme_cs_close_relay(nvme, epcmd);
	}

	return epcmd->status;
}

static u16 tsp_nvme_cs_comm(struct pci_epf_nvme *nvme,
			    struct pci_epf_nvme_cmd *epcmd)
{
	struct msghdr msg;
	struct kvec vec;
	struct socket *sock;
	u32 write_nread = epcmd->cmd.common.cdw11;
	u32 length = epcmd->cmd.common.cdw12;
	u32 desc = epcmd->cmd.common.cdw13;
	u16 ret;

	if (desc >= TSP_RELAY_MAX_RELAYS) {
		dev_warn(&nvme->epf->dev,
			 "Relay with descriptor %d does not exist\n", desc);
		return NVME_SC_INVALID_FIELD;
	}
	if (!nvme->tsp_relays[desc]) {
		dev_warn(&nvme->epf->dev,
			 "Relay with descriptor %d is not allocated\n", desc);
		return NVME_SC_INVALID_FIELD;
	}

	sock = nvme->tsp_relays[desc]->socket;

	if (!sock) {
		dev_warn(&nvme->epf->dev,
			 "Relay with descriptor %d has no socket\n", desc);
		return NVME_SC_DATA_XFER_ERROR;
	}

	if (length > SZ_4K) {
		dev_warn(&nvme->epf->dev,
			 "Cannot send more than 4k for the moment\n");
		return NVME_SC_DATA_XFER_ERROR;
	}

	if (write_nread) {
		/* If this is a write get data from host */
		ret = pci_epf_nvme_h2c(nvme, SZ_4K, epcmd);
		if (ret != NVME_SC_SUCCESS) {
			return ret;
		}

		dev_dbg(&nvme->epf->dev,
			"Relay %d : Writing %d bytes to socket\n", desc, length);

		msg.msg_name = 0;
		msg.msg_namelen = 0;
		msg.msg_control = NULL;
		msg.msg_controllen = 0;
		msg.msg_flags = 0;

		vec.iov_len = length;
		vec.iov_base = epcmd->buffer;
		/* And send it into the socket */
		ret = kernel_sendmsg(sock, &msg, &vec, length, length);
		if (ret < 0) {
			dev_warn(&nvme->epf->dev,
				 "Relay %d : Failed to write to socket\n", desc);
			return NVME_SC_INTERNAL;
		} else if (ret < length) {
			dev_warn(&nvme->epf->dev,
				 "Relay %d : Failed to write all data to socket\n",
				 desc);
			return NVME_SC_DATA_XFER_ERROR;
		}
		return NVME_SC_SUCCESS;
	} else {
		/* This is a read */
		dev_dbg(&nvme->epf->dev,
			"Relay %d : A socket read was requested...\n", desc);
		/* If there is no data yet so we will defer the request */
		if (!nvme->tsp_relays[desc]->buffer_full) {
			dev_dbg(&nvme->epf->dev,
				"Relay %d : no data, outstanding req\n", desc);
			/* This request is a relay related outstanding req */
			epcmd->relay = true;
			epcmd->relay_desc = desc;
			return (u16)-1; /* deferred */
		} else {
			/* Data available, move it to host */
			return tsp_nvme_cs_comm_to_host(nvme, epcmd);
		}
	}

	return NVME_SC_SUCCESS;
}

static u16 pci_epf_nvme_process_custom_admin_cmd(struct pci_epf_nvme *nvme,
						 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd;
	cmd = &epcmd->cmd;
	dev_dbg(&nvme->epf->dev, "TSP: Opcode: %#x, CDW10: %#x, CDW11: %#x\n",
		cmd->common.opcode, cmd->common.cdw10, cmd->common.cdw11);
	switch (cmd->common.cdw10) {
	case TSP_CS_IDENTIFY:
		return tsp_nvme_cs_identify(nvme, epcmd);
	case TSP_CS_GET:
		return tsp_nvme_cs_get(nvme, epcmd);
	case TSP_CS_ALLOCATE:
		/* fallthrough */
	case TSP_CS_DEALLOCATE:
		return tsp_nvme_cs_mm(nvme, epcmd);
	case TSP_CS_COMPUTE:
		return tsp_nvme_cs_compute(nvme, epcmd);
	case TSP_CS_COMM:
		return tsp_nvme_cs_comm(nvme, epcmd);
	case TSP_CS_OPEN_RELAY:
		return tsp_nvme_cs_open_relay(nvme, epcmd);
	case TSP_CS_CLOSE_RELAY:
		return tsp_nvme_cs_close_relay(nvme, epcmd);
	default:
		break;
	}

	return NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
}

static bool pci_epf_nvme_process_admin_cmd(struct pci_epf_nvme *epf_nvme)
{
	void (*post_process_hook)(struct pci_epf_nvme *,
				  struct pci_epf_nvme_cmd *) = NULL;
	struct pci_epf_nvme_cmd *epcmd;
	size_t transfer_len = 0;
	bool deferred = false;
	u16 status;
	int ret = 0;

	/* Fetch new command from admin submission queue */
	epcmd = pci_epf_nvme_fetch_cmd(epf_nvme, 0);
	if (!epcmd)
		return false;

	if (epcmd->cmd.common.opcode >= nvme_admin_vendor_start) {
		status = pci_epf_nvme_process_custom_admin_cmd(epf_nvme, epcmd);
		/* The above returns -1 to show that the cmd is deferred */
		if (status == (u16)-1) {
			/* Request is handled by another thread / task */
			deferred = true;
		} else if (status < 0) {
			dev_warn(&epf_nvme->epf->dev, "Negative status from custom admin command\n");
		} else {
			/* This might not be needed, it should be set */
			epcmd->status = status;
		}
		goto complete;
	}

	switch (epcmd->cmd.common.opcode) {
	case nvme_admin_identify:
		post_process_hook = pci_epf_nvme_admin_identify_hook;
		transfer_len = NVME_IDENTIFY_DATA_SIZE;
		break;

	case nvme_admin_get_log_page:
		transfer_len = nvme_get_log_page_len(&epcmd->cmd);
		break;

	case nvme_admin_async_event:
		/* XXX For now XXX */
		dev_info(&epf_nvme->epf->dev, "nvme_admin_async_event\n");
		return true;

	case nvme_admin_get_features:
	case nvme_admin_set_features:
	case nvme_admin_abort_cmd:
		break;

	case nvme_admin_create_cq:
		pci_epf_nvme_admin_create_cq(epf_nvme, epcmd);
		goto complete;

	case nvme_admin_create_sq:
		pci_epf_nvme_admin_create_sq(epf_nvme, epcmd);
		goto complete;

	case nvme_admin_delete_cq:
	case nvme_admin_delete_sq:
	case nvme_admin_ns_attach:
		goto complete;

	default:
		dev_err(&epf_nvme->epf->dev,
			"Unhandled admin command %s (0x%02x)\n",
			pci_epf_nvme_cmd_name(epcmd), epcmd->cmd.common.opcode);
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	if (transfer_len) {
		/* Get the host buffer segments and an internal buffer */
		ret = pci_epf_nvme_cmd_parse_dptr(epf_nvme, epcmd,
						  transfer_len);
		if (ret)
			goto complete;
	}

	pci_epf_nvme_submit_cmd(epf_nvme, epcmd, deferred);

	/* Command done: post process it and transfer data if needed */
	if (post_process_hook)
		post_process_hook(epf_nvme, epcmd);
	if (transfer_len)
		pci_epf_nvme_cmd_transfer(epf_nvme, epcmd, DMA_TO_DEVICE);

complete:
	if (deferred)
		list_add_tail(&epcmd->list_node,
			      &epcmd->epf_nvme->deferred_reqs);
	else
		pci_epf_nvme_queue_response(epcmd);

	return true;
}

static inline size_t pci_epf_nvme_rw_data_len(struct pci_epf_nvme_cmd *epcmd)
{
	return ((u32)le16_to_cpu(epcmd->cmd.rw.length) + 1) <<
		epcmd->ns->lba_shift;
}

static bool pci_epf_nvme_process_io_cmd(struct pci_epf_nvme *epf_nvme, int qid)
{
	enum dma_data_direction dir = DMA_NONE;
	struct pci_epf_nvme_cmd *epcmd;
	size_t transfer_len = 0;
	bool deferred = false;
	int ret;

	/* Fetch new command from IO submission queue */
	epcmd = pci_epf_nvme_fetch_cmd(epf_nvme, qid);
	if (!epcmd)
		return false;

	/* Get the command target namespace */
	epcmd->ns = nvme_find_get_ns(epf_nvme->ctrl.ctrl,
				     le32_to_cpu(epcmd->cmd.common.nsid));
	if (!epcmd->ns) {
		epcmd->status = NVME_SC_INVALID_NS | NVME_SC_DNR;
		goto complete;
	}

	switch (epcmd->cmd.common.opcode) {
	case nvme_cmd_read:
		transfer_len = pci_epf_nvme_rw_data_len(epcmd);
		dir = DMA_TO_DEVICE;
		break;

	case nvme_cmd_write:
		transfer_len = pci_epf_nvme_rw_data_len(epcmd);
		dir = DMA_FROM_DEVICE;
		deferred = true;
		break;

	case nvme_cmd_dsm:
		transfer_len = (le32_to_cpu(epcmd->cmd.dsm.nr) + 1) *
			sizeof(struct nvme_dsm_range);
		dir = DMA_FROM_DEVICE;
		goto complete;

	case nvme_cmd_flush:
	case nvme_cmd_write_zeroes:
		break;

	default:
		dev_err(&epf_nvme->epf->dev,
			"Unhandled IO command %s (0x%02x)\n",
			pci_epf_nvme_cmd_name(epcmd),
			epcmd->cmd.common.opcode);
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	if (transfer_len) {
		/* Setup the command buffer */
		ret = pci_epf_nvme_cmd_parse_dptr(epf_nvme, epcmd,
						  transfer_len);
		if (ret)
			goto complete;

		/* Get data from the host if needed */
		if (dir == DMA_FROM_DEVICE) {
			ret = pci_epf_nvme_cmd_transfer(epf_nvme, epcmd, dir);
			if (ret)
				goto complete;
		}
	}

	pci_epf_nvme_submit_cmd(epf_nvme, epcmd, deferred);

	/* Transfer command data to the host */
	if (transfer_len && dir == DMA_TO_DEVICE)
		pci_epf_nvme_cmd_transfer(epf_nvme, epcmd, dir);

complete:
	if (!deferred)
		pci_epf_nvme_queue_response(epcmd);

	return true;
}

static void pci_epf_nvme_chk_deferred_completions(struct pci_epf_nvme* epf_nvme)
{
	struct pci_epf_nvme_cmd *epcmd;
	struct pci_epf_nvme_cmd *epcmd_tmp;
	bool completed;

	list_for_each_entry_safe(epcmd, epcmd_tmp, &epf_nvme->deferred_reqs,
				 list_node) {
		/*
		 * Delete list entries here because pci_epf_nvme_queue_response
		 * frees epcmd
		 */
		if (epcmd->relay) {
			if (epf_nvme->tsp_relays[epcmd->relay_desc] == NULL) {
				/* The relay has been closed */
				dev_dbg(&epf_nvme->epf->dev, "Relay %d is closed\n",
					epcmd->relay_desc);
				list_del(&epcmd->list_node);
				epcmd->status = NVME_SC_DATA_XFER_ERROR;
				pci_epf_nvme_queue_response(epcmd);
			} else if (epf_nvme->tsp_relays[epcmd->relay_desc]->buffer_full) {
				/* There is data to move */
				list_del(&epcmd->list_node);
				/* Move it to host */
				tsp_nvme_cs_comm_to_host(epf_nvme, epcmd);
				pci_epf_nvme_queue_response(epcmd);
			}
		} else {
			completed = try_wait_for_completion(&epcmd->done);
			if (completed) {
				list_del(&epcmd->list_node);
				pci_epf_nvme_queue_response(epcmd);
			}
		}
	}
}

static void pci_epf_nvme_sq_poll(struct work_struct *work)
{
	struct pci_epf_nvme *epf_nvme =
		container_of(work, struct pci_epf_nvme, sq_poll.work);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	bool did_work = true;
	int qid;

	/* Process pending commands, starting with the IO queues */
	while (did_work && pci_epf_nvme_ctrl_ready(ctrl)) {
		did_work = false;
		for (qid = 1; qid < ctrl->nr_queues; qid++)
			did_work |= pci_epf_nvme_process_io_cmd(epf_nvme, qid);
		did_work |= pci_epf_nvme_process_admin_cmd(epf_nvme);
	}

	pci_epf_nvme_chk_deferred_completions(epf_nvme);

	if (!pci_epf_nvme_ctrl_ready(ctrl))
		return;

	queue_delayed_work(epf_nvme_sq_wq, &epf_nvme->sq_poll, 1);
}

static void pci_epf_nvme_reg_poll(struct work_struct *work)
{
	struct pci_epf_nvme *epf_nvme =
		container_of(work, struct pci_epf_nvme, reg_poll.work);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	bool shutdown;
	u32 old_cc;

	/* Check CC.EN to determine what we need to do */
	old_cc = ctrl->cc;
	ctrl->cc = pci_epf_nvme_reg_read32(ctrl, NVME_REG_CC);

	/* If not enabled yet, wait */
	if (!(old_cc & NVME_CC_ENABLE) && !(ctrl->cc & NVME_CC_ENABLE))
		goto again;

	/* If CC.EN was set by the host, enbale the controller */
	if (!(old_cc & NVME_CC_ENABLE) && (ctrl->cc & NVME_CC_ENABLE)) {
		pci_epf_nvme_enable_ctrl(epf_nvme);
		goto again;
	}

	/* If CC.EN was cleared by the host, disable the controller */
	shutdown = ctrl->cc & NVME_CC_SHN_NORMAL;
	if (((old_cc & NVME_CC_ENABLE) && !(ctrl->cc & NVME_CC_ENABLE)) ||
	    shutdown) {
		pci_epf_nvme_disable_ctrl(epf_nvme, shutdown);
		if (shutdown)
			return;
	}

again:
	queue_delayed_work(epf_nvme_reg_wq, &epf_nvme->reg_poll,
			   msecs_to_jiffies(5));
}

static int pci_epf_nvme_set_bars(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	enum pci_barno reg_bar = epf_nvme->reg_bar;
	struct pci_epf_bar *epf_bar;
	int bar, add;
	int ret;

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];

		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation requires a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		if (features->reserved_bar & (1 << bar))
			continue;

		if (features->fixed_bar & (1 << bar))
			continue;

		ret = pci_epf_set_bar(epf, epf_bar);
		if (ret) {
			dev_err(&epf->dev, "Failed to set BAR%d\n", bar);
			pci_epf_free_space(epf, epf_nvme->reg[bar], bar,
					   PRIMARY_INTERFACE);
			if (bar == reg_bar)
				return ret;
		}
	}

	/* Init CMB memory management context */
	tsp_mm_init_context(epf_nvme->reg[CMB_BIR],
			    epf_nvme->epf->bar[CMB_BIR].size,
			    &epf_nvme->tsp_alloc_ctx);

	return 0;
}

static int pci_epf_nvme_alloc_reg_bar(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	enum pci_barno reg_bar = epf_nvme->reg_bar;
	size_t reg_size, reg_bar_size;
	size_t msix_table_size = 0;
	void *reg_base;
	int ret;

	/*
	 * We want one MSI or MSIX per queue and are we want to keep queue
	 * pairs mapped. So we are also limited by the number of memory windows
	 * we have. Set our maximum number of queues based on these limits.
	 */
	epf_nvme->max_nr_queues = epf->epc->num_windows / 2 - 1;
	if (features->msix_capable)
		epf_nvme->max_nr_queues =
			min_t(unsigned int, epf->msix_interrupts,
			      epf_nvme->max_nr_queues);
	if (features->msi_capable)
		epf_nvme->max_nr_queues =
			min_t(unsigned int, epf->msi_interrupts,
			      epf_nvme->max_nr_queues);

	dev_info(&epf->dev, "Maximum number of queues: %u\n",
		 epf_nvme->max_nr_queues);

	/*
	 * Calculate the size of the register bar: NVMe registers first with
	 * enough space for the doorbells, followed by the MSIX table
	 * if supported.
	 */
	reg_size = NVME_REG_DBS + ((epf_nvme->max_nr_queues * 2) * sizeof(u32));
	reg_size = ALIGN(reg_size, 8);

	if (features->msix_capable) {
		size_t pba_size;

		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_nvme->msix_table_offset = reg_size;
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);

		reg_size += msix_table_size + pba_size;
	}

	reg_bar_size = ALIGN(reg_size, 4096);

	if (features->bar_fixed_size[reg_bar]) {
		if (reg_bar_size > features->bar_fixed_size[reg_bar]) {
			dev_err(&epf->dev,
				"Reg BAR %d size %llu B too small, need %zu B\n",
				reg_bar,
				features->bar_fixed_size[reg_bar],
				reg_bar_size);
			return -ENOMEM;
		}
		reg_bar_size = features->bar_fixed_size[reg_bar];
	}

	if (features->fixed_bar & (1 << reg_bar)) {
		ret = pci_epc_get_fixed_bar(epf->epc, epf->func_no,
					    epf->vfunc_no, reg_bar,
					    &epf->bar[reg_bar]);
		if (ret < 0) {
			dev_err(&epf->dev, "Failed to get fixed bar");
			return ret;
		}
		reg_base = epf->bar[reg_bar].addr;
	} else {
		reg_base = pci_epf_alloc_space(epf, reg_bar_size, reg_bar,
					   PAGE_SIZE, PRIMARY_INTERFACE);
	}

	if (!reg_base) {
		dev_err(&epf->dev, "Allocate register BAR failed\n");
		return -ENOMEM;
	}

	epf_nvme->reg[reg_bar] = reg_base;

	//memset_io(epf_nvme->reg[reg_bar], 0, reg_bar_size);
	/* Try clearing only 4k to see if it also fails */
	memset_io(epf_nvme->reg[reg_bar], 0, min(reg_bar_size, (size_t)SZ_4K));

	dev_dbg(&epf->dev,
		"BAR %d, virt addr 0x%llx, phys addr 0x%llx, %zu B\n",
		reg_bar, (u64)epf_nvme->reg[reg_bar],
		epf->bar[reg_bar].phys_addr, epf->bar[reg_bar].size);

	return 0;
}

static int pci_epf_nvme_configure_bars(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	struct pci_epf_bar *epf_bar;
	int bar, add, ret;
	size_t bar_size;
	void *base;

	/* The first free BAR will be our register BAR */
	bar = pci_epc_get_first_free_bar(features);
	if (bar < 0) {
		dev_err(&epf->dev, "No free BAR\n");
		return -EINVAL;
	}
	epf_nvme->reg_bar = bar;

	/* Initialize BAR flags */
	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];
		if (features->bar_fixed_64bit & (1 << bar))
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	}

	/* Allocate the register BAR */
	ret = pci_epf_nvme_alloc_reg_bar(epf);
	if (ret)
		return ret;

	/* Allocate remaining BARs */
	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		/*
		 * Skip the register BAR (already allocated) and
		 * reserved BARs.
		 */
		if (epf_nvme->reg[bar] || features->reserved_bar & (1 << bar))
			continue;

		if (features->fixed_bar & (1 << bar)) {
			ret = pci_epc_get_fixed_bar(epf->epc, epf->func_no,
						    epf->vfunc_no, bar,
						    epf_bar);
			if (ret < 0)
				base = NULL;
			else
				base = epf->bar[bar].addr;
		} else {
			bar_size = max_t(size_t, features->bar_fixed_size[bar], SZ_4K);
			base = pci_epf_alloc_space(epf, bar_size, bar,
					PAGE_SIZE, PRIMARY_INTERFACE);
		}

		if (!base) {
			dev_err(&epf->dev, "Allocate BAR%d failed\n", bar);
			return -ENOMEM;
		}

		epf_nvme->reg[bar] = base;

		/* Don't clear the other BARs for the moment */
		//memset_io(epf_nvme->reg[bar], 0, bar_size);
	}

	return 0;
}

static int pci_epf_nvme_init_irq(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	int ret;

	/* Enable MSIX if supported, otherwise, use MSI */
	if (epf_nvme->epc_features->msix_capable) {
		dev_info(&epf->dev, "MSIX capable, %d vectors\n",
			 epf->msix_interrupts);
		ret = pci_epf_set_msix(epf, epf->msix_interrupts,
				       epf_nvme->reg_bar,
				       epf_nvme->msix_table_offset);
		if (ret) {
			dev_err(&epf->dev, "MSI-X configuration failed\n");
			return ret;
		}

		epf_nvme->nr_vectors = epf->msix_interrupts;
		epf_nvme->irq_type = PCI_IRQ_MSIX;

		return 0;
	}

	if (epf_nvme->epc_features->msi_capable) {
		dev_info(&epf->dev, "MSI capable, %d vectors\n",
			 epf->msi_interrupts);
		ret = pci_epf_set_msi(epf, epf->msi_interrupts);
		if (ret) {
			dev_err(&epf->dev, "MSI configuration failed\n");
			return ret;
		}

		epf_nvme->nr_vectors = epf->msi_interrupts;
		epf_nvme->irq_type = PCI_IRQ_MSI;

		return 0;
	}

	/* MSI and MSIX are not supported. Use INTX */
	epf_nvme->nr_vectors = 1;
	epf_nvme->irq_type = PCI_IRQ_LEGACY;

	return 0;
}

static int pci_epf_nvme_core_init(struct pci_epf *epf)
{
	int ret;

	if (epf->vfunc_no <= 1) {
		/* Set device ID, class, etc */
		ret = pci_epf_write_header(epf, epf->header);
		if (ret) {
			dev_err(&epf->dev,
				"Write configuration header failed %d\n", ret);
			return ret;
		}
	}

	/* Setup the PCIe BARs and enable interrupts */
	ret = pci_epf_nvme_set_bars(epf);
	if (ret)
		return ret;

	ret = pci_epf_nvme_init_irq(epf);
	if (ret)
		return ret;

	/*
	 * Create the fabrics host controller and initialize the attributes
	 * of the PCI controller for the host to see.
	 */
	ret = pci_epf_nvme_create_ctrl(epf);
	if (ret)
		return ret;

	return 0;
}

static int pci_epf_nvme_link_up(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	int ret;

	if (!epf_nvme->ctrl.ctrl) {
		/*
		 * We come here if the host was restarted after
		 * shutting down the controller, e.g. a normal reboot.
		 */
		ret = pci_epf_nvme_create_ctrl(epf);
		if (ret)
			return ret;
	}

	/* Start polling the BAR registers to detect controller enable */
	queue_delayed_work(epf_nvme_reg_wq, &epf_nvme->reg_poll,
			   msecs_to_jiffies(1));

	return 0;
}

static const struct pci_epf_event_ops pci_epf_nvme_event_ops = {
	.core_init = pci_epf_nvme_core_init,
	.link_up = pci_epf_nvme_link_up,
};

static int pci_epf_nvme_bind(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	int ret;

	if (!epc) {
		dev_err(&epf->dev, "No endpoint controller\n");
		return -EINVAL;
	}

	epc_features = pci_epf_get_features(epf);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}
	epf_nvme->epc_features = epc_features;

	ret = pci_epf_nvme_configure_bars(epf);
	if (ret)
		return ret;

	if (!epc_features->core_init_notifier) {
		ret = pci_epf_nvme_core_init(epf);
		if (ret)
			return ret;
	}

	if (epf_nvme->dma_enable) {
		epf_nvme->dma_supported = pci_epf_nvme_init_dma(epf_nvme);
		if (epf_nvme->dma_supported) {
			dev_info(&epf->dev, "DMA supported\n");
		} else {
			dev_info(&epf->dev,
				 "DMA not supported, falling back to mmio\n");
			epf_nvme->dma_enable = false;
		}
	} else {
		dev_info(&epf->dev, "DMA disabled\n");
	}

	if (!epc_features->linkup_notifier && !epc_features->core_init_notifier)
		queue_delayed_work(epf_nvme_reg_wq, &epf_nvme->reg_poll,
				   msecs_to_jiffies(1));

	return 0;
}

static void pci_epf_nvme_unbind(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	int bar;

	cancel_delayed_work(&epf_nvme->reg_poll);

	pci_epf_nvme_disable_ctrl(epf_nvme, true);

	pci_epf_nvme_clean_dma(epf_nvme);

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_nvme->reg[bar])
			continue;
		pci_epf_clear_bar(epf, &epf->bar[bar]);
		pci_epf_free_space(epf, epf_nvme->reg[bar], bar,
				   PRIMARY_INTERFACE);
	}
}

static struct pci_epf_header epf_nvme_pci_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.progif_code	= 0x02, /* NVM Express */
	.baseclass_code = PCI_BASE_CLASS_STORAGE,
	.subclass_code	= 0x08, /* Non-Volatile Memory controller */
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static int pci_epf_nvme_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct pci_epf_nvme *epf_nvme;

	epf_nvme = devm_kzalloc(&epf->dev, sizeof(*epf_nvme), GFP_KERNEL);
	if (!epf_nvme)
		return -ENOMEM;

	epf_nvme->epf = epf;
	INIT_DELAYED_WORK(&epf_nvme->reg_poll, pci_epf_nvme_reg_poll);
	INIT_DELAYED_WORK(&epf_nvme->sq_poll, pci_epf_nvme_sq_poll);

	dev_info(&epf_nvme->epf->dev, "Version with deferred REQs\n");
	INIT_LIST_HEAD(&epf_nvme->deferred_reqs);

	epf_nvme->prp_list_buf = devm_kzalloc(&epf->dev, NVME_CTRL_PAGE_SIZE,
					      GFP_KERNEL);
	if (!epf_nvme->prp_list_buf)
		return -ENOMEM;

	/* Set default attribute values */
	epf_nvme->dma_enable = true;

	epf->event_ops = &pci_epf_nvme_event_ops;
	epf->header = &epf_nvme_pci_header;
	epf_set_drvdata(epf, epf_nvme);

	return 0;
}

#define to_epf_nvme(epf_group)						\
	container_of((epf_group), struct pci_epf_nvme, group)

static ssize_t pci_epf_nvme_ctrl_opts_show(struct config_item *item,
					   char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	if (!epf_nvme->ctrl_opts_buf)
		return 0;

	return sysfs_emit(page, "%s\n", epf_nvme->ctrl_opts_buf);
}

#define PCI_EPF_NVME_OPT_HIDDEN_NS	"hidden_ns"

static ssize_t pci_epf_nvme_ctrl_opts_store(struct config_item *item,
					    const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	size_t opt_buf_size;

	/* Do not allow setting options when the function is already started */
	if (epf_nvme->ctrl.ctrl)
		return -EBUSY;

	if (!len)
		return -EINVAL;

	if (epf_nvme->ctrl_opts_buf)
		kfree(epf_nvme->ctrl_opts_buf);

	/*
	 * Make sure we have enough room to add the hidden_ns option
	 * if it is missing.
	 */
	opt_buf_size = len + strlen(PCI_EPF_NVME_OPT_HIDDEN_NS) + 2;
	epf_nvme->ctrl_opts_buf = kzalloc(opt_buf_size, GFP_KERNEL);
	if (!epf_nvme->ctrl_opts_buf)
		return -ENOMEM;

	strcpy(epf_nvme->ctrl_opts_buf, page);
	if (!strnstr(page, PCI_EPF_NVME_OPT_HIDDEN_NS, len))
		strncat(epf_nvme->ctrl_opts_buf,
			"," PCI_EPF_NVME_OPT_HIDDEN_NS, opt_buf_size);

	dev_dbg(&epf_nvme->epf->dev,
		"NVMe fabrics controller options: %s\n",
		epf_nvme->ctrl_opts_buf);

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, ctrl_opts);

static ssize_t pci_epf_nvme_dma_enable_show(struct config_item *item,
					    char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	return sysfs_emit(page, "%d\n", epf_nvme->dma_enable);
}

static ssize_t pci_epf_nvme_dma_enable_store(struct config_item *item,
					     const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	int ret;

	ret = kstrtobool(page, &epf_nvme->dma_enable);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, dma_enable);

static struct configfs_attribute *pci_epf_nvme_attrs[] = {
	&pci_epf_nvme_attr_ctrl_opts,
	&pci_epf_nvme_attr_dma_enable,
	NULL,
};

static const struct config_item_type pci_epf_nvme_group_type = {
	.ct_attrs	= pci_epf_nvme_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *pci_epf_nvme_add_cfs(struct pci_epf *epf,
						 struct config_group *group)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);

	/* Add the NVMe target attributes */
	config_group_init_type_name(&epf_nvme->group, "nvme",
				    &pci_epf_nvme_group_type);

	return &epf_nvme->group;
}

static const struct pci_epf_device_id pci_epf_nvme_ids[] = {
	{ .name = "pci_epf_nvme" },
	{},
};

static struct pci_epf_ops pci_epf_nvme_ops = {
	.bind	= pci_epf_nvme_bind,
	.unbind	= pci_epf_nvme_unbind,
	.add_cfs = pci_epf_nvme_add_cfs,
};

static struct pci_epf_driver epf_nvme_driver = {
	.driver.name	= "pci_epf_nvme",
	.probe		= pci_epf_nvme_probe,
	.id_table	= pci_epf_nvme_ids,
	.ops		= &pci_epf_nvme_ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_nvme_init(void)
{
	int ret;


	epf_nvme_reg_wq = alloc_workqueue("epf_nvme_reg",
					  WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!epf_nvme_reg_wq)
		return -ENOMEM;

	epf_nvme_sq_wq = alloc_workqueue("epf_nvme_sq",
					 WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!epf_nvme_sq_wq)
		goto out_reg_wq;

	epf_nvme_cmd_cache = kmem_cache_create("epf_nvme_cmd",
					sizeof(struct pci_epf_nvme_cmd),
					0, SLAB_HWCACHE_ALIGN, NULL);
	if (!epf_nvme_cmd_cache)
		goto out_sq_wq;

	ret = pci_epf_register_driver(&epf_nvme_driver);
	if (ret)
		goto out_cache;

	pr_info("Registered driver\n");

	return 0;

out_cache:
	kmem_cache_destroy(epf_nvme_cmd_cache);
out_sq_wq:
	destroy_workqueue(epf_nvme_sq_wq);
out_reg_wq:
	destroy_workqueue(epf_nvme_reg_wq);

	pr_info("Register driver failed\n");

	return ret;
}
module_init(pci_epf_nvme_init);

static void __exit pci_epf_nvme_exit(void)
{
	if (epf_nvme_reg_wq)
		destroy_workqueue(epf_nvme_reg_wq);
	if (epf_nvme_sq_wq)
		destroy_workqueue(epf_nvme_sq_wq);

	pci_epf_unregister_driver(&epf_nvme_driver);

	kmem_cache_destroy(epf_nvme_cmd_cache);

	pr_info("Unregistered driver\n");
}
module_exit(pci_epf_nvme_exit);

MODULE_DESCRIPTION("PCI endpoint NVMe function driver");
MODULE_AUTHOR("Damien Le Moal <dlemoal@kernel.org>");
MODULE_IMPORT_NS(NVME_TARGET_PASSTHRU);
MODULE_IMPORT_NS(NVME_FABRICS);
MODULE_LICENSE("GPL v2");
