/*
 * Coherent per-device memory handling.
 * Borrowed from i386
 */
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

struct dma_block {
	/* Offset of this block in the DMA memory range */
	size_t pg_offset;
	/* Size of this block in pages */
	size_t pg_size;
	/* True if the block is available */
	int    free;
};

#define DMA_BLOCK_DEFAULT_CAPACITY 64

struct dma_block_list {
	/* List of blocks */
	struct dma_block *blocks;
	/* Number of blocks in the list */
	size_t           count;
	/* Capacity of the block list (how much we can grow before we have to
	 * reallocate) */
	size_t           capacity;
	spinlock_t       lock;
};

struct dma_coherent_mem {
	void                  *virt_base;
	dma_addr_t            device_base;
	int                   size;
	int                   flags;
	struct dma_block_list block_list;
};

static struct dma_coherent_mem dma_sys_mem;

static int dma_init_block_list(struct dma_block_list *block_list,
			       size_t pg_size)
{
	spin_lock_init(&block_list->lock);

	block_list->blocks =
		kzalloc(sizeof(*block_list->blocks) *
			DMA_BLOCK_DEFAULT_CAPACITY,
			GFP_KERNEL);

	if (block_list->blocks == NULL) {
		return -ENOMEM;
	}

	block_list->capacity = DMA_BLOCK_DEFAULT_CAPACITY;

	/* Start with a single block spanning the entire memory range */
	block_list->blocks[0].pg_offset = 0;
	block_list->blocks[0].pg_size   = pg_size;
	block_list->blocks[0].free      = 1;

	block_list->count = 1;

	return 0;
}

static void dma_free_block_list(struct dma_block_list *block_list)
{
	kfree(block_list->blocks);
	block_list->capacity = 0;
	block_list->count = 0;
}

/* Must be called with block_list->lock held */
static int dma_insert_block_after(struct dma_block_list *block_list,
				  struct dma_block *new_block,
				  size_t pos)
{
	size_t i;

	/* Make sure we have room for the new entry */
	if (block_list->count == block_list->capacity) {
		struct dma_block *tmp;
		size_t new_capacity = block_list->capacity * 2;

		tmp = krealloc(block_list->blocks,
			       sizeof(*block_list->blocks) * new_capacity,
			       GFP_ATOMIC);

		if (tmp == NULL) {
			return -ENOMEM;
		}

		block_list->capacity = new_capacity;
		block_list->blocks = tmp;
	}

	/* Move all entries after pos forward to leave room for the new entry */
	for (i = block_list->count - 1; i > pos; i--) {
		block_list->blocks[i + 1] = block_list->blocks[i];
	}

	/* Add the new entry */
	block_list->blocks[pos + 1] = *new_block;
	block_list->count++;

	return 0;
}

/* Must be called with block_list->lock held */
static void dma_block_delete(struct dma_block_list *block_list, size_t pos)
{
	size_t i;

	/* Overwrite deleted entry */
	for (i = pos + 1; i < block_list->count; i++) {
		block_list->blocks[i - 1] = block_list->blocks[i];
	}

	block_list->count--;
}

static int dma_block_alloc(struct dma_block_list *block_list,
			   size_t pg_size,
			   size_t *pg_offset)
{
	struct dma_block *blocks;
	struct dma_block new_block;
	unsigned long    flags;
	size_t           i;
	int              ret = 0;

	if (pg_size == 0) {
		return -EINVAL;
	}

	spin_lock_irqsave(&block_list->lock, flags);

	blocks = block_list->blocks;

	for (i = 0; i < block_list->count; i++) {
		if (blocks[i].free && blocks[i].pg_size >= pg_size) {
			/* Found a block big enough */
			break;
		}
	}

	if (i >= block_list->count) {
		/* Not enough memory left (or too fragmented) */
		ret = -ENOMEM;
		goto out;
	}

	*pg_offset = blocks[i].pg_offset;

	if (blocks[i].pg_size == pg_size) {
		/* The block is exactly the size we need, we're done */
		blocks[i].free = 0;
		goto out;
	}

	/* We need to split the block to salvage the remaining free space */
	new_block.pg_offset = blocks[i].pg_offset + pg_size;
	new_block.pg_size   = blocks[i].pg_size   - pg_size;
	new_block.free      = 1;

	ret = dma_insert_block_after(block_list, &new_block, i);
	if (ret) {
		goto out;
	}

	/* dma_insert_block_after can reallocate the block buffer */
	blocks = block_list->blocks;

	blocks[i].pg_size = pg_size;
	blocks[i].free = 0;

 out:
	spin_unlock_irqrestore(&block_list->lock, flags);
	return ret;
}

static void dma_block_free(struct dma_block_list *block_list,
			   size_t pg_offset)
{
	struct dma_block *blocks;
	unsigned long    flags;
	size_t           i;

	spin_lock_irqsave(&block_list->lock, flags);

	blocks = block_list->blocks;

	for (i = 0; i < block_list->count; i++) {
		if (blocks[i].pg_offset == pg_offset) {
			/* Found the block */
			break;
		}
	}

	/* Make sure the block has been found */
	if (i >= block_list->count) {
		spin_unlock_irqrestore(&block_list->lock, flags);
		WARN_ON(1);
		goto out;
	}

	WARN_ON(blocks[i].free);

	blocks[i].free = 1;

	/* Check if we can merge adjacent blocks */
	if (i + 1 < block_list->capacity && blocks[i + 1].free) {
		blocks[i].pg_size += blocks[i + 1].pg_size;
		dma_block_delete(block_list, i + 1);
	}

	if (i > 0 && blocks[i - 1].free) {
		blocks[i - 1].pg_size += blocks[i].pg_size;
		dma_block_delete(block_list, i);
	}

out:
	spin_unlock_irqrestore(&block_list->lock, flags);
}

void __init dma_init_coherent_mem(dma_addr_t bus_addr, dma_addr_t device_addr,
                                  size_t size, int flags)
{
	void __iomem *mem_base = NULL;
	int pages = size >> PAGE_SHIFT;

	pr_info("Dma coherent allocator extended enabled\n");

#ifdef DEBUG
	BUG_ON(!(flags & (DMA_MEMORY_MAP | DMA_MEMORY_IO)));
	BUG_ON(!size);
	BUG_ON(dma_sys_mem.virt_base);
	BUG_ON(dma_sys_mem.size);
	BUG_ON(dma_sys_mem.bitmap);
#endif

	mem_base = ioremap(bus_addr, size);
	if (!mem_base)
		goto out;

	if (dma_init_block_list(&dma_sys_mem.block_list, pages))
		goto out;

	dma_sys_mem.virt_base = mem_base;
	dma_sys_mem.device_base = device_addr;
	dma_sys_mem.size = pages;
	dma_sys_mem.flags = flags;

	return;

out:
	if (mem_base)
		iounmap(mem_base);
	panic("failed to remap global coherent memory pool\n");
}

int dma_declare_coherent_memory(struct device *dev, dma_addr_t bus_addr,
				dma_addr_t device_addr, size_t size, int flags)
{
	void __iomem *mem_base = NULL;
	int pages = size >> PAGE_SHIFT;

	if ((flags & (DMA_MEMORY_MAP | DMA_MEMORY_IO)) == 0)
		goto out;
	if (!size)
		goto out;
	if (dev->dma_mem)
		goto out;

	/* FIXME: this routine just ignores DMA_MEMORY_INCLUDES_CHILDREN */

	mem_base = ioremap(bus_addr, size);
	if (!mem_base)
		goto out;

	dev->dma_mem = kzalloc(sizeof(struct dma_coherent_mem), GFP_KERNEL);
	if (!dev->dma_mem)
		goto out;

	if (dma_init_block_list(&dev->dma_mem->block_list, pages))
		goto free1_out;

	dev->dma_mem->virt_base = mem_base;
	dev->dma_mem->device_base = device_addr;
	dev->dma_mem->size = pages;
	dev->dma_mem->flags = flags;

	if (flags & DMA_MEMORY_MAP)
		return DMA_MEMORY_MAP;

	return DMA_MEMORY_IO;

 free1_out:
	kfree(dev->dma_mem);
 out:
	if (mem_base)
		iounmap(mem_base);
	return 0;
}
EXPORT_SYMBOL(dma_declare_coherent_memory);

void dma_release_declared_memory(struct device *dev)
{
	struct dma_coherent_mem *mem = dev->dma_mem;

	if (!mem)
		return;
	dev->dma_mem = NULL;
	iounmap(mem->virt_base);
	dma_free_block_list(&mem->block_list);
	kfree(mem);
}
EXPORT_SYMBOL(dma_release_declared_memory);

void *dma_mark_declared_memory_occupied(struct device *dev,
					dma_addr_t device_addr, size_t size)
{
	struct dma_coherent_mem *mem = dev->dma_mem;
	/* int pos, err; */

	size += device_addr & ~PAGE_MASK;

	if (!mem)
		return ERR_PTR(-EINVAL);

	/* Feature not supported for now */
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(dma_mark_declared_memory_occupied);

static int dma_alloc_from_mem(struct dma_coherent_mem *mem, ssize_t size,
                              dma_addr_t *dma_handle, void **ret)
{
	size_t pg_size = (roundup(size, 1 << PAGE_SHIFT)) >> PAGE_SHIFT;
	size_t pageno;

	if (unlikely(pg_size > mem->size))
		goto err;

	if (dma_block_alloc(&mem->block_list, pg_size, &pageno)) {
		goto err;
	}

	/*
	 * Memory was found in the per-device area.
	 */
	*dma_handle = mem->device_base + (pageno << PAGE_SHIFT);
	*ret = mem->virt_base + (pageno << PAGE_SHIFT);
	memset(*ret, 0, size);

	return 1;

err:
	/*
	 * In the case where the allocation can not be satisfied from the
	 * per-device area, try to fall back to generic memory if the
	 * constraints allow it.
	 */

	return mem->flags & DMA_MEMORY_EXCLUSIVE;
}

/**
 * dma_alloc_from_coherent() - try to allocate memory from the per-device coherent area
 *
 * @dev:	device from which we allocate memory
 * @size:	size of requested memory area
 * @dma_handle:	This will be filled with the correct dma handle
 * @ret:	This pointer will be filled with the virtual address
 *		to allocated area.
 *
 * This function should be only called from per-arch dma_alloc_coherent()
 * to support allocation from per-device coherent memory pools.
 *
 * Returns 0 if dma_alloc_coherent should continue with allocating from
 * generic memory areas, or !0 if dma_alloc_coherent should return @ret.
 */
int dma_alloc_from_coherent(struct device *dev, ssize_t size,
				       dma_addr_t *dma_handle, void **ret)
{
	struct dma_coherent_mem *mem = NULL;

	*ret = NULL;

	if (dev)
		mem = dev->dma_mem;
	if(!mem)
		mem = &dma_sys_mem;

	return dma_alloc_from_mem(mem, size, dma_handle, ret);
}
EXPORT_SYMBOL(dma_alloc_from_coherent);

/**
 * dma_release_from_coherent() - try to free the memory allocated from per-device coherent memory pool
 * @dev:	device from which the memory was allocated
 * @order:	the order of pages allocated
 * @vaddr:	virtual address of allocated pages
 *
 * This checks whether the memory was allocated from the per-device
 * coherent memory pool and if so, releases that memory.
 *
 * Returns 1 if we correctly released the memory, or 0 if
 * dma_release_coherent() should proceed with releasing memory from
 * generic pools.
 */
int dma_release_from_coherent(struct device *dev, int order, void *vaddr)
{
	struct dma_coherent_mem *mem = dev ? dev->dma_mem : NULL;

	if (!mem && dma_sys_mem.size)
		mem = &dma_sys_mem;

	if (mem && vaddr >= mem->virt_base && vaddr <
		   (mem->virt_base + (mem->size << PAGE_SHIFT))) {
		int page = (vaddr - mem->virt_base) >> PAGE_SHIFT;

		dma_block_free(&mem->block_list, page);
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(dma_release_from_coherent);

/**
 * dma_mmap_from_coherent - map a coherent DMA allocation into user space
 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
 * @vma: vm_area_struct describing requested user mapping
 * @cpu_addr: kernel CPU-view address returned from dma_alloc_coherent
 * @dma_addr: device-view address returned from dma_alloc_coherent
 * @size: size of memory originally requested in dma_alloc_coherent
 *
 * Map a coherent DMA buffer previously allocated by dma_alloc_coherent
 * into user space.  The coherent DMA buffer must not be freed by the
 * driver until the user space mapping has been released.
 */
int dma_mmap_from_coherent(struct device *dev,
                           struct vm_area_struct *vma,
                           void *cpu_addr,
                           dma_addr_t dma_addr,
                           size_t size)
{
	struct dma_coherent_mem *mem = NULL;

	if (dev)
		mem = dev->dma_mem;
	if(!mem)
		mem = &dma_sys_mem;

	if (cpu_addr < mem->virt_base ||
	    ((size_t) cpu_addr + size) >
	    ((size_t) mem->virt_base + ((size_t) mem->size << PAGE_SHIFT)))
		return -ENXIO;

	vma->vm_page_prot = pgprot_dmacoherent(vm_get_page_prot(vma->vm_flags));
	return remap_pfn_range(vma,
	                       vma->vm_start,
	                       PFN_DOWN(dma_addr),
	                       PAGE_ALIGN(size),
	                       vma->vm_page_prot);

}
EXPORT_SYMBOL(dma_mmap_from_coherent);
