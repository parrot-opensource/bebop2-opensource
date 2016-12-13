/*
 * Coherent per-device memory handling.
 * Borrowed from i386
 */
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

struct dma_coherent_mem {
	void		*virt_base;
	dma_addr_t	device_base;
	int		size;
	int		flags;
	unsigned long	*bitmap;
};

static struct dma_coherent_mem dma_sys_mem;

void __init dma_init_coherent_mem(dma_addr_t bus_addr, dma_addr_t device_addr,
                                  size_t size, int flags)
{
	void __iomem *mem_base = NULL;
	int pages = size >> PAGE_SHIFT;

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

	dma_sys_mem.bitmap = kzalloc(BITS_TO_LONGS(pages) * sizeof(long), GFP_KERNEL);
	if (!dma_sys_mem.bitmap)
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
	int bitmap_size = BITS_TO_LONGS(pages) * sizeof(long);

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
	dev->dma_mem->bitmap = kzalloc(bitmap_size, GFP_KERNEL);
	if (!dev->dma_mem->bitmap)
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
	kfree(mem->bitmap);
	kfree(mem);
}
EXPORT_SYMBOL(dma_release_declared_memory);

void *dma_mark_declared_memory_occupied(struct device *dev,
					dma_addr_t device_addr, size_t size)
{
	struct dma_coherent_mem *mem = dev->dma_mem;
	int pos, err;

	size += device_addr & ~PAGE_MASK;

	if (!mem)
		return ERR_PTR(-EINVAL);

	pos = (device_addr - mem->device_base) >> PAGE_SHIFT;
	err = bitmap_allocate_region(mem->bitmap, pos, get_order(size));
	if (err != 0)
		return ERR_PTR(err);
	return mem->virt_base + (pos << PAGE_SHIFT);
}
EXPORT_SYMBOL(dma_mark_declared_memory_occupied);

static int dma_alloc_from_mem(struct dma_coherent_mem *mem, ssize_t size,
                              dma_addr_t *dma_handle, void **ret)
{
	int order = get_order(size);
	int pageno;

	if (unlikely(size > (mem->size << PAGE_SHIFT)))
		goto err;

	pageno = bitmap_find_free_region(mem->bitmap, mem->size, order);
	if (unlikely(pageno < 0))
		goto err;

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

		bitmap_release_region(mem->bitmap, page, order);
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
