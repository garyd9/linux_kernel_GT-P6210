/*
 * videobuf2-core.h - V4L2 driver helper framework
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Pawel Osciak <p.osciak@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */
#ifndef _MEDIA_VIDEOBUF2_CORE_H
#define _MEDIA_VIDEOBUF2_CORE_H

#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/videodev2.h>

struct vb2_alloc_ctx;
struct vb2_fileio_data;

/**
 * struct vb2_mem_ops - memory handling/memory allocator operations
 * @alloc:	allocate video memory and, optionally, allocator private data,
 *		return NULL on failure or a pointer to allocator private,
 *		per-buffer data on success; the returned private structure
 *		will then be passed as buf_priv argument to other ops in this
 *		structure
 * @put:	inform the allocator that the buffer will no longer be used;
 *		usually will result in the allocator freeing the buffer (if
 *		no other users of this buffer are present); the buf_priv
 *		argument is the allocator private per-buffer structure
 *		previously returned from the alloc callback
 * @get_userptr: acquire userspace memory for a hardware operation; used for
 *		 USERPTR memory types; alloc_ctx is the allocator private data,
 *		 vaddr is the address passed to the videobuf layer when
 *		 queuing a video buffer of USERPTR type, size is the size of
 *		 that buffer and write=1 if the buffer will be written to
 *		 by kernel/hardware (i.e. is a CAPTURE buffer);
 *		 should return an allocator private per-buffer structure
 *		 associated with the buffer on success, NULL on failure;
 *		 the returned private structure will then be passed as buf_priv
 *		 argument to other ops in this structure
 * @put_userptr: inform the allocator that a USERPTR buffer will no longer
 *		 be used
 * @vaddr:	return a kernel virtual address to a given memory buffer
 *		associated with the passed private structure or NULL if no
 *		such mapping exists
 * @cookie:	return allocator-specific cookie for a given memory buffer,
 *		associated with the passed private structure, or NULL if not
 *		available; a cookie is a unique identifier for a buffer that
 *		driver will be able to use when communicating with hardware;
 *		for example, for devices accessing physical memory directly via
 *		physical addresses, it can point to a variable containing
 *		a physical address for the given buffer, which the driver can
 *		then pass to the device when setting up a HW operation
 * @num_users:	return the current number of users of a memory buffer;
 *		return 1 if the videobuf layer (or actually the driver using
 *		it) is the only user
 * @mmap:	setup a userspace mapping for a given memory buffer under
 *		the provided virtual memory region
 *
 * Required ops for USERPTR types: get_userptr, put_userptr.
 * Required ops for MMAP types: alloc, put, num_users, mmap.
 * Required ops for read/write access types: alloc, put, num_users, vaddr
 */
struct vb2_mem_ops {
	void		*(*alloc)(void *alloc_ctx, unsigned long size);
	void		(*put)(void *buf_priv);

	void		*(*get_userptr)(void *alloc_ctx, unsigned long vaddr,
					unsigned long size, int write);
	void		(*put_userptr)(void *buf_priv);

	void		*(*vaddr)(void *buf_priv);
	void		*(*cookie)(void *buf_priv);

	unsigned int	(*num_users)(void *buf_priv);

	int		(*mmap)(void *buf_priv, struct vm_area_struct *vma);
};

struct vb2_plane {
	void		*mem_priv;
	unsigned int	mapped:1;
};

/**
 * enum vb2_io_modes - queue access methods
 * @VB2_MMAP:		driver supports MMAP with streaming API
 * @VB2_USERPTR:	driver supports USERPTR with streaming API
 * @VB2_READ:		driver supports read() style access
 * @VB2_WRITE:		driver supports write() style access
 */
enum vb2_io_modes {
	VB2_MMAP	= (1 << 0),
	VB2_USERPTR	= (1 << 1),
	VB2_READ	= (1 << 2),
	VB2_WRITE	= (1 << 3),
};

/**
 * enum vb2_fileio_flags - flags for selecting file IO (read/write) behavior
 * @VB2_FILEIO_READ_ONCE:	if set, the userspace will be able to read one
 * 				full buffer only, an EOF will be reported after
 * 				the first buffer has been read completely
 * @VB2_FILEIO_WRITE_IMMEDIATELY: if set, do not wait for the buffer to be
 * 				  filled completely by userspace (possibly
 * 				  using multiple write() calls), but send it
 * 				  to the device immediately after the first
 * 				  write() call
 */
enum vb2_fileio_flags {
	VB2_FILEIO_READ_ONCE		= (1 << 0),
	VB2_FILEIO_WRITE_IMMEDIATELY	= (1 << 1),
};

/**
 * enum vb2_buffer_state - current video buffer state
 * @VB2_BUF_STATE_DEQUEUED:	buffer under userspace control
 * @VB2_BUF_STATE_QUEUED:	buffer queued in videobuf, but not in driver
 * @VB2_BUF_STATE_ACTIVE:	buffer queued in driver and possibly used
 *				in a hardware operation
 * @VB2_BUF_STATE_DONE:		buffer returned from driver to videobuf, but
 *				not yet dequeued to userspace
 * @VB2_BUF_STATE_ERROR:	same as above, but the operation on the buffer
 *				has ended with an error, which will be reported
 *				to the userspace when it is dequeued
 */
enum vb2_buffer_state {
	VB2_BUF_STATE_DEQUEUED,
	VB2_BUF_STATE_QUEUED,
	VB2_BUF_STATE_ACTIVE,
	VB2_BUF_STATE_DONE,
	VB2_BUF_STATE_ERROR,
};

struct vb2_queue;

/**
 * struct vb2_buffer - represents a video buffer
 * @v4l2_buf:		struct v4l2_buffer associated with this buffer; can
 *			be read by the driver and relevant entries can be
 *			changed by the driver in case of CAPTURE types
 *			(such as timestamp); NOTE that even for single-planar
 *			types, the v4l2_planes[0] struct should be used
 *			instead of v4l2_buf for filling bytesused - drivers
 *			should use the vb2_set_plane_payload() function for that
 * @v4l2_planes:	struct v4l2_planes associated with this buffer; can
 *			be read by the driver and relevant entries can be
 *			changed by the driver in case of CAPTURE types
 *			(such as bytesused); NOTE that even for single-planar
 *			types, the v4l2_planes[0] struct should be used
 *			instead of v4l2_buf for filling bytesused - drivers
 *			should use the vb2_set_plane_payload() function for that
 * @vb2_queue:		the queue to which this driver belongs
 * @num_planes:		number of planes in the buffer
 * @state:		current buffer state; do not use
 * @queued_entry:	entry on the queued buffers list, which holds all
 *			buffers queued from userspace; do not use
 * @done_entry:		entry on the list that stores all buffers ready to
 *			be dequeued to userspace; do not use
 * @planes:		private per-plane information; do not use
 * @num_planes_mapped:	number of mapped planes; do not use
 */
struct vb2_buffer {
	struct v4l2_buffer	v4l2_buf;
	struct v4l2_plane	v4l2_planes[VIDEO_MAX_PLANES];

	struct vb2_queue	*vb2_queue;

	unsigned int		num_planes;

/* Private: internal use only */
	enum vb2_buffer_state	state;

	struct list_head	queued_entry;
	struct list_head	done_entry;

	struct vb2_plane	planes[VIDEO_MAX_PLANES];
	unsigned int		num_planes_mapped;
};

/**
 * struct vb2_ops - driver-specific callbacks to be implemented by the driver
 * Required: queue_setup, buf_queue. The rest is optional.
 *
 * @queue_setup:	used to negotiate queue parameters between the userspace
 *			and the driver; called before memory allocation;
 *			the number of buffers requested by userspace will be
 *			passed in num_buffers, which the driver can change;
 *			the driver has to set the required number of planes per
 *			buffer for the current format in num_planes and put
 *			the size of each plane in the sizes[] array (sizes[i]
 *			being the size of i-th plane for all buffers);
 *			the driver can also put optional, per-plane
 *			allocator-specific contexts in alloc_ctxs[] array;
 *			if provided, allocator contexts will be passed to the
 *			memory allocator when allocating each plane,
 *			alloc_ctx[i] being passed to the alloc() memory
 *			operation on allocating i-th plane for each buffer;
 * @wait_prepare:	asks the driver to release any locks that should not be
 *			held while sleeping and all locks protecting ioctl calls
 *			in the driver; it is called before videobuf needs
 *			to put the driver to sleep, e.g. to wait for new buffers
 *			to arrive; as new buffers can only arrive via an another
 *			ioctl call, locks that protect those calls have
 *			to be released here as well;
 * @wait_finish:	asks the driver to reacquire locks released in
 *			wait_prepare; called after waking up;
 * @buf_init:		called once after allocating a buffer (in MMAP case)
 *			or after acquiring a new USERPTR buffer; drivers may
 *			perform additional buffer-related initialization here;
 *			a failure (return != 0) will prevent queue setup from
 *			completing successfully;
 * @buf_prepare:	called each time a buffer is queued from userspace;
 *			drivers may perform any additional initialization steps
 *			that need to be done before every hardware operation
 *			in this callback; if an error is returned, the buffer
 *			will not be queued;
 * @buf_finish:		a counterpart to buf_prepare; called each time a buffer
 *			is about to be dequeued back to the userspace; drivers
 *			may perform any operations required before the buffer
 *			can be accessed by userspace here;
 * @buf_cleanup:	a counterpart to buf_init; called once before a buffer
 *			is freed; drivers may perform any additional cleanup
 *			here;
 * @start_streaming:	called once before entering the 'streaming' state;
 *			can be used to perform any additional steps required by
 *			the driver before streaming begins (such as enabling
 *			the device);
 * @stop_streaming:	called when the 'streaming' state must be disabled;
 * 			drivers should stop any DMA transactions here (or wait
 * 			until they are finished) and give back all the buffers
 * 			received via buf_queue() by calling vb2_buffer_done()
 * 			for each of them;
 * 			drivers can use the vb2_wait_for_all_buffers() function
 * 			here to wait for asynchronous completion events that
 * 			call vb2_buffer_done(), such as ISRs;
 * @buf_queue:		passes a buffer to the driver; the driver may start
 *			a hardware operation on that buffer; this callback
 *			MUST return immediately, i.e. it may NOT wait for
 *			the end of a hardware operation; the driver should use
 *			the vb2_buffer_done() function to give the buffer back
 *			after an operation is finished;
 */
struct vb2_ops {
	int (*queue_setup)(struct vb2_queue *q, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned long sizes[],
			   void *alloc_ctxs[]);

	void (*wait_prepare)(struct vb2_queue *q);
	void (*wait_finish)(struct vb2_queue *q);

	int (*buf_init)(struct vb2_buffer *vb);
	int (*buf_prepare)(struct vb2_buffer *vb);
	int (*buf_finish)(struct vb2_buffer *vb);
	void (*buf_cleanup)(struct vb2_buffer *vb);

	int (*start_streaming)(struct vb2_queue *q);
	int (*stop_streaming)(struct vb2_queue *q);

	void (*buf_queue)(struct vb2_buffer *vb);
};

/**
 * struct vb2_queue - a videobuf queue
 *
 * @type:	queue type (see V4L2_BUF_TYPE_* in linux/videodev2.h
 * @io_modes:	supported io methods (see vb2_io_modes enum)
 * @io_flags:	additional io flags (see vb2_fileio_flags enum)
 * @ops:	driver-specific callbacks
 * @mem_ops:	memory allocator specific callbacks
 * @drv_priv:	driver private data
 * @buf_struct_size: size of the driver-specific buffer structure;
 *		"0" indicates the driver doesn't want to use a custom buffer
 *		structure type, so sizeof(struct vb2_buffer) will is used
 *
 * @memory:	current memory type used
 * @bufs:	videobuf buffer structures
 * @num_buffers: number of allocated/used buffers
 * @queued_list: list of buffers currently queued from userspace
 * @queued_count: number of buffers owned by the driver
 * @done_list:	list of buffers ready to be dequeued to userspace
 * @done_lock:	lock to protect done_list list
 * @done_wq:	waitqueue for processes waiting for buffers ready to be dequeued
 * @alloc_ctx:	memory type/allocator-specific contexts for each plane
 * @streaming:	current streaming state
 * @fileio:	file io emulator internal data, used only if emulator is active
 */
struct vb2_queue {
	enum v4l2_buf_type		type;
	unsigned int			io_modes;
	unsigned int			io_flags;

	const struct vb2_ops		*ops;
	const struct vb2_mem_ops	*mem_ops;
	void				*drv_priv;
	unsigned int			buf_struct_size;

/* private: internal use only */
	enum v4l2_memory		memory;
	struct vb2_buffer		*bufs[VIDEO_MAX_FRAME];
	unsigned int			num_buffers;

	struct list_head		queued_list;

	atomic_t			queued_count;
	struct list_head		done_list;
	spinlock_t			done_lock;
	wait_queue_head_t		done_wq;

	void				*alloc_ctx[VIDEO_MAX_PLANES];

	unsigned int			streaming:1;

	struct vb2_fileio_data		*fileio;
};

void *vb2_plane_vaddr(struct vb2_buffer *vb, unsigned int plane_no);
void *vb2_plane_cookie(struct vb2_buffer *vb, unsigned int plane_no);

void vb2_buffer_done(struct vb2_buffer *vb, enum vb2_buffer_state state);
int vb2_wait_for_all_buffers(struct vb2_queue *q);

int vb2_querybuf(struct vb2_queue *q, struct v4l2_buffer *b);
int vb2_reqbufs(struct vb2_queue *q, struct v4l2_requestbuffers *req);

int vb2_queue_init(struct vb2_queue *q);

void vb2_queue_release(struct vb2_queue *q);

int vb2_qbuf(struct vb2_queue *q, struct v4l2_buffer *b);
int vb2_dqbuf(struct vb2_queue *q, struct v4l2_buffer *b, bool nonblocking);

int vb2_streamon(struct vb2_queue *q, enum v4l2_buf_type type);
int vb2_streamoff(struct vb2_queue *q, enum v4l2_buf_type type);

int vb2_mmap(struct vb2_queue *q, struct vm_area_struct *vma);
unsigned int vb2_poll(struct vb2_queue *q, struct file *file, poll_table *wait);
size_t vb2_read(struct vb2_queue *q, char __user *data, size_t count,
		loff_t *ppos, int nonblock);
size_t vb2_write(struct vb2_queue *q, char __user *data, size_t count,
		loff_t *ppos, int nonblock);

/**
 * vb2_is_streaming() - return streaming status of the queue
 * @q:		videobuf queue
 */
static inline bool vb2_is_streaming(struct vb2_queue *q)
{
	return q->streaming;
}

/**
 * vb2_is_busy() - return busy status of the queue
 * @q:		videobuf queue
 *
 * This function checks if queue has any buffers allocated.
 */
static inline bool vb2_is_busy(struct vb2_queue *q)
{
	return (q->num_buffers > 0);
}

/**
 * vb2_get_drv_priv() - return driver private data associated with the queue
 * @q:		videobuf queue
 */
static inline void *vb2_get_drv_priv(struct vb2_queue *q)
{
	return q->drv_priv;
}

/**
 * vb2_set_plane_payload() - set bytesused for the plane plane_no
 * @vb:		buffer for which plane payload should be set
 * @plane_no:	plane number for which payload should be set
 * @size:	payload in bytes
 */
static inline void vb2_set_plane_payload(struct vb2_buffer *vb,
				 unsigned int plane_no, unsigned long size)
{
	if (plane_no < vb->num_planes)
		vb->v4l2_planes[plane_no].bytesused = size;
}

/**
 * vb2_get_plane_payload() - get bytesused for the plane plane_no
 * @vb:		buffer for which plane payload should be set
 * @plane_no:	plane number for which payload should be set
 * @size:	payload in bytes
 */
static inline unsigned long vb2_get_plane_payload(struct vb2_buffer *vb,
				 unsigned int plane_no)
{
	if (plane_no < vb->num_planes)
		return vb->v4l2_planes[plane_no].bytesused;
	return 0;
}

/**
 * vb2_plane_size() - return plane size in bytes
 * @vb:		buffer for which plane size should be returned
 * @plane_no:	plane number for which size should be returned
 */
static inline unsigned long
vb2_plane_size(struct vb2_buffer *vb, unsigned int plane_no)
{
	if (plane_no < vb->num_planes)
		return vb->v4l2_planes[plane_no].length;
	return 0;
}

#endif /* _MEDIA_VIDEOBUF2_CORE_H */
