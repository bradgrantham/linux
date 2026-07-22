// SPDX-License-Identifier: GPL-2.0
/*
 * Griffin CompactFlash (True IDE, 8-bit PIO) block driver.
 *
 * The CF sits at 0xF40000 with byte-wide ATA registers on odd addresses
 * (stride 2), put into 8-bit data mode (SET FEATURES 0x01) so the data
 * register is read/written a byte at a time.  Register map and PIO sequence
 * mirror the Griffin ROM firmware (firmware/rom.cpp, firmware/cf.h) and the
 * u-boot driver of the same name (drivers/block/griffin_cf.c in the u-boot
 * tree) -- three independent ports of the same protocol, no shared code since
 * they're different projects (ROM/u-boot/kernel) with different APIs.
 *
 * No IRQ: griffin.yml gives CF no interrupt line, so I/O is fully polled;
 * .queue_rq runs synchronously (BLK_MQ_F_BLOCKING), matching how short these
 * transfers are.
 */

#include <linux/blk-mq.h>
#include <linux/blkdev.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define CF_DRIVER_NAME	"griffin-cf"
#define CF_DEVICE_NAME	"cf"

/* Register offsets from the CF base (byte-wide, odd addresses). */
#define CF_DATA		0x01
#define CF_FEATURES	0x03	/* w */
#define CF_SECCNT	0x05
#define CF_LBA0		0x07
#define CF_LBA1		0x09
#define CF_LBA2		0x0B
#define CF_DEVICE	0x0D
#define CF_STATUS	0x0F	/* r */
#define CF_COMMAND	0x0F	/* w */

#define CF_ST_BSY	0x80
#define CF_ST_DRDY	0x40
#define CF_ST_DRQ	0x08
#define CF_ST_ERR	0x01

#define CF_DH_LBA	0xE0

#define CF_CMD_READ	0x20
#define CF_CMD_WRITE	0x30
#define CF_CMD_IDENTIFY	0xEC
#define CF_CMD_SETFEAT	0xEF
#define CF_FEAT_8BIT	0x01

#define CF_POLL		1000000

struct griffin_cf_dev {
	struct device *dev;
	void __iomem *base;
	struct gendisk *disk;
	struct blk_mq_tag_set tag_set;
};

static inline u8 cf_rd(struct griffin_cf_dev *cf, unsigned reg)
{
	return readb(cf->base + reg);
}

static inline void cf_wr(struct griffin_cf_dev *cf, unsigned reg, u8 val)
{
	writeb(val, cf->base + reg);
}

static int cf_wait_not_busy(struct griffin_cf_dev *cf)
{
	for (u32 i = 0; i < CF_POLL; i++)
		if (!(cf_rd(cf, CF_STATUS) & CF_ST_BSY))
			return 0;
	return -ETIMEDOUT;
}

static int cf_wait_drq(struct griffin_cf_dev *cf)
{
	for (u32 i = 0; i < CF_POLL; i++) {
		u8 s = cf_rd(cf, CF_STATUS);
		if (s & CF_ST_ERR)
			return -EIO;
		if (!(s & CF_ST_BSY) && (s & CF_ST_DRQ))
			return 0;
	}
	return -ETIMEDOUT;
}

static void cf_set_lba(struct griffin_cf_dev *cf, sector_t lba, u8 count)
{
	cf_wr(cf, CF_SECCNT, count);
	cf_wr(cf, CF_LBA0, lba & 0xFF);
	cf_wr(cf, CF_LBA1, (lba >> 8) & 0xFF);
	cf_wr(cf, CF_LBA2, (lba >> 16) & 0xFF);
	cf_wr(cf, CF_DEVICE, CF_DH_LBA | ((lba >> 24) & 0x0F));
}

static int cf_read_sector(struct griffin_cf_dev *cf, sector_t lba, void *buf)
{
	int ret = cf_wait_not_busy(cf);
	if (ret)
		return ret;

	cf_set_lba(cf, lba, 1);
	cf_wr(cf, CF_COMMAND, CF_CMD_READ);

	ret = cf_wait_drq(cf);
	if (ret)
		return ret;

	for (int i = 0; i < SECTOR_SIZE; i++)
		((u8 *)buf)[i] = cf_rd(cf, CF_DATA);
	return 0;
}

static int cf_write_sector(struct griffin_cf_dev *cf, sector_t lba, const void *buf)
{
	int ret = cf_wait_not_busy(cf);
	if (ret)
		return ret;

	cf_set_lba(cf, lba, 1);
	cf_wr(cf, CF_COMMAND, CF_CMD_WRITE);

	ret = cf_wait_drq(cf);
	if (ret)
		return ret;

	for (int i = 0; i < SECTOR_SIZE; i++)
		cf_wr(cf, CF_DATA, ((const u8 *)buf)[i]);
	return cf_wait_not_busy(cf);	/* wait for the write to actually land */
}

static int cf_init(struct griffin_cf_dev *cf)
{
	if (cf_wait_not_busy(cf))
		return -ETIMEDOUT;
	for (u32 i = 0; i < CF_POLL; i++)
		if (cf_rd(cf, CF_STATUS) & CF_ST_DRDY)
			goto ready;
	return -ETIMEDOUT;
ready:
	cf_wr(cf, CF_FEATURES, CF_FEAT_8BIT);
	cf_wr(cf, CF_COMMAND, CF_CMD_SETFEAT);
	return cf_wait_not_busy(cf);
}

/* Total user-addressable sectors from IDENTIFY words 60:61 (LBA28). */
static sector_t cf_identify_capacity(struct griffin_cf_dev *cf)
{
	u8 id[SECTOR_SIZE];
	u32 w60, w61;

	if (cf_wait_not_busy(cf))
		return 0;
	cf_wr(cf, CF_DEVICE, CF_DH_LBA);
	cf_wr(cf, CF_COMMAND, CF_CMD_IDENTIFY);
	if (cf_wait_drq(cf))
		return 0;
	for (int i = 0; i < SECTOR_SIZE; i++)
		id[i] = cf_rd(cf, CF_DATA);

	w60 = id[120] | (id[121] << 8);
	w61 = id[122] | (id[123] << 8);
	return (sector_t)(w60 | (w61 << 16));
}

static blk_status_t griffin_cf_queue_rq(struct blk_mq_hw_ctx *hctx,
					const struct blk_mq_queue_data *bd)
{
	struct griffin_cf_dev *cf = hctx->queue->queuedata;
	struct request *rq = bd->rq;
	struct req_iterator iter;
	struct bio_vec bvec;
	sector_t lba = blk_rq_pos(rq);
	int ret = 0;

	blk_mq_start_request(rq);

	switch (req_op(rq)) {
	case REQ_OP_READ:
		rq_for_each_segment(bvec, rq, iter) {
			unsigned nsec = bvec.bv_len >> SECTOR_SHIFT;
			void *dst = page_address(bvec.bv_page) + bvec.bv_offset;

			for (unsigned i = 0; i < nsec; i++) {
				ret = cf_read_sector(cf, lba, dst);
				if (ret)
					goto done;
				dst += SECTOR_SIZE;
				lba++;
				cond_resched();
			}
		}
		break;
	case REQ_OP_WRITE:
		rq_for_each_segment(bvec, rq, iter) {
			unsigned nsec = bvec.bv_len >> SECTOR_SHIFT;
			void *src = page_address(bvec.bv_page) + bvec.bv_offset;

			for (unsigned i = 0; i < nsec; i++) {
				ret = cf_write_sector(cf, lba, src);
				if (ret)
					goto done;
				src += SECTOR_SIZE;
				lba++;
				cond_resched();
			}
		}
		break;
	default:
		ret = -EIO;
		break;
	}

done:
	blk_mq_end_request(rq, ret ? BLK_STS_IOERR : BLK_STS_OK);
	return BLK_STS_OK;
}

static const struct blk_mq_ops griffin_cf_mq_ops = {
	.queue_rq = griffin_cf_queue_rq,
};

static const struct block_device_operations griffin_cf_fops = {
	.owner = THIS_MODULE,
};

static int griffin_cf_probe(struct platform_device *pdev)
{
	struct queue_limits lim = {
		.logical_block_size  = SECTOR_SIZE,
		.physical_block_size = SECTOR_SIZE,
	};
	struct device *dev = &pdev->dev;
	struct griffin_cf_dev *cf;
	sector_t capacity;
	int ret;

	cf = devm_kzalloc(dev, sizeof(*cf), GFP_KERNEL);
	if (!cf)
		return -ENOMEM;
	cf->dev = dev;

	cf->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cf->base))
		return PTR_ERR(cf->base);

	ret = cf_init(cf);
	if (ret) {
		dev_err(dev, "card init failed (%d)\n", ret);
		return ret;
	}

	capacity = cf_identify_capacity(cf);
	if (!capacity) {
		dev_err(dev, "IDENTIFY returned no capacity\n");
		return -EIO;
	}

	ret = blk_mq_alloc_sq_tag_set(&cf->tag_set, &griffin_cf_mq_ops, 1,
				      BLK_MQ_F_BLOCKING);
	if (ret)
		return ret;

	cf->disk = blk_mq_alloc_disk(&cf->tag_set, &lim, cf);
	if (IS_ERR(cf->disk)) {
		ret = PTR_ERR(cf->disk);
		goto err_free_tagset;
	}
	cf->disk->fops = &griffin_cf_fops;
	cf->disk->private_data = cf;
	snprintf(cf->disk->disk_name, DISK_NAME_LEN, CF_DEVICE_NAME);
	set_capacity(cf->disk, capacity);

	platform_set_drvdata(pdev, cf);

	ret = device_add_disk(dev, cf->disk, NULL);
	if (ret)
		goto err_put_disk;

	dev_info(dev, "%llu sectors (%llu MiB)\n",
		 (unsigned long long)capacity,
		 (unsigned long long)(capacity >> 11));
	return 0;

err_put_disk:
	put_disk(cf->disk);
err_free_tagset:
	blk_mq_free_tag_set(&cf->tag_set);
	return ret;
}

static void griffin_cf_remove(struct platform_device *pdev)
{
	struct griffin_cf_dev *cf = platform_get_drvdata(pdev);

	del_gendisk(cf->disk);
	put_disk(cf->disk);
	blk_mq_free_tag_set(&cf->tag_set);
}

static const struct of_device_id griffin_cf_ids[] = {
	{ .compatible = "griffin,cf-ide" },
	{ }
};
MODULE_DEVICE_TABLE(of, griffin_cf_ids);

static struct platform_driver griffin_cf_driver = {
	.probe	= griffin_cf_probe,
	.remove	= griffin_cf_remove,
	.driver	= {
		.name		= CF_DRIVER_NAME,
		.of_match_table	= griffin_cf_ids,
	},
};
module_platform_driver(griffin_cf_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Griffin CompactFlash (True IDE 8-bit PIO) block driver");
