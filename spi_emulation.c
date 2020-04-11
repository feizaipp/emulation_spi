/* 
 *	Author:feizaipp
 *	simulate spi for mtk
 *	spi_simulate.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <cust_gpio_boot.h>
#include <mach/mt_gpio.h>

#define GPIO_DOUT11 0x100054A0
#define GPIO_DIN11  0x100055A0

struct spi_simulate_t {
	struct platform_device *pdev;
	struct wake_lock wk_lock;
	struct mt_chip_conf *config;	
	struct spi_master *master;
	struct spi_transfer *cur_transfer;
	struct spi_transfer *next_transfer;
	spinlock_t lock;
	struct list_head	queue;
	struct work_struct read_work;
};

static struct workqueue_struct *spi_simulate_wq;

static const struct of_device_id spi_simulate_of_match[] = {
	{ .compatible = "mediatek,SPI1",},
	{},
};

volatile static unsigned short *gpio_dout11;
volatile static unsigned short *gpio_din11;

static void spi_simulate_next_message(struct spi_simulate_t *ss);
static void spi_simulate_msg_done(struct spi_simulate_t *ss, struct spi_message *msg, int status);
static int spi_simulate_next_xfer(struct spi_simulate_t *ss, struct spi_message *msg);

static inline int is_last_xfer(struct spi_message *msg, struct spi_transfer *xfer)
{
	return msg->transfers.prev == &xfer->transfer_list;
}

static void cs_enable(int enable)
{
	if (enable) {
		mt_set_gpio_out(GPIO_SPI_CS_PIN, 0);
	} else {
		mt_set_gpio_out(GPIO_SPI_CS_PIN, 1);
	}
}

/*
static void cs_enable(int enable)
{
	if (enable) {
		*gpio_dout11 &= ~(1<<9);	
	} else {
		*gpio_dout11 |= (1<<9);	
	}
}
*/

/*
static void cs_enable(int enable)
{
	if (enable) {
		mt_set_simulate_dout(gpio_dout11, 9, 0);
	} else {
		mt_set_simulate_dout(gpio_dout11, 9, 1);
	}
}
*/

static int spi_simulate_setup (struct spi_device *spidev)
{
	struct spi_master *master;
	struct spi_simulate_t *ss;

	master = spidev->master;
	ss = spi_master_get_devdata(master);
	if(!spidev){
		dev_err( &spidev->dev,"spi device %s: error.\n", 
			dev_name( &spidev->dev ) );
	}
	if(spidev->chip_select >= master->num_chipselect){
		dev_err( &spidev->dev,"spi device chip select excesses \
			the number of master's chipselect number.\n");
		return -EINVAL;
	}
	
	/* 166 SCK */
	mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_SPI_SCK_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_DISABLE);
	
	/* 167 MISO */
	mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_SPI_MISO_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_DISABLE);
	
	/* 168 MOSI */
	mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_SPI_MOSI_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_DISABLE);
	
	/* 169 CS */
	mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_DISABLE);

	mt_set_gpio_out(GPIO_SPI_SCK_PIN, 1); /* SCK = 1 */
	mt_set_gpio_out(GPIO_SPI_CS_PIN, 1);  /* CS   = 1 */
	return 0;
}

static unsigned char spi_readwrite(unsigned char buf)
{
	unsigned char i, val, bit;

	val = 0;
	bit = 0;
	for (i=0; i<8; i++) {
		if (buf & 0x80) {
			mt_set_gpio_out(GPIO_SPI_MOSI_PIN, 1);
		} else {
			mt_set_gpio_out(GPIO_SPI_MOSI_PIN, 0);
		}
		buf <<= 1;
		mt_set_gpio_out(GPIO_SPI_SCK_PIN, 0);
		mt_set_gpio_out(GPIO_SPI_SCK_PIN, 1);
		val <<= 1;
		bit = mt_get_gpio_in(GPIO_SPI_MISO_PIN);
		val |= bit;
	}
	return val;
}

static void spi_simulate_transfer_bytes(unsigned char *tx_buf, unsigned char *rx_buf, int len)
{
	int i;

	cs_enable(1);
	for (i=0; i<len; i++) {
		if (rx_buf)
			rx_buf[i] = spi_readwrite(tx_buf[i]);
		else 
			spi_readwrite(tx_buf[i]);
	}
	cs_enable(0);
}

static void spi_simulate_read(struct work_struct *work)
{}

static void spi_simulate_msg_done(struct spi_simulate_t *ss, struct spi_message *msg, int status)
{
	list_del(&msg->queue);
	msg->status = status;
	msg->complete(msg->context);

	ss->cur_transfer	= NULL;
	ss->next_transfer	= NULL;

	if(list_empty(&ss->queue)){
		wake_unlock(&ss->wk_lock);
	}else {
		spi_simulate_next_message(ss);
	}
}

static int spi_simulate_next_xfer(struct spi_simulate_t *ss, struct spi_message *msg)
{
	struct spi_transfer	*xfer;
	int ret = 0;

	if(unlikely(!ss)){
		dev_err(&msg->spi->dev, "master wrapper is invalid \n");
		ret = -EINVAL;
		goto fail;
	}
	if(unlikely(!msg)){
		dev_err(&msg->spi->dev, "msg is invalid \n");
		ret = -EINVAL;
		goto fail;
	}

	xfer = ss->cur_transfer;

	if(is_last_xfer(msg,xfer)){
		ss->next_transfer = NULL;
	}else{
		ss->next_transfer = list_entry(xfer->transfer_list.next, struct spi_transfer, transfer_list );
	}

	/* send data */
	spi_simulate_transfer_bytes((unsigned char *)xfer->tx_buf, (unsigned char *)xfer->rx_buf, xfer->len);

	msg->actual_length += xfer->len;
	if (is_last_xfer(msg, xfer)) {
		spi_simulate_msg_done(ss, msg, 0);
	} else {
		ss->cur_transfer = ss->next_transfer;		
		spi_simulate_next_xfer(ss, msg);
	}
	
	//queue_work(spi_simulate_wq, &ss->read_work);
	
	return 0;
fail:
	spi_simulate_msg_done(ss, msg, ret);
	return ret;
}

static void spi_simulate_next_message(struct spi_simulate_t *ss)
{
	struct spi_message	*msg;

	msg = list_entry(ss->queue.next, struct spi_message, queue);
	ss->cur_transfer = list_entry( msg->transfers.next, struct spi_transfer, transfer_list);
	spi_simulate_next_xfer(ss, msg);
}

static int spi_simulate_transfer(struct spi_device *spidev, struct spi_message *msg)
{
	struct spi_master *master ;
	struct spi_simulate_t *ss;	
	struct spi_transfer *xfer;
	unsigned long		flags;

	master = spidev->master;
	ss = spi_master_get_devdata(master);
	if(unlikely(!msg)){
		dev_err(&spidev->dev, "msg is NULL pointer. \n" );
		msg->status = -EINVAL;
		goto out;
	}
	if(unlikely(list_empty(&msg->transfers))) {		
		dev_err(&spidev->dev, "the message is NULL.\n" );
		msg->status = -EINVAL;
		msg->actual_length = 0;
		goto out;
	}
	if(master->setup(spidev)){
		dev_err(&spidev->dev, "set up error.\n");
		msg->status = -EINVAL;
		msg->actual_length = 0;
		goto out;
	}
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!((xfer->tx_buf || xfer->rx_buf) && xfer->len)) {
			dev_err(&spidev->dev,"missing tx %p or rx %p buf, len%d\n",xfer->tx_buf,xfer->rx_buf,xfer->len);
			msg->status = -EINVAL;
			goto out;
		}
	}
	msg->status = -EINPROGRESS;
	msg->actual_length = 0;	

	spin_lock_irqsave(&ss->lock, flags);
	list_add_tail(&msg->queue, &ss->queue);
	if (!ss->cur_transfer){
		wake_lock(&ss->wk_lock);
		spi_simulate_next_message(ss);
	}
	spin_unlock_irqrestore(&ss->lock, flags);
	
	return 0;
out:
	return -1;
}

static void spi_simulate_cleanup(struct spi_device *spidev)
{
	struct spi_master *master;
	struct spi_simulate_t *ss;

	master = spidev->master;
	ss = spi_master_get_devdata(master);
	spidev->master = NULL;
}

static int spi_simulate_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct spi_master *master;
	struct spi_simulate_t *ss;

#ifdef CONFIG_OF
	if (of_property_read_u32(pdev->dev.of_node, "cell-index", &pdev->id)) {
		dev_err(&pdev->dev, "SPI get cell-index failed\n");
		return -ENODEV;
	}
#endif
	master= spi_alloc_master(&pdev->dev, sizeof (struct spi_simulate_t));
	if (!master) {
		dev_err(&pdev->dev, " device %s: alloc spi master fail.\n", dev_name ( &pdev->dev ) );
		goto out;
	}
	master->num_chipselect = 1;
	master->mode_bits = (SPI_CPOL | SPI_CPHA);
	master->bus_num   = pdev->id;
	master->setup     = spi_simulate_setup;
	master->transfer  = spi_simulate_transfer;
	master->cleanup   = spi_simulate_cleanup;
	platform_set_drvdata(pdev, master);
	ss = spi_master_get_devdata(master);
	ss->pdev	      =	pdev;
	ss->cur_transfer  = NULL;
	ss->next_transfer = NULL;
	wake_lock_init (&ss->wk_lock, WAKE_LOCK_SUSPEND, "spi_simulate_wakelock");
	spin_lock_init(&ss->lock);
	INIT_LIST_HEAD(&ss->queue);
	INIT_WORK(&ss->read_work, spi_simulate_read);
	spi_master_set_devdata(master, ss);

	ret = spi_register_master(master);
		
out:
	spi_master_put(master);
	return ret;
}

static int spi_simulate_remove(struct platform_device *pdev)
{
	struct spi_simulate_t *ss;
	struct spi_message *msg;
	struct spi_master *master = platform_get_drvdata (pdev);
	if (!master ) {
		dev_err(&pdev->dev, 
			"master %s: is invalid. \n", 
			dev_name ( &pdev->dev ) );
		return -EINVAL;
	}
	ss = spi_master_get_devdata ( master );
	
	list_for_each_entry(msg, &ss->queue, queue) {
		msg->status = -ESHUTDOWN;
		msg->complete(msg->context);
	}
	ss->cur_transfer = NULL;
	flush_work(&ss->read_work);
	spi_unregister_master(master);
	return 0;
}

static int spi_simulate_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int spi_simulate_resume(struct platform_device *dev)
{
	return 0;
}

struct platform_driver spi_simulate_drv = {
	.driver = {
		.name           = "spi-simulate",
		.owner          = THIS_MODULE,
		.of_match_table = spi_simulate_of_match,
	},
	.probe   = spi_simulate_probe,
	.suspend = spi_simulate_suspend,
	.resume  = spi_simulate_resume,
	.remove  = spi_simulate_remove,
};

static int __init spi_simulate_init(void)
{
	int ret;

	spi_simulate_wq = alloc_workqueue("spi-simulate", 0, 0);
	if (!spi_simulate_wq)
		return -ENOMEM;
	gpio_dout11 = ioremap(GPIO_DOUT11, sizeof(unsigned short));
	if (!gpio_dout11) {
		printk(KERN_ERR "%s,%d,Uable to map I/O register.\n", __FUNCTION__, __LINE__);
		goto fail_dout11;
	}
	gpio_din11 = ioremap(GPIO_DIN11, sizeof(unsigned short));
	if (!gpio_din11) {
		printk(KERN_ERR "%s,%d,Uable to map I/O register.\n", __FUNCTION__, __LINE__);
		goto fail_din11;
	}
	ret = platform_driver_register(&spi_simulate_drv);

	return ret;
fail_din11:
	iounmap(gpio_dout11);
fail_dout11:
	destroy_workqueue(spi_simulate_wq);
	return -ENOMEM;
}

static void __init spi_simulate_exit(void)
{
	platform_driver_unregister(&spi_simulate_drv);
	destroy_workqueue(spi_simulate_wq);
	iounmap(gpio_dout11);
	iounmap(gpio_din11);
}
module_init(spi_simulate_init);
module_exit(spi_simulate_exit);
MODULE_DESCRIPTION ( "simulate SPI Controller driver for mtk" );
MODULE_AUTHOR ( "feizaipp <zpehome@yeah.net>" );
MODULE_LICENSE ( "GPL" );
