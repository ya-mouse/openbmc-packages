/*
 * simple driver for PWM (Pulse Width Modulator) controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Derived from pxa PWM driver by eric miao <eric.miao@marvell.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <dt-bindings/pwm/ast-pwm.h>

#define PWMTACH_GEN			0x00
#define PWMTACH_CLK			0x04
#define PWMTACH_DUTY_0			0x08
#define PWMTACH_DUTY_1			0x0C
#define PWMTACH_EXT_GEN			0x40
#define PWMTACH_EXT_CLK			0x44
#define PWMTACH_DUTY_2			0x48
#define PWMTACH_DUTY_3			0x4C

#define SCU_PASSWORD	0x1688A8A8
#define AST_BASE_SCU	0x1E6E2000

#define AST_IO_VA	0xf0000000
#define AST_IO_PA	0x1e600000
#define AST_IO_SZ	0x00200000

#define AST_IO(__pa)	((void __iomem *)(((__pa) & 0x001fffff) | AST_IO_VA))

#define ast_writel(v, reg)		writel((v), ast->mmio_base + (reg))
#define ast_readl(reg)			readl(ast->mmio_base + (reg))

struct ast_pwm_device {
	struct ast_chip *ast;
	unsigned int channel;
	u32 duty_ns;
	u16 type;
	u16 merged;
};

struct ast_pwm_type {
	u16 reg;
	u16 shift;
	u32 period;
	u32 period_ns;
};

struct ast_chip {
	struct clk	*clk;

	void __iomem	*mmio_base;

	struct pwm_chip	chip;
	struct ast_pwm_type	*types;
};

#define to_ast_chip(chip)	container_of(chip, struct ast_chip, chip)

static struct ast_pwm_type ast_pwm_types_tbl[] = {
	{ .reg = 0x04, .shift = 0,  .period_ns = 42667 }, /* Type M */
	{ .reg = 0x04, .shift = 16, .period_ns = 42667 }, /* Type N */
	{ .reg = 0x44, .shift = 0,  .period_ns = 42667 }, /* Type O */
};

/* from sysfs.c */

struct pwm_export {
	struct device child;
	struct pwm_device *pwm;
};

static struct pwm_export *child_to_pwm_export(struct device *child)
{
	return container_of(child, struct pwm_export, child);
}

static struct pwm_device *child_to_pwm_device(struct device *child)
{
	struct pwm_export *export = child_to_pwm_export(child);

	return export->pwm;
}

/* end from sysfs.c */

static int chip_get_first(struct device *child, void *data)
{
	return 1;
}

static ssize_t type_show(struct device *child,
			   struct device_attribute *attr,
			   char *buf)
{
	struct pwm_device *_pwm = child_to_pwm_device(child);
	struct ast_pwm_device *pwm = pwm_get_chip_data(_pwm);

	return sprintf(buf, "%d\n", pwm->type);
}

static int ast_pwm_config(struct pwm_chip *chip,
		struct pwm_device *_pwm, int duty_ns, int period_ns);

static ssize_t type_store(struct device *child,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct pwm_device *_pwm = child_to_pwm_device(child);
	struct ast_pwm_device *pwm = pwm_get_chip_data(_pwm);
	int val, ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (pwm_is_enabled(_pwm))
		return -EINVAL;

	switch (val) {
	case PWM_TYPE_M:
	case PWM_TYPE_N:
	case PWM_TYPE_O:
		pwm->type = val;
		ast_pwm_config(_pwm->chip, _pwm,
			       pwm->ast->types[val].period_ns, pwm->duty_ns);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret ? : size;
}

static DEVICE_ATTR_RW(type);

static struct attribute *ast_pwm_attrs[] = {
	&dev_attr_type.attr,
	NULL
};

static struct attribute_group ast_pwm_group = {
	.attrs	= ast_pwm_attrs,
};

static int ast_pwm_request(struct pwm_chip *chip, struct pwm_device *_pwm)
{
	struct ast_chip *ast = to_ast_chip(chip);
	struct ast_pwm_device *pwm;

//	if (_pwm->hwpwm >= TPU_CHANNEL_MAX)
//		return -EINVAL;

	pwm = kzalloc(sizeof(*pwm), GFP_KERNEL);
	if (pwm == NULL)
		return -ENOMEM;

	pwm->ast = ast;
	pwm->channel = _pwm->hwpwm;
	pwm->type = PWM_TYPE_M;
	pwm->duty_ns = 1;
	pwm->merged = 0;
	pwm_set_period(_pwm, ast->types[pwm->type].period_ns);
	pwm_set_duty_cycle(_pwm, pwm->duty_ns);

	pwm_set_chip_data(_pwm, pwm);

	return 0;
}

static void ast_pwm_free(struct pwm_chip *chip, struct pwm_device *_pwm)
{
	struct ast_pwm_device *pwm = pwm_get_chip_data(_pwm);

	kfree(pwm);
}

static int ast_pwm_config(struct pwm_chip *chip,
		struct pwm_device *_pwm, int duty_ns, int period_ns)
{
	char pwm_name[] = "pwmXX";
	struct ast_chip *ast = to_ast_chip(chip);
	struct device *child;
	struct device *dev = chip->dev;
	unsigned long long c;
	bool enable = pwm_is_enabled(_pwm);
	struct ast_pwm_device *pwm = pwm_get_chip_data(_pwm);
	struct ast_pwm_type *type = &ast->types[pwm->type];
	u32 period_cycles, duty_cycles;
	u32 reg, val, offset;
	int ret;

	/* HACK: expose "type" attribute to pwm channel group */
	if (!enable && !pwm->merged) {
		child = device_find_child(dev, _pwm, chip_get_first);
		if (!child)
			return -ENODEV;

		snprintf(pwm_name, 7, "pwm%d", pwm->channel);
		ast_pwm_group.name = pwm_name;
		ret = sysfs_merge_group(&child->kobj, &ast_pwm_group);
		put_device(child);
		if (ret)
			return ret;

		pwm->merged = 1;
		return 0;
	}

	if (!enable) {
		if (pwm->channel < 4) {
			reg = PWMTACH_GEN;
			offset = pwm->channel;
		} else {
			reg = PWMTACH_EXT_GEN;
			offset = pwm->channel - 4;
		}
		val = ast_readl(reg);
		/* NOTE: Type O is "1x", but we check exactly for "10" */
		if ((val & (0x1010 << offset)) != pwm->type) {
			printk("type=%08x -> ", val);
			val &= ~ (0x1010 << offset);
			val |= (pwm->type & 0x2) << (offset + 3);
			val |= (pwm->type & 0x1) << (offset + 12);
			printk("%08x\n", val);
			ast_writel(val, reg);
		}
	}

	/* 0xFF11 --> 24000000 / (2 * 2 * 256) = 23437.5 Hz */

	/* (2 * 2 * 256) / 24000000 = sec */
	c = clk_get_rate(ast->clk);
	c = c * period_ns;
	do_div(c, 1000000000);
	period_cycles = c;

	if (!period_cycles)
		return -EINVAL;

	if (type->period_ns != period_ns && type->period != period_cycles) {
		int minv;
		int min_i = 0;
		int min_j = 0;
		register int i, j, k, v, pv, p;

		p = period_cycles >> 8;
		minv = p;
		/* Brute force searching */
		for (i=0; i<16; i++) {
			v = p;
			pv = 0;
			k = (i ? i*2 : 1);
			for (j = 0; j < 16; j++) {
				v = (1 << j) * k;
				if (v >= p) {
					if ((p - pv) > (v - p)) {
						if ((v - p) < minv) {
							minv = (v-p);
							min_i = i;
							min_j = j;
						}
					} else {
						if ((v - pv) < minv) {
							minv = (v-pv);
							min_i = i;
							min_j = j-1;
						}
					}
					break;
				}
				pv = v;
			}
			/* Other values much bigger */
			if (v >= p && j == 0)
				break;
			/* Find exact value */
			if (v == p)
				break;
		}
		printk("minv=%d i=%d j=%d\n", minv, min_i, min_j);
		type->period = period_cycles;
		type->period_ns = period_ns;

		/* Set period and clock divisions */
		val = ast_readl(type->reg);
		val &= ~(0xffff << type->shift);
		val |= (0xff00 | (min_j << 4) | min_i) << type->shift;
		ast_writel(val, type->reg);
	}

	c = (unsigned long long)0xff * duty_ns;
	do_div(c, period_ns);
	duty_cycles = c;

	pwm->duty_ns = duty_ns;

	if (pwm->channel < 4) {
		reg = PWMTACH_DUTY_0 + ((pwm->channel >> 1) << 2);
		offset = (pwm->channel % 2) << 4;
	} else {
		reg = PWMTACH_DUTY_2 + (((pwm->channel - 4) >> 1) << 2);
		offset = ((pwm->channel - 4) % 2) << 4;
	}

	val = ast_readl(reg);
	printk("reg%02x=%08x ", reg, val);
	val &= ~(0xffff << offset);
	ast_writel(val | ((duty_cycles << 4 | 0x00) << offset), reg);
	printk("-> %08x\n", val | ((duty_cycles << 4 | 0x00) << offset));

	printk("period_cycles=%ld duty_cycles=%ld\n", period_cycles, duty_cycles);

	return 0;
}

static int ast_pwm_start_stop(struct ast_chip *ast, struct pwm_device *_pwm, int start)
{
	struct ast_pwm_device *pwm = pwm_get_chip_data(_pwm);
	u32 reg;
	u32 val;
	u32 mask;

	if (pwm->channel < 4) {
		reg = PWMTACH_GEN;
		mask = 0x100 << pwm->channel;
	} else {
		reg = PWMTACH_EXT_GEN;
		mask = 0x100 << (pwm->channel - 4);
	}

	val = ast_readl(reg);
	if (start)
		val |= mask;
	else
		val &= ~mask;
	ast_writel(val, reg);

	return 0;
}

static int ast_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ast_chip *ast = to_ast_chip(chip);
	int ret;

	ret = clk_prepare_enable(ast->clk);
	if (ret)
		return ret;

	return ast_pwm_start_stop(ast, pwm, true);
}

static void ast_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ast_chip *ast = to_ast_chip(chip);

	ast_pwm_start_stop(ast, pwm, false);

	clk_disable_unprepare(ast->clk);
}

static struct pwm_ops ast_pwm_ops = {
	.request = ast_pwm_request,
	.free = ast_pwm_free,
	.enable = ast_pwm_enable,
	.disable = ast_pwm_disable,
	.config = ast_pwm_config,
	.owner = THIS_MODULE,
};

static const struct of_device_id ast_pwm_dt_ids[] = {
	{ .compatible = "aspeed,pwm" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ast_pwm_dt_ids);

static void ast_pwm_init_one(struct ast_chip *ast)
{
	u32 reg;

	/* FIXME: should be a part of board init function */
	writel(SCU_PASSWORD, AST_IO(AST_BASE_SCU));

	/* SCU Pin-MUX: PWM & TACHO */
	reg = readl(AST_IO(AST_BASE_SCU | 0x88)); // MULTI_FN_PIN_3
	reg &= ~0xcfffff;
	writel(reg | 0xc000ff, AST_IO(AST_BASE_SCU | 0x88));

	reg = readl(AST_IO(AST_BASE_SCU | 0x04));
	reg &= ~0x200; /* stop the reset */
	writel(reg, AST_IO(AST_BASE_SCU | 0x04));
	/* Lock SCU */
	writel(0, AST_IO(AST_BASE_SCU));
	/* END FIXME */


	/* Set clock division and period of type M/N */
	/* 0xFF11 --> 24000000 / (2 * 2 * 256) = 23437.5 Hz */
	ast_writel(0xff11ff11, PWMTACH_CLK);
	reg = ast_readl(PWMTACH_GEN);
	reg &= ~(0xf0f0);
	ast_writel(reg, PWMTACH_GEN);

	/* Set clock division and period of type O */
	ast_writel(0xff11, PWMTACH_EXT_CLK);
	reg = ast_readl(PWMTACH_EXT_GEN);
	reg &= ~(0xf0f0);
	ast_writel(reg, PWMTACH_EXT_GEN);

	/* Initialize all channels to lowest speed (0x01) */
	ast_writel(0x01000100, PWMTACH_DUTY_0);
	ast_writel(0x01000100, PWMTACH_DUTY_1);

	if (ast->chip.npwm > 4) {
		ast_writel(0x01000100, PWMTACH_DUTY_2);
		ast_writel(0x01000100, PWMTACH_DUTY_3);
	}
}

static int ast_pwm_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(ast_pwm_dt_ids, &pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	struct ast_chip *ast;
	struct resource *r;
	int ret = 0;

	if (!of_id)
		return -ENODEV;

	ast = devm_kzalloc(&pdev->dev, sizeof(*ast), GFP_KERNEL);
	if (ast == NULL)
		return -ENOMEM;

	ast->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ast->clk)) {
		dev_err(&pdev->dev, "getting clock failed with %ld\n",
				PTR_ERR(ast->clk));
		return PTR_ERR(ast->clk);
	}

	ast->chip.ops = &ast_pwm_ops;
	ast->chip.dev = &pdev->dev;
	ast->chip.base = -1;
	ast->chip.can_sleep = true;
	ret = of_property_read_u32(np, "pwm-number", &ast->chip.npwm);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get pwm number: %d\n", ret);
		return ret;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ast->mmio_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(ast->mmio_base))
		return PTR_ERR(ast->mmio_base);

	ast->types = ast_pwm_types_tbl;

	ast_pwm_init_one(ast);

	ret = pwmchip_add(&ast->chip);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, ast);
	return 0;
}

static int ast_pwm_remove(struct platform_device *pdev)
{
	struct ast_chip *ast;

	ast = platform_get_drvdata(pdev);
	if (ast == NULL)
		return -ENODEV;

	return pwmchip_remove(&ast->chip);
}

static struct platform_driver ast_pwm_driver = {
	.driver		= {
		.name	= "ast-pwm",
		.of_match_table = ast_pwm_dt_ids,
	},
	.probe		= ast_pwm_probe,
	.remove		= ast_pwm_remove,
};

module_platform_driver(ast_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Anton D. Kachalov <mouse@yandex-team.ru>");
